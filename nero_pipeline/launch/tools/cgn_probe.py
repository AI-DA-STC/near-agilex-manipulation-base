#!/usr/bin/env python3
"""
cgn_probe.py - why does Contact-GraspNet find no grasps for this object? Runs inside the `contact_graspnet` conda env on the depth and mask of
the LAST /contact_graspnet/detect_grasps call (/tmp/last_depth.npy, /tmp/last_segmap.npy) and tries variants, one result line each:
  - the pipeline's own settings, three times (is the result random?)
  - filter_grasps OFF: does the network find grasps at all, and how close to the object do their contact points land?
  - the mask grown by a few pixels: are the contact points just missing the (tight) mask?
  - more forward passes
It moves nothing and changes no files except printing. Needs /tmp/last_K.json too (written by the one-liner in the instructions).
    conda run -n contact_graspnet --no-capture-output python cgn_probe.py --ckpt_dir ~/contact_graspnet_pytorch/checkpoints/contact_graspnet
"""
import argparse, contextlib, io, json, os, sys, time, traceback
import numpy as np
from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator
from contact_graspnet_pytorch import config_utils
from contact_graspnet_pytorch.checkpoints import CheckpointIO


def dilate(mask, r):
    """Grow a boolean mask by r pixels (3x3 steps, numpy only)."""
    out = mask.copy()
    H, W = out.shape
    for _ in range(r):
        p = np.pad(out, 1)
        out = np.max([p[dy:dy + H, dx:dx + W] for dy in range(3) for dx in range(3)], axis=0)
    return out


def build(ckpt_dir, passes):
    est = GraspEstimator(config_utils.load_config(ckpt_dir, batch_size=passes))
    CheckpointIO(checkpoint_dir=os.path.join(ckpt_dir, "checkpoints"), model=est.model).load("model.pt")
    return est


def nearest_dist(points, ref):
    """distance from each of `points` to the closest point of `ref`"""
    out = []
    for i in range(0, len(points), 400):
        d = np.sqrt(((points[i:i + 400, None, :] - ref[None, :, :]) ** 2).sum(2))
        out.append(d.min(1))
    return np.concatenate(out) if out else np.zeros(0)


def run(est, depth, K, mask, passes, filt, z_range):
    quiet = io.StringIO()
    with contextlib.redirect_stdout(quiet):
        pc_full, pc_segments, _ = est.extract_point_clouds(depth, K, segmap=mask.astype(np.int32), rgb=None, skip_border_objects=False, z_range=z_range)
        seg = pc_segments.get(1)
        if seg is None or len(seg) == 0:
            return dict(points=0)
        t0 = time.time()
        pred, scores, contact, _ = est.predict_scene_grasps(pc_full, pc_segments=pc_segments, local_regions=True, filter_grasps=filt, forward_passes=passes)
        dt = time.time() - t0
    n = int(sum(len(v) for v in pred.values()))
    sc = np.concatenate([np.asarray(v).ravel() for v in scores.values()]) if scores else np.zeros(0)
    cp = np.concatenate([np.asarray(v).reshape(-1, 3) for v in contact.values()]) if contact else np.zeros((0, 3))
    d = nearest_dist(cp, seg) if len(cp) else np.zeros(0)
    return dict(points=len(seg), n=n, best=float(sc.max()) if len(sc) else None, near1=int((d <= 0.01).sum()), near2=int(((d > 0.01) & (d <= 0.02)).sum()),
                far=int((d > 0.02).sum()), secs=dt, ext=seg.max(0) - seg.min(0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt_dir", required=True)
    ap.add_argument("--depth", default="/tmp/last_depth.npy"); ap.add_argument("--segmap", default="/tmp/last_segmap.npy"); ap.add_argument("--K_json", default="/tmp/last_K.json")
    ap.add_argument("--passes", default="3", help="comma list of extra forward_passes values to try, e.g. 3,5")
    a = ap.parse_args()
    depth = np.load(a.depth).astype(np.float64); mask = np.load(a.segmap) > 0
    K = np.array(json.load(open(a.K_json)), dtype=float).reshape(3, 3)
    ys, xs = np.nonzero(mask)
    print(f"input: depth {depth.shape}, valid depth {100 * (depth > 0.05).mean():.0f}% of pixels, mask {int(mask.sum())} px, bbox x {xs.min()}-{xs.max()} y {ys.min()}-{ys.max()}, K fx={K[0,0]:.1f} cx={K[0,2]:.1f}")
    z = [0.2, 1.8]
    extra = [int(p) for p in a.passes.split(",") if p.strip()]
    ests = {1: build(a.ckpt_dir, 1)}
    for p in extra:
        ests[p] = build(a.ckpt_dir, p)
    variants = [("pipeline settings, try 1", 1, 0, True), ("pipeline settings, try 2", 1, 0, True), ("pipeline settings, try 3", 1, 0, True),
                ("filter_grasps OFF", 1, 0, False)]
    variants += [(f"mask grown by {r} px", 1, r, True) for r in (3, 6, 10)]
    for p in extra:
        variants += [(f"forward_passes={p}", p, 0, True), (f"forward_passes={p} + mask grown 6 px", p, 6, True)]
    print()
    print(f"{'variant':<38}{'object pts':>11}{'grasps':>8}{'best':>7}   contact points: <=1cm  1-2cm  >2cm from object   {'sec':>5}")
    for name, p, grow, filt in variants:
        m = dilate(mask, grow) if grow else mask
        try:
            r = run(ests[p], depth, K, m, p, filt, z)
        except Exception:
            print(f"{name:<38}  ERROR: {traceback.format_exc().strip().splitlines()[-1][:100]}")
            continue
        if r.get("points", 0) == 0:
            print(f"{name:<38}{0:>11}   (no points in the object after extraction)")
            continue
        best = f"{r['best']:.2f}" if r["best"] is not None else "  -"
        print(f"{name:<38}{r['points']:>11}{r['n']:>8}{best:>7}   {r['near1']:>22}{r['near2']:>7}{r['far']:>6}   {r['secs']:>5.1f}")
        if name == "pipeline settings, try 1":
            e = r["ext"]; print(f"{'':<38}  object extent: {100*e[0]:.1f} x {100*e[1]:.1f} x {100*e[2]:.1f} cm (x, y, z in the camera frame)")
    print("\nfilter OFF row: grasps the network proposes anywhere in the 20 cm region; the three right-hand columns say how far their contact points are from the object.")


if __name__ == "__main__":
    main()
