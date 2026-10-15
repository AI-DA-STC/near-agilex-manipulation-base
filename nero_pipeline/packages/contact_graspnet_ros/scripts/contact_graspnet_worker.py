#!/usr/bin/env python3
"""
Persistent Contact-GraspNet worker. Runs inside the `contact_graspnet` conda env (like contact_graspnet_inference.py) but stays alive:
it builds the network and loads model.pt ONCE, then answers requests. The old way paid `conda run`, the imports and the checkpoint load on
every /contact_graspnet/detect_grasps call (about 16 of the 20 seconds measured on the Jetson, for a call that found nothing).

Started and talked to by contact_graspnet_ros (contact_graspnet_ros/worker_client.py); you normally never run it by hand:
    conda run -n contact_graspnet --no-capture-output python -u contact_graspnet_worker.py --ckpt_dir DIR --spool /tmp/x --parent_pid 1234

Same file protocol as grounded_sam2_worker.py (spool directory, so it does not depend on how `conda run` forwards stdin/stdout):
    worker -> ready.json | client -> req_<id>.json | worker -> res_<id>.json | client -> quit | exits when its parent is gone
Request:  {"cmd": "infer", "depth_npy": path, "segmap_npy": path, "K": [9 floats], "z_range": [0.2, 1.8], "local_regions": true,
           "filter_grasps": true, "skip_border_objects": false, "attempts": 3}
Reply:    {"ok": true, "result": {"detected": bool, "num_grasps": n, "grasps": [...]}, "attempts": used, "seconds": s}
The "result" is exactly the json contact_graspnet_inference.py writes. forward_passes is fixed when the worker starts (it sets the network's
batch size), so changing it needs a restart of the node. "attempts" > 1 re-runs the prediction when it comes back with 0 grasps.
"""
import argparse
import json
import os
import sys
import time
import traceback

import numpy as np
import torch

from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator
from contact_graspnet_pytorch import config_utils
from contact_graspnet_pytorch.checkpoints import CheckpointIO

POLL_S = 0.05


def log(msg):
    print(f"[contact_graspnet worker] {msg}", file=sys.stderr, flush=True)


def write_json_atomic(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f)
    os.replace(tmp, path)


def rotation_matrix_to_quaternion(R):
    """3x3 rotation matrix -> [x, y, z, w] quaternion, no scipy dependency. (verbatim from contact_graspnet_inference.py)"""
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return [float(x), float(y), float(z), float(w)]


def build_grasps(pred_grasps_cam, scores, gripper_openings):
    """Same conversion as contact_graspnet_inference.py: dict of 4x4 poses per segment -> list of grasp dicts, best score first."""
    grasps_out = []
    for seg_id, poses in pred_grasps_cam.items():
        seg_scores = scores.get(seg_id, [])
        seg_openings = gripper_openings.get(seg_id, [None] * len(poses)) if gripper_openings else [None] * len(poses)
        for i, pose in enumerate(poses):
            R = pose[:3, :3]
            t = pose[:3, 3]
            grasps_out.append({
                "segment_id": int(seg_id),
                "position": [float(t[0]), float(t[1]), float(t[2])],
                "quaternion": rotation_matrix_to_quaternion(R),
                "score": float(seg_scores[i]) if i < len(seg_scores) else None,
                "gripper_opening": float(seg_openings[i]) if seg_openings[i] is not None else None,
            })
    grasps_out.sort(key=lambda g: g["score"] if g["score"] is not None else 0, reverse=True)
    return grasps_out


def infer(estimator, req, forward_passes):
    depth = np.load(req["depth_npy"])
    segmap = np.load(req["segmap_npy"])
    K = np.array(req["K"], dtype=float).reshape(3, 3)
    pc_full, pc_segments, pc_colors = estimator.extract_point_clouds(
        depth, K, segmap=segmap, rgb=None,
        skip_border_objects=bool(req.get("skip_border_objects", False)), z_range=req.get("z_range", [0.2, 1.8]),
    )
    attempts = max(1, int(req.get("attempts", 1)))
    grasps_out, used = [], 0
    for used in range(1, attempts + 1):
        pred_grasps_cam, scores, contact_pts, gripper_openings = estimator.predict_scene_grasps(
            pc_full, pc_segments=pc_segments,
            local_regions=bool(req.get("local_regions", True)), filter_grasps=bool(req.get("filter_grasps", True)),
            forward_passes=forward_passes,
        )
        grasps_out = build_grasps(pred_grasps_cam, scores, gripper_openings)
        if grasps_out:
            break
        log(f"attempt {used}/{attempts}: 0 grasps")
    return {"detected": len(grasps_out) > 0, "num_grasps": len(grasps_out), "grasps": grasps_out}, used


def warmup(estimator, forward_passes):
    """One throw-away prediction on a synthetic table + box, so the first real call does not pay CUDA kernel start-up."""
    depth = np.full((480, 640), 0.87, np.float32)
    depth[200:240, 300:340] = 0.82
    segmap = np.zeros((480, 640), np.int32)
    segmap[200:240, 300:340] = 1
    K = np.array([600, 0, 320, 0, 600, 240, 0, 0, 1], dtype=float).reshape(3, 3)
    pc_full, pc_segments, _ = estimator.extract_point_clouds(depth, K, segmap=segmap, rgb=None, skip_border_objects=False, z_range=[0.2, 1.8])
    estimator.predict_scene_grasps(pc_full, pc_segments=pc_segments, local_regions=True, filter_grasps=True, forward_passes=forward_passes)


def parent_alive(pid):
    if not pid:
        return True
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spool", required=True, help="directory used to talk to the client")
    ap.add_argument("--parent_pid", type=int, default=0, help="exit when this process is gone")
    ap.add_argument("--ckpt_dir", required=True)
    ap.add_argument("--forward_passes", type=int, default=1)
    args = ap.parse_args()
    os.makedirs(args.spool, exist_ok=True)

    t0 = time.time()
    global_config = config_utils.load_config(args.ckpt_dir, batch_size=args.forward_passes)
    estimator = GraspEstimator(global_config)
    checkpoint_io = CheckpointIO(checkpoint_dir=os.path.join(args.ckpt_dir, "checkpoints"), model=estimator.model)
    try:
        checkpoint_io.load("model.pt")
    except FileExistsError:
        log("no model checkpoint found - exiting (the node falls back to the old script, which reports it)")
        sys.exit(2)
    load_s = time.time() - t0
    log(f"model loaded in {load_s:.1f} s (torch {torch.__version__}, forward_passes={args.forward_passes})")
    try:
        t1 = time.time()
        warmup(estimator, args.forward_passes)
        log(f"warm-up done in {time.time() - t1:.1f} s")
    except Exception:
        log("warm-up failed (ignored):\n" + traceback.format_exc()[-600:])
    write_json_atomic(os.path.join(args.spool, "ready.json"), {"load_s": round(load_s, 2), "forward_passes": args.forward_passes, "pid": os.getpid()})

    while True:
        if not os.path.isdir(args.spool) or os.path.exists(os.path.join(args.spool, "quit")) or not parent_alive(args.parent_pid):
            log("exiting (quit requested, spool removed, or parent gone)")
            return
        reqs = sorted(f for f in os.listdir(args.spool) if f.startswith("req_") and f.endswith(".json"))
        if not reqs:
            time.sleep(POLL_S)
            continue
        req_file = os.path.join(args.spool, reqs[0])
        req_id = reqs[0][len("req_"):-len(".json")]
        try:
            with open(req_file) as f:
                req = json.load(f)
            os.remove(req_file)
            t1 = time.time()
            if req.get("cmd") == "ping":
                reply = {"ok": True, "pong": True}
            else:
                result, used = infer(estimator, req, args.forward_passes)
                reply = {"ok": True, "result": result, "attempts": used, "seconds": round(time.time() - t1, 3)}
        except Exception:
            reply = {"ok": False, "error": traceback.format_exc()[-1500:]}
            try:
                os.remove(req_file)
            except OSError:
                pass
        write_json_atomic(os.path.join(args.spool, f"res_{req_id}.json"), reply)


if __name__ == "__main__":
    main()
