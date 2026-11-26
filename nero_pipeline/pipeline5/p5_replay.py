#!/usr/bin/env python3
"""p5_replay.py - PIPELINE 5: rerun the grasp generator OFFLINE on a saved look (no ROS, no robot). Every /p5/grasps call saves one in
~/p3_runs/p5_snaps/. Settings come from the same environment switches as the node (P5_MAX_TILT_DEG, P5_OPEN_MARGIN_M, P5_OFFSET_M,
P5_GRASPS_TO_MOVEIT, P5_TIP_CLEARANCE, P5_CLOUD_CHECK) and ~/.p5_gripper.yaml, so you can try a change before using it on the arm.
    /usr/bin/python3 ~/nero_pipeline_launch/pipeline5/p5_replay.py              # the newest snapshot
    /usr/bin/python3 ~/nero_pipeline_launch/pipeline5/p5_replay.py <file.npz>   # a given one
    P5_MAX_TILT_DEG=0 /usr/bin/python3 ~/nero_pipeline_launch/pipeline5/p5_replay.py
Writes a top-down picture next to the snapshot (<name>_replay.png): grey = scene, orange = object, cyan = GraspMolmo's point,
green/yellow = the fingers of G1/G2-3, red dots = fused-cloud obstacles near the object.
"""
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE]
import p5_grasp_core as GC  # noqa: E402
import p5_grasp_node as GN  # noqa: E402
import p5_gripper  # noqa: E402

P5_REPLAY_VERSION = "p5-replay-v2"   # v2: opens its picture itself (P5_REPLAY_SHOW=0: not); draws what blocked the fingers (red rings)


def picture(path, snap, res, geom, px_per_m=1500, half=0.16):
    try:
        import cv2
    except ImportError:
        return "no picture (cv2 not available)"
    c = np.asarray(snap["gpoint"])[:2]
    size = int(2 * half * px_per_m)
    img = np.full((size, size, 3), 255, np.uint8)

    def uv(p):   # base_link x,y -> pixel (image up = -x = away from the arm, right = -y)
        d = np.asarray(p)[..., :2] - c
        return np.stack([size / 2 - d[..., 1] * px_per_m, size / 2 - d[..., 0] * px_per_m], -1).astype(int)

    def dots(P, col, r=1):
        for u, v in uv(P):
            if 0 <= u < size and 0 <= v < size:
                cv2.circle(img, (int(u), int(v)), r, col, -1)

    z = snap["scene"][:, 2]
    dots(snap["scene"][z > snap["table_z"] + 0.006], (190, 190, 190))
    dots(snap["obj"], (0, 140, 255))
    F = snap["fused"]
    if len(F):
        dots(F[np.hypot(F[:, 0] - c[0], F[:, 1] - c[1]) < half], (40, 40, 220), 2)
    for cand in res["candidates"]:                                # what blocked a finger: red rings
        if not cand.get("ok") and cand.get("hit_pts"):
            for u, v in uv(np.asarray(cand["hit_pts"])):
                if 0 <= u < size and 0 <= v < size:
                    cv2.circle(img, (int(u), int(v)), 3, (0, 0, 255), 1)
    for s in res["selected"][::-1]:
        col = (40, 200, 40) if s["rank"] == 1 else (0, 210, 230)
        for name, (lo, hi) in p5_gripper.boxes(geom, s["open"]).items():
            if not name.startswith("finger"):
                continue
            corners = np.array([[x, y, hi[2]] for x, y in ((lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]), (lo[0], hi[1]))])
            cv2.polylines(img, [uv(corners @ s["R"].T + s["tcp"]).reshape(-1, 1, 2)], True, col, 2)
    u, v = uv(snap["gpoint"])
    cv2.drawMarker(img, (int(u), int(v)), (230, 200, 0), cv2.MARKER_CROSS, 18, 2)
    cv2.putText(img, "%s  %s" % (P5_REPLAY_VERSION, "OK" if res["ok"] else "NO GRASP"), (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.putText(img, "up = away from the arm (-x), 10 cm = %d px | red rings = what blocked a finger" % int(0.1 * px_per_m), (8, size - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1)
    cv2.imwrite(path, img)
    return path


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv:
        path = os.path.expanduser(argv[0])
    else:
        snaps = sorted(glob.glob(os.path.join(GN.SNAPS, "p5_*.npz")))
        if not snaps:
            print("no snapshots in %s (run p5_plan.sh once)" % GN.SNAPS)
            return 1
        path = snaps[-1]
    snap = GN.load_snapshot(path)
    cfg, fixed, cloud = GN.env_cfg()
    geom, gnote = p5_gripper.load()
    print("%s: %s (saved with %s)" % (P5_REPLAY_VERSION, path, snap["extra"].get("fused_note", "?")))
    print("settings now: %s; clearance %s; fused check %s; gripper: %s" % (
        cfg, "fixed %.1f mm" % (1000 * fixed) if fixed else "own rule", "on" if cloud else "off", gnote))
    if cfg != {k: snap["cfg"].get(k) for k in cfg}:
        print("  (at the look: %s)" % {k: snap["cfg"].get(k) for k in cfg})
    res = GC.generate(snap["obj"], snap["scene"], snap["fused"] if cloud else None, snap["gpoint"], snap["table_z"], fixed, cfg=cfg, geom=geom)
    for line in GC.summary(res, 15):
        print(line)
    pic = picture(os.path.splitext(path)[0] + "_replay.png", snap, res, geom)
    print("picture:", pic)
    if pic.endswith(".png") and os.environ.get("P5_REPLAY_SHOW", "1") != "0" and (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        import shutil
        import subprocess
        for viewer in ("eog", "xdg-open"):
            if shutil.which(viewer):
                subprocess.Popen([viewer, pic], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
                break
    return 0 if res["ok"] else 2


if __name__ == "__main__":
    sys.exit(main())
