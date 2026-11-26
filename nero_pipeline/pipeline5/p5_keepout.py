#!/usr/bin/env python3
"""p5_keepout.py - PIPELINE 5: the gripper's checked corridor, kept free of wrist-camera boxes (no ROS in here, tested offline).

Why: the wrist boxes are coarse (2 cm voxels + 1 cm margin) and count everything 2.5 cm above the TABLE as an obstacle - in the sink that includes
the sink floor (+31 mm). MoveIt then saw the fingers' last 2 cm as a collision and MTC (approach 2-8 cm) closed 2 cm above the eggplant.
The grasp step (p5_grasp_core) already checked exactly this volume - the fingers and the gripper body at the grasp and along the 8 cm approach -
against the same cameras, finer (1 cm voxels + 5 mm, top-camera points + 3 mm) and with the real gripper shape. So the wrist points inside that
volume, grown by GROW_M, are left out of the wrist boxes; everything else (walls beyond it, things above it, the place side) stays a box.
GROW_M = a voxel (2 cm) + the box margin (1 cm): a box built from the remaining points can never reach back into the corridor.

The grasp node writes the corridor into ~/p3_runs/p5_grasp_latest.json ("keepout"); p5_fuse_node reads it when it builds the wrist boxes
(only if it belongs to the same look: written within MAX_SKEW_S of the mask it uses). P5_KEEPOUT=off disables it.
"""
import json
import os
import time

import numpy as np

import p5_gripper

P5_KEEPOUT_VERSION = "p5-keepout-v1"
GROW_M = 0.03
MAX_SKEW_S = 120.0
REPORT = os.path.expanduser("~/p3_runs/p5_grasp_latest.json")


def corridor(selected, geom, open_w):
    """The grasps passed to MoveIt -> list of oriented boxes {"tcp", "R", "lo", "hi"} (tcp_link frame of each grasp): every gripper box swept
    along the approach, not yet grown."""
    out = []
    for s in selected:
        for name, (lo, hi) in p5_gripper.boxes(geom, open_w, sweep=geom["APPROACH_M"]).items():
            out.append({"grasp": int(s["rank"]), "part": name, "tcp": [float(v) for v in s["tcp"]],
                        "R": [[float(v) for v in row] for row in np.asarray(s["R"])], "lo": list(map(float, lo)), "hi": list(map(float, hi))})
    return out


def inside(P, boxes, grow=GROW_M):
    """Boolean mask: which points lie in any corridor box grown by `grow`."""
    P = np.asarray(P, float).reshape(-1, 3)
    m = np.zeros(len(P), bool)
    for b in boxes:
        q = (P - np.asarray(b["tcp"])) @ np.asarray(b["R"])
        m |= np.all((q >= np.asarray(b["lo"]) - grow) & (q <= np.asarray(b["hi"]) + grow), axis=1)
    return m


def apply_latest(P, mask_time=None, report=REPORT, env=None):
    """Wrist points -> (points outside the latest grasp step's corridor, note). Unchanged (with the reason) when off, missing, failed or from
    another look."""
    e = os.environ if env is None else env
    P = np.asarray(P, float).reshape(-1, 3)
    if e.get("P5_KEEPOUT", "on") == "off":
        return P, "keep-out OFF (P5_KEEPOUT=off)"
    try:
        d = json.load(open(report))
    except Exception:
        return P, "keep-out: no grasp-step report (%s) - wrist boxes unchanged" % report
    boxes, t = d.get("keepout") or [], float(d.get("unix_time", 0))
    if not boxes:
        return P, "keep-out: the grasp-step report has no corridor - wrist boxes unchanged"
    if mask_time is not None and abs(t - mask_time) > MAX_SKEW_S:
        return P, "keep-out: the grasp-step report is from another look (%.0f s apart) - wrist boxes unchanged" % abs(t - mask_time)
    if mask_time is None and time.time() - t > 600:
        return P, "keep-out: the grasp-step report is %.0f min old - wrist boxes unchanged" % ((time.time() - t) / 60)
    m = inside(P, boxes)
    n_grasps = len({b["grasp"] for b in boxes})
    return P[~m], "keep-out: %d wrist point(s) in the gripper's checked corridor (%d grasp(s), +%.0f cm) left out" % (int(m.sum()), n_grasps, 100 * GROW_M)
