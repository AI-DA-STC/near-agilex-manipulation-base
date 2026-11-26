#!/usr/bin/env python3
"""Offline tests for p5_keepout.py: the sink-floor bug (wrist boxes in the fingers' path -> MTC closed 2 cm high) and its fix.
Run: /usr/bin/python3 ~/nero_pipeline_launch/pipeline5/tests/test_p5_keepout.py"""
import json, os, sys, tempfile, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); P5 = os.path.join(HERE, "..")
sys.path[:0] = [P5, os.path.join(P5, "core")]
import p5_keepout as KO
import p5_grasp_core as GC
import p5_gripper
import p5_wrist_obstacles as W

TABLE = -0.0109
results = []


def check(name, ok, info=""):
    results.append(bool(ok)); print("%-5s %s %s" % ("PASS" if ok else "FAIL", name, info))


def sink_points(egg=(-0.40, 0.03), floor_up=0.031, rim=0.09, inner=(0.09, 0.07), gap=0.05, step=0.004):
    """Points of the real scene's kind: a raised sink basin (floor +31 mm like the real look) with walls, a toy eggplant (6 x 3.4 x 3 cm) -> all, eggplant."""
    bx, by = egg[0], egg[1] - 0.017 - gap + inner[1]
    xs = np.arange(bx - inner[0] - 0.05, bx + inner[0] + 0.05, step); ys = np.arange(by - inner[1] - 0.05, by + inner[1] + 0.05, step)
    X, Y = np.meshgrid(xs, ys); X, Y = X.ravel(), Y.ravel()
    dx, dy = np.abs(X - bx), np.abs(Y - by)
    H = np.zeros(len(X))
    H[(dx <= inner[0] + 0.015) & (dy <= inner[1] + 0.015)] = rim
    H[(dx <= inner[0]) & (dy <= inner[1])] = floor_up
    u, v = (X - egg[0]) / 0.03, (Y - egg[1]) / 0.017
    q = 1 - u ** 2 - v ** 2
    m = q > 0
    H[m] = floor_up + 0.015 + 0.015 * np.sqrt(q[m])
    P = np.column_stack([X, Y, TABLE + H])
    # the walls' sides too (the wrist camera sees them from the side)
    wall = []
    for z in np.arange(floor_up, rim, step):
        for x in xs[(np.abs(xs - bx) <= inner[0])]:
            wall += [[x, by - inner[1], TABLE + z], [x, by + inner[1], TABLE + z]]
    P = np.vstack([P, np.array(wall)])
    return P, P[: len(X)][m]


def gripper_samples(sel, geom, open_w, sweep=0.0, step=0.004):
    """Points filling the fingers' boxes (at the grasp, or swept along the approach) of a grasp, in base_link."""
    pts = []
    for name, (lo, hi) in p5_gripper.boxes(geom, open_w, sweep=sweep).items():
        if not name.startswith("finger"):
            continue
        g = np.stack(np.meshgrid(*[np.arange(lo[i], hi[i] + 1e-9, step) for i in range(3)]), -1).reshape(-1, 3)
        pts.append(g @ np.asarray(sel["R"]).T + sel["tcp"])
    return np.vstack(pts)


def in_any_box(P, boxes):
    m = np.zeros(len(P), bool)
    for b in boxes:
        c, s = np.asarray(b["centre"]), np.asarray(b["size"])
        m |= np.all(np.abs(P - c) <= s / 2, axis=1)
    return m


geom = dict(p5_gripper.DEFAULTS)
P, egg = sink_points()
g = egg[np.argmin(np.hypot(egg[:, 0] + 0.40, egg[:, 1] - 0.03))]
res = GC.generate(egg, P, None, g, TABLE, None)
check("sink scene: a grasp is found", res["ok"], res["why"])
sel, open_w = res["selected"], res["info"]["open_width"]
tbox = W.target_box(egg, TABLE)
boxes0, note0 = W.wrist_boxes(P, TABLE, tbox, None)
fing = np.vstack([gripper_samples(s, geom, open_w) for s in sel])
hit0 = in_any_box(fing, boxes0).sum()
check("BUG reproduced: without the corridor, wrist boxes fill the fingers' place (the sink floor counts as an obstacle)", hit0 > 0,
      "%d of %d finger sample points inside %d wrist boxes" % (hit0, len(fing), len(boxes0)))
corr = KO.corridor(sel, geom, open_w)
keep = ~KO.inside(P, corr)
boxes1, note1 = W.wrist_boxes(P[keep], TABLE, tbox, None)
swept = np.vstack([gripper_samples(s, geom, open_w, sweep=geom["APPROACH_M"]) for s in sel])
hit1 = in_any_box(swept, boxes1).sum()
check("FIX: with the corridor, no wrist box anywhere in the fingers' path (grasp + 8 cm approach)", hit1 == 0,
      "%d hits; %d points left out, %d boxes" % (hit1, (~keep).sum(), len(boxes1)))
walls = P[(P[:, 2] > TABLE + 0.06)]
far_walls = walls[~KO.inside(walls, corr, grow=0.06)]
check("the sink walls away from the gripper stay obstacles", len(far_walls) > 0 and in_any_box(far_walls, boxes1).mean() > 0.95,
      "%.0f %% of far wall points covered" % (100 * in_any_box(far_walls, boxes1).mean()))

# apply_latest: from the report the grasp node writes
tmp = tempfile.mkdtemp(); rep = os.path.join(tmp, "p5_grasp_latest.json"); now = time.time()
json.dump({"unix_time": now, "keepout": corr}, open(rep, "w"))
Q, note = KO.apply_latest(P, now - 3, rep, env={})
check("apply_latest: same look -> corridor left out", len(Q) == keep.sum() and "left out" in note, note)
Q, note = KO.apply_latest(P, now - 900, rep, env={})
check("apply_latest: report from another look -> unchanged", len(Q) == len(P) and "another look" in note, note)
Q, note = KO.apply_latest(P, now, rep, env={"P5_KEEPOUT": "off"})
check("apply_latest: P5_KEEPOUT=off -> unchanged", len(Q) == len(P) and "OFF" in note)
Q, note = KO.apply_latest(P, now, os.path.join(tmp, "missing.json"), env={})
check("apply_latest: no report -> unchanged", len(Q) == len(P) and "no grasp-step report" in note)
json.dump({"unix_time": now, "ok": False}, open(rep, "w"))
Q, note = KO.apply_latest(P, now, rep, env={})
check("apply_latest: report without a corridor -> unchanged", len(Q) == len(P))
print("%s (%s): %d/%d passed" % ("ALL OK" if all(results) else "FAILURES", KO.P5_KEEPOUT_VERSION, sum(results), len(results)))
sys.exit(0 if all(results) else 1)
