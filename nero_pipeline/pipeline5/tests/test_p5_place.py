#!/usr/bin/env python3
"""Offline tests for p5_place_core.py: GraspMolmo's place point at a basket wall -> moved to where the gripper fits; the top camera's boxes rebuilt
around the new point (the wall the old landing zone hid becomes an obstacle). Run: /usr/bin/python3 .../pipeline5/tests/test_p5_place.py"""
import math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); P5 = os.path.join(HERE, "..")
sys.path[:0] = [P5, os.path.join(P5, "core")]
import p5_place_core as PC
import p5_gripper
import p3_obstacles

TABLE = -0.0109
K = np.array([[489.0, 0, 320.0], [0, 489.0, 240.0], [0, 0, 1.0]])
R_BC = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
T_BC = np.array([-0.35, 0.05, TABLE + 0.86])
G = dict(p5_gripper.DEFAULTS)
BASKET = np.array([-0.45, -0.05])
results = []


def check(name, ok, info=""):
    results.append(bool(ok)); print("%-5s %s %s" % ("PASS" if ok else "FAIL", name, info))


def scene(inner=(0.08, 0.06), floor_up=0.02, rim=0.08, cup=None):
    """Depth of a basket (inner half sizes, floor and rim heights above the table, 1 cm walls) -> depth, basket mask, pixel->xy map."""
    vs, us = np.mgrid[0:480, 0:640]
    ray = np.stack([(us - K[0, 2]) / K[0, 0], (vs - K[1, 2]) / K[1, 1], np.ones_like(us, float)], -1) @ R_BC.T
    p = T_BC + ray * ((TABLE + 0.04 - T_BC[2]) / ray[..., 2])[..., None]
    dx, dy = np.abs(p[..., 0] - BASKET[0]), np.abs(p[..., 1] - BASKET[1])
    H = np.zeros(p.shape[:2])
    outer = (dx <= inner[0] + 0.01) & (dy <= inner[1] + 0.01)
    H[outer] = rim
    H[(dx <= inner[0]) & (dy <= inner[1])] = floor_up
    if cup is not None:
        H[np.hypot(p[..., 0] - cup[0], p[..., 1] - cup[1]) <= 0.03] = floor_up + 0.05     # a 5 cm cup (below the gripper body)
    d = (T_BC[2] - TABLE - H + np.random.default_rng(4).normal(0, 0.0007, H.shape)).astype(np.float32)
    return d, outer, p


def px_of(p, xy):
    i = np.argmin(np.hypot(p[..., 0] - xy[0], p[..., 1] - xy[1]))
    v, u = np.unravel_index(i, p.shape[:2]); return (float(u), float(v))


def inside_basket(xyz, inner):
    return abs(xyz[0] - BASKET[0]) <= inner[0] and abs(xyz[1] - BASKET[1]) <= inner[1]


def walls_clear(d, res, open_w, inner):
    """independent check: no wall/rim point inside the release footprint at the chosen jaw angle (margin 0)."""
    P, _, _ = PC.points_px(d, K, R_BC, T_BC, *[a.ravel() for a in np.mgrid[0:640:2, 0:480:2]])
    F = P[P[:, 2] > res["floor_z"] + 0.01]; B = P[P[:, 2] > res["floor_z"] + 0.01 + 0.076]
    return PC.footprint_hits(F, B, np.array(res["xyz"]), math.radians(res["jaw_deg"]), open_w, G, 0.0, res["floor_z"] + 0.01) <= 3


inner = (0.08, 0.06)
d, mask, p = scene(inner)
# A: GraspMolmo's point 1.5 cm from the +x inner wall
uv0 = px_of(p, (BASKET[0] + inner[0] - 0.015, BASKET[1]))
r = PC.choose_place(d, K, R_BC, T_BC, uv0, mask, G, 0.07, grasp_jaw_deg=120.0)
check("point at the wall -> moved to where the gripper fits", r["ok"] and r["shift_m"] > 0.01 and inside_basket(r["xyz"], inner) and walls_clear(d, r, 0.07, inner),
      "%s; jaw %.0f deg; %s" % (r["why"], r.get("jaw_deg", -1), r["container"]))
check("  ... the release yaw stays within 60 deg of the grasp's", r["ok"] and min(abs((r["jaw_deg"] - 120 + 90) % 180 - 90), 180) <= 60.0001)
check("  ... the basket's floor is measured", abs(r["floor_z"] - (TABLE + 0.02)) < 0.004, "%+.1f mm vs table" % (1000 * (r["floor_z"] - TABLE)))
# B: in the middle: unchanged
r = PC.choose_place(d, K, R_BC, T_BC, px_of(p, BASKET), mask, G, 0.07, grasp_jaw_deg=120.0)
check("point in the middle -> kept", r["ok"] and r["shift_m"] < 0.006, r["why"])
# C: a basket too small for the gripper -> flagged, best effort
d2, m2, p2 = scene((0.03, 0.03))
r = PC.choose_place(d2, K, R_BC, T_BC, px_of(p2, BASKET), m2, G, 0.07, grasp_jaw_deg=120.0)
check("basket too small -> flagged, least-blocked spot given", (not r["ok"]) and r["uv"] is not None and "least blocked" in r["why"], r["why"])
# D: no SAM2 mask: the fallback stays on the basket floor (never jumps to the table outside)
r = PC.choose_place(d, K, R_BC, T_BC, uv0, None, G, 0.07, grasp_jaw_deg=120.0)
check("no SAM2 mask -> fallback stays inside the basket", r["ok"] and inside_basket(r["xyz"], inner) and walls_clear(d, r, 0.07, inner),
      "%s; %s" % (r["why"], r["container"]))
# E: a cup in the basket next to the point
big = (0.12, 0.09)                                                                # a real basket's size (24 x 18 cm inside)
d3, m3, p3 = scene(big, cup=(BASKET[0] - 0.035, BASKET[1]))
r = PC.choose_place(d3, K, R_BC, T_BC, px_of(p3, (BASKET[0] - 0.01, BASKET[1])), m3, G, 0.07, grasp_jaw_deg=0.0)
check("a cup next to the point -> avoided", r["ok"] and r["shift_m"] > 0.005 and walls_clear(d3, r, 0.07, big), "%s; jaw %.0f" % (r["why"], r.get("jaw_deg", -1)))
r = PC.choose_place(d3, K, R_BC, T_BC, px_of(p3, (BASKET[0] - 0.035, BASKET[1] + 0.06)), m3, G, 0.07, grasp_jaw_deg=0.0)
check("  ... a point beside the cup and near the wall -> both avoided", r["ok"] and walls_clear(d3, r, 0.07, big), "%s; jaw %.0f" % (r["why"], r.get("jaw_deg", -1)))
# F: top-camera boxes rebuilt around the NEW point: the wall the old landing zone hid is an obstacle again
r = PC.choose_place(d, K, R_BC, T_BC, uv0, mask, G, 0.07, grasp_jaw_deg=120.0)
old = PC.lift_px(d, K, R_BC, T_BC, uv0)
b_old = p3_obstacles.obstacle_boxes(d, K, R_BC, T_BC, TABLE, None, old.tolist(), ())
b_new = p3_obstacles.obstacle_boxes(d, K, R_BC, T_BC, TABLE, None, r["xyz"], ())
wall = np.array([BASKET[0] + inner[0] + 0.005, BASKET[1], TABLE + 0.05])


def covered(boxes, q):
    for b in boxes:
        c, s = np.asarray(b["centre"]), np.asarray(b["size"])
        if np.all(np.abs(q - c) <= s / 2 + 1e-6):
            return True
    return False


check("top boxes: the wall next to GraspMolmo's point was hidden by its landing zone", not covered(b_old, wall), "%d boxes" % len(b_old))
check("top boxes rebuilt around the new point: that wall is an obstacle again", covered(b_new, wall), "%d boxes" % len(b_new))
newpt = np.array([r["xyz"][0], r["xyz"][1], TABLE + 0.04])
check("  ... and the new landing spot itself stays free", not covered(b_new, newpt))
# the taper is never smaller than the finger: a point at the fingertips' height uses the tip sizes, one at the body's underside the body-end sizes
F = np.array([[0.0, 0.035 + 0.0069, 0.0], [0.0, 0.035 + 0.0071, 0.0], [0.0, 0.035 + 0.0249, 0.076], [0.0, 0.035 + 0.0251, 0.076]])
hits = [PC.footprint_hits(F[i:i + 1], np.zeros((0, 3)), np.zeros(3), math.radians(90), 0.07, G, 0.0, 0.0) for i in range(4)]
check("taper: tip sizes at the fingertips, body-end sizes at the top", hits == [1, 0, 1, 0], hits)
print("%s (%s): %d/%d passed" % ("ALL OK" if all(results) else "FAILURES", PC.P5_PLACE_CORE_VERSION, sum(results), len(results)))
sys.exit(0 if all(results) else 1)
