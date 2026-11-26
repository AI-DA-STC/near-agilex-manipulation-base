#!/usr/bin/env python3
"""Offline tests for p5_grasp_core.py (no ROS, no robot): synthetic top-camera scenes - the camera sees only TOP surfaces, like the real one.
Run:  /usr/bin/python3 ~/nero_pipeline_launch/pipeline5/tests/test_p5_grasp_core.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import p5_grasp_core as C  # noqa: E402
import p5_gripper  # noqa: E402

RNG = np.random.default_rng(0)
TABLE = -0.0109
STEP = 0.003          # ~ the top camera's point spacing at the table (0.86 m, fx 489: 1.8 mm/px, stride ~2)
X0, Y0 = -0.35, 0.05  # a typical object position (workspace is on -x)


def clearance(h):   # pipeline 5's rule
    return C.tip_clearance(h)


def expected_pad_z(floor, h):
    return max(floor + 0.5 * h, floor + clearance(h) + 0.5 * p5_gripper.DEFAULTS["PAD_H"])


def grid(cx, cy, r, step=STEP):
    xs = np.arange(cx - r, cx + r + 1e-9, step)
    X, Y = np.meshgrid(xs, np.arange(cy - r, cy + r + 1e-9, step))
    return np.column_stack([X.ravel(), Y.ravel()])


def rot(xy, c, yaw):
    """xy in base_link -> object frame (u along the long axis, v across)."""
    d = xy - c
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.column_stack([d[:, 0] * cy + d[:, 1] * sy, -d[:, 0] * sy + d[:, 1] * cy])


class Scene:
    def __init__(self, floor=TABLE, cx=X0, cy=Y0, r=0.15):
        self.floor, self.c = floor, np.array([cx, cy])
        self.xy = grid(cx, cy, r)
        self.z = np.full(len(self.xy), floor)
        self.label = np.zeros(len(self.xy), int)     # 0 floor, 1 target, 2+ others
        self.fused = np.zeros((0, 3))

    def box(self, c, L, W, H, yaw, label):
        uv = rot(self.xy, np.asarray(c), yaw)
        m = (np.abs(uv[:, 0]) <= L / 2) & (np.abs(uv[:, 1]) <= W / 2)
        self.z[m] = np.maximum(self.z[m], self.floor + H)
        self.label[m] = label
        return self

    def ellipsoid(self, c, a, b, cz, yaw, label):
        uv = rot(self.xy, np.asarray(c), yaw)
        q = 1 - (uv[:, 0] / a) ** 2 - (uv[:, 1] / b) ** 2
        m = q > 0
        z = self.floor + cz + cz * np.sqrt(np.clip(q, 0, 1))
        self.z[m] = np.maximum(self.z[m], z[m])
        self.label[m] = label
        return self

    def wall_ring(self, inner, H):   # a basin: walls outside `inner` (half size), floor stays at self.floor inside
        d = np.max(np.abs(self.xy - self.c), axis=1)
        m = d > inner
        self.z[m] = self.floor + H
        self.label[m] = 9
        return self

    def pts(self, noise=0.001):
        P = np.column_stack([self.xy, self.z + RNG.normal(0, noise, len(self.z))])
        return P

    def run(self, gpoint_xy=None, **kw):
        P = self.pts()
        obj = P[self.label == 1]
        g = obj[np.argmin(np.linalg.norm(obj[:, :2] - (self.c if gpoint_xy is None else np.asarray(gpoint_xy)), axis=1))]
        if gpoint_xy is not None:
            g = np.array([gpoint_xy[0], gpoint_xy[1], g[2]])
        h = float(np.percentile(obj[:, 2], 95) - self.floor)
        res = C.generate(obj, P, self.fused, g, TABLE, clearance(h), **kw)
        return res


def jaw_vs(res_sel, yaw):
    """angle between the selected jaw line and the object's long axis (deg, 0-90)."""
    return math.degrees(C.angle_diff180(math.radians(res_sel["jaw_deg"]), yaw))


def fingers_free_of(res, pts, open_w=None):
    G = dict(p5_gripper.DEFAULTS)
    for s in res["selected"]:
        bx = p5_gripper.boxes(G, open_w or s["open"], sweep=G["APPROACH_M"])
        q = C.to_frame(pts, s["tcp"], s["R"])
        for k in bx:
            if C.in_box(q, bx[k]).sum() > 0:
                return False
    return True


# ---------------------------------------------------------------- tests
def test_frame():
    for th in (0.0, 0.7, 2.0):
        for tl in (0.0, math.radians(20), -math.radians(10)):
            R = C.jaw_frame(th, tl)
            assert np.allclose(R.T @ R, np.eye(3), atol=1e-9) and abs(np.linalg.det(R) - 1) < 1e-9
            assert abs(R[2, 1]) < 1e-12, "jaw line must be level"
            assert abs(math.degrees(math.acos(-R[2, 2])) - abs(math.degrees(tl))) < 1e-6
    R = C.jaw_frame(0.0, 0.0)
    assert np.allclose(R[:, 2], [0, 0, -1])


def test_box_across_narrow():
    yaw = math.radians(30)
    s = Scene().box((X0, Y0), 0.12, 0.04, 0.05, yaw, 1)
    r = s.run()
    assert r["ok"], r["why"]
    g1 = r["selected"][0]
    assert jaw_vs(g1, yaw) > 80, "jaw must close across the narrow side (got %.0f deg from the long axis)" % jaw_vs(g1, yaw)
    assert abs(g1["width"] - 0.04) < 0.006, g1["width"]
    assert abs(g1["open"] - 0.06) < 0.008, g1["open"]
    assert abs(g1["tilt_deg"]) < 1e-6, "best grasp should be straight down"
    assert abs(g1["pad"][2] - expected_pad_z(TABLE, 0.05)) < 0.004, "pads at mid-height (or raised for the fingertip clearance)"
    assert abs(g1["tip_mm"] - 10.0) < 1.0, "a 5 cm object is shorter than the 70 mm grip face: fingertips go down to the 10 mm clearance"


def test_eggplant():
    yaw = math.radians(-40)
    s = Scene().ellipsoid((X0, Y0), 0.10, 0.03, 0.03, yaw, 1)
    r = s.run()
    assert r["ok"], r["why"]
    g1 = r["selected"][0]
    assert jaw_vs(g1, yaw) > 75, jaw_vs(g1, yaw)
    assert abs(g1["width"] - 0.06) < 0.008, g1["width"]
    assert 0.07 <= g1["open"] <= 0.10
    assert len(set(round(s_["open"], 6) for s_ in r["selected"])) == 1, "one common opening"
    for s_ in r["selected"]:
        assert s_["open"] >= s_["width"] + 0.02 - 1e-9
    assert r["selected"][0]["open"] <= r["selected"][0]["width"] + 0.02 + 0.015 + 1e-9, "the opening stays close to what the best grasp needs"


def test_point_at_one_end_stays_there():
    yaw = 0.0
    s = Scene().box((X0, Y0), 0.16, 0.04, 0.04, yaw, 1)
    r = s.run(gpoint_xy=(X0 + 0.06, Y0))
    assert r["ok"], r["why"]
    assert abs(r["selected"][0]["pad"][0] - (X0 + 0.06)) <= 0.0105, "stays at GraspMolmo's point (+-1 cm slide)"


def test_too_wide():
    s = Scene().box((X0, Y0), 0.12, 0.12, 0.05, 0.0, 1)
    r = s.run()
    assert not r["ok"] and "gripper" in r["why"], r["why"]


def test_too_flat():
    s = Scene().box((X0, Y0), 0.08, 0.04, 0.005, 0.0, 1)
    r = s.run()
    assert not r["ok"], r["why"]


def test_point_off_object():
    s = Scene().box((X0, Y0), 0.08, 0.04, 0.04, 0.0, 1)
    r = s.run(gpoint_xy=(X0, Y0 + 0.05))
    assert not r["ok"] and "from the object" in r["why"], r["why"]


def test_neighbour_blocks_one_side():
    # target 8 x 4 cm (long axis x); a tall box 1.2 cm away along +y blocks a finger of the across-narrow grasp
    s = Scene().box((X0, Y0), 0.08, 0.04, 0.04, 0.0, 1).box((X0, Y0 + 0.02 + 0.012 + 0.03), 0.12, 0.06, 0.08, 0.0, 2)
    r = s.run()
    P = s.pts(0)
    neighbour = P[s.label == 2]
    if r["ok"]:
        assert fingers_free_of(r, neighbour), "a selected grasp hits the neighbour"
    assert any("neighbour" in c["why"] for c in r["candidates"] if not c["ok"]), "the blocked grasps must be rejected as hitting the neighbour"


def test_neighbour_low_long_side_used():
    # same, but the neighbour is as low as the target: the gripper body passes over it, so closing along the 8 cm side (opening 10 cm) is chosen
    s = Scene().box((X0, Y0), 0.08, 0.04, 0.04, 0.0, 1).box((X0, Y0 + 0.02 + 0.012 + 0.03), 0.12, 0.06, 0.04, 0.0, 2)
    r = s.run()
    assert r["ok"], r["why"]
    assert jaw_vs(r["selected"][0], 0.0) < 10, "jaw along the long axis (got %.0f deg)" % jaw_vs(r["selected"][0], 0.0)
    assert fingers_free_of(r, s.pts(0)[s.label == 2])


def test_sink_floor():
    # a basin: floor 4 cm above the table, walls 12 cm tall at +-0.10 m; object 6 x 3 x 4 cm on the basin floor
    fl = TABLE + 0.04
    s = Scene(floor=fl).wall_ring(0.10, 0.12).box((X0, Y0), 0.06, 0.03, 0.04, 0.3, 1)
    r = s.run()
    assert r["ok"], r["why"]
    assert abs(r["info"]["floor_z"] - fl) < 0.004, r["info"]["floor_z"]
    assert abs(r["info"]["height"] - 0.04) < 0.004, r["info"]["height"]
    assert abs(r["selected"][0]["pad"][2] - expected_pad_z(fl, 0.04)) < 0.004
    assert r["selected"][0]["tip_mm"] > 9, "fingertips measured from the basin floor"


def test_overhang_in_fused_cloud():
    # a faucet bar 6 cm above the object's top, seen only in the fused cloud: straight-down approaches pass through it
    s = Scene().box((X0, Y0), 0.08, 0.04, 0.04, 0.0, 1)
    bar = np.array([[X0 + dx, Y0 + dy, TABLE + 0.10] for dx in np.arange(-0.03, 0.031, 0.01) for dy in np.arange(-0.10, 0.101, 0.01)])
    s.fused = bar
    r = s.run()
    straight = [c for c in r["candidates"] if c.get("tilt_deg") == 0.0]
    assert straight and all(not c["ok"] for c in straight), "straight-down grasps must be rejected"
    assert any("fused" in c["why"] for c in straight)
    if r["ok"]:
        assert fingers_free_of(r, bar), "a selected grasp passes through the bar"


def test_object_points_in_fused_ignored():
    # the fused cloud contains the target itself (1 cm voxels): it must not block its own grasp
    s = Scene().box((X0, Y0), 0.10, 0.04, 0.05, 0.0, 1)
    vox = np.array([[X0 + dx, Y0 + dy, TABLE + dz] for dx in np.arange(-0.05, 0.051, 0.01) for dy in np.arange(-0.02, 0.021, 0.01)
                    for dz in np.arange(0.005, 0.0551, 0.01)])
    s.fused = vox
    r = s.run()
    assert r["ok"], r["why"]


def test_tall_object_pads_raised():
    s = Scene().box((X0, Y0), 0.06, 0.04, 0.12, 0.0, 1)
    r = s.run()
    assert r["ok"], r["why"]
    g = dict(p5_gripper.DEFAULTS)
    g1 = r["selected"][0]
    palm_z = g1["tcp"][2] - g["FINGER_ROOT_Z"] * g1["R"][2, 2]
    assert palm_z >= TABLE + 0.12 + 0.0099, "body must stay 1 cm above the top"


def test_tilted_object_has_tilted_options():
    s = Scene().box((X0, Y0), 0.10, 0.04, 0.05, 0.0, 1)
    r = s.run()
    tilts = sorted(set(round(c["tilt_deg"]) for c in r["candidates"] if c["ok"]))
    assert tilts == [-20, -10, 0, 10, 20], tilts
    assert all(abs(c["tilt_deg"]) <= 20 + 1e-9 for c in r["candidates"] if "tilt_deg" in c)


def test_max_tilt_zero():
    s = Scene().box((X0, Y0), 0.10, 0.04, 0.05, 0.0, 1)
    r = s.run(cfg={"max_tilt_deg": 0})
    assert r["ok"] and all(c["tilt_deg"] == 0 for c in r["candidates"] if "tilt_deg" in c)


def test_big_object_narrow_side_found():
    # 18 x 6 cm, bigger than the 5 cm patch: the jaw must still close across the 6 cm side (v3 took the angles from a disc of it)
    yaw = math.radians(20)
    s = Scene().box((X0, Y0), 0.18, 0.06, 0.05, yaw, 1)
    r = s.run()
    assert r["ok"], r["why"]
    assert jaw_vs(r["selected"][0], yaw) > 82, jaw_vs(r["selected"][0], yaw)
    assert abs(r["selected"][0]["width"] - 0.06) < 0.006
    assert abs(math.degrees(C.angle_diff180(math.radians(r["info"]["narrow_axis_deg"]), yaw + math.pi / 2))) < 5


def sink_corner_scene(gap, r=0.03, egg_yaw=-19.0):
    """The 2026-11-23 15:52 look: a sink floor 33 mm up that curves (radius r) into a wall on the +y side, a 26 mm tall eggplant lying `gap`
    from where the curve starts (its narrow axis ~71 deg like the real one)."""
    fl = TABLE + 0.033
    s = Scene(floor=fl)
    wall_y = Y0 + 0.017 + gap + r
    d = wall_y - s.xy[:, 1]
    curve = np.where(d <= 0, 0.10, np.where(d < r, r - np.sqrt(np.clip(r * r - (r - d) ** 2, 0, None)), 0.0))
    s.z = fl + curve
    s.label[d <= 0] = 9
    s.ellipsoid((X0, Y0), 0.04, 0.017, 0.013, math.radians(egg_yaw), 1)
    return s


def egg_end(frac=0.7, yaw=-19.0):
    """GraspMolmo's point near one end of the eggplant (as in the real look: centring along it needed 28 mm)."""
    return (X0 + frac * 0.04 * math.cos(math.radians(yaw)), Y0 + frac * 0.04 * math.sin(math.radians(yaw)))


def test_sink_wall_point_near_end_closes_along():
    # case 1 (as the real look): the wall is within the finger's reach across the eggplant; GraspMolmo's point near one end
    s = sink_corner_scene(0.0)
    old = s.run(gpoint_xy=egg_end(), cfg={"raise_steps": (0.0,), "min_tip_corner": 0.003, "recentre_max": 0.02})
    assert not old["ok"] and "finger+ would hit a neighbour" in old["why"] and "off-centre" in old["why"], "real failure reproduced: " + old["why"]
    new = s.run(gpoint_xy=egg_end())
    assert new["ok"], new["why"]
    assert jaw_vs(new["selected"][0], math.radians(-19)) < 15, "closes along the eggplant (got %.0f deg)" % new["selected"][0]["jaw_deg"]
    P = s.pts(0)
    assert fingers_free_of(new, P[s.label == 9]), "clear of the wall"


def test_sink_curve_higher_grasp_fits():
    # case 2: the sink floor rises like a shallow bowl beside the eggplant (6 mm at 2 cm): across it, upright, the usual height is blocked by the
    # rising floor and the 5 / 10 mm higher versions of the SAME grasp fit
    fl = TABLE + 0.033
    s = Scene(floor=fl)
    e = s.xy[:, 1] - (Y0 + 0.017)
    s.z = fl + np.where(e > 0, 0.006 * (e / 0.02) ** 2, 0.0)
    s.ellipsoid((X0, Y0), 0.04, 0.017, 0.017, math.radians(-19), 1)
    r = s.run()
    cs = {c["raise_mm"]: c for c in r["candidates"] if "raise_mm" in c and abs(c["tilt_deg"]) < 1e-6 and c["offset_mm"] == 0 and c.get("shift_mm", 0) == 0
          and abs(((c["jaw_deg"] - 71 + 90) % 180) - 90) < 8}
    assert not cs[0.0]["ok"] and "finger+ would hit a neighbour" in cs[0.0]["why"], cs[0.0]["why"]
    assert cs[5.0]["ok"] and cs[10.0]["ok"], (cs[5.0]["why"], cs[10.0]["why"])
    assert r["ok"]
    P = s.pts(0)
    floor_up = P[(s.label == 0) & (P[:, 2] > fl + 0.002)]
    assert fingers_free_of(r, floor_up), "every grasp sent stays above the rising floor"


def test_blocking_points_kept():
    s = sink_corner_scene(0.0)
    r = s.run(cfg={"raise_steps": (0.0,)})
    rej = [c for c in r["candidates"] if not c["ok"] and ("neighbour" in c["why"] or "fused" in c["why"] or "camera" in c["why"])]
    assert rej and all(len(c.get("hit_pts", [])) > 0 for c in rej), "every 'hit a neighbour' rejection keeps the points that blocked it"
    assert np.all(np.array(rej[0]["hit_pts"])[:, 1] > Y0), "and they are on the wall side (+y)"
    assert any("mm above the floor" in c["why"] for c in rej if c["why"].startswith("finger"))


def test_real_sink_scene_2026_11_23():
    """The real look that failed (eggplant 28 mm from a 5 cm step of the sink, GraspMolmo's point near one end): v7 closes ACROSS it, the jaws
    shifted toward the open side. (This snapshot's fused points are voxel CENTRES - the old fusion - so it is a little harder than new looks.)"""
    import p5_grasp_node as N
    snap = N.load_snapshot(os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "sink_20261123_155234.npz"))
    args = (snap["obj"], snap["scene"], snap["fused"], snap["gpoint"], snap["table_z"], None)
    v6 = C.generate(*args, cfg={"shift_steps": (0.0,), "fused_margin": 0.005, "max_hits_fused": 1})
    assert not v6["ok"], "v6 settings: no grasp (as on the arm)"
    r = C.generate(*args)
    assert r["ok"], r["why"]
    g1 = r["selected"][0]
    assert abs(((g1["jaw_deg"] - 71 + 90) % 180) - 90) < 25, "across the eggplant (got %.0f deg)" % g1["jaw_deg"]
    assert g1["open"] < 0.06, "a normal opening, not end to end (%.0f mm)" % (1000 * g1["open"])
    # independent check: no top-camera point that is not the eggplant inside any finger / the body / the camera box (swept), margin 0
    G = dict(p5_gripper.DEFAULTS)
    cells = C.CellSet(snap["obj"], 0.004)
    other = snap["scene"][(snap["scene"][:, 2] > r["info"]["floor_z"] + 0.006) & ~cells.contains(snap["scene"])]
    for sel in r["selected"]:
        q = C.to_frame(other, sel["tcp"], sel["R"])
        for k, b in p5_gripper.boxes(G, sel["open"], sweep=G["APPROACH_M"]).items():
            assert C.in_box(q, b).sum() <= 4, "%s of G%d hits %d points" % (k, sel["rank"], C.in_box(q, b).sum())


def test_shift_keeps_object_off_the_fingers():
    s = Scene().box((X0, Y0), 0.10, 0.04, 0.05, 0.0, 1)
    r = s.run()
    for c in r["candidates"]:
        if "shift_mm" in c:
            assert abs(c["shift_mm"]) / 1000 <= 0.5 * (c["own_open"] - c["width"]) - 0.002 + 1e-9


def test_floor_below_table_uses_the_table():
    """7l (2026-11-24 table picks): the floor around the eggplant measured 12 mm BELOW MoveIt's table; its height came out 28 mm from that floor,
    the pads (fingertips kept above the table) covered only 8-9 mm -> no grasp. The table is the floor: height 16 mm, the grasp passes."""
    s = Scene(floor=TABLE - 0.012).ellipsoid((X0, Y0), 0.04, 0.017, 0.014, math.radians(-19), 1)   # the surface reads 12 mm low, top 16 mm above the table
    P = s.pts(); obj = P[s.label == 1]
    g = obj[np.argmin(np.linalg.norm(obj[:, :2] - s.c, axis=1))]
    r = C.generate(obj, P, None, g, TABLE, None)
    assert r["ok"], r["why"]
    assert abs(r["info"]["floor_z"] - TABLE) < 1e-9 and "the table is used" in r["info"]["floor_note"], r["info"]["floor_note"]
    assert r["selected"][0]["tip_mm"] >= 6 - 1e-6, "fingertips still above the table by the clearance"
    assert abs(r["info"]["height"] - 0.016) < 0.004, r["info"]["height"]
    g1 = r["selected"][0]
    assert g1["overlap"] >= 0.5 * r["info"]["height"] - 1e-9, "the pads cover at least half of it"


def test_clearance_rule():
    assert C.tip_clearance(None) == 0.010 and C.tip_clearance(0.01) == 0.006 and C.tip_clearance(0.03) == 0.0075 and C.tip_clearance(0.2) == 0.010


def test_clearance_none_uses_local_height():
    s = Scene().box((X0, Y0), 0.08, 0.04, 0.03, 0.0, 1)
    P = s.pts()
    obj = P[s.label == 1]
    r = C.generate(obj, P, None, obj[np.argmin(np.linalg.norm(obj[:, :2] - s.c, axis=1))], TABLE, None)
    assert r["ok"], r["why"]
    assert abs(r["info"]["clearance"] - C.tip_clearance(r["info"]["height"])) < 1e-12
    assert r["selected"][0]["tip_mm"] >= 1000 * r["info"]["clearance"] - 0.2


def test_gripper_yaml(tmp="/tmp/p5_gripper_test.yaml"):
    open(tmp, "w").write("pad_h: 0.065\nfinger_w_tip: 0.018\n")
    g, note = p5_gripper.load(tmp)
    assert g["PAD_H"] == 0.065 and g["FINGER_W_TIP"] == 0.018 and "pad_h".upper() in note and g["BODY_Y"] == 0.165
    g, note = p5_gripper.load("/nonexistent.yaml")
    assert g == p5_gripper.DEFAULTS


def test_tapered_finger_boxes():
    g = dict(p5_gripper.DEFAULTS)
    bx = p5_gripper.boxes(g, 0.06)
    for side, sgn in (("finger+", 1), ("finger-", -1)):
        segs = [bx["%s%d" % (side, k)] for k in range(g["FINGER_SEGMENTS"])]
        assert abs(segs[0][1][2] - 0.138) < 1e-9 and abs(segs[-1][0][2] - 0.062) < 1e-9, "fingers span 62-138 mm"
        widths = [s[1][0] - s[0][0] for s in segs]
        thick = [s[1][1] - s[0][1] for s in segs]
        assert widths == sorted(widths) and thick == sorted(thick), "wider and thicker towards the body"
        assert abs(widths[-1] - 0.055) < 1e-9 and abs(thick[-1] - 0.025) < 1e-9, "body end = the measured A, B"
        assert widths[0] >= 0.020 and thick[0] >= 0.007, "never smaller than the finger"
        inner = [s[0][1] if sgn > 0 else s[1][1] for s in segs]
        assert all(abs(abs(v) - 0.03) < 1e-9 for v in inner), "grip faces flat, at +-opening/2"
    assert abs(bx["body"][1][1] - bx["body"][0][1] - 0.165) < 1e-9 and abs(bx["body"][1][0] - bx["body"][0][0] - 0.055) < 1e-9
    assert abs(p5_gripper.pad_mid(g) - 0.103) < 1e-9


if __name__ == "__main__":
    import time
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_")]
    bad = 0
    for k, f in tests:
        t0 = time.time()
        try:
            f()
            print("PASS  %-40s %.2f s" % (k, time.time() - t0))
        except AssertionError as e:
            bad += 1
            print("FAIL  %-40s %s" % (k, e))
        except Exception as e:
            bad += 1
            print("ERROR %-40s %s: %s" % (k, type(e).__name__, e))
    print("%s (%s, %s): %d/%d passed" % ("ALL OK" if not bad else "FAILURES", C.P5_GRASP_CORE_VERSION, p5_gripper.P5_GRIPPER_VERSION,
                                        len(tests) - bad, len(tests)))
    sys.exit(1 if bad else 0)
