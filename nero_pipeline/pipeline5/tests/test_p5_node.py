#!/usr/bin/env python3
"""Offline tests for p5_grasp_node's pure parts, p5_replay, and the edited functions of the core copies (no ROS, no robot).
Run: /usr/bin/python3 ~/nero_pipeline_launch/pipeline5/tests/test_p5_node.py"""
import ast, json, math, os, sys, tempfile, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); P5 = os.path.join(HERE, "..")
sys.path[:0] = [P5, os.path.join(P5, "core")]
import p5_grasp_node as N
import p5_grasp_core as GC
import p5_gripper

TABLE = -0.0109
K = np.array([[489.0, 0, 320.0], [0, 489.0, 240.0], [0, 0, 1.0]])
# camera 0.86 m above the table looking straight down, image x = base -y, image y = base -x  (camera z = base -z)
R_BC = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
T_BC = np.array([-0.35, 0.05, TABLE + 0.86])
results = []


def check(name, ok, info=""):
    results.append(bool(ok)); print("%-5s %s %s" % ("PASS" if ok else "FAIL", name, info))


def depth_scene(box_xy=(-0.35, 0.05), L=0.12, W=0.04, H=0.05, yaw=0.5):
    """Depth image (m) of a box on the table, and the box's mask."""
    vs, us = np.mgrid[0:480, 0:640]
    d = np.full((480, 640), 0.86)
    # ray of each pixel in base_link; intersect with the box top plane z = TABLE + H
    ray = np.stack([(us - K[0, 2]) / K[0, 0], (vs - K[1, 2]) / K[1, 1], np.ones_like(us, float)], -1) @ R_BC.T
    s = (TABLE + H - T_BC[2]) / ray[..., 2]
    p = T_BC + ray * s[..., None]
    dx, dy = p[..., 0] - box_xy[0], p[..., 1] - box_xy[1]
    u = dx * math.cos(yaw) + dy * math.sin(yaw); v = -dx * math.sin(yaw) + dy * math.cos(yaw)
    m = (np.abs(u) <= L / 2) & (np.abs(v) <= W / 2)
    d[m] = 0.86 - H
    d = d + np.random.default_rng(1).normal(0, 0.0007, d.shape)
    return d.astype(np.float32), (m * 255).astype(np.uint8)


# env settings
cfg, clr, cloud = N.env_cfg({"P5_MAX_TILT_DEG": "10", "P5_TIP_CLEARANCE": "0.008", "P5_CLOUD_CHECK": "off", "P5_GRASPS_TO_MOVEIT": "2"})
check("env switches", cfg["max_tilt_deg"] == 10 and cfg["n_out"] == 2 and clr == 0.008 and cloud is False)
cfg, clr, cloud = N.env_cfg({})
check("env defaults", cfg == {"max_tilt_deg": 20.0, "open_margin": 0.02, "n_out": 3, "offset_m": 0.01} and clr is None and cloud)
cfg, _, _ = N.env_cfg({"P5_MIN_CORNER_MM": "0", "P5_RAISE_MM": "0,5,10,15"})
check("env: corner limit and raise steps", cfg["min_tip_corner"] == 0.0 and cfg["raise_steps"] == (0.0, 0.005, 0.01, 0.015))

# quaternions
rng = np.random.default_rng(3); ok = True
for _ in range(100):
    q = rng.normal(size=4); q /= np.linalg.norm(q)
    R = N.quat_to_R(*q); q2 = N.R_to_quat(R)
    ok &= np.allclose(R, N.quat_to_R(*q2), atol=1e-9)
check("quaternion <-> matrix (100 random)", ok)

# GraspMolmo result pairing
r = {(1, 0): {"mask": 1, "point": 1}, (2, 0): {"mask": 1}}
k, e = N.pick_result(r, None); check("newest COMPLETE result is used", k == (1, 0))
k, e = N.pick_result(r, (1, 0)); check("the same result is never used twice", k is None and "already used" in e)
k, e = N.pick_result({}, None); check("no result -> refused", k is None and "no GraspMolmo result" in e)

# depth -> points -> grasps (end to end, synthetic camera)
depth, mask = depth_scene()
uv = np.argwhere(mask > 0).mean(0)[::-1]
z = float(depth[int(uv[1]), int(uv[0])])
p_cam = np.array([(uv[0] - K[0, 2]) * z / K[0, 0], (uv[1] - K[1, 2]) * z / K[1, 1], z])
obj, scene = N.gather_points(mask, depth, K, R_BC, T_BC, N.project(p_cam, K))
check("object points at the box top", abs(np.median(obj[:, 2]) - (TABLE + 0.05)) < 0.002, "%.4f" % np.median(obj[:, 2]))
check("scene points include the table", np.percentile(scene[:, 2], 10) < TABLE + 0.003 and len(scene) > 5000, "%d pts" % len(scene))
gpoint = R_BC @ p_cam + T_BC
res = GC.generate(obj, scene, None, gpoint, TABLE, None)
check("grasp found from a depth image", res["ok"], res["why"])
g1 = res["selected"][0]
check("jaw across the 4 cm side", abs(g1["width"] - 0.04) < 0.006 and abs(((g1["jaw_deg"] - math.degrees(0.5)) % 180) - 90) < 12,
      "jaw %.0f width %.0f mm" % (g1["jaw_deg"], 1000 * g1["width"]))

# reports
g_rep, r_rep = N.reports(res, "test note", "2026-11-18 10:00:00", 1234.5)
check("report: opening + clearance for MTC/picture", g_rep["open_width_m"] == res["info"]["open_width"] and g_rep["clearance_m"] == res["info"]["clearance"]
      and g_rep["unix_time"] == 1234.5 and len(g_rep["grasps"]) == len(res["selected"]))
check("relay report: picture format, passed ones first", all(set(x) >= {"rank", "tilt_deg", "roll_deg", "lowest_finger_mm", "result"} for x in r_rep["grasps"])
      and r_rep["grasps"][0]["result"].startswith("PASSED to MoveIt as G1"))
json.dumps(g_rep); json.dumps(r_rep)
check("reports are JSON", True)

# markers
geom = dict(p5_gripper.DEFAULTS)
mk = N.build_markers(res, gpoint, geom)
g1m = [m for m in mk if m["ns"] == "G1" and m["type"] == "cube"]
nseg = geom["FINGER_SEGMENTS"]
check("markers: G1 = 2 x %d finger boxes + body + camera" % nseg, len(g1m) == 2 * nseg + 2 and all(m["rgba"][1] > 0.8 for m in g1m))
tips = [m for m in g1m if abs(m["scale"][2] - (0.076 / nseg)) < 1e-6]
lowest = min(m["position"][2] - 0.5 * m["scale"][2] for m in tips)
check("markers: lowest finger box at the fingertips", abs(lowest - (TABLE + g1["tip_mm"] / 1000)) < 0.002, "%.4f" % lowest)
check("markers: GraspMolmo point", any(m["ns"] == "graspmolmo_point" for m in mk))

# snapshot round trip + pruning + replay
tmp = tempfile.mkdtemp()
old_keep = N.KEEP_SNAPS; N.KEEP_SNAPS = 3
for i in range(5):
    N.save_snapshot(os.path.join(tmp, "p5_2026111%d_000000.npz" % i), obj, scene, None, gpoint, TABLE, None, {"max_tilt_deg": 20}, geom)
N.KEEP_SNAPS = old_keep
snaps = sorted(os.listdir(tmp))
check("snapshots: only the newest are kept", snaps == ["p5_20261112_000000.npz", "p5_20261113_000000.npz", "p5_20261114_000000.npz"], snaps)
s = N.load_snapshot(os.path.join(tmp, snaps[-1]))
res2 = GC.generate(s["obj"], s["scene"], s["fused"], s["gpoint"], s["table_z"], s["clearance"])
check("snapshot replays to the same G1", res2["ok"] and abs(res2["selected"][0]["jaw_deg"] - g1["jaw_deg"]) < 1e-6
      and np.allclose(res2["selected"][0]["tcp"], g1["tcp"], atol=1e-4))
import p5_replay
os.environ["P5_REPLAY_SHOW"] = "0"
rc = p5_replay.main([os.path.join(tmp, snaps[-1])])
check("p5_replay runs on a snapshot", rc == 0)

# the edited functions of the core copies
os.environ["HOME"] = tmp
os.makedirs(os.path.join(tmp, "p3_runs"), exist_ok=True)
rep = os.path.join(tmp, "p3_runs", "p5_grasp_latest.json")
import p3_picture
json.dump({"unix_time": time.time(), "clearance_m": 0.0075, "clearance_why": "x"}, open(rep, "w"))
c, why = p3_picture.run_clearance({"object_height_m": 0.05})
check("picture checks pipeline 5's clearance", abs(c - 0.007) < 1e-9 and "pipeline 5" in why, "%.4f %s" % (c, why))
json.dump({"unix_time": time.time() - 3600, "clearance_m": 0.0075}, open(rep, "w"))
c, why = p3_picture.run_clearance({"object_height_m": 0.05})
check("old pipeline 5 report -> pipeline 3's rule (safe side)", "pipeline 5" not in why and c > 0.0075)
src = open(os.path.join(P5, "core", "p3_mtc.py")).read()
fn = [n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef) and n.name == "_p4_open_width"][0]
ns = {}; exec(compile(ast.Module(body=[fn], type_ignores=[]), "p3_mtc", "exec"), ns)
json.dump({"unix_time": time.time(), "open_width_m": 0.077, "time": "now"}, open(rep, "w"))
w, why = ns["_p4_open_width"]()
check("MTC copy reads pipeline 5's opening", w == 0.077 and "pipeline 5" in why, why)
os.environ["P5_GRIPPER_WIDTH"] = "off"; w, why = ns["_p4_open_width"](); del os.environ["P5_GRIPPER_WIDTH"]
check("P5_GRIPPER_WIDTH=off -> fully open", w is None)
os.remove(rep); w, why = ns["_p4_open_width"]()
check("no pipeline 5 report -> fully open", w is None)

# ---------------- v2: object masks (SAM2 choice, merged depth regions), on a synthetic copy of the real sink scene
def basin_scene(egg=(-0.40, 0.03), floor_up=0.028, rim=0.09, inner=(0.09, 0.07), gap=0.03):
    """Depth image of a raised basin like the real sink's left one (inner 18 x 14 cm, floor 28 mm above the table, 15 mm walls up to 9 cm) with a
    toy eggplant (6 x 3.4 cm, 3.4 cm tall) `gap` from its -y wall; -> depth, exact eggplant mask, eggplant pixel."""
    vs, us = np.mgrid[0:480, 0:640]
    ray = np.stack([(us - K[0, 2]) / K[0, 0], (vs - K[1, 2]) / K[1, 1], np.ones_like(us, float)], -1) @ R_BC.T
    p = T_BC + ray * ((TABLE + 0.05 - T_BC[2]) / ray[..., 2])[..., None]          # xy of each pixel (parallax neglected: < 2 mm)
    bx, by = egg[0], egg[1] - 0.017 - gap + inner[1]                             # basin centre: the eggplant `gap` from the -y inner wall
    dx, dy = np.abs(p[..., 0] - bx), np.abs(p[..., 1] - by)
    H = np.zeros(p.shape[:2])
    H[(dx <= inner[0] + 0.015) & (dy <= inner[1] + 0.015)] = rim                  # walls, 15 mm thick
    H[(dx <= inner[0]) & (dy <= inner[1])] = floor_up                             # inner floor
    u_, v_ = (p[..., 0] - egg[0]) / 0.03, (p[..., 1] - egg[1]) / 0.017
    q = 1 - u_ ** 2 - v_ ** 2
    m = q > 0
    H[m] = np.maximum(H[m], floor_up + 0.017 + 0.017 * np.sqrt(q[m]))
    d = (T_BC[2] - TABLE - H + np.random.default_rng(2).normal(0, 0.0007, H.shape)).astype(np.float32)
    uvs = np.argwhere(m).mean(0)[::-1]
    return d, (m * 255).astype(np.uint8), uvs


import p3_geometry
EGG = np.array([-0.40, 0.03])


def sink_case(gap):
    depth_b, egg_mask, egg_uv = basin_scene(gap=gap)
    z = float(depth_b[int(egg_uv[1]), int(egg_uv[0])])
    pc = np.array([(egg_uv[0] - K[0, 2]) * z / K[0, 0], (egg_uv[1] - K[1, 2]) * z / K[1, 1], z])
    gm, _ = p3_geometry.object_mask(depth_b, egg_uv, 45, above_table_m=0.008)    # GraspMolmo's own region, exactly as its node makes it
    return depth_b, egg_mask, egg_uv, gm, R_BC @ pc + T_BC


# A: eggplant 4 mm from a wall (as in the real look): GraspMolmo's region takes the wall too
d_a, egg_a, uv_a, gm_a, gp_a = sink_case(0.004)
merged, f = N.region_merged(gm_a, uv_a)
check("sink, eggplant at a wall: GraspMolmo's region is merged -> refused", merged, "%.0f %% of its outline on the circle, %d px (eggplant %d px)" % (
    100 * f, (gm_a > 0).sum(), (egg_a > 0).sum()))
o1, s1 = N.gather_points(gm_a, d_a, K, R_BC, T_BC, uv_a)
r_bad = GC.generate(o1, s1, None, gp_a, TABLE, None)
check("  ... and grasping that region would be wrong (why the check exists)", (not r_bad["ok"]) or r_bad["selected"][0]["width"] > 0.05,
      r_bad["why"][:90] or "G1 %.0f mm wide" % (1000 * r_bad["selected"][0]["width"]))
merged2, f2 = N.region_merged(mask, uv)
check("box alone on the table: its region is not merged", not merged2, "%.0f %%" % (100 * f2))

# B: 3 cm from a wall, the object's own (SAM2-like) mask: a grasp ON the eggplant, fingers clear of the basin
d_b, egg_b, uv_b, gm_b, gp_b = sink_case(0.03)
o2, s2 = N.gather_points(egg_b, d_b, K, R_BC, T_BC, uv_b)
r_b = GC.generate(o2, s2, None, gp_b, TABLE, None)
if r_b["ok"]:
    g = r_b["selected"][0]
    walls = s2[s2[:, 2] > TABLE + 0.028 + 0.04]                                   # the basin walls (above the eggplant's top)
    free = True
    for sel in r_b["selected"]:
        bx = p5_gripper.boxes(geom, sel["open"], sweep=geom["APPROACH_M"])
        q = GC.to_frame(walls, sel["tcp"], sel["R"])
        free &= not any(GC.in_box(q, bx[k]).any() for k in bx)
    check("sink, 3 cm from a wall: G1 on the eggplant, all grasps clear of the walls", np.hypot(*(g["pad"][:2] - EGG)) < 0.015 and free,
          "jaw %.0f deg, %.0f mm wide, %.0f mm from its centre" % (g["jaw_deg"], 1000 * g["width"], 1000 * np.hypot(*(g["pad"][:2] - EGG))))
    check("sink: fingertips measured from the BASIN floor", abs(r_b["info"]["floor_z"] - (TABLE + 0.028)) < 0.004 and g["tip_mm"] >= 6,
          r_b["info"]["floor_note"])
else:
    check("sink, 3 cm from a wall: G1 on the eggplant", False, r_b["why"])

# C: in the middle of the basin: closes ACROSS the eggplant
d_c, egg_c, uv_c, gm_c, gp_c = sink_case(0.05)
o3, s3 = N.gather_points(egg_c, d_c, K, R_BC, T_BC, uv_c)
r_c = GC.generate(o3, s3, None, gp_c, TABLE, None)
check("sink, eggplant mid-basin: closes across it (~34 mm)", r_c["ok"] and abs(r_c["selected"][0]["width"] - 0.034) < 0.008
      and abs(((r_c["selected"][0]["jaw_deg"]) % 180) - 90) < 16, r_c["why"] or "jaw %.0f, %.0f mm" % (r_c["selected"][0]["jaw_deg"], 1000 * r_c["selected"][0]["width"]))

# SAM2 proposal choice
h, w = 480, 640
yy, xx = np.mgrid[0:h, 0:w]
part = (np.hypot(xx - 300, yy - 200) < 6)
eggm = (np.hypot((xx - 300) / 25.0, (yy - 200) / 12.0) < 1)
sink = (np.abs(xx - 320) < 160) & (np.abs(yy - 220) < 110)
i, note = N.choose_sam_mask([part, eggm, sink], [0.70, 0.86, 0.97], (300, 200))
check("SAM2: the sink proposal (too big) is skipped, the eggplant chosen", i == 1, note)
piece = (np.hypot(xx - 300, yy - 200) < 7)
i, note = N.choose_sam_mask([piece, eggm], [0.03, 0.57], (300, 200))
check("SAM2: a tiny low-score piece (as on 2026-11-24) loses to the whole eggplant", i == 1, note)
blob = (np.hypot((xx - 300) / 60.0, (yy - 200) / 40.0) < 1)    # eggplant + sink ledge: bigger, rated lower (2026-11-24)
i, note = N.choose_sam_mask([piece, eggm, blob], [0.03, 0.62, 0.50], (300, 200))
check("SAM2: eggplant + ledge (bigger, rated lower) loses to the eggplant", i == 1, note)
region = (np.abs(xx - 300) < 110) & (np.abs(yy - 200) < 90)    # a sink-sized region under the 30000 px cap (2026-11-24 13:43)
i, note = N.choose_sam_mask([eggm, region], [0.70, 0.55], (300, 200))
check("SAM2: a sink-sized region (bigger, rated lower) loses to the eggplant", i == 0, note)
i, note = N.choose_sam_mask([piece, eggm], [0.9, 0.2], (300, 200))
check("SAM2: a proposal with a score below 0.3 is not used", i == 0, note)
i, note = N.choose_sam_mask([eggm], [0.9], (400, 400))
check("SAM2: a proposal missing GraspMolmo's point is not used", i is None and "misses" in note, note)
border = (xx < 30) & (np.abs(yy - 200) < 20)
i, note = N.choose_sam_mask([border], [0.9], (10, 200))
check("SAM2: a proposal touching the image border is not used", i is None and "border" in note, note)

# container choice at the PLACE point: the whole basket, not a slat of it; never the table
slat = (np.abs(xx - 400) < 4) & (np.abs(yy - 300) < 30)
basket = (np.abs(xx - 400) < 70) & (np.abs(yy - 300) < 50)
table = np.ones((h, w), bool)
i, note = N.choose_container_mask([slat, basket, table], [0.9, 0.8, 0.95], (400, 300))
check("place: SAM2 container = the whole basket (largest usable), not a slat, not the table", i == 1, note)

# the SAM2 worker's file protocol, end to end (--fake: no model)
try:
    import cv2
    from worker_client import Worker
    wk = Worker([sys.executable, os.path.join(P5, "p5_sam2_worker.py"), "--fake"], log=lambda m: None, startup_timeout=60)
    d = tempfile.mkdtemp(); img = os.path.join(d, "rgb.png"); cv2.imwrite(img, np.zeros((480, 640, 3), np.uint8))
    rep = wk.request({"cmd": "segment", "image": img, "point": [320, 240], "out_dir": d}, timeout=60)
    wk.stop()
    ms = [cv2.imread(pth, cv2.IMREAD_GRAYSCALE) for pth in rep["masks"]]
    i, note = N.choose_sam_mask(ms, rep["scores"], (320, 240))
    check("SAM2 worker protocol (fake model): 3 proposals, the best-rated usable one chosen", rep["ok"] and len(ms) == 3 and i == 1, note)
except ImportError:
    check("SAM2 worker protocol (needs cv2)", False, "cv2 missing in this Python")

print("%s (%s): %d/%d passed" % ("ALL OK" if all(results) else "FAILURES", N.P5_NODE_VERSION, sum(results), len(results)))
sys.exit(0 if all(results) else 1)
