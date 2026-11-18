"""Offline tests for p4_grasp's geometry (no ROS). Run: python3 test_p4_grasp.py"""
import math, os, sys
import numpy as np
_here = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(_here, ".."), os.path.join(_here, "..", "pipeline4")]
import p4_grasp as G

rng = np.random.default_rng(4)
results = []
def check(name, ok, info=""):
    results.append(bool(ok)); print("%-4s %s %s" % ("PASS" if ok else "FAIL", name, info))

def rot(axis, deg):
    a = np.asarray(axis, float) / np.linalg.norm(axis); t = math.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K

TZ = -0.0109
DOWNR = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)        # approach straight down, jaws along -y... (X=x, Y=-y, Z=-z)

# quaternion round trip
for _ in range(50):
    R = rot(rng.normal(size=3), rng.uniform(0, 180))
    R2 = G.quat_to_R(*G.R_to_quat(R))
    if not np.allclose(R, R2, atol=1e-9):
        break
check("quaternion <-> matrix round trip (50 random)", np.allclose(R, R2, atol=1e-9))

# v4 straightening: jaw line always level; tilt kept only ACROSS the jaws, limited
RA = rot([0, 1, 0], 41) @ DOWNR                       # tilted 41 deg ACROSS the jaws (jaw line stays level)
RB = rot([1, 0, 0], 41) @ DOWNR                       # tilted 41 deg ALONG the jaws = jaw line rolled 41 deg (one finger lower)
check("across-the-jaws 41 deg -> 20 deg (default limit)", abs(G.tilt_deg(G.straighten(RA)) - 20) < 1e-6)
check("across-the-jaws 41 deg with limit 0 -> straight down", abs(G.tilt_deg(G.straighten(RA, 0))) < 1e-6)
check("along-the-jaws (rolled) 41 deg -> jaw level, approach straight down", G.roll_deg(G.straighten(RB)) < 1e-6 and abs(G.tilt_deg(G.straighten(RB))) < 1e-6)
mixed = rot([1, 0, 0], 25) @ rot([0, 1, 0], 30) @ DOWNR
Rm = G.straighten(mixed)
check("mixed tilt + roll -> roll 0, tilt <= 20", G.roll_deg(Rm) < 1e-6 and G.tilt_deg(Rm) <= 20 + 1e-6, "tilt %.1f" % G.tilt_deg(Rm))
check("result is a proper rotation", np.allclose(Rm.T @ Rm, np.eye(3), atol=1e-9) and abs(np.linalg.det(Rm) - 1) < 1e-9)
jy = mixed[:, 1].copy(); jy[2] = 0; jy /= np.linalg.norm(jy)
check("jaw compass direction kept", abs(Rm[:, 1] @ jy) > 0.999999)
R10 = rot([0, 1, 0], 10) @ DOWNR
check("10 deg across the jaws, limit 20: unchanged", np.allclose(G.straighten(R10), R10, atol=1e-9))
Rbad = np.column_stack([[1, 0, 0], [0, 0, -1], [0, 1, 0]])          # jaw line vertical
check("vertical jaw line -> rejected (None)", G.straighten(Rbad) is None)
R41 = mixed

# width of an eggplant-like object (18 x 6 x 6 cm, lying along x) across vs along
n = 4000
egg = np.c_[rng.uniform(-0.09, 0.09, n), rng.uniform(-0.03, 0.03, n), rng.uniform(TZ, TZ + 0.06, n)]
egg = np.vstack([egg, [[0.5, 0.5, 0.5]]])                                       # one stray pixel
w_across = G.object_width(egg, [0, 1, 0]); w_along = G.object_width(egg, [1, 0, 0])
check("width across the eggplant ~6 cm (stray pixel ignored)", abs(w_across - 0.06) < 0.004, "%.1f mm" % (1000 * w_across))
check("width along it ~18 cm", abs(w_along - 0.18) < 0.01, "%.1f mm" % (1000 * w_along))
check("opening = width + 2 cm", abs(G.opening(0.06) - 0.08) < 1e-9)
check("opening clamped: tiny object -> 3 cm, huge -> 10 cm, unknown -> 10 cm",
      G.opening(0.001) == 0.03 and G.opening(0.2) == 0.10 and G.opening(None) == 0.10)

# full grasp: Contact-GraspNet-like tilted grasp over the eggplant's middle, eggplant 6.3 cm tall
contact = np.array([-0.40, 0.05, TZ + 0.045])
o = contact - G.CGN_CONTACT_DEPTH_M * R41[:, 2]
egg2 = egg[:-1] + np.array([-0.40, 0.05, 0])
g = G.plan_grasp(o, R41, TZ, 0.015, 0.063, egg2, max_tilt_deg=0)
check("planned grasp with limit 0 is vertical and level", abs(g["tilt_to"]) < 1e-6 and g["roll_to"] < 1e-6 and g["tilt_from"] > 30)
check("pads at the eggplant's mid-height (3.15 cm above the table)", abs(g["pad_z"] - (TZ + 0.0315)) < 1e-6, "pad z %+.4f" % g["pad_z"])
check("pads above Contact-GraspNet's x/y", np.allclose((g["tcp"] + G.PAD_DEPTH_M * g["R"][:, 2])[:2], contact[:2]))
check("opening ~8 cm (6 cm + 2 cm), not 10", abs(g["open"] - 0.08) < 0.005, "%.1f mm" % (1000 * g["open"]))
check("fingertips at least the clearance above the table", g["lowest_tip_mm"] >= 15 - 1e-6, "%.1f mm" % g["lowest_tip_mm"])
check("no lift needed for this object", g["raised_mm"] < 0.5, "%.1f mm" % g["raised_mm"])
flat = egg2.copy(); flat[:, 2] = np.minimum(flat[:, 2], TZ + 0.02)
gf = G.plan_grasp(o, R41, TZ, 0.015, 0.02, flat, max_tilt_deg=0)
check("a 2 cm flat object: raised so the tips keep the clearance", gf["raised_mm"] > 0 and abs(gf["lowest_tip_mm"] - 15) < 1e-6,
      "raised %.1f mm" % gf["raised_mm"])
gn = G.plan_grasp(o, R41, TZ, 0.015, None, None, max_tilt_deg=0)
check("no height / no object points: CGN contact height, fully open", gn["open"] == 0.10 and abs(gn["pad_z"] - max(contact[2], gn["pad_z"])) < 1e-9)

# near-duplicates after straightening are dropped
R30 = rot([0, 1, 0], 30) @ DOWNR
g2 = G.plan_grasp(contact + np.array([0.003, 0, 0]) - G.CGN_CONTACT_DEPTH_M * R30[:, 2], R30, TZ, 0.015, 0.063, egg2, 0)
g3 = G.plan_grasp(o + np.array([0.05, 0, 0]), R41, TZ, 0.015, 0.063, egg2, 0)
check("near-duplicate dropped, distinct one kept", len(G.distinct([g, g2, g3])) == 2)

# the run that stopped: pipeline 3's picture checks the fingertips assuming FULLY open (100 mm); with a level jaw line it must agree
g20 = G.plan_grasp(o, mixed, TZ, 0.016, 0.061, egg2)
full = G.lowest_tip_z(g20["tcp"], g20["R"], 0.10) - TZ
real = G.lowest_tip_z(g20["tcp"], g20["R"], g20["open"]) - TZ
check("fingertip height the same at 100 mm and at the real opening (level jaw)", abs(full - real) < 1e-9, "%.1f mm" % (1000 * full))
check("...and at least the 15 mm pipeline 3 demands", 1000 * full >= 15.0, "%.1f mm" % (1000 * full))
check("default tilt limit is 20 deg", G.MAX_TILT_DEG == 20.0)
print("\n%d/%d passed" % (sum(results), len(results)))
sys.exit(0 if all(results) else 1)
