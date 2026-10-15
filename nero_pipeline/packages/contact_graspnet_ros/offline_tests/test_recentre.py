import math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); BASE = os.path.dirname(HERE)
sys.path.insert(0, BASE)
import numpy as np
from contact_graspnet_ros.recentre import quat_to_R, object_top_middle, recentre_grasps, CGN_GRIPPER_DEPTH
ok = True
def check(c, m):
    global ok; print(("  PASS  " if c else "  FAIL  ") + m); ok = ok and c

K = [[488.8, 0, 317.8], [0, 488.8, 216.8], [0, 0, 1]]
depth = np.full((480, 640), 0.90); mask = np.zeros((480, 640), bool)
depth[200:230, 100:130] = 0.85; mask[200:230, 100:130] = True            # a 30 x 30 px top face at 0.85 m on the left of the image
mask[231:236, 100:130] = True; depth[231:236, 100:130] = 0.88            # plus some pixels of a side face, farther away
top = object_top_middle(depth, mask, K)
exp = np.array([(114.5 - 317.8) * 0.85 / 488.8, (214.5 - 216.8) * 0.85 / 488.8, 0.85])
check(top is not None and np.allclose(top[0], exp, atol=2e-3), "top-face middle found from depth + mask: (%.3f, %.3f, %.3f) m, expected (%.3f, %.3f, %.3f)" % (*top[0], *exp))
check(top[1] == 900, "only the top face counts (900 points), the farther side-face pixels are ignored")
check(object_top_middle(depth, np.zeros((480, 640), bool), K) is None, "an empty mask gives None, not a crash")
check(abs(top[2][0] - 29 * 0.85 / 488.8) < 2e-3, "top-face size measured (%.1f cm across)" % (100 * top[2][0]))

R0 = quat_to_R(0, 0, 0, 1)
check(np.allclose(R0, np.eye(3)), "identity quaternion -> identity rotation")
a = math.radians(40.0); q = (math.sin(a / 2), 0, 0, math.cos(a / 2))       # 40 deg about camera x
check(abs(quat_to_R(*q)[2, 2] - math.cos(a)) < 1e-9, "40 degree rotation about x: the approach axis tilts by 40 degrees")

middle = top[0]; target = middle + [0, 0, 0.02]
def grasp(close_off, quat):
    ax = quat_to_R(*quat)[:, 2]; return {"segment_id": 1, "position": list(middle + close_off - CGN_GRIPPER_DEPTH * ax), "quaternion": list(quat), "score": 0.2, "gripper_opening": 0.009}
gs = [grasp([0.03, -0.01, 0.0], (0, 0, 0, 1)), grasp([-0.02, 0.0, 0.01], q), grasp([0.08, 0.0, 0.0], (0, 0, 0, 1)), grasp([0.0, -0.045, 0.0], q)]
new, dropped = recentre_grasps(gs, middle, 0.02, 0.05)
check(len(new) == 3 and dropped == 1, "a grasp 8 cm sideways of the middle is dropped (max offset 5 cm): kept %d, dropped %d" % (len(new), dropped))
for g in new:
    ax = quat_to_R(*g["quaternion"])[:, 2]; close = np.array(g["position"]) + CGN_GRIPPER_DEPTH * ax
    check(np.allclose(close, target, atol=1e-9), "closing centre moved onto the target (%.3f, %.3f, %.3f), tilt %.0f deg kept" % (*close, math.degrees(math.acos(ax[2]))))
check(new[0]["quaternion"] == [0, 0, 0, 1] and new[1]["quaternion"] == list(q) and all(g["score"] == 0.2 and g["gripper_opening"] == 0.009 for g in new), "orientation, score and opening are untouched; order preserved")
check(all("recentred_from" in g for g in new) and gs[0]["position"] != new[0]["position"] and "recentred_from" not in gs[0], "the original grasp dicts are not modified (a copy is returned)")
empty, d = recentre_grasps([gs[2]], middle, 0.02, 0.05); check(empty == [] and d == 1, "if every grasp is too far away the result is empty")
sys.exit(0 if ok else 1)
