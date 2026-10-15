import os
HERE = os.path.dirname(os.path.abspath(__file__)); BASE = os.path.dirname(HERE)
import sys, importlib.util, numpy as np, math
sys.path.insert(0, os.path.join(HERE, "nstubs")); sys.path.insert(0, BASE)
spec = importlib.util.spec_from_file_location("target_pose_node", os.path.join(BASE, "target_pose_ros", "target_pose_node.py"))
import types; pkg = types.ModuleType("target_pose_ros"); pkg.__path__ = [os.path.join(BASE, "target_pose_ros")]; sys.modules["target_pose_ros"] = pkg
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
H, W = 480, 640
K = np.array([[488.778, 0, 317.809], [0, 488.778, 216.849], [0, 0, 1.0]])
yy, xx = np.mgrid[0:H, 0:W]; dist = np.hypot(xx - 360, yy - 250)
plate = dist <= 90
depth_mm = np.full((H, W), 900, np.uint16); depth_mm[plate] = 850; depth_mm[plate & (dist >= 83)] = 825; depth_mm[260, 330] = 700
mask = plate.astype(np.uint8); mask[180:230, 375:425] = 0                      # an object-shaped hole
node = m.TargetPoseNode(); node._latest_mask, node._latest_depth, node._camera_K = mask, depth_mm, K
class Resp: pass
ok = True
def check(c, msg):
    global ok; print(("  PASS  " if c else "  FAIL  ") + msg); ok = ok and c
print("defaults declared by the node:", {k: node._p[k] for k in ("z_mode", "center_mode", "center_fraction")})
r = Resp(); node._extract_cb(None, r); p = node.published[-1].pose.position
print("response:", r.message[:230], "...")
check(r.success and node._p["z_mode"] == "floor" and node._p["center_mode"] == "hull", "service succeeds with the new defaults (floor, hull)")
check(abs(p.z - 0.850) < 0.004, "published z = plate floor 0.850 (not the rim 0.825 or the stray pixel 0.700), got %.4f" % p.z)
tx, ty = (360 - K[0, 2]) * 0.85 / K[0, 0], (250 - K[1, 2]) * 0.85 / K[1, 1]
check(math.hypot(p.x - tx, p.y - ty) < 0.004, "published x,y at the true centre despite the hole (err %.1f mm)" % (1000 * math.hypot(p.x - tx, p.y - ty)))
# live parameter change takes effect on the next call (as 'ros2 param set' would)
node.set_param("center_mode", "mean"); node.set_param("z_mode", "top_surface"); r2 = Resp(); node._extract_cb(None, r2); p2 = node.published[-1].pose.position
check(abs(p2.z - 0.700) < 1e-9 and math.hypot(p2.x - tx, p2.y - ty) > 0.008, "switching back to the old modes (mean + top_surface) reproduces the old behaviour")
node._latest_mask = np.zeros((H, W), np.uint8); r3 = Resp(); node._extract_cb(None, r3)
check(not r3.success and "empty" in r3.message, "an empty mask gives a clean failure message: '%s'" % r3.message)
node._latest_mask = np.zeros((10, 10), np.uint8) + 1; r4 = Resp(); node._extract_cb(None, r4)
check(not r4.success and "depth_registration" in r4.message, "shape mismatch still reports the depth_registration hint")

# ---- plate pose frame: follows the depth camera_info (auto) unless forced
print("\nframe label of the plate pose")
node._latest_mask, node._latest_depth, node._camera_K = mask, depth_mm, K; node.set_param("center_mode", "hull"); node.set_param("z_mode", "floor")
check(node._p["camera_frame"] == "", "default camera_frame is empty = automatic")
r5 = Resp(); node._extract_cb(None, r5); f0 = node.published[-1].header.frame_id
check(r5.success and f0 == "topcam_depth_optical_frame", "no camera_info frame known yet: falls back to the old default ('%s')" % f0)
node._camera_frame_info = "topcam_color_optical_frame"; r6 = Resp(); node._extract_cb(None, r6); f1 = node.published[-1].header.frame_id
check(f1 == "topcam_color_optical_frame" and "topcam_color_optical_frame" in r6.message, "camera_info says colour frame: the plate pose is labelled '%s' and the response says so" % f1)
node.set_param("camera_frame", "my_forced_frame"); r7 = Resp(); node._extract_cb(None, r7)
check(node.published[-1].header.frame_id == "my_forced_frame", "an explicit camera_frame parameter still wins")
node.set_param("camera_frame", ""); node._camera_frame_info = "topcam_depth_optical_frame"; r8 = Resp(); node._extract_cb(None, r8)
check(node.published[-1].header.frame_id == "topcam_depth_optical_frame", "unregistered depth (camera_info says depth frame): labelled with the depth frame")
check(sum(1 for k, l in node.get_logger().lines if "plate pose is labelled with frame" in l) >= 3, "every change of frame is logged once")
node._camera_info_cb(type("M", (), {"k": [488.8, 0, 317.8, 0, 488.8, 216.8, 0, 0, 1], "header": type("H", (), {"frame_id": "topcam_color_optical_frame"})()})())
check(node._camera_frame_info == "topcam_color_optical_frame" and abs(node._camera_K[0, 0] - 488.8) < 1e-9, "the camera_info callback stores both the matrix and the frame")
sys.exit(0 if ok else 1)
