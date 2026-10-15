import os, sys, types, importlib.util, tempfile, time, json
HERE = os.path.dirname(os.path.abspath(__file__)); BASE = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "nstubs")); sys.path.insert(0, BASE)
import numpy as np
spec = importlib.util.spec_from_file_location("cgn_node", os.path.join(BASE, "contact_graspnet_ros", "contact_graspnet_node.py"))
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
from contact_graspnet_ros.worker_client import SpoolWorker
PY = os.environ.get("TEST_PYTHON", sys.executable); WORKER = os.path.join(BASE, "scripts", "contact_graspnet_worker.py")
ok = True
def check(c, msg):
    global ok; print(("  PASS  " if c else "  FAIL  ") + msg); ok = ok and c
class Resp: pass
def make(empty_first=0, retries=2, worker=True):
    n = m.ContactGraspNetNode()
    n._latest_depth = np.full((480, 640), 870, np.uint16); n._latest_depth[300:332, 324:356] = 840      # millimetres, like the camera
    n._latest_mask = np.zeros((480, 640), np.uint8); n._latest_mask[300:332, 324:356] = 255
    n._camera_K = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], float)
    n.set_param("retries_if_empty", retries)
    if worker:
        env = dict(os.environ, PYTHONPATH=os.path.join(HERE, "mstubs"), STUB_LOAD_S="0.2", STUB_EMPTY_FIRST=str(empty_first))
        n._worker = SpoolWorker([PY, "-u", WORKER, "--ckpt_dir", "/x"], env=env, log=lambda s: None); n._worker.start()
    return n
def call(n): r = Resp(); n._detect_cb(None, r); return r
def logs(n, kind=None): return [l for k, l in n.get_logger().lines if kind is None or k == kind]

n0 = m.ContactGraspNetNode()
check(n0._worker is None and any("does not exist" in l for l in logs(n0, "warn")), "worker script missing -> a warning and the old path, no crash")
print("declared defaults:", {k: n0._p[k] for k in ("use_worker", "forward_passes", "retries_if_empty", "worker_timeout_sec")})

print("\n=== 1. normal call through the real worker")
n = make(); t0 = time.time(); r = call(n); dt = time.time() - t0
print("   response:", r.message)
check(r.success and r.message.startswith("Found 3 grasp(s). Best score=0.900 at position="), "success, message format identical to before")
pa = n.published[-1][1]
check(n.published[-1][0] == "/contact_graspnet/grasps" and len(pa.poses) == 3 and pa.header.frame_id == "topcam_depth_optical_frame", "PoseArray with 3 poses published on the same topic and frame")
check(any("persistent worker" in l and "attempts used: 1" in l for l in logs(n)), "log says the persistent worker was used and how long it took")
t0 = time.time(); call(n); check(time.time() - t0 < 0.6, "a second call is fast (%.2f s)" % (time.time() - t0))
n._worker.stop()

print("\n=== 2. empty random draws are retried")
n = make(empty_first=3, retries=2); r = call(n)       # the warm-up uses the first call, so 2 empty draws follow
check(r.success and any("attempts used: 3" in l for l in logs(n)), "2 empty draws then success: the call succeeds (%s)" % [l[-40:] for l in logs(n) if "attempts" in l])
n._worker.stop()
n = make(empty_first=2, retries=0); r = call(n)
check((not r.success) and r.message == "No grasps found", "retries_if_empty=0 keeps the old behaviour: 'No grasps found'")
n.published.clear(); check(call(n).success, "next call succeeds (random draw)"); n._worker.stop()

print("\n=== 3. fallback to the old script (worker dies, or use_worker=false); old messages unchanged")
seen = {}
def fake_run(cmd, **kw):
    seen["cmd"] = cmd; out = cmd[cmd.index("--output_json") + 1]; mode = seen.get("mode", "ok")
    class P: returncode = 0; stdout = "Found 2 grasp(s)"; stderr = ""
    if mode == "ok":
        json.dump({"detected": True, "num_grasps": 2, "grasps": [{"segment_id": 1, "position": [0, 0, 0.8], "quaternion": [0, 0, 0, 1], "score": 0.5, "gripper_opening": 0.05}] * 2}, open(out, "w"))
    elif mode == "fail": P.returncode = 1; P.stderr = "boom " * 200
    return P
m.subprocess.run = fake_run
n = make(); n._worker._proc.kill(); n._worker._proc.wait(); flag = tempfile.mkdtemp() + "/crash"
env = dict(os.environ, PYTHONPATH=os.path.join(HERE, "mstubs"), STUB_LOAD_S="0.2", STUB_CRASH_FLAG=flag)
n._worker = SpoolWorker([PY, "-u", WORKER, "--ckpt_dir", "/x"], env=env, log=lambda s: None); n._worker.start(); open(flag, "w").close()   # crash only after it is up
seen["mode"] = "ok"; r = call(n)
check(r.success and "Found 2 grasp(s)" in r.message and "--forward_passes" in seen["cmd"], "worker died mid-call -> old script used, call still succeeds")
check(any("falling back" in l for l in logs(n, "error")), "the failure is logged as an error that says it is falling back")
n._worker.stop(); n.set_param("use_worker", False); seen.clear(); seen["mode"] = "ok"; r = call(n)
check(r.success and "cmd" in seen, "use_worker=false (live parameter) goes straight to the old path")
seen["mode"] = "fail"; r = call(n); check((not r.success) and r.message.startswith("Inference script failed: boom") and len(r.message) <= 40 + 500, "old error message unchanged: '%s...'" % r.message[:40])
seen["mode"] = "nojson"; r = call(n); check((not r.success) and r.message == "Inference ran but produced no output json", "old 'no output json' message unchanged")

print("\n=== 5. optional re-centring (live parameter, default off)")
from contact_graspnet_ros.recentre import quat_to_R, CGN_GRIPPER_DEPTH
def closing_points(pa):
    out = []
    for q in pa.poses:
        ax = quat_to_R(q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w)[:, 2]
        out.append(np.array([q.position.x, q.position.y, q.position.z]) + CGN_GRIPPER_DEPTH * ax)
    return np.array(out)
n = make(); n._worker.stop(); n._worker = None; n.set_param("use_worker", False)
def fake_old(cmd, **kw):                  # the old one-process path, returning grasps whose closing centres are 0 / 1 / 2 / 6 cm to the right of the cube
    out = cmd[cmd.index("--output_json") + 1]
    gs = [{"segment_id": 1, "position": [0.01 * k - 0.0 + X0, Y0, Z0 - CGN_GRIPPER_DEPTH], "quaternion": [0, 0, 0, 1], "score": 0.3 - 0.01 * k, "gripper_opening": 0.009} for k in (0, 1, 2, 6)]
    json.dump({"detected": True, "num_grasps": 4, "grasps": gs}, open(out, "w"))
    class P: returncode = 0; stdout = ""; stderr = ""
    return P
v, u = np.nonzero(n._latest_mask > 0); z = n._latest_depth[v, u] * 0.001; Kc = n._camera_K
mid = np.array([((u - Kc[0, 2]) * z / Kc[0, 0]).mean(), ((v - Kc[1, 2]) * z / Kc[1, 1]).mean(), z.mean()])     # independent expectation (flat top face)
X0, Y0, Z0 = mid[0], mid[1], mid[2]
m.subprocess.run = fake_old
r = call(n); base = closing_points(n.published[-1][1])
check(r.success and len(base) == 4 and abs(base[3, 0] - (X0 + 0.06)) < 1e-9, "default (off): the poses are published exactly as the network gave them")
n.published.clear(); n.set_param("recentre_on_object", True); r = call(n); cl = closing_points(n.published[-1][1])
check(r.success and len(cl) == 3, "on: the grasp 6 cm to the side is dropped (default max offset 5 cm): %d left" % len(cl))
check(np.allclose(cl, np.array([X0, Y0, Z0 + 0.02]), atol=2e-3), "on: every remaining closing centre is the top-face middle, 2 cm below the top (%.3f, %.3f, %.3f)" % tuple(cl[0]))
check(any("recentred 3 grasp(s)" in l for l in logs(n, "info")), "the log says how many were re-centred and how many dropped")
X0 += 0.10; n.published.clear(); r = call(n); X0 -= 0.10            # now every grasp closes 10+ cm to the side of the cube
check((not r.success) and r.message.startswith("No grasps left after re-centring") and n.published == [], "if nothing is close enough the answer is a clear failure and nothing is published: '%s'" % r.message[:60])
n.set_param("recentre_max_offset_m", 0.005); n.published.clear(); r = call(n); k = len(closing_points(n.published[-1][1]))
check(r.success and k == 1, "a stricter limit (5 mm) keeps only the grasp that already closed at the middle: %d left" % k)
n.set_param("recentre_on_object", False); n.set_param("recentre_max_offset_m", 0.05); n.published.clear(); r = call(n)
check(r.success and len(closing_points(n.published[-1][1])) == 4, "switching it off again (live) restores the original behaviour")
n._latest_mask = np.zeros((480, 640), np.uint8); n.set_param("recentre_on_object", True); n.published.clear(); r = call(n)
check(r.success and any("could not find the object's top face" in l for l in logs(n, "warn")), "no usable top face: warning, grasps left as they are")

print("\n=== 6. the grasp frame follows the depth camera_info (auto), unless forced")
n = make(); n._worker.stop(); n._worker = None; n.set_param("use_worker", False); m.subprocess.run = fake_old
check(n0._p["camera_frame"] == "", "default: camera_frame is empty = automatic")
n.published.clear(); call(n); f0 = n.published[-1][1].header.frame_id
check(f0 == "topcam_depth_optical_frame", "no camera_info frame known yet: falls back to the old default ('%s')" % f0)
n._camera_frame_info = "topcam_color_optical_frame"; n.published.clear(); call(n); f1 = n.published[-1][1].header.frame_id
check(f1 == "topcam_color_optical_frame", "camera_info says the data is in the colour frame: the grasps are labelled '%s'" % f1)
n.set_param("camera_frame", "my_forced_frame"); n.published.clear(); call(n); f2 = n.published[-1][1].header.frame_id
check(f2 == "my_forced_frame", "an explicit camera_frame parameter still wins ('%s')" % f2)
n.set_param("camera_frame", ""); n._camera_frame_info = "topcam_depth_optical_frame"; n.published.clear(); call(n)
check(n.published[-1][1].header.frame_id == "topcam_depth_optical_frame", "unregistered depth (camera_info says depth frame): labelled with the depth frame")
check(sum(1 for l in logs(n, "info") if "grasp poses are labelled with frame" in l) >= 3, "every change of frame is logged once")
n._camera_info_cb(type("M", (), {"k": [488.8, 0, 317.8, 0, 488.8, 216.8, 0, 0, 1], "header": type("H", (), {"frame_id": "topcam_color_optical_frame"})()})())
check(n._camera_frame_info == "topcam_color_optical_frame" and abs(n._camera_K[0, 0] - 488.8) < 1e-9, "the camera_info callback stores both the matrix and the frame")
sys.exit(0 if ok else 1)
