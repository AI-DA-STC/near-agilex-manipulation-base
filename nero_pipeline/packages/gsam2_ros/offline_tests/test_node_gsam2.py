import os, sys, types, importlib.util, tempfile, time
HERE = os.path.dirname(os.path.abspath(__file__)); BASE = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "nstubs")); sys.path.insert(0, BASE)
import numpy as np
spec = importlib.util.spec_from_file_location("gsam2_node", os.path.join(BASE, "gsam2_ros", "gsam2_segmentation_node.py"))
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
from gsam2_ros.worker_client import Gsam2Worker
PY = os.environ.get("TEST_PYTHON", sys.executable)
env = dict(os.environ, PYTHONPATH=os.path.join(HERE, "mstubs"), STUB_LOAD_S="0.3")
ok = True
def check(c, msg):
    global ok; print(("  PASS  " if c else "  FAIL  ") + msg); ok = ok and c
class Resp: pass
def fresh(worker=True):
    n = m.GSam2SegmentationNode(); n._latest_frame = np.full((120, 160, 3), 128, np.uint8)
    if worker:
        n._worker = Gsam2Worker([PY, "-u", os.path.join(BASE, "scripts", "grounded_sam2_worker.py")], env=env, log=lambda s: None); n._worker.start()
    return n
def call(n, src, tgt):
    n.set_param("source_prompt", src); n.set_param("target_prompt", tgt); r = Resp(); n._segment_cb(None, r); return r

n0 = m.GSam2SegmentationNode()
check(n0._worker is None and any("does not exist" in l for k, l in n0.get_logger().lines if k == "warn"), "use_worker on but the worker script missing -> a warning and the old path (no crash)")
print("declared defaults:", {k: n0._p[k] for k in ("use_worker", "worker_script", "worker_timeout_sec")})

print("\n=== 1. normal call through the real worker")
n = fresh(); t0 = time.time(); r = call(n, "a green cube.", "plate."); dt = time.time() - t0
print("   response:", r.message); print("   took %.2f s" % dt)
check(r.success and r.message.startswith("Source: 'plate' (box=0.800, mask=0.930). Target:"), "success, message format identical to the old node")
topics = [t for t, _ in n.published]
check(topics == ["/gsam2/mask_source", "/gsam2/annotated_source", "/gsam2/mask_target", "/gsam2/annotated_target"], "four images published on the same topics, same order")
check(dict(n.published)["/gsam2/mask_source"][1] == "mono8" and dict(n.published)["/gsam2/annotated_source"][1] == "bgr8", "masks mono8, annotated bgr8 as before")
check(any("persistent worker" in l for k, l in n.get_logger().lines), "the log line says the persistent worker was used and how long it took")
t0 = time.time(); r_b = call(n, "a green cube.", "plate."); dt2 = time.time() - t0
check(r_b.success and dt2 < 1.5, "a second call is fast (%.2f s) - no model loading" % dt2)

print("\n=== 2. failures keep the old wording")
r2 = call(n, "nothing here.", "plate.")
check((not r2.success) and r2.message == "Source ('nothing here.') failed: No detection for prompt 'nothing here.'", "source not found: '%s'" % r2.message)
r3 = call(n, "a green cube.", "nothing here too.")
check((not r3.success) and r3.message == "Target ('nothing here too.') failed: No detection for prompt 'nothing here too.'", "target not found: '%s'" % r3.message)
n.published.clear(); call(n, "nothing here.", "plate."); check(n.published == [], "nothing is published when a prompt fails")

print("\n=== 3. worker dies during a call -> automatic fallback to the old per-prompt path")
calls = []
def fake_run(prompt, frame):
    calls.append(prompt); d = tempfile.mkdtemp(); mk = np.zeros((120, 160), np.uint8); mk[10:50, 10:50] = 255
    return True, "ok", mk, np.zeros((120, 160, 3), np.uint8), {"label": "fallback-" + prompt, "box_score": 0.5, "mask_score": 0.6}
n._run_segmentation = fake_run
r4 = call(n, "crash now.", "plate.")
check(r4.success and calls == ["crash now.", "plate."] and "fallback-crash now." in r4.message, "the call still succeeded through the old path (%s)" % calls)
check(any(k == "error" and "falling back" in l for k, l in n.get_logger().lines), "the failure is logged as an error that says it is falling back")
check(call(n, "a green cube.", "plate.").success and calls == ["crash now.", "plate."], "next call uses the (restarted) worker again, not the fallback")

print("\n=== 4. use_worker=false (live parameter) goes straight to the old path")
calls.clear(); n.set_param("use_worker", False); r5 = call(n, "a green cube.", "plate.")
check(r5.success and calls == ["a green cube.", "plate."], "old behaviour restored without restarting anything")
n._worker.stop()
sys.exit(0 if ok else 1)
