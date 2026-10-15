import os, sys, time, json, subprocess, tempfile
HERE = os.path.dirname(os.path.abspath(__file__)); BASE = os.path.dirname(HERE)
sys.path.insert(0, BASE)
from contact_graspnet_ros.worker_client import SpoolWorker, WorkerError
import numpy as np
PY = os.environ.get("TEST_PYTHON", sys.executable)
WORKER = os.path.join(BASE, "scripts", "contact_graspnet_worker.py"); ORIG = os.path.join(BASE, "scripts", "contact_graspnet_inference.py")
tmp = tempfile.mkdtemp(); ok = True
def check(c, m):
    global ok; print(("  PASS  " if c else "  FAIL  ") + m); ok = ok and c
def envx(**kw):
    e = dict(os.environ, PYTHONPATH=os.path.join(HERE, "mstubs"), STUB_LOAD_S="0.5"); e.update(kw); return e
depth = np.full((480, 640), 0.87); depth[300:332, 324:356] = 0.84; seg = np.zeros((480, 640), np.int32); seg[300:332, 324:356] = 1
np.save(tmp + "/depth.npy", depth); np.save(tmp + "/segmap.npy", seg); np.save(tmp + "/empty_seg.npy", np.zeros((480, 640), np.int32))
K = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
def worker(**kw): return SpoolWorker([PY, "-u", WORKER, "--ckpt_dir", "/nonexistent"], env=envx(**kw), log=lambda s: None, startup_timeout=60)
def req(w, attempts=1, seg_file="segmap.npy", **kw): return w.request(dict(cmd="infer", depth_npy=tmp + "/depth.npy", segmap_npy=tmp + "/" + seg_file, K=K, z_range=[0.2, 1.8], local_regions=True, filter_grasps=True, skip_border_objects=False, attempts=attempts, **kw), timeout=30)

print("=== normal operation")
w = worker(); t0 = time.time(); w.start(); ts = time.time() - t0
r = req(w); t0 = time.time(); r2 = req(w); d2 = time.time() - t0
check(r["ok"] and r["result"]["detected"] and r["result"]["num_grasps"] == 3 and r["attempts"] == 1, "worker answers with 3 grasps in one attempt")
check(ts > 0.8 and d2 < 0.5, "model loaded once at start-up (%.1f s), the next request is fast (%.2f s)" % (ts, d2))
check([g["score"] for g in r["result"]["grasps"]] == [0.9, 0.8, 0.7], "grasps sorted best score first")

print("=== identical to the ORIGINAL inference script on the same input")
out = tmp + "/orig.json"
subprocess.run([PY, ORIG, "--depth_npy", tmp + "/depth.npy", "--segmap_npy", tmp + "/segmap.npy", "--K", str(K), "--ckpt_dir", "/nonexistent", "--output_json", out], env=envx(), check=True, capture_output=True)
orig = json.load(open(out))
check(orig == r["result"], "worker result == original script json (same positions, quaternions, scores, openings, order)")
w.stop()

print("=== empty predictions and retry (the stand-in returns nothing for the first calls; the warm-up uses the first)")
w = worker(STUB_EMPTY_FIRST="3"); w.start(); r = req(w, attempts=3)
check(r["ok"] and r["result"]["detected"] and r["attempts"] == 3, "2 empty draws, then success on attempt %s of 3" % r["attempts"]); w.stop()
w = worker(STUB_EMPTY_FIRST="2"); w.start(); r = req(w, attempts=1)
check(r["ok"] and r["result"] == {"detected": False, "num_grasps": 0, "grasps": []} and r["attempts"] == 1, "with attempts=1 the empty draw is reported as detected=false (same as the old 'No grasps found')")
check(req(w, attempts=1)["result"]["detected"], "and the next request succeeds (worker still fine)")
r = req(w, attempts=2, seg_file="empty_seg.npy"); check(r["ok"] and not r["result"]["detected"] and r["attempts"] == 2, "an empty mask gives no grasps after all attempts, no crash")
rb = w.request(dict(cmd="infer", depth_npy="/nonexistent.npy", segmap_npy="/nonexistent.npy", K=K), timeout=30)
check(rb["ok"] is False and "No such file" in rb["error"], "missing input file -> ok=false with the error text"); check(req(w)["ok"], "worker survived it"); w.stop()

print("=== failure handling")
w = worker(STUB_NO_CKPT="1")
try: w.start(); check(False, "missing checkpoint should raise")
except WorkerError as e: check("exited during start-up" in str(e), "no checkpoint -> worker exits, client raises (the node then uses the old script, which reports the checkpoint error)")
flag = tmp + "/crash"; w = worker(STUB_CRASH_FLAG=flag); w.start(); open(flag, "w").close()
try: req(w); check(False, "a worker dying mid-request should raise")
except WorkerError as e: check("died while processing" in str(e), "worker dying during a request raises WorkerError")
os.remove(flag); check(req(w)["ok"], "next request restarts it and succeeds"); w.stop()
sp = tempfile.mkdtemp(); p = subprocess.Popen([PY, WORKER, "--ckpt_dir", "/x", "--spool", sp, "--parent_pid", "999999"], env=envx(STUB_LOAD_S="0.05"))
try: p.wait(timeout=30); check(p.returncode == 0, "worker exits by itself when its parent process is gone")
except subprocess.TimeoutExpired: p.kill(); check(False, "worker did not exit when its parent was gone")
sys.exit(0 if ok else 1)
