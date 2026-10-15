import os, sys, time, json, subprocess, tempfile
HERE = os.path.dirname(os.path.abspath(__file__)); BASE = os.path.dirname(HERE)
sys.path.insert(0, BASE)
from gsam2_ros.worker_client import Gsam2Worker, WorkerError
import numpy as np
from PIL import Image

PY = os.environ.get("TEST_PYTHON", sys.executable)
env = dict(os.environ, PYTHONPATH=os.path.join(HERE, "mstubs"), STUB_LOAD_S="1.0")      # each stub model "loads" for 1 s
cmd = [PY, "-u", os.path.join(BASE, "scripts", "grounded_sam2_worker.py")]
tmp = tempfile.mkdtemp(); img = os.path.join(tmp, "in.jpg"); Image.fromarray(np.full((120, 160, 3), 128, np.uint8)).save(img)
msgs = []
w = Gsam2Worker(cmd, env=env, log=msgs.append, startup_timeout=60)
ok = True
def check(c, m):
    global ok; print(("  PASS  " if c else "  FAIL  ") + m); ok = ok and c
def req(prompts, out, **kw):
    return w.request(dict(cmd="segment", image=img, prompts=prompts, output_dir=out, box_threshold=0.25, text_threshold=0.3, **kw), timeout=30)

t0 = time.time(); w.start(); t_start = time.time() - t0
check(w.alive() and w.ready_info and w.ready_info["device"] == "cpu", "worker started and reported ready (took %.1f s: it loads the 3 models once)" % t_start)
out1 = os.path.join(tmp, "o1"); t0 = time.time(); r = req(["a green cube.", "plate."], out1); d1 = time.time() - t0
check(r["ok"] and len(r["results"]) == 2 and all(x["detected"] for x in r["results"]), "two prompts answered in ONE request")
check(os.path.exists(os.path.join(out1, "0", "mask.png")) and os.path.exists(os.path.join(out1, "1", "annotated.jpg")) and os.path.exists(os.path.join(out1, "1", "result.json")), "mask.png / annotated.jpg / result.json written per prompt")
check(set(r["results"][0]) >= {"detected", "prompt", "label", "box_score", "mask_score", "box_xyxy", "mask_path"}, "result fields match what the inference script wrote")
t0 = time.time(); r2 = req(["plate."], os.path.join(tmp, "o2")); d2 = time.time() - t0
print("   first request %.2f s, second %.2f s, start-up %.2f s" % (d1, d2, t_start))
check(d1 < 1.0 and d2 < 1.0 and t_start > 2.5, "requests are fast (< 1 s) because the models were loaded once at start-up (that took %.1f s, paid once)" % t_start)
r3 = req(["nothing here."], os.path.join(tmp, "o3"))
check(r3["ok"] and r3["results"][0]["detected"] is False, "a prompt with no detection gives detected=false (no crash)")
rb = w.request(dict(cmd="segment", image="/nonexistent.jpg", prompts=["plate."], output_dir=os.path.join(tmp, "o4")), timeout=30)
check(rb["ok"] is False and "No such file" in rb["error"], "a bad request returns ok=false with the error text")
r5 = req(["plate."], os.path.join(tmp, "o5")); check(r5["ok"], "the worker survived the bad request and still answers")
# crash recovery
pid = w._proc.pid; os.kill(pid, 9); time.sleep(0.3)
t0 = time.time(); r6 = req(["plate."], os.path.join(tmp, "o6"))
check(r6["ok"] and w._proc.pid != pid, "a worker that was already dead is restarted transparently by the next request (%.1f s)" % (time.time() - t0))
pid = w._proc.pid
try: req(["crash now."], os.path.join(tmp, "o6b")); check(False, "a worker dying DURING a request should raise")
except WorkerError as e: check("died while processing" in str(e), "a worker that dies during a request raises WorkerError: %s" % e)
t0 = time.time(); r7 = req(["plate."], os.path.join(tmp, "o7")); check(r7["ok"] and w._proc.pid != pid, "the request after that restarts the worker and succeeds (%.1f s)" % (time.time() - t0))
# timeout
try: w.request(dict(cmd="segment", image=img, prompts=["plate."], output_dir=os.path.join(tmp, "o8")), timeout=0.0001); check(False, "tiny timeout should raise")
except WorkerError as e: check("did not answer" in str(e), "a timeout raises WorkerError and stops the worker")
r9 = req(["plate."], os.path.join(tmp, "o9")); check(r9["ok"], "after a timeout the worker is restarted by the next request")
w.stop(); check(not w.alive(), "stop() shuts the worker down")
# parent-death exit
sp = tempfile.mkdtemp(); p = subprocess.Popen(cmd + ["--spool", sp, "--parent_pid", "999999"], env=dict(env, STUB_LOAD_S="0.05"))
try: p.wait(timeout=30); check(p.returncode == 0, "a worker whose parent process is gone exits by itself (no GPU hog left behind)")
except subprocess.TimeoutExpired: p.kill(); check(False, "worker did not exit when its parent was gone")
sys.exit(0 if ok else 1)
