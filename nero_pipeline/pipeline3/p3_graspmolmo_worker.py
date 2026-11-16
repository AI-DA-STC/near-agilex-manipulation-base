#!/usr/bin/env python3
"""Persistent GraspMolmo worker for PIPELINE 3 (GraspMolmo + Contact-GraspNet). Runs in the `graspmolmo` conda env, loads the model ONCE, answers requests.
Started by graspmolmo_node.py through worker_client.py (the same file protocol as gsam2_ros's worker):

    worker -> ready.json        once the model is loaded and warmed up
    client -> req_<id>.json     {"cmd": "point", "image": path, "queries": [{"name": "grasp", "text": "..."}, ...]}
    worker -> res_<id>.json     {"ok": true, "results": [{"name", "uv", "answer", "seconds", "tokens"}], "seconds": s}
                                or {"ok": false, "error": "..."}
    client -> quit              asks the worker to exit (it also exits when its parent or the spool directory is gone)

--fake answers every query with the image centre and loads no model (offline tests only).
"""
import argparse
import glob
import json
import os
import re
import sys
import time
import traceback
import xml.etree.ElementTree as ET

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "3")
os.environ["TORCHDYNAMO_DISABLE"] = "1"          # no torch.compile on the Jetson (no Triton), as in pipeline 2
P3_WORKER_VERSION = "p3-worker-v1"

MODEL = "allenai/GraspMolmo"
STOP = ["<|endoftext|>", "</point>", "</points>"]
POLL_S = 0.05


def log(msg):
    print(f"[graspmolmo worker] {msg}", file=sys.stderr, flush=True)


def write_json_atomic(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f)
    os.replace(tmp, path)


def parse_point(text, w, h):
    """Molmo answers with x/y in percent of the image. Returns [u, v] in pixels or None."""
    m = re.findall(r"<points?.*?</points?>", text, re.DOTALL)
    if not m:
        return None
    try:
        e = ET.fromstring(m[0])
        x, y = (e.get("x"), e.get("y")) if e.tag == "point" else (e.get("x1"), e.get("y1"))
        return [float(x) / 100.0 * w, float(y) / 100.0 * h]
    except (ET.ParseError, TypeError, ValueError):
        return None


class Pointer:
    def __init__(self, dtype="bf16"):
        import torch
        from transformers import AutoModelForCausalLM, AutoProcessor
        self.torch = torch
        self.proc = AutoProcessor.from_pretrained(MODEL, trust_remote_code=True)
        # bf16 (default; pipeline 2's compare check: the same points as fp32, within 6 px) or fp32, straight onto the GPU
        self.model = AutoModelForCausalLM.from_pretrained(
            MODEL, trust_remote_code=True, torch_dtype=torch.bfloat16 if dtype == "bf16" else torch.float32, device_map={"": 0}, low_cpu_mem_usage=True)
        self.dtype = dtype
        self.model.eval()

    def ask(self, image, text, max_new_tokens):
        from transformers import GenerationConfig
        torch = self.torch
        inputs = self.proc.process(text=text, images=[image], return_tensors="pt")
        if inputs["input_ids"].dim() == 1:                 # this processor returns unbatched tensors
            inputs = {k: v.unsqueeze(0) for k, v in inputs.items()}
        fdt = torch.bfloat16 if self.dtype == "bf16" else torch.float32
        inputs = {k: (v.to(self.model.device, fdt) if v.is_floating_point() else v.to(self.model.device))
                  for k, v in inputs.items()}
        gen = GenerationConfig(max_new_tokens=max_new_tokens, stop_strings=STOP)
        torch.cuda.synchronize(); t0 = time.time()
        import contextlib
        ac = torch.autocast(device_type="cuda", dtype=torch.bfloat16) if self.dtype == "bf16" else contextlib.nullcontext()
        with torch.inference_mode(), ac:
            out = self.model.generate_from_batch(inputs, gen, tokenizer=self.proc.tokenizer)
        torch.cuda.synchronize()
        new = out[0, inputs["input_ids"].size(1):]
        return self.proc.tokenizer.decode(new, skip_special_tokens=True), time.time() - t0, int(new.numel())


class FakePointer:
    def ask(self, image, text, max_new_tokens):
        return '<point x="50.0" y="50.0" alt="fake">fake</point>', 0.01, 10


def handle(pointer, req):
    from PIL import Image
    if req.get("cmd") != "point":
        return {"ok": False, "error": f"unknown cmd {req.get('cmd')!r}"}
    image = Image.open(req["image"]).convert("RGB")
    w, h = image.size
    t0 = time.time()
    results = []
    for q in req["queries"]:
        ans, dt, ntok = pointer.ask(image, q["text"], int(req.get("max_new_tokens", 256)))
        results.append({"name": q["name"], "uv": parse_point(ans, w, h), "answer": ans.strip()[:400],
                        "seconds": round(dt, 2), "tokens": ntok})
    return {"ok": True, "results": results, "seconds": round(time.time() - t0, 2)}


def parent_alive(pid):
    if not pid:
        return True
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spool", required=True)
    ap.add_argument("--parent_pid", type=int, default=0)
    ap.add_argument("--fake", action="store_true", help="no model; for offline tests")
    ap.add_argument("--dtype", default=os.environ.get("P3_GM_DTYPE", "bf16"), choices=["bf16", "fp32"])
    a = ap.parse_args()
    os.makedirs(a.spool, exist_ok=True)

    t0 = time.time()
    pointer = FakePointer() if a.fake else Pointer(a.dtype)
    load_s = time.time() - t0
    log(f"{P3_WORKER_VERSION}: model loaded in {load_s:.1f} s ({a.dtype})" + (" (FAKE)" if a.fake else ""))
    try:                                                   # warm-up: the first real query otherwise pays CUDA start-up
        from PIL import Image
        t1 = time.time()
        pointer.ask(Image.new("RGB", (640, 480), (128, 128, 128)), "Point to the plate.", 32)
        log(f"warm-up done in {time.time() - t1:.1f} s")
    except Exception:
        log("warm-up failed (ignored):\n" + traceback.format_exc()[-600:])
    write_json_atomic(os.path.join(a.spool, "ready.json"), {"load_s": round(load_s, 1), "pid": os.getpid(), "fake": a.fake, "dtype": a.dtype, "version": P3_WORKER_VERSION})

    while True:
        if not os.path.isdir(a.spool) or os.path.exists(os.path.join(a.spool, "quit")) or not parent_alive(a.parent_pid):
            log("exiting")
            return 0
        for req_path in sorted(glob.glob(os.path.join(a.spool, "req_*.json"))):
            rid = os.path.basename(req_path)[4:-5]
            try:
                with open(req_path) as f:
                    req = json.load(f)
                os.remove(req_path)
                reply = handle(pointer, req)
            except Exception as e:
                reply = {"ok": False, "error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc()[-800:]}
            write_json_atomic(os.path.join(a.spool, f"res_{rid}.json"), reply)
        time.sleep(POLL_S)


if __name__ == "__main__":
    sys.exit(main())
