#!/usr/bin/env python3
"""MoSAIC-DUO model server. Runs on the LAPTOP GPU (conda env p5_gpu). Loads GraspMolmo and SAM2 ONCE and answers
HTTP requests from the Jetson, so the slow, GPU-heavy models run on the RTX 5090 while the arm side stays on the Jetson.

  GET  /health    -> {"ok": true, "version": ..., "graspmolmo": bool, "sam2": bool}
  POST /point     {"image_b64": png, "queries": [{"name", "text"}], "max_new_tokens": 256}
                  -> the same reply as p3_graspmolmo_worker: {"ok", "results": [{"name","uv","answer","seconds","tokens"}], "seconds"}
  POST /segment   {"image_b64": png, "point": [u, v]}
                  -> {"ok", "masks_png_b64": [3 x 0/255 png], "scores": [3 floats], "seconds"}   (SAM2's 3 proposals, like p5_sam2_worker)

Only run it on a trusted network (it has no password). --fake loads no models (offline tests only).
"""
import argparse
import base64
import io
import os
import re
import sys
import threading
import time
import traceback
import xml.etree.ElementTree as ET

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "3")
os.environ["TORCHDYNAMO_DISABLE"] = "1"
P5_SERVER_VERSION = "p5-server-v2"   # v2: GraspMolmo pinned to the Jetson's snapshot, one log line per request, /health shows the revision

MODEL = "allenai/GraspMolmo"
REVISION = os.environ.get("P5_GM_REVISION", "9aa184d698ffc41382524875f0c585e7499c62f5")   # the Jetson's snapshot (same code, same weights)
STOP = ["<|endoftext|>", "</point>", "</points>"]
SAM2_CFG = "configs/sam2.1/sam2.1_hiera_l.yaml"


def log(msg):
    print(f"[p5 server] {msg}", file=sys.stderr, flush=True)


def parse_point(text, w, h):
    """Molmo answers with x/y in percent of the image. Returns [u, v] in pixels or None (same as the Jetson worker)."""
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
    """GraspMolmo, loaded the same way as pipeline 3's worker (bf16 by default, straight onto the GPU)."""

    def __init__(self, dtype="bf16"):
        import torch
        from transformers import AutoModelForCausalLM, AutoProcessor
        self.torch = torch
        self.proc = AutoProcessor.from_pretrained(MODEL, revision=REVISION, trust_remote_code=True)
        self.model = AutoModelForCausalLM.from_pretrained(
            MODEL, revision=REVISION, trust_remote_code=True, torch_dtype=torch.bfloat16 if dtype == "bf16" else torch.float32,
            device_map={"": 0}, low_cpu_mem_usage=True)
        self.dtype = dtype
        self.model.eval()

    def ask(self, image, text, max_new_tokens):
        import contextlib
        from transformers import GenerationConfig
        torch = self.torch
        inputs = self.proc.process(text=text, images=[image], return_tensors="pt")
        if inputs["input_ids"].dim() == 1:
            inputs = {k: v.unsqueeze(0) for k, v in inputs.items()}
        fdt = torch.bfloat16 if self.dtype == "bf16" else torch.float32
        inputs = {k: (v.to(self.model.device, fdt) if v.is_floating_point() else v.to(self.model.device))
                  for k, v in inputs.items()}
        gen = GenerationConfig(max_new_tokens=max_new_tokens, stop_strings=STOP)
        torch.cuda.synchronize()
        t0 = time.time()
        ac = torch.autocast(device_type="cuda", dtype=torch.bfloat16) if self.dtype == "bf16" else contextlib.nullcontext()
        with torch.inference_mode(), ac:
            out = self.model.generate_from_batch(inputs, gen, tokenizer=self.proc.tokenizer)
        torch.cuda.synchronize()
        new = out[0, inputs["input_ids"].size(1):]
        return self.proc.tokenizer.decode(new, skip_special_tokens=True), time.time() - t0, int(new.numel())


class FakePointer:
    def ask(self, image, text, max_new_tokens):
        return '<point x="50.0" y="50.0" alt="fake">fake</point>', 0.01, 10


class Segmenter:
    """SAM2.1 image predictor: one positive click -> 3 mask proposals."""

    def __init__(self, ckpt, cfg=SAM2_CFG):
        import torch
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        self.torch = torch
        self.pred = SAM2ImagePredictor(build_sam2(cfg, ckpt, device="cuda"))

    def segment(self, rgb, uv):
        import numpy as np
        torch = self.torch
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
            self.pred.set_image(rgb)
            masks, scores, _ = self.pred.predict(
                point_coords=np.array([uv], dtype=np.float32), point_labels=np.array([1]), multimask_output=True)
        return [((m > 0).astype(np.uint8) * 255) for m in masks], [float(s) for s in scores]


class FakeSegmenter:
    def segment(self, rgb, uv):
        import numpy as np
        h, w = rgb.shape[:2]
        yy, xx = np.mgrid[0:h, 0:w]
        out = []
        for r in (20, 40, 60):
            out.append((((xx - uv[0]) ** 2 + (yy - uv[1]) ** 2) <= r * r).astype(np.uint8) * 255)
        return out, [0.9, 0.8, 0.7]


def decode_image(b64):
    from PIL import Image
    return Image.open(io.BytesIO(base64.b64decode(b64))).convert("RGB")


def create_app(pointer, segmenter):
    from fastapi import FastAPI
    from pydantic import BaseModel
    from typing import Any

    class PointReq(BaseModel):
        image_b64: str
        queries: list[dict[str, Any]]
        max_new_tokens: int = 256

    class SegReq(BaseModel):
        image_b64: str
        point: list[float]

    app = FastAPI(title="MoSAIC-DUO model server", version=P5_SERVER_VERSION)
    lock = threading.Lock()                                  # one GPU job at a time

    @app.get("/health")
    def health():
        return {"ok": True, "version": P5_SERVER_VERSION, "graspmolmo": pointer is not None, "sam2": segmenter is not None,
                "revision": REVISION[:8], "fake": isinstance(pointer, FakePointer) or isinstance(segmenter, FakeSegmenter)}

    @app.post("/point")
    def point(req: PointReq):
        if pointer is None:
            return {"ok": False, "error": "GraspMolmo is not loaded on the server"}
        try:
            image = decode_image(req.image_b64)
            w, h = image.size
            t0 = time.time()
            results = []
            with lock:
                for q in req.queries:
                    ans, dt, ntok = pointer.ask(image, q["text"], int(req.max_new_tokens))
                    results.append({"name": q["name"], "uv": parse_point(ans, w, h), "answer": ans.strip()[:400],
                                    "seconds": round(dt, 2), "tokens": ntok})
            dt = round(time.time() - t0, 2)
            log("point: " + ", ".join("%s %.2f s %s" % (r["name"], r["seconds"], "uv=(%.0f,%.0f)" % tuple(r["uv"]) if r["uv"] else "NO POINT") for r in results) + " | %.2f s total" % dt)
            return {"ok": True, "results": results, "seconds": dt}
        except Exception as e:
            return {"ok": False, "error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc()[-800:]}

    @app.post("/segment")
    def segment(req: SegReq):
        if segmenter is None:
            return {"ok": False, "error": "SAM2 is not loaded on the server"}
        try:
            import numpy as np
            from PIL import Image
            rgb = np.array(decode_image(req.image_b64))
            t0 = time.time()
            with lock:
                masks, scores = segmenter.segment(rgb, [float(req.point[0]), float(req.point[1])])
            pngs = []
            for m in masks:
                buf = io.BytesIO()
                Image.fromarray(m).save(buf, "PNG")
                pngs.append(base64.b64encode(buf.getvalue()).decode())
            dt = round(time.time() - t0, 2)
            log("segment at (%.0f,%.0f): scores %s, %.2f s" % (req.point[0], req.point[1], [round(x, 2) for x in scores], dt))
            return {"ok": True, "masks_png_b64": pngs, "scores": scores, "seconds": dt}
        except Exception as e:
            return {"ok": False, "error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc()[-800:]}

    return app


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--sam2-ckpt", default=os.path.expanduser("~/p5_server/checkpoints/sam2.1_hiera_large.pt"))
    ap.add_argument("--sam2-cfg", default=SAM2_CFG)
    ap.add_argument("--dtype", default="bf16", choices=["bf16", "fp32"])
    ap.add_argument("--no-graspmolmo", action="store_true")
    ap.add_argument("--no-sam2", action="store_true")
    ap.add_argument("--fake", action="store_true", help="no models; for offline tests")
    a = ap.parse_args()

    t0 = time.time()
    pointer = None if a.no_graspmolmo else (FakePointer() if a.fake else Pointer(a.dtype))
    log(f"{P5_SERVER_VERSION}: GraspMolmo {'off' if pointer is None else 'ready (revision ' + REVISION[:8] + ')'} after {time.time() - t0:.1f} s" + (" (FAKE)" if a.fake else ""))
    segmenter = None
    if not a.no_sam2:
        t1 = time.time()
        segmenter = FakeSegmenter() if a.fake else Segmenter(a.sam2_ckpt, a.sam2_cfg)
        log(f"SAM2 ready after {time.time() - t1:.1f} s" + (" (FAKE)" if a.fake else ""))
    if pointer is not None and not a.fake:
        try:                                                 # warm-up, so the first real request does not pay CUDA start-up
            from PIL import Image
            t2 = time.time()
            pointer.ask(Image.new("RGB", (640, 480), (128, 128, 128)), "Point to the plate.", 32)
            log(f"GraspMolmo warm-up done in {time.time() - t2:.1f} s")
        except Exception:
            log("warm-up failed (ignored):\n" + traceback.format_exc()[-600:])
    import uvicorn
    log(f"listening on http://{a.host}:{a.port}  (trusted network only)")
    uvicorn.run(create_app(pointer, segmenter), host=a.host, port=a.port, log_level="warning")


if __name__ == "__main__":
    main()
