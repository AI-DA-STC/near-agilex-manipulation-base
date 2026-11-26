#!/usr/bin/env python3
"""Persistent SAM2 worker for PIPELINE 5: the object under GraspMolmo's point, as a real mask. Runs in the `grounded_sam2` conda env (pipeline 1's),
with the Grounded-SAM-2 folder as working directory (SAM2 finds its configs there), loads the model ONCE. Started by p5_grasp_node.py through
core/worker_client.py (the same file protocol as GraspMolmo's worker):

    worker -> ready.json        once the model is loaded and warmed up
    client -> req_<id>.json     {"cmd": "segment", "image": path (BGR png), "point": [u, v], "out_dir": path}
    worker -> res_<id>.json     {"ok": true, "masks": [png path, ...], "scores": [...], "seconds": s}   (SAM2's 3 proposals, 0/255 png)
                                or {"ok": false, "error": "..."}
    client -> quit              asks the worker to exit (it also exits when its parent or the spool directory is gone)

--fake: no model, answers with 3 discs (radius 10/25/60 px) around the point (offline tests only; needs only numpy + cv2).
"""
import argparse
import glob
import json
import os
import sys
import time
import traceback

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "3")
os.environ["TORCHDYNAMO_DISABLE"] = "1"          # no torch.compile on the Jetson (no Triton)
P5_SAM2_WORKER_VERSION = "p5-sam2-worker-v1"
POLL_S = 0.03
MODELS = {"large": ("sam2.1_hiera_large.pt", "configs/sam2.1/sam2.1_hiera_l.yaml"),
          "base_plus": ("sam2.1_hiera_base_plus.pt", "configs/sam2.1/sam2.1_hiera_b+.yaml"),
          "small": ("sam2.1_hiera_small.pt", "configs/sam2.1/sam2.1_hiera_s.yaml"),
          "tiny": ("sam2.1_hiera_tiny.pt", "configs/sam2.1/sam2.1_hiera_t.yaml")}


def log(msg):
    print(f"[sam2 worker] {msg}", file=sys.stderr, flush=True)


def write_json_atomic(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f)
    os.replace(tmp, path)


class Segmenter:
    def __init__(self, model, repo):
        import torch
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        ckpt, cfg = MODELS[model]
        self.torch = torch
        self.pred = SAM2ImagePredictor(build_sam2(cfg, os.path.join(repo, "checkpoints", ckpt), device="cuda"))

    def segment(self, rgb, uv):
        import numpy as np
        torch = self.torch
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
            self.pred.set_image(rgb)
            masks, scores, _ = self.pred.predict(point_coords=np.array([uv], dtype=np.float32), point_labels=np.array([1]), multimask_output=True)
        return [m > 0 for m in masks], [float(s) for s in scores]


class FakeSegmenter:
    def segment(self, rgb, uv):
        import numpy as np
        h, w = rgb.shape[:2]
        yy, xx = np.mgrid[0:h, 0:w]
        d = np.hypot(xx - uv[0], yy - uv[1])
        return [d <= r for r in (10, 25, 60)], [0.6, 0.9, 0.8]


def handle(seg, req):
    import cv2
    import numpy as np
    if req.get("cmd") != "segment":
        return {"ok": False, "error": f"unknown cmd {req.get('cmd')!r}"}
    bgr = cv2.imread(req["image"], cv2.IMREAD_COLOR)
    if bgr is None:
        return {"ok": False, "error": "cannot read %s" % req["image"]}
    t0 = time.time()
    masks, scores = seg.segment(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), [float(v) for v in req["point"]])
    os.makedirs(req["out_dir"], exist_ok=True)
    paths = []
    for i, m in enumerate(masks):
        p = os.path.join(req["out_dir"], "sam2_mask_%d.png" % i)
        cv2.imwrite(p, (np.asarray(m, bool) * 255).astype(np.uint8))
        paths.append(p)
    return {"ok": True, "masks": paths, "scores": scores, "seconds": round(time.time() - t0, 2)}


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
    ap.add_argument("--model", default=os.environ.get("P5_SAM2_MODEL", "large"), choices=sorted(MODELS))
    ap.add_argument("--repo", default=os.path.expanduser(os.environ.get("P5_SAM2_REPO", "~/near-agilex-manipulation-base/external/Grounded-SAM-2")))
    a = ap.parse_args()
    os.makedirs(a.spool, exist_ok=True)
    t0 = time.time()
    seg = FakeSegmenter() if a.fake else Segmenter(a.model, a.repo)
    load_s = time.time() - t0
    log(f"{P5_SAM2_WORKER_VERSION}: {'FAKE' if a.fake else 'SAM2.1 ' + a.model} loaded in {load_s:.1f} s")
    try:                                                   # warm-up: the first real request otherwise pays CUDA start-up
        import numpy as np
        t1 = time.time()
        seg.segment(np.full((480, 640, 3), 128, np.uint8), [320.0, 240.0])
        log(f"warm-up done in {time.time() - t1:.1f} s")
    except Exception:
        log("warm-up failed (ignored):\n" + traceback.format_exc()[-600:])
    write_json_atomic(os.path.join(a.spool, "ready.json"), {"load_s": round(load_s, 1), "pid": os.getpid(), "fake": a.fake,
                                                             "model": "fake" if a.fake else a.model, "version": P5_SAM2_WORKER_VERSION})
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
                reply = handle(seg, req)
            except Exception as e:
                reply = {"ok": False, "error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc()[-800:]}
            write_json_atomic(os.path.join(a.spool, f"res_{rid}.json"), reply)
        time.sleep(POLL_S)


if __name__ == "__main__":
    sys.exit(main())
