#!/usr/bin/env python3
"""
Persistent Grounded-SAM-2 worker. Runs inside the `grounded_sam2` conda env (like grounded_sam2_inference.py) but stays alive:
it loads Grounding DINO and SAM2 ONCE and then answers requests, so a segmentation costs seconds instead of paying for
`conda run`, the imports and both model loads on every call (about 17 of every 20 seconds, measured on the Jetson).

Started and talked to by gsam2_ros (gsam2_ros/worker_client.py); you normally never run it by hand:
    conda run -n grounded_sam2 --no-capture-output python -u grounded_sam2_worker.py --spool /tmp/some_dir --parent_pid 1234

Protocol (plain files in the spool directory, so it does not depend on how `conda run` forwards stdin/stdout):
    worker  -> ready.json              written once the models are loaded and warmed up
    client  -> req_<id>.json           {"cmd": "segment", "image": path, "prompts": [...], "output_dir": dir, "box_threshold": f, "text_threshold": f}
    worker  -> res_<id>.json           {"ok": true, "results": [one dict per prompt], "seconds": s}   or {"ok": false, "error": "..."}
    client  -> quit                    (an empty file) asks the worker to exit
The worker also exits by itself when its parent process (the ROS node) is gone or the spool directory disappears.
Per prompt it writes <output_dir>/<index>/mask.png, annotated.jpg and result.json, exactly what grounded_sam2_inference.py writes.
The image is encoded by SAM2 once and shared by all prompts of a request.
"""
import argparse
import json
import os
import sys
import time
import traceback

import cv2
import numpy as np
import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
GROUNDING_DINO_MODEL_ID = "IDEA-Research/grounding-dino-tiny"
SAM2_CHECKPOINT = os.environ.get(
    "SAM2_CHECKPOINT",
    os.path.expanduser("~/near-agilex-manipulation-base/external/Grounded-SAM-2/checkpoints/sam2.1_hiera_large.pt"),
)
SAM2_MODEL_CONFIG = os.environ.get("SAM2_MODEL_CONFIG", "configs/sam2.1/sam2.1_hiera_l.yaml")
POLL_S = 0.05


def log(msg):
    print(f"[gsam2 worker] {msg}", file=sys.stderr, flush=True)


def write_json_atomic(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f)
    os.replace(tmp, path)


def load_models():
    t0 = time.time()
    processor = AutoProcessor.from_pretrained(GROUNDING_DINO_MODEL_ID)
    dino = AutoModelForZeroShotObjectDetection.from_pretrained(GROUNDING_DINO_MODEL_ID).to(DEVICE)
    predictor = SAM2ImagePredictor(build_sam2(SAM2_MODEL_CONFIG, SAM2_CHECKPOINT, device=DEVICE))
    return processor, dino, predictor, time.time() - t0


def segment(processor, dino, predictor, image_path, prompts, out_dir, box_threshold, text_threshold):
    """Same detection + segmentation as grounded_sam2_inference.py, for several prompts on one image."""
    image = Image.open(image_path).convert("RGB")
    image_np = np.array(image)
    predictor.set_image(image_np)                       # the heavy image encoder runs ONCE for all prompts
    results = []
    for i, prompt in enumerate(prompts):
        sub = os.path.join(out_dir, str(i))
        os.makedirs(sub, exist_ok=True)
        inputs = processor(images=image, text=prompt, return_tensors="pt").to(DEVICE)
        with torch.no_grad():
            outputs = dino(**inputs)
        det = processor.post_process_grounded_object_detection(
            outputs, inputs.input_ids, box_threshold=box_threshold, text_threshold=text_threshold,
            target_sizes=[image.size[::-1]],
        )[0]
        if len(det["boxes"]) == 0:
            res = {"detected": False, "prompt": prompt}
            with open(os.path.join(sub, "result.json"), "w") as f:
                json.dump(res, f, indent=2)
            results.append(res)
            continue
        best = int(torch.argmax(det["scores"]))          # highest-confidence detection only, as before
        box = det["boxes"][best].cpu().numpy().tolist()
        score = float(det["scores"][best])
        label = det["labels"][best]
        masks, mask_scores, _ = predictor.predict(box=np.array(box), multimask_output=False)
        mask = masks[0].astype(np.uint8) * 255
        mask_path = os.path.join(sub, "mask.png")
        cv2.imwrite(mask_path, mask)
        annotated = image_np.copy()
        x0, y0, x1, y1 = [int(v) for v in box]
        cv2.rectangle(annotated, (x0, y0), (x1, y1), (0, 255, 0), 2)
        overlay = annotated.copy()
        overlay[mask > 0] = (0, 0, 255)
        annotated = cv2.addWeighted(overlay, 0.4, annotated, 0.6, 0)
        cv2.imwrite(os.path.join(sub, "annotated.jpg"), cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
        res = {"detected": True, "prompt": prompt, "label": label, "box_score": score, "mask_score": float(mask_scores[0]),
               "box_xyxy": box, "mask_path": mask_path}
        with open(os.path.join(sub, "result.json"), "w") as f:
            json.dump(res, f, indent=2)
        results.append(res)
    return results


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
    ap.add_argument("--spool", required=True, help="directory used to talk to the client")
    ap.add_argument("--parent_pid", type=int, default=0, help="exit when this process is gone")
    args = ap.parse_args()
    os.makedirs(args.spool, exist_ok=True)

    processor, dino, predictor, load_s = load_models()
    log(f"models loaded in {load_s:.1f} s on {DEVICE} (torch {torch.__version__})")
    try:                                                # warm-up: the first real inference otherwise pays CUDA kernel start-up
        t0 = time.time()
        dummy = np.full((480, 640, 3), 128, np.uint8)
        Image.fromarray(dummy).save(os.path.join(args.spool, "warmup.jpg"))
        segment(processor, dino, predictor, os.path.join(args.spool, "warmup.jpg"), ["object."], os.path.join(args.spool, "warmup_out"), 0.99, 0.99)
        log(f"warm-up done in {time.time() - t0:.1f} s")
    except Exception:
        log("warm-up failed (ignored):\n" + traceback.format_exc()[-600:])
    write_json_atomic(os.path.join(args.spool, "ready.json"), {"device": DEVICE, "load_s": round(load_s, 2), "pid": os.getpid()})

    while True:
        if not os.path.isdir(args.spool) or os.path.exists(os.path.join(args.spool, "quit")) or not parent_alive(args.parent_pid):
            log("exiting (quit requested, spool removed, or parent gone)")
            return
        reqs = sorted(f for f in os.listdir(args.spool) if f.startswith("req_") and f.endswith(".json"))
        if not reqs:
            time.sleep(POLL_S)
            continue
        req_file = os.path.join(args.spool, reqs[0])
        req_id = reqs[0][len("req_"):-len(".json")]
        try:
            with open(req_file) as f:
                req = json.load(f)
            os.remove(req_file)
            t0 = time.time()
            if req.get("cmd") == "ping":
                reply = {"ok": True, "pong": True}
            else:
                results = segment(processor, dino, predictor, req["image"], req["prompts"], req["output_dir"],
                                  float(req.get("box_threshold", 0.25)), float(req.get("text_threshold", 0.3)))
                reply = {"ok": True, "results": results, "seconds": round(time.time() - t0, 3)}
        except Exception:
            reply = {"ok": False, "error": traceback.format_exc()[-1500:]}
            try:
                os.remove(req_file)
            except OSError:
                pass
        write_json_atomic(os.path.join(args.spool, f"res_{req_id}.json"), reply)


if __name__ == "__main__":
    main()
