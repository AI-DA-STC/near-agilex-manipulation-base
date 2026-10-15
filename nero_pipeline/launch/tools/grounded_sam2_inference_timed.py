#!/usr/bin/env python3
"""
Standalone Grounded-SAM-2 inference script. Runs inside the `grounded_sam2`
conda env (has its own torch/torchvision/transformers - do not run this
with system Python). Called as a subprocess by the ROS2 node in
gsam2_ros/gsam2_segmentation_node.py, so it keeps the heavy perception
dependencies fully isolated from the ROS Python environment.

Usage:
    conda run -n grounded_sam2 python grounded_sam2_inference.py \
        --image /path/to/input.jpg \
        --text_prompt "carrot." \
        --output_dir /path/to/output \
        --box_threshold 0.4 \
        --text_threshold 0.3

Writes to output_dir:
    - mask.png                 (single-channel binary mask, 255 = object)
    - annotated.jpg             (visualization, boxes + mask overlay)
    - result.json               (boxes, scores, labels, mask path)

Text prompts must end with a period per Grounding DINO's convention, e.g.
"carrot." or "red cup. blue plate." for multiple classes.
"""

import time
_T0 = time.time()
_LAST = [_T0]


def lap(label):
    """[timing] line: time since the previous lap and since the start of the script"""
    now = time.time()
    print(f"[timing] {label}: {now - _LAST[0]:6.2f} s   (total {now - _T0:6.2f} s)", flush=True)
    _LAST[0] = now


def sync():
    try:
        if torch.cuda.is_available():
            torch.cuda.synchronize()
    except Exception:
        pass


import argparse
import json
import os

import cv2
import numpy as np
import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
lap("1. python start + imports (torch, transformers, sam2)")
print(f"[timing] DEVICE = {DEVICE}   torch {torch.__version__}   cuda available: {torch.cuda.is_available()}", flush=True)

GROUNDING_DINO_MODEL_ID = "IDEA-Research/grounding-dino-tiny"
SAM2_CHECKPOINT = os.environ.get(
    "SAM2_CHECKPOINT",
    os.path.expanduser(
        "~/near-agilex-manipulation-base/external/Grounded-SAM-2/checkpoints/sam2.1_hiera_large.pt"
    ),
)
SAM2_MODEL_CONFIG = os.environ.get("SAM2_MODEL_CONFIG", "configs/sam2.1/sam2.1_hiera_l.yaml")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True, help="Path to input RGB image")
    parser.add_argument("--text_prompt", required=True, help='e.g. "carrot."')
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--box_threshold", type=float, default=0.4)
    parser.add_argument("--text_threshold", type=float, default=0.3)
    return parser.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    image = Image.open(args.image).convert("RGB")
    image_np = np.array(image)

    processor = AutoProcessor.from_pretrained(GROUNDING_DINO_MODEL_ID)
    grounding_model = AutoModelForZeroShotObjectDetection.from_pretrained(
        GROUNDING_DINO_MODEL_ID
    ).to(DEVICE)
    sync(); lap("2. load Grounding DINO (processor + weights)")

    inputs = processor(images=image, text=args.text_prompt, return_tensors="pt").to(DEVICE)
    with torch.no_grad():
        outputs = grounding_model(**inputs)

    sync(); lap("3. Grounding DINO detection (the actual compute)")
    results = processor.post_process_grounded_object_detection(
        outputs,
        inputs.input_ids,
        box_threshold=args.box_threshold,
        text_threshold=args.text_threshold,
        target_sizes=[image.size[::-1]],
    )[0]

    if len(results["boxes"]) == 0:
        print(f"No detections for prompt: '{args.text_prompt}'")
        with open(os.path.join(args.output_dir, "result.json"), "w") as f:
            json.dump({"detected": False, "prompt": args.text_prompt}, f, indent=2)
        return

    # Take the highest-confidence detection only - one object per call.
    # For multi-object scenes, call this script once per detected box instead.
    best_idx = int(torch.argmax(results["scores"]))
    box = results["boxes"][best_idx].cpu().numpy().tolist()
    score = float(results["scores"][best_idx])
    label = results["labels"][best_idx]

    sam2_model = build_sam2(SAM2_MODEL_CONFIG, SAM2_CHECKPOINT, device=DEVICE)
    sync(); lap("4. build SAM2 (load the checkpoint)")
    sam2_predictor = SAM2ImagePredictor(sam2_model)
    sam2_predictor.set_image(image_np)
    sync(); lap("5. SAM2 image encoder (set_image, the heavy compute)")

    masks, mask_scores, _ = sam2_predictor.predict(
        box=np.array(box),
        multimask_output=False,
    )
    sync(); lap("6. SAM2 predict (box -> mask)")
    mask = masks[0].astype(np.uint8) * 255  # binary mask, single channel

    mask_path = os.path.join(args.output_dir, "mask.png")
    cv2.imwrite(mask_path, mask)

    annotated = image_np.copy()
    x0, y0, x1, y1 = [int(v) for v in box]
    cv2.rectangle(annotated, (x0, y0), (x1, y1), (0, 255, 0), 2)
    overlay = annotated.copy()
    overlay[mask > 0] = (0, 0, 255)
    annotated = cv2.addWeighted(overlay, 0.4, annotated, 0.6, 0)
    cv2.imwrite(
        os.path.join(args.output_dir, "annotated.jpg"),
        cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR),
    )

    with open(os.path.join(args.output_dir, "result.json"), "w") as f:
        json.dump(
            {
                "detected": True,
                "prompt": args.text_prompt,
                "label": label,
                "box_score": score,
                "mask_score": float(mask_scores[0]),
                "box_xyxy": box,
                "mask_path": mask_path,
            },
            f,
            indent=2,
        )

    print(f"Detected '{label}' (score={score:.3f}), mask saved to {mask_path}")
    lap("7. write mask, picture and json")
    print(f"[timing] TOTAL inside the script: {time.time() - _T0:.2f} s", flush=True)


if __name__ == "__main__":
    main()
