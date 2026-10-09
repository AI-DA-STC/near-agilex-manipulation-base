#!/usr/bin/env python3
"""
Standalone Contact-GraspNet (PyTorch port) inference script. Runs inside
the `contact_graspnet` conda env. Calls the library's Python API directly
(GraspEstimator.extract_point_clouds + predict_scene_grasps) rather than
their CLI script, so no GUI/Open3D visualization window ever opens - this
is meant to be called headless as a subprocess from a ROS2 node.

Usage:
    conda run -n contact_graspnet python contact_graspnet_inference.py \
        --depth_npy /tmp/depth.npy \
        --segmap_npy /tmp/segmap.npy \
        --K "[fx, 0, cx, 0, fy, cy, 0, 0, 1]" \
        --ckpt_dir /path/to/contact_graspnet_pytorch/checkpoints/contact_graspnet \
        --output_json /tmp/output/grasps.json

depth_npy: depth map in METERS (not mm), shape (H, W), float32/float64
segmap_npy: integer segmentation map, shape (H, W), 0 = background,
            nonzero = object id. Convert a binary 0/255 mask from
            gsam2/mask to this by (mask > 0).astype(int).
K: flat 3x3 camera intrinsics matrix, row-major, as a string list

Writes to output_json:
    - detected: bool
    - grasps: list of {position: [x,y,z], quaternion: [x,y,z,w], score, gripper_opening},
      all in the CAMERA optical frame - a separate TF transform is still
      needed to get these into base_link (see nero-cam-config for the
      hand-eye calibration this depends on).
"""

import argparse
import json
import os

import numpy as np
import torch

from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator
from contact_graspnet_pytorch import config_utils
from contact_graspnet_pytorch.checkpoints import CheckpointIO


def rotation_matrix_to_quaternion(R):
    """3x3 rotation matrix -> [x, y, z, w] quaternion, no scipy dependency."""
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return [float(x), float(y), float(z), float(w)]


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--depth_npy", required=True, help="Depth map in meters, .npy")
    parser.add_argument("--segmap_npy", required=True, help="Integer segmentation map, .npy")
    parser.add_argument("--rgb_npy", default=None, help="Optional RGB image, .npy")
    parser.add_argument("--K", required=True, help='Flat 3x3 camera matrix, e.g. "[fx,0,cx,0,fy,cy,0,0,1]"')
    parser.add_argument("--ckpt_dir", required=True)
    parser.add_argument("--output_json", required=True)
    parser.add_argument("--z_range", default="[0.2,1.8]")
    parser.add_argument("--local_regions", action="store_true", default=True)
    parser.add_argument("--filter_grasps", action="store_true", default=True)
    parser.add_argument("--skip_border_objects", action="store_true", default=False)
    parser.add_argument("--forward_passes", type=int, default=1)
    return parser.parse_args()


def main():
    args = parse_args()
    os.makedirs(os.path.dirname(args.output_json), exist_ok=True)

    depth = np.load(args.depth_npy)
    segmap = np.load(args.segmap_npy)
    rgb = np.load(args.rgb_npy) if args.rgb_npy else None
    K = np.array(eval(args.K)).reshape(3, 3)
    z_range = eval(args.z_range)

    global_config = config_utils.load_config(args.ckpt_dir, batch_size=args.forward_passes)
    grasp_estimator = GraspEstimator(global_config)

    model_checkpoint_dir = os.path.join(args.ckpt_dir, "checkpoints")
    checkpoint_io = CheckpointIO(checkpoint_dir=model_checkpoint_dir, model=grasp_estimator.model)
    try:
        checkpoint_io.load("model.pt")
    except FileExistsError:
        with open(args.output_json, "w") as f:
            json.dump({"detected": False, "error": "No model checkpoint found"}, f, indent=2)
        return

    pc_full, pc_segments, pc_colors = grasp_estimator.extract_point_clouds(
        depth, K, segmap=segmap, rgb=rgb,
        skip_border_objects=args.skip_border_objects, z_range=z_range,
    )

    pred_grasps_cam, scores, contact_pts, gripper_openings = grasp_estimator.predict_scene_grasps(
        pc_full, pc_segments=pc_segments,
        local_regions=args.local_regions, filter_grasps=args.filter_grasps,
        forward_passes=args.forward_passes,
    )

    grasps_out = []
    # pred_grasps_cam is a dict keyed by segment id -> array of 4x4 poses
    for seg_id, poses in pred_grasps_cam.items():
        seg_scores = scores.get(seg_id, [])
        seg_openings = gripper_openings.get(seg_id, [None] * len(poses)) if gripper_openings else [None] * len(poses)
        for i, pose in enumerate(poses):
            R = pose[:3, :3]
            t = pose[:3, 3]
            grasps_out.append({
                "segment_id": int(seg_id),
                "position": [float(t[0]), float(t[1]), float(t[2])],
                "quaternion": rotation_matrix_to_quaternion(R),
                "score": float(seg_scores[i]) if i < len(seg_scores) else None,
                "gripper_opening": float(seg_openings[i]) if seg_openings[i] is not None else None,
            })

    grasps_out.sort(key=lambda g: g["score"] if g["score"] is not None else 0, reverse=True)

    with open(args.output_json, "w") as f:
        json.dump({"detected": len(grasps_out) > 0, "num_grasps": len(grasps_out), "grasps": grasps_out}, f, indent=2)

    print(f"Found {len(grasps_out)} grasp(s), saved to {args.output_json}")


if __name__ == "__main__":
    main()
