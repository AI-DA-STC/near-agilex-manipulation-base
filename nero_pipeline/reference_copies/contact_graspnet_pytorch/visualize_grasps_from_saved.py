#!/usr/bin/env python3
import argparse
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "contact_graspnet_pytorch"))
import numpy as np

from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator
from contact_graspnet_pytorch import config_utils
from contact_graspnet_pytorch.checkpoints import CheckpointIO
from contact_graspnet_pytorch.visualization_utils_o3d import visualize_grasps


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--depth_npy", required=True)
    parser.add_argument("--segmap_npy", required=True)
    parser.add_argument("--K", required=True)
    parser.add_argument("--rgb_npy", default=None)
    parser.add_argument("--ckpt_dir", required=True)
    parser.add_argument("--z_range", default="[0.2,1.8]")
    parser.add_argument("--local_regions", action="store_true", default=True)
    parser.add_argument("--filter_grasps", action="store_true", default=True)
    parser.add_argument("--forward_passes", type=int, default=1)
    return parser.parse_args()


def main():
    args = parse_args()

    depth = np.load(args.depth_npy)
    segmap = np.load(args.segmap_npy)
    K = np.array(eval(args.K)).reshape(3, 3)
    z_range = eval(args.z_range)

    global_config = config_utils.load_config(args.ckpt_dir, batch_size=args.forward_passes)
    grasp_estimator = GraspEstimator(global_config)

    checkpoint_io = CheckpointIO(
        checkpoint_dir=f"{args.ckpt_dir}/checkpoints", model=grasp_estimator.model
    )
    checkpoint_io.load("model.pt")

    pc_full, pc_segments, pc_colors = grasp_estimator.extract_point_clouds(
        depth, K, segmap=segmap, rgb=(np.load(args.rgb_npy) if args.rgb_npy else None), skip_border_objects=False, z_range=z_range,
    )

    if pc_colors is None:
        pc_colors = np.full((pc_full.shape[0], 3), 160, dtype=np.uint8)

    pred_grasps_cam, scores, contact_pts, gripper_openings = grasp_estimator.predict_scene_grasps(
        pc_full, pc_segments=pc_segments,
        local_regions=args.local_regions, filter_grasps=args.filter_grasps,
        forward_passes=args.forward_passes,
    )

    total = sum(len(v) for v in pred_grasps_cam.values())
    print(f"Found {total} grasp(s) across {len(pred_grasps_cam)} segment(s). Opening viewer...")
    print("(rotate/zoom with mouse, close the window to exit)")

    visualize_grasps(pc_full, pred_grasps_cam, scores, pc_colors=pc_colors, gripper_openings=gripper_openings)


if __name__ == "__main__":
    main()
