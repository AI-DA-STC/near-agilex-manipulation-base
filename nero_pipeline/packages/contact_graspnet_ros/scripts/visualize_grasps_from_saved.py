#!/usr/bin/env python3
"""
Loads a saved depth map + segmentation map (from the .npy files produced
by contact_graspnet_node.py's debug saver, or your own), re-runs the full
Contact-GraspNet pipeline, and opens their real Open3D visualization window
showing the point cloud with grasp gripper markers overlaid - same window
you saw with their bundled test_data, but on your own live carrot capture.

Run inside the `contact_graspnet` conda env:

    conda activate contact_graspnet
    cd ~/contact_graspnet_pytorch
    python visualize_grasps_from_saved.py \
        --depth_npy /tmp/last_depth.npy \
        --segmap_npy /tmp/last_segmap.npy \
        --K "[476.9205627441406, 0, 322.4602355957031, 0, 476.9205627441406, 200.17709350585938, 0, 0, 1]" \
        --ckpt_dir ~/contact_graspnet_pytorch/checkpoints/contact_graspnet
"""

import argparse
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
        depth, K, segmap=segmap, rgb=None, skip_border_objects=False, z_range=z_range,
    )

    if pc_colors is None:
        # No RGB was provided, so give every point a plain grey color -
        # visualize_grasps() requires pc_colors to not be None.
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
