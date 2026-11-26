#!/bin/bash
# p5_clean.sh: stops everything pipelines 1-5 may have left running and restarts the ROS daemon. start_pipeline5.sh runs it first.
# On the REAL arm run it with the arm powered and at home.
P5_CLEAN_VERSION="p5-clean-v2f"   # v2: + the SAM2 worker
echo "$P5_CLEAN_VERSION"
P3="agx_arm_ctrl_single|move_group|rviz2|orbbec_camera|handeye_publisher|mtc_pick_and_place|p2_mtc|grasp_watch|grasp_view|grounded_sam2_worker|contact_graspnet_worker|contact_graspnet_node|gsam2_segmentation_node|target_pose_node|pose_transform|p2_debug_tf|p2_perceive|graspmolmo_select|m2t2_run_scene|m2t2_capture|gm_worker|gm_client|p3_graspmolmo|p3_jaw_fix|debug_pose_tf_broadcaster"
P4="realsense2_camera|armcam_calib_publisher|p4_fuse_node|p4_grasp.py|aruco_tracker|rqt_image_view|easy_handeye2|p3_mtc.py"
P5="p5_fuse_node|p5_grasp_node|p5_sam2_worker|p5_view.py|p5_view/dashboard|/tmp/p5_view/look"   # + its image viewer   # 7f: + the live window
PATTERN="$P3|$P4|$P5"
source /opt/ros/humble/setup.bash 2>/dev/null; source "$HOME/agx_arm_ws/install/setup.bash" 2>/dev/null
pkill -9 -f "$PATTERN"
ros2 daemon stop >/dev/null 2>&1; sleep 2; ros2 daemon start >/dev/null 2>&1; sleep 1
if pgrep -af "$PATTERN" | grep -v pgrep; then echo "NOT clean: the processes above are still running"; exit 1; else echo "clean"; fi
