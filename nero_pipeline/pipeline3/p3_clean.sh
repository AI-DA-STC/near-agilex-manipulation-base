#!/bin/bash
# Clean slate for pipeline 3 (also stops anything pipeline 1 or 2 left running) and restarts the ROS daemon - the docs/12 3.1 block plus the
# GraspMolmo processes of pipelines 2 and 3. On the REAL arm run it with the arm powered and at home.
# Afterwards, in your terminal:  unset REAL_ARM TABLE_TOP_Z JOINT_STATES_TOPIC HOME_MOVE_SECONDS ARM_WAIT
PATTERN="agx_arm_ctrl_single|move_group|rviz2|orbbec_camera|handeye_publisher|mtc_pick_and_place|p2_mtc|grasp_watch|grasp_view|grounded_sam2_worker|contact_graspnet_worker|contact_graspnet_node|gsam2_segmentation_node|target_pose_node|pose_transform|p2_debug_tf|p2_perceive|graspmolmo_select|m2t2_run_scene|m2t2_capture|gm_worker|gm_client|p3_graspmolmo|p3_jaw_fix|debug_pose_tf_broadcaster"
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/agx_arm_ws/install/setup.bash" 2>/dev/null
pkill -9 -f "$PATTERN"
ros2 daemon stop
sleep 2
ros2 daemon start
sleep 1
if pgrep -af "$PATTERN" | grep -v pgrep; then echo "NOT clean: the processes above are still running"; exit 1; else echo clean; fi
