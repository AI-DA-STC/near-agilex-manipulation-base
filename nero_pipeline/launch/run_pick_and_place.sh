#!/bin/bash
call_checked() { out=$(ros2 service call "$1" std_srvs/srv/Trigger 2>&1); echo "$out"; if echo "$out" | grep -q "success=False"; then echo "!!! $1 FAILED - stopping so the previous detection is not reused"; exit 1; fi; }
# Runs the full perception chain, then immediately runs task planning -
# combined into one script specifically to avoid the "topic published
# before the next node subscribed" timing gap that happens when these are
# run as separate manual commands with any delay between them.
#
# Requires start_pipeline.sh to already be running (all nodes up).
#
# Usage:
#   chmod +x run_pick_and_place.sh
#   ./run_pick_and_place.sh

set -e

echo "=== 0/5: Moving arm to the start pose (${START_POSE:-initial}) ==="
/usr/bin/python3 "$HOME/nero_pipeline_launch/scripts/move_to_pose_90deg.py" || { echo "start-pose move failed"; exit 1; }

source /opt/ros/humble/setup.bash
source "$HOME/agx_arm_ws/install/setup.bash"

echo "=== 1/5: Segmenting source + target ==="
call_checked /gsam2/segment

echo ""
echo "=== 2/5: Detecting grasp pose ==="
call_checked /contact_graspnet/detect_grasps

echo ""
echo "=== 3/5: Extracting target pose ==="
call_checked /target_pose/extract

echo ""
echo "=== 4/5: Transforming both poses into base_link ==="
call_checked /pose_transform/transform_all

echo ""
echo "=== 5/5: Running task planning (plan + execute) ==="
ros2 launch nero_mtc_task mtc_pick_and_place.launch.py arm_type:=nero effector_type:=agx_gripper tcp_offset:="[0.037, 0, -0.0235, -1.5708, 0, -1.5708]"
