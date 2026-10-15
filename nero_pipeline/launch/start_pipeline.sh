#!/bin/bash
# Brings up the entire perception + task-planning pipeline in one command,
# instead of manually opening 8+ terminals every session. Everything runs
# backgrounded with its own log file under ~/nero_pipeline_logs/<timestamp>/
# - press Ctrl+C in THIS terminal to cleanly stop everything at once.
#
# Uses the SIM arm by default (demo.launch.py) since that's what daily
# perception-pipeline testing has been using. Swap to the real arm by
# editing the ARM_LAUNCH_CMD line below (see the commented-out real-arm
# version) - do this deliberately, not by default, given the real arm
# needs CAN activated first (see nero-arm-setup/docs/01-can-setup.md).
#
# Usage:
#   chmod +x start_pipeline.sh
#   ./start_pipeline.sh
#   (Ctrl+C to stop everything)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$HOME/nero_pipeline_logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
echo "Logs: $LOG_DIR"

source /opt/ros/humble/setup.bash
source "$HOME/agx_arm_ws/install/setup.bash"

PIDS=()

start() {
    local name="$1"
    shift
    echo "Starting $name..."
    "$@" > "$LOG_DIR/$name.log" 2>&1 &
    PIDS+=($!)
}

cleanup() {
    echo ""
    echo "Stopping all pipeline processes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "Stopped. Logs kept at $LOG_DIR"
}
trap cleanup SIGINT SIGTERM

# --- Arm (sim by default) ---
if [ "${REAL_ARM:-0}" = 1 ]; then ARM_LAUNCH_CMD="$HOME/nero_pipeline_launch/scripts/launch_arm_real.sh"; else ARM_LAUNCH_CMD="$HOME/nero_pipeline_launch/scripts/launch_arm.sh"; fi
# Real arm instead (requires CAN already activated - see nero-arm-setup docs):
# ARM_LAUNCH_CMD="ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can_piper arm_type:=nero effector_type:=agx_gripper control:=false follow:=true"
start "arm" bash -c "$ARM_LAUNCH_CMD"
sleep "${ARM_WAIT:-5}"

# --- Safety: table collision object ---
# MUST run every session - MoveIt's planning scene is in-memory only and
# does not persist across restarts. Without this, the arm has zero
# awareness the table physically exists and could plan a path straight
# through it. This is a one-shot script (not a persistent node), run
# directly (not backgrounded) so its success/failure prints immediately -
# if it fails, stop and investigate before doing anything else.
echo "Adding table collision object (safety-critical, not backgrounded)..."
python3 "$SCRIPT_DIR/scripts/add_table_collision.py" 2>&1 | tee "$LOG_DIR/table_collision.log"
echo ""
echo "!!! REMINDER: table collision only stops the arm going THROUGH the"
echo "!!! table - it does NOT stop swinging past the table's real edges."
echo "!!! Supervise the arm directly during any real execution."
echo ""

# --- Top camera ---
start "camera" ros2 launch orbbec_camera ob_camera.launch.py \
    camera_name:=topcam \
    enable_depth:=true depth_width:=640 depth_height:=400 depth_fps:=30 depth_format:=Y12 \
    enable_color:=true color_width:=640 color_height:=480 color_fps:=30 color_format:=MJPG \
    enable_ir:=true ir_format:=Y10 \
    enable_point_cloud:=true \
    depth_registration:=true
sleep 5

# --- Hand-eye calibration publisher ---
start "handeye_publisher" ros2 launch easy_handeye2 publish.launch.py name:=nero_eye_to_hand_calib
sleep 2

# --- Perception + task-planning pipeline nodes ---
start "gsam2_ros" ros2 launch gsam2_ros groundedsam2.launch.py \
    source_prompt:="a green cube." target_prompt:="plate."
sleep 3

start "contact_graspnet_ros" ros2 launch contact_graspnet_ros contactgraspnet.launch.py forward_passes:=${CGN_FORWARD_PASSES:-5} recentre_on_object:=${CGN_RECENTRE:-true} recentre_depth_below_top_m:=${CGN_DEPTH:-0.006}
sleep 3

start "target_pose_ros" ros2 launch target_pose_ros targetpose.launch.py
sleep 2

start "pose_transform_ros" ros2 launch pose_transform_ros posetransform.launch.py
sleep 2

# --- Debug: live-updating grasp/target markers in RViz ---
# Automatically re-broadcasts the latest /grasp_pose_transformed and
# /target_pose_transformed as TF frames (debug_grasp_point,
# debug_target_point) - updates every time the perception chain runs, no
# manual static_transform_publisher commands needed.
start "debug_pose_tf_broadcaster" python3 "$SCRIPT_DIR/scripts/debug_pose_tf_broadcaster.py"
sleep 1

echo ""
echo "=== All nodes started ==="
echo "Logs: $LOG_DIR"
echo ""
echo "Run the perception chain with:"
echo "  ros2 service call /gsam2/segment std_srvs/srv/Trigger"
echo "  ros2 service call /contact_graspnet/detect_grasps std_srvs/srv/Trigger"
echo "  ros2 service call /target_pose/extract std_srvs/srv/Trigger"
echo "  ros2 service call /pose_transform/transform_all std_srvs/srv/Trigger"
echo ""
echo "Then run task planning with:"
echo "  ros2 run nero_mtc_task mtc_pick_and_place"
echo ""
echo "Press Ctrl+C here to stop everything."

wait
