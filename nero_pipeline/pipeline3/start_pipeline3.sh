#!/bin/bash
# PIPELINE 3 (GraspMolmo + Contact-GraspNet). A copy of start_pipeline.sh (pipeline 1); ONLY the perception lines differ:
# Grounded-SAM-2 is replaced by GraspMolmo (pipeline3/p3_graspmolmo_node.py), whose masks go to the same Contact-GraspNet and target_pose nodes.
# Prompts: P3_SOURCE (what to pick, default "green cube"), P3_TARGET (where to place, default "plate"). P3_GM_DTYPE=fp32 for full precision.
#   P3_SOURCE="carrot" ./start_pipeline3.sh
#
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
LOG_DIR="$HOME/nero_pipeline_logs/p3_$(date +%Y%m%d_%H%M%S)"
P3_START_VERSION="p3-start-v3"
echo "start_pipeline3 $P3_START_VERSION"
# Only ONE pipeline at a time: another GraspMolmo (pipeline 2's worker) or pipeline 1/2/3 nodes would clash or not fit in memory.
LEFT=$(pgrep -af "handeye_publisher|easy_handeye2 publish|gm_worker|p3_graspmolmo|p3_jaw_fix|gsam2_segmentation_node|contact_graspnet_node|target_pose_node|pose_transform_node|orbbec_camera|move_group" | grep -v pgrep || true)
if [ -n "$LEFT" ]; then
    echo "STOP: processes of an earlier run (pipeline 1, 2 or 3) are still running:"
    echo "$LEFT" | cut -c1-150
    echo "Stop them first:  bash ~/nero_pipeline_launch/pipeline3/p3_clean.sh   then start again."
    exit 1
fi
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
# PIPELINE 3: GraspMolmo points at the grasp and the plate (it loads the model once, about a minute); its masks go to the nodes below
start "graspmolmo" /usr/bin/python3 "$SCRIPT_DIR/pipeline3/p3_graspmolmo_node.py" --ros-args \
    -p source_prompt:="${P3_SOURCE:-green cube}" -p target_prompt:="${P3_TARGET:-plate}" -p dtype:="${P3_GM_DTYPE:-bf16}"
sleep 3

start "contact_graspnet_ros" ros2 launch contact_graspnet_ros contactgraspnet.launch.py mask_topic:=/graspmolmo/mask_source forward_passes:=${CGN_FORWARD_PASSES:-5} recentre_on_object:=${CGN_RECENTRE:-true} recentre_depth_below_top_m:=${CGN_DEPTH:-0.006}
sleep 3

start "target_pose_ros" ros2 launch target_pose_ros targetpose.launch.py mask_topic:=/graspmolmo/mask_target
sleep 2

# PIPELINE 3: Contact-GraspNet's jaws are on X, the Nero's tcp_link has them on Y, and pipeline 1's MTC uses the raw pose (APPLY_TCP_CONVERSION
# = False): the real fingers would close 90 deg turned from the intended grasp. p3_jaw_fix turns every grasp -90 deg about its approach axis
# before pose_transform (pipeline 1's own to_tcp_pose turn). P3_JAW_FIX=0 = the old behaviour.
if [ "${P3_JAW_FIX:-1}" != 0 ]; then
    start "p3_jaw_fix" /usr/bin/python3 "$SCRIPT_DIR/pipeline3/p3_jaw_fix.py"
    P3_GRASPS_TOPIC="/p3/grasps"
else
    echo "P3_JAW_FIX=0: grasps go to MoveIt unturned (the real fingers close 90 deg from the intended grasp)"
    P3_GRASPS_TOPIC="/contact_graspnet/grasps"
fi
start "pose_transform_ros" ros2 launch pose_transform_ros posetransform.launch.py grasps_topic:="$P3_GRASPS_TOPIC"
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
echo "  (GraspMolmo loads for about a minute: grep \"worker ready\" $LOG_DIR/graspmolmo.log)"
echo "  ros2 service call /graspmolmo/segment std_srvs/srv/Trigger"
echo "  ros2 service call /contact_graspnet/detect_grasps std_srvs/srv/Trigger"
echo "  ros2 service call /target_pose/extract std_srvs/srv/Trigger"
echo "  ros2 service call /pose_transform/transform_all std_srvs/srv/Trigger"
echo ""
echo "Or the whole run with timing (asks EXECUTE on the real arm):"
echo "  /usr/bin/python3 ~/nero_pipeline_launch/pipeline3/p3_go.py"
echo ""
echo "Press Ctrl+C here to stop everything."

wait
