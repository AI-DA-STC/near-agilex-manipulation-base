#!/bin/bash
# PIPELINE 4 - COMMAND 1 of 3: brings up EVERYTHING pipeline 4 needs, in one terminal (Ctrl+C here stops it all):
#   arm + MoveIt + RViz, table, top camera + its calibration, wrist camera (D405, cloud on) + its calibration, GraspMolmo, Contact-GraspNet,
#   target_pose, pipeline 4's grasp step (straight-down grasps, gripper opened to the object's width), pose_transform, the fusion node
#   (both cameras -> 3D wrist obstacles), a second RViz with the fused cloud; then checks everything and says READY.
#   Then:  bash ~/nero_pipeline_launch/pipeline4/p4_plan.sh "eggplant" "basket"      and      bash ~/nero_pipeline_launch/pipeline4/p4_execute.sh
# Options (set before the command): REAL_ARM=0 (simulated arm), P4_GRASP=off (pipeline 3's grasps, full opening), P4_CLOUD_RVIZ=0 (no 2nd RViz),
#   CGN_RECENTRE=true, P4_SOURCE / P4_TARGET (default prompts). Uses pipeline4/core (copies of pipeline 3) - pipeline 3 itself is not used.
P4_START_VERSION="p4-start-v2"   # v2: conda left like `conda deactivate` (CONDA_SHLVL=0), so GraspMolmo's `conda run` works
echo "start_pipeline4 $P4_START_VERSION"
# no conda Python in this script (ROS needs the system Python); the `conda` command stays available for GraspMolmo's worker
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'miniconda3/(bin|envs/[^/]+/bin)$|anaconda3/(bin|envs/[^/]+/bin)$' | paste -sd:)"
command -v conda >/dev/null 2>&1 || { [ -x "$HOME/miniconda3/condabin/conda" ] && export PATH="$PATH:$HOME/miniconda3/condabin"; }
# exactly what `conda deactivate` leaves behind: level 0, no active prefix (a level above 0 without a prefix makes `conda run` crash)
export CONDA_SHLVL=0
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER $(env | sed -n "s/^\(CONDA_PREFIX_[0-9]*\)=.*/\1/p")
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; ROOT="$(dirname "$D")"; C="$D/core"
export REAL_ARM="${REAL_ARM:-1}" ARM_WAIT="${ARM_WAIT:-30}" CGN_RECENTRE="${CGN_RECENTRE:-false}"
unset TABLE_TOP_Z JOINT_STATES_TOPIC HOME_MOVE_SECONDS
CALIB="$HOME/.ros2/easy_handeye2/calibrations/nero_wrist_calib.calib"
[ -f "$C/p3_graspmolmo_node.py" ] && [ -f "$D/p4_run.sh" ] || { echo "STOP: pipeline4/core is missing - reinstall pipeline 4"; exit 1; }
[ -f "$CALIB" ] || { echo "STOP: wrist calibration not found: $CALIB"; exit 1; }
if [ "$REAL_ARM" = 1 ]; then
  echo "REAL ARM. Before starting: the arm is at HOME, the workspace is clear, the STOP terminal is open (STOP typed, no Enter)."
  read -r -p "Type y to start: " a; [ "$a" = y ] || { echo "not started"; exit 1; }
fi
echo "=== clean slate (stops any earlier pipeline 1/2/3/4 processes) ==="
bash "$D/p4_clean.sh" || { echo "STOP: could not clean up"; exit 1; }
LOG_DIR="$HOME/nero_pipeline_logs/p4_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$LOG_DIR"; echo "Logs: $LOG_DIR"
source /opt/ros/humble/setup.bash; source "$HOME/agx_arm_ws/install/setup.bash"
PIDS=()
start() { local name="$1"; shift; echo "Starting $name..."; "$@" > "$LOG_DIR/$name.log" 2>&1 & PIDS+=($!); }
cleanup() { echo ""; echo "Stopping all pipeline 4 processes..."; for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
            wait 2>/dev/null || true; echo "Stopped. Logs kept at $LOG_DIR"; exit 0; }
trap cleanup SIGINT SIGTERM

# --- arm, MoveIt, RViz (shared scripts in ~/nero_pipeline_launch/scripts, incl. your 'initial' pose) ---
if [ "$REAL_ARM" = 1 ]; then ARM_CMD="$ROOT/scripts/launch_arm_real.sh"; else ARM_CMD="$ROOT/scripts/launch_arm.sh"; fi
start "arm" bash -c "$ARM_CMD"
sleep "$ARM_WAIT"
echo "Adding the table collision object (safety-critical)..."
/usr/bin/python3 "$ROOT/scripts/add_table_collision.py" 2>&1 | tee "$LOG_DIR/table_collision.log"

# --- top camera + its calibration ---
start "camera" ros2 launch orbbec_camera ob_camera.launch.py camera_name:=topcam \
    enable_depth:=true depth_width:=640 depth_height:=400 depth_fps:=30 depth_format:=Y12 \
    enable_color:=true color_width:=640 color_height:=480 color_fps:=30 color_format:=MJPG \
    enable_ir:=true ir_format:=Y10 enable_point_cloud:=true depth_registration:=true
sleep 5
start "handeye_publisher" ros2 launch easy_handeye2 publish.launch.py name:=nero_eye_to_hand_calib
sleep 2

# --- wrist camera + its calibration ---
WARGS=$(/usr/bin/python3 - "$CALIB" <<'PY'
import sys, yaml
c = yaml.safe_load(open(sys.argv[1])); t, r, p = c["transform"]["translation"], c["transform"]["rotation"], c["parameters"]
print("--x %r --y %r --z %r --qx %r --qy %r --qz %r --qw %r --frame-id %s --child-frame-id %s"
      % (t["x"], t["y"], t["z"], r["x"], r["y"], r["z"], r["w"], p["robot_effector_frame"], p["tracking_base_frame"]))
PY
)
start "armcam_calib" ros2 run tf2_ros static_transform_publisher $WARGS --ros-args -r __node:=armcam_calib_publisher
start "armcam" ros2 launch realsense2_camera rs_launch.py camera_name:=armcam enable_depth:=true enable_color:=true align_depth.enable:=true
( for i in $(seq 1 60); do
    for n in pointcloud__neon_.enable pointcloud.enable; do
      if ros2 param get /camera/armcam "$n" >/dev/null 2>&1; then ros2 param set /camera/armcam "$n" true; exit 0; fi
    done; sleep 1
  done; echo "wrist camera node never came up" ) > "$LOG_DIR/armcam_cloud_on.log" 2>&1 &

# --- perception ---
start "graspmolmo" /usr/bin/python3 "$C/p3_graspmolmo_node.py" --ros-args \
    -p source_prompt:="${P4_SOURCE:-eggplant}" -p target_prompt:="${P4_TARGET:-basket}" -p dtype:="${P3_GM_DTYPE:-bf16}"
sleep 3
start "contact_graspnet_ros" ros2 launch contact_graspnet_ros contactgraspnet.launch.py mask_topic:=/graspmolmo/mask_source \
    forward_passes:=${CGN_FORWARD_PASSES:-5} recentre_on_object:=${CGN_RECENTRE} recentre_depth_below_top_m:=${CGN_DEPTH:-0.006}
sleep 3
start "target_pose_ros" ros2 launch target_pose_ros targetpose.launch.py mask_topic:=/graspmolmo/mask_target
sleep 2
if [ "${P4_GRASP:-on}" != off ]; then GRASP_NAME="pipeline 4's grasp step (tilt <= ${P4_MAX_TILT_DEG:-20} deg, opened to the object's width)"; start "p4_grasp" /usr/bin/python3 "$D/p4_grasp.py"
else GRASP_NAME="pipeline 3's relay copy (tilted grasps, full opening)"; rm -f "$HOME/p3_runs/p4_grasp_latest.json"; start "p3_jaw_fix" /usr/bin/python3 "$C/p3_jaw_fix.py"; fi
start "pose_transform_ros" ros2 launch pose_transform_ros posetransform.launch.py grasps_topic:=/p3/grasps
sleep 2
start "debug_pose_tf_broadcaster" /usr/bin/python3 "$ROOT/scripts/debug_pose_tf_broadcaster.py"

# --- fusion (must run before every plan: it keeps GraspMolmo's masks for the wrist obstacles) ---
start "p4_fuse" /usr/bin/python3 "$D/p4_fuse_node.py"
[ "${P4_CLOUD_RVIZ:-1}" != 0 ] && start "rviz_clouds" rviz2 -d "$D/p4_clouds.rviz" --ros-args -r __node:=rviz_clouds

# --- checks ---
echo ""; echo "=== checks ==="
BAD=0
ok() { echo "  ok        $*"; }; bad() { echo "  PROBLEM   $*"; BAD=1; }
TZ=$(/usr/bin/python3 -c "import sys; sys.path.insert(0, '$C'); import p3_tablez as t; v, s = t.table_z(); print('%.4f|%+.1f|%s' % (v, 1000 * v, s))")
TZV=${TZ%%|*}; REST=${TZ#*|}; TZMM=${REST%%|*}
OUT=$(/usr/bin/python3 "$C/add_table_box.py" --top_z "$TZV" 2>&1)
echo "$OUT" | grep -q -- "$(printf 'z = %+.3f' "$TZV")" && ok "table box at the real table height ($TZMM mm)" || bad "table box: $(echo "$OUT" | tail -1)"
printf "  ...       waiting for GraspMolmo to load (about a minute)"
for i in $(seq 1 60); do
  grep -q "worker ready" "$LOG_DIR/graspmolmo.log" 2>/dev/null && break
  grep -q "worker start-up failed\|Traceback" "$LOG_DIR/graspmolmo.log" 2>/dev/null && break
  printf "."; sleep 5
done; echo ""
grep -q "worker ready" "$LOG_DIR/graspmolmo.log" 2>/dev/null && ok "GraspMolmo loaded" || bad "GraspMolmo not ready: see $LOG_DIR/graspmolmo.log"
W=$(timeout 15 ros2 topic echo --once --field width /camera/armcam/depth/color/points 2>/dev/null | head -1)
[ -n "$W" ] && [ "$W" -gt 0 ] 2>/dev/null && ok "wrist cloud publishing ($W points), wrist calibration published" || bad "no wrist cloud: see $LOG_DIR/armcam.log and armcam_cloud_on.log"
ros2 service list 2>/dev/null | grep -qx "/p4/wrist_obstacles" && ok "fusion node up (/p4/wrist_obstacles)" || bad "fusion node not answering: see $LOG_DIR/p4_fuse.log"
N=$(ros2 topic info /p3/grasps 2>/dev/null | sed -n 's/Publisher count: //p')
[ "${N:-0}" = 1 ] && ok "grasp step: $GRASP_NAME" || bad "grasp relay: ${N:-0} publisher(s) on /p3/grasps (must be 1)"
echo ""
if [ "$BAD" = 0 ]; then
  echo "READY ($([ "$REAL_ARM" = 1 ] && echo REAL || echo SIMULATED) arm). Next, in another terminal:"
  echo "  bash $D/p4_plan.sh \"eggplant\" \"basket\"        (look + both cameras' obstacles + plan; Ctrl+C after looking)"
  echo "  bash $D/p4_execute.sh                         (asks EXECUTE; hand on STOP)"
else
  echo "NOT READY: fix the PROBLEM line(s) above (logs: $LOG_DIR), then Ctrl+C here and start again."
fi
echo "Ctrl+C here stops everything."
wait
