#!/bin/bash
# PIPELINE 5 - COMMAND 1 of 3: brings up EVERYTHING pipeline 5 needs, in one terminal (Ctrl+C here stops it all):
#   arm + MoveIt + RViz, table, top camera + its calibration, wrist camera (D405, cloud on) + its calibration, GraspMolmo, target_pose,
#   pipeline 5's grasp node (/p5/grasps - NO Contact-GraspNet), pose_transform, the fusion node (/p5/...), a second RViz with the fused cloud and
#   the grasp boxes; then checks everything and says READY.
#   Then:  bash ~/nero_pipeline_launch/pipeline5/p5_plan.sh "eggplant" "basket"      and      bash ~/nero_pipeline_launch/pipeline5/p5_execute.sh
# Options (set before the command): REAL_ARM=0 (simulated arm), P5_CLOUD_RVIZ=0 (no 2nd RViz), P5_SOURCE / P5_TARGET (default prompts),
#   grasp switches: P5_MAX_TILT_DEG P5_OPEN_MARGIN_M P5_GRASPS_TO_MOVEIT P5_OFFSET_M P5_CLOUD_CHECK P5_TIP_CLEARANCE P5_MASK P5_SAM2_MODEL (README.txt).
# Uses only pipeline5/ (its own copies). Shared with every pipeline: ~/nero_pipeline_launch/scripts (arm launch, start pose, table),
# the calibrations, ~/.p3_table_z, the ROS workspace.
P5_START_VERSION="p5-start-v8a"   # 8a: no live window; one picture pops up per look |   # v6f (7f): Ctrl+C stops for sure (10 s, then forced)
# v6: the live window "Pipeline 5"; its picture folder (/tmp/p5_view) is deleted on Ctrl+C
# v5: v5: target_pose reads /p5/mask_target (the place point after /p5/place)
# v4: v4: CAN = link check before launch + the arm must publish /feedback/joint_states after it
# v3: CAN check/bring-up first (real arm)
# v2: SAM2 object masks (P5_MASK=depth: GraspMolmo's region only)
echo "start_pipeline5 $P5_START_VERSION"
# no conda Python in this script (ROS needs the system Python); the `conda` command stays available for GraspMolmo's worker
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'miniconda3/(bin|envs/[^/]+/bin)$|anaconda3/(bin|envs/[^/]+/bin)$' | paste -sd:)"
command -v conda >/dev/null 2>&1 || { [ -x "$HOME/miniconda3/condabin/conda" ] && export PATH="$PATH:$HOME/miniconda3/condabin"; }
export CONDA_SHLVL=0
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER $(env | sed -n "s/^\(CONDA_PREFIX_[0-9]*\)=.*/\1/p")
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; ROOT="$(dirname "$D")"; C="$D/core"
export REAL_ARM="${REAL_ARM:-1}" ARM_WAIT="${ARM_WAIT:-30}"
unset TABLE_TOP_Z JOINT_STATES_TOPIC HOME_MOVE_SECONDS
CALIB="$HOME/.ros2/easy_handeye2/calibrations/nero_wrist_calib.calib"
[ -f "$C/p3_graspmolmo_node.py" ] && [ -f "$D/p5_run.sh" ] && [ -f "$D/p5_grasp_node.py" ] || { echo "STOP: pipeline5 is incomplete - reinstall pipeline 5"; exit 1; }
[ -f "$CALIB" ] || { echo "STOP: wrist calibration not found: $CALIB"; exit 1; }
if [ "$REAL_ARM" = 1 ]; then
  echo "REAL ARM. Before starting: the arm is at HOME, the workspace is clear, the STOP terminal is open (STOP typed, no Enter)."
  read -r -p "Type y to start: " a; [ "$a" = y ] || { echo "not started"; exit 1; }
  # --- CAN link (P5_CAN = interface name, default can_piper). The Nero stays SILENT until its driver starts, so this checks the LINK only
  #     (up, 1 Mbit/s, no bus errors); that the arm really talks is checked after the arm launch (/feedback/joint_states). ---
  CAN="${P5_CAN:-can_piper}"; CS="$HOME/agx_arm_ws/src/agx_arm_ros/scripts"
  can_link() { ip -details link show "$CAN" 2>/dev/null | grep -q "state UP" && ip -details link show "$CAN" | grep -q "bitrate 1000000" \
               && ! ip -details link show "$CAN" | grep -q "BUS-OFF"; }
  if can_link; then
    echo "  ok        CAN $CAN up at 1 Mbit/s ($(ip -details link show "$CAN" | grep -oE "ERROR-[A-Z]+|BUS-[A-Z]+" | head -1))"
  else
    echo "  ...       CAN $CAN not up - looking for the adapter"
    PORTS=$(cd "$CS" 2>/dev/null && bash find_all_can_port.sh 2>/dev/null | grep -oE "[0-9]+-[0-9.]+:[0-9]+\.[0-9]+" | sort -u)
    N=$(echo "$PORTS" | grep -c .)
    if [ "$N" = 1 ]; then
      echo "  ...       one CAN adapter (USB port $PORTS): can_activate.sh $CAN 1000000 $PORTS  (may ask for your sudo password)"
      (cd "$CS" && bash can_activate.sh "$CAN" 1000000 "$PORTS"); sleep 1
    fi
    if can_link; then
      echo "  ok        CAN $CAN up at 1 Mbit/s (brought up just now)"
    else
      echo "STOP: CAN $CAN is not up (found $N adapter(s)). Nothing was started. Bring it up by hand:"
      echo "    cd $CS && bash find_all_can_port.sh"
      echo "    bash can_activate.sh $CAN 1000000 \"<usb_port from the line above>\""
      echo "    ip -details link show $CAN        # want: state UP, bitrate 1000000, not BUS-OFF"
      echo "  Replugging the adapter changes its USB port. Then start again."
      exit 1
    fi
  fi
fi
echo "=== clean slate (stops any earlier pipeline 1-5 processes) ==="
bash "$D/p5_clean.sh" || { echo "STOP: could not clean up"; exit 1; }
LOG_DIR="$HOME/nero_pipeline_logs/p5_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$LOG_DIR"; echo "Logs: $LOG_DIR"
rm -f "$HOME/p3_runs/p5_grasp_latest.json"      # no opening from an earlier session
source /opt/ros/humble/setup.bash; source "$HOME/agx_arm_ws/install/setup.bash"
PIDS=()
start() { local name="$1"; shift; echo "Starting $name..."; "$@" > "$LOG_DIR/$name.log" 2>&1 & PIDS+=($!); }
cleanup() {   # 7f: polite stop, up to 10 s, then force; extra Ctrl+C ignored (7e waited forever on one stubborn process)
  trap '' SIGINT SIGTERM
  echo ""; echo "Stopping all pipeline 5 processes (up to 10 s, then forced)..."
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  for i in $(seq 1 10); do
    alive=0; for pid in "${PIDS[@]}"; do kill -0 "$pid" 2>/dev/null && alive=1; done
    [ "$alive" = 0 ] && break; sleep 1
  done
  for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null || true; done
  bash "$D/p5_clean.sh" > /dev/null 2>&1 || true              # anything they started (workers, camera nodes, the arm driver)
  rm -rf /tmp/p5_view; /usr/bin/python3 "$D/p5_tidy.py"
  echo "Stopped. Live-view pictures deleted; logs kept at $LOG_DIR"
  echo "The arm stays where it is: put it at 'home' before the next start."
  exit 0
}
export P5_VIEW_DIR=/tmp/p5_view
rm -rf /tmp/p5_view && mkdir -p /tmp/p5_view
trap cleanup SIGINT SIGTERM

# --- arm, MoveIt, RViz (shared scripts in ~/nero_pipeline_launch/scripts, incl. your 'initial' pose) ---
if [ "$REAL_ARM" = 1 ]; then ARM_CMD="$ROOT/scripts/launch_arm_real.sh"; else ARM_CMD="$ROOT/scripts/launch_arm.sh"; fi
start "arm" bash -c "$ARM_CMD"
sleep "$ARM_WAIT"
if [ "$REAL_ARM" = 1 ]; then          # the real proof the arm talks: its driver publishes its joint states
  JS=$(timeout 15 ros2 topic echo --once --field position /feedback/joint_states 2>/dev/null | head -3)
  if [ -n "$JS" ]; then echo "  ok        the arm talks (/feedback/joint_states)"
  else
    echo "STOP: the arm driver started but the arm sends nothing (/feedback/joint_states silent for 15 s)."
    echo "  Arm power and CAN cable at the arm; see $LOG_DIR/arm.log. Stopping everything."
    cleanup
  fi
fi
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

# --- perception: GraspMolmo, target, pipeline 5's grasp node (instead of Contact-GraspNet), pose_transform ---
start "graspmolmo" /usr/bin/python3 "$C/p3_graspmolmo_node.py" --ros-args \
    -p source_prompt:="${P5_SOURCE:-eggplant}" -p target_prompt:="${P5_TARGET:-basket}" -p dtype:="${P3_GM_DTYPE:-bf16}"
sleep 3
start "target_pose_ros" ros2 launch target_pose_ros targetpose.launch.py mask_topic:=/p5/mask_target
sleep 2
start "p5_grasp" /usr/bin/python3 "$D/p5_grasp_node.py"
start "pose_transform_ros" ros2 launch pose_transform_ros posetransform.launch.py grasps_topic:=/p5/grasp_poses
sleep 2
start "debug_pose_tf_broadcaster" /usr/bin/python3 "$ROOT/scripts/debug_pose_tf_broadcaster.py"

# --- fusion (both cameras; keeps GraspMolmo's masks for the wrist obstacles; feeds the grasp node's obstacle check) ---
start "p5_fuse" /usr/bin/python3 "$D/p5_fuse_node.py"
[ "${P5_CLOUD_RVIZ:-1}" != 0 ] && start "rviz_clouds" rviz2 -d "$D/p5_clouds.rviz" --ros-args -r __node:=rviz_clouds

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
if [ "${P5_MASK:-sam2}" = sam2 ]; then
  for i in $(seq 1 40); do grep -q "sam2 worker ready\|sam2 worker start-up failed" "$LOG_DIR/p5_grasp.log" 2>/dev/null && break; sleep 3; done
  grep -q "sam2 worker ready" "$LOG_DIR/p5_grasp.log" && ok "SAM2 loaded (object masks from GraspMolmo's point)" \
    || bad "SAM2 not ready: see $LOG_DIR/p5_grasp.log (or start with P5_MASK=depth: GraspMolmo's region, merged ones refused)"
else
  ok "object masks: GraspMolmo's depth region (P5_MASK=${P5_MASK}); merged regions are refused"
fi
W=$(timeout 15 ros2 topic echo --once --field width /camera/armcam/depth/color/points 2>/dev/null | head -1)
[ -n "$W" ] && [ "$W" -gt 0 ] 2>/dev/null && ok "wrist cloud publishing ($W points), wrist calibration published" || bad "no wrist cloud: see $LOG_DIR/armcam.log and armcam_cloud_on.log"
ros2 service list 2>/dev/null | grep -qx "/p5/wrist_obstacles" && ok "fusion node up (/p5/wrist_obstacles)" || bad "fusion node not answering: see $LOG_DIR/p5_fuse.log"
ros2 service list 2>/dev/null | grep -qx "/p5/place" || bad "no /p5/place service: see $LOG_DIR/p5_grasp.log"
ros2 service list 2>/dev/null | grep -qx "/p5/grasps" && ok "grasp node up (/p5/grasps): $(grep -m1 -o 'p5-grasp-node-v[0-9]*.*' "$LOG_DIR/p5_grasp.log" | cut -c1-110)" \
  || bad "grasp node not answering: see $LOG_DIR/p5_grasp.log"
ok "pictures: one pops up per look - after the place step (grasps + place point), or at the grasp step if no grasp (P5_POPUP=off: none)"
N=$(ros2 topic info /p5/grasp_poses 2>/dev/null | sed -n 's/Publisher count: //p')
[ "${N:-0}" = 1 ] && ok "one publisher of grasps for pose_transform (/p5/grasp_poses)" || bad "${N:-0} publisher(s) on /p5/grasp_poses (must be 1)"
ros2 node list 2>/dev/null | grep -q "contact_graspnet" && bad "a Contact-GraspNet node is running (not part of pipeline 5)"
echo ""
if [ "$BAD" = 0 ]; then
  echo "READY ($([ "$REAL_ARM" = 1 ] && echo REAL || echo SIMULATED) arm). Next, in another terminal:"
  echo "  bash $D/p5_go.sh \"eggplant\" \"the middle of the basket\"     (look + plan + EXECUTE right away; hand on STOP)"
  echo "  or: bash $D/p5_plan.sh \"eggplant\" \"the middle of the basket\"  (look + plan, saved)  then  bash $D/p5_execute.sh  (runs it, no re-plan)"
  echo "  pictures: one pops up per look, once the grasps and the place point are ready"
else
  echo "NOT READY: fix the PROBLEM line(s) above (logs: $LOG_DIR), then Ctrl+C here and start again."
fi
echo "Ctrl+C here stops everything."
wait
