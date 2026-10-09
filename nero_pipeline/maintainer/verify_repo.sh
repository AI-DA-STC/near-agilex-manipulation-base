#!/bin/bash
# verify_repo.sh - is this folder complete and does it contain our final edits?   Run from anywhere:
#     maintainer/verify_repo.sh
# Prints OK / WARN / FAIL per check, then the key tuning constants as they are in the collected script.
# Exit code 1 if any FAIL. WARN means "works, but check it" (for example an optional edit that is not present).

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAILS=0; WARNS=0
ok()   { echo "OK    $*"; }
warn() { echo "WARN  $*"; WARNS=$((WARNS+1)); }
fail() { echo "FAIL  $*"; FAILS=$((FAILS+1)); }
need_file() { [ -f "$HERE/$1" ] && ok "file $1" || fail "missing file $1  (run maintainer/collect_from_workspace.sh on the lab machine)"; }
marker() {   # marker FILE PATTERN WHY [warn]
  local f="$HERE/$1"; [ -f "$f" ] || return
  if grep -q -- "$2" "$f"; then ok "$1 contains: $2"
  elif [ "${4:-}" = "warn" ]; then warn "$1 does not contain '$2'  ($3)"
  else fail "$1 does not contain '$2'  ($3)"; fi
}

echo "== structure =="
for f in README.md .gitignore docs/01_system_overview.md docs/02_install_and_environment.md docs/03_calibration.md docs/04_run_simulation.md \
         docs/05_run_real_arm.md docs/06_tuning_reference.md docs/07_development_log.md docs/08_troubleshooting.md \
         docs/09_known_issues_and_next_steps.md docs/10_grasp_quality_investigation.md launch/README.md launch/tools/README.md \
         launch/start_pipeline.sh launch/run_pick_and_place.sh launch/stop_arm.sh launch/scripts/launch_arm.sh launch/scripts/launch_arm_real.sh \
         launch/scripts/add_table_collision.py launch/scripts/move_to_pose_90deg.py launch/tools/grasp_watch.py launch/tools/grasp_view.py \
         launch/tools/calib_adjust.awk launch/tools/adjust_calib_yaw.py launch/tools/reach_check.py launch/tools/nero_fk.py \
         reference_copies/nero_mtc_task/nero_mtc_task/mtc_pick_and_place.py config_snapshots/agx_arm_moveit/agx_arm.srdf.xacro \
         config_snapshots/agx_arm_moveit/initial_positions.yaml config_snapshots/calibration/nero_eye_to_hand_calib.calib \
         docs/PACKAGE_VERSIONS.md; do need_file "$f"; done

echo; echo "== syntax =="
while IFS= read -r f; do bash -n "$f" 2>/dev/null && ok "bash -n ${f#$HERE/}" || fail "bash syntax error in ${f#$HERE/}"; done < <(find "$HERE" -name '*.sh' -not -path '*/.git/*')
while IFS= read -r f; do python3 -m py_compile "$f" 2>/dev/null && ok "py_compile ${f#$HERE/}" || fail "python syntax error in ${f#$HERE/}"; done < <(find "$HERE" -name '*.py' -not -path '*/.git/*')
find "$HERE" -name __pycache__ -type d -prune -exec rm -rf {} + 2>/dev/null

echo; echo "== our edits are present in the collected files =="
marker launch/start_pipeline.sh "REAL_ARM" "real/sim switch"
marker launch/start_pipeline.sh "ARM_WAIT" "waits for MoveIt before the table step"
marker launch/start_pipeline.sh "source_prompt" "object prompt"
marker launch/run_pick_and_place.sh "tcp_offset" "corrected gripper frame passed to MTC's robot model"
marker launch/run_pick_and_place.sh "call_checked" "stops on a failed perception step" warn
marker launch/scripts/move_to_pose_90deg.py "JOINT_STATES_TOPIC" "reads the real joint topic"
marker launch/scripts/move_to_pose_90deg.py "HOME_MOVE_SECONDS" "slow start-pose move"
marker launch/scripts/add_table_collision.py "TABLE_TOP_Z" "table height from the shell"
marker launch/scripts/launch_arm_real.sh "control_enabled:=true" "driver control path open"
marker launch/scripts/launch_arm_real.sh "capabilities" "ExecuteTaskSolution capability"
marker launch/scripts/launch_arm_real.sh "tcp_offset" "corrected gripper frame"
marker launch/scripts/launch_arm_real.sh "follow:=true" "plan from real feedback"
marker launch/scripts/launch_arm.sh "capabilities" "ExecuteTaskSolution capability"
marker launch/scripts/launch_arm.sh "tcp_offset" "corrected gripper frame"
M=reference_copies/nero_mtc_task/nero_mtc_task/mtc_pick_and_place.py
marker $M "def retime_solution" "per-segment timing (controller rejects untimed trajectories)"
marker $M "ARM_MAX_VEL" "the speed constant"
marker $M "def to_tcp_pose" "Contact-GraspNet to tcp_link conversion"
marker $M "CartesianPath" "straight-line approach"
marker $M "GenerateRandomPose" "place orientation cone"
marker $M "def tilt_from_down_deg" "tilt filter"
marker $M "GRASP_OFFSET_X" "world-frame grasp nudge"
if grep -q "MTC_EXECUTE" "$HERE/launch/run_pick_and_place.sh" "$HERE/$M" 2>/dev/null; then ok "MTC_EXECUTE switch found (plan-only vs execute)"; else warn "MTC_EXECUTE not found in run_pick_and_place.sh or the MTC script"; fi
marker $M "usable.sort(" "candidates are sorted most top-down first (the fix that made the grasps work; docs/07 stage 12)"
marker config_snapshots/agx_arm_moveit/agx_arm.srdf.xacro 'group_state name="home"' "home state"
marker config_snapshots/agx_arm_moveit/agx_arm.srdf.xacro "pose_90deg" "start pose state"
marker config_snapshots/agx_arm_moveit/initial_positions.yaml "joint1" "simulation start pose"
marker config_snapshots/calibration/nero_eye_to_hand_calib.calib "topcam_link" "calibration expressed relative to topcam_link"
marker config_snapshots/calibration/nero_eye_to_hand_calib.calib "eye_on_base" "calibration type"

echo; echo "== key constants as collected (compare with docs/06_tuning_reference.md) =="
if [ -f "$HERE/$M" ]; then
  grep -nE "^(MAX_TILT_DEG|MAX_CANDIDATES_IN_TASK|CGN_GRIPPER_DEPTH|OUR_FINGER_REACH|GRASP_OFFSET_X|GRASP_OFFSET_Y|PREGRASP_OFFSET_M|PLACE_Z_CLEARANCE_M|APPROACH_MIN|LIFT_MIN|RETREAT_MIN|ARM_MAX_VEL|ARM_MAX_ACC|GRIPPER_MAX_VEL|GRIPPER_MAX_ACC)\b" "$HERE/$M" | sed 's/^/    /'
fi
CAL="$HERE/config_snapshots/calibration/nero_eye_to_hand_calib.calib"
if [ -f "$CAL" ]; then
  awk '/rotation:/{r=1} r&&$1=="x:"{x=$2} r&&$1=="y:"{y=$2} r&&$1=="z:"{z=$2} r&&$1=="w:"{w=$2} END{n=sqrt(x*x+y*y+z*z+w*w); printf "    calibration quaternion length %.6f %s\n", n, (n>0.999&&n<1.001)?"(ok)":"(NOT unit length: normalise it, docs/08)"}' "$CAL"
fi

echo; echo "== privacy (the repository is public) =="
TERMS="${PRIVACY_TERMS:-$(id -un)}"          # run on the lab machine: your user name. Override: PRIVACY_TERMS="name1 name2"
PRIV=0
for t in $TERMS; do
  while IFS= read -r hit; do [ -n "$hit" ] && { fail "'$t' appears in: ${hit}   (replace /home/$t with \$HOME or ~ in that file)"; PRIV=1; }; done < <(grep -rIl --exclude-dir=.git --exclude=verify_repo.sh -e "$t" "$HERE" 2>/dev/null | sed "s#^$HERE/##" | head -8)
done
[ "$PRIV" -eq 0 ] && ok "no trace of: $TERMS"

echo; echo "== things that must NOT be committed =="
if find "$HERE" \( -name '*.npy' -o -name '*.log' -o -name '*.bak*' -o -name '*.new' -o -name '*.orig' -o -name '__pycache__' \) -not -path '*/.git/*' | grep -q .; then
  warn "scratch/runtime files exist (they are covered by .gitignore, but check 'git status'):"; find "$HERE" \( -name '*.npy' -o -name '*.log' -o -name '*.bak*' -o -name '*.new' -o -name '*.orig' \) -not -path '*/.git/*' | sed 's/^/    /'
else ok "no scratch or runtime files"; fi

echo; echo "== result: $FAILS FAIL, $WARNS WARN =="
[ "$FAILS" -eq 0 ]
