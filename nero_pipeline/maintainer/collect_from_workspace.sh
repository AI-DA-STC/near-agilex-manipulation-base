#!/bin/bash
# collect_from_workspace.sh  -  run ON THE LAB MACHINE, as the person who maintains this folder.
#
# Copies the FINAL, hand-edited versions of our files into this folder and records the environment, so the
# repository is self-contained and a new person can reproduce the setup. Safe to re-run (it reports what changed).
# It only READS from your workspace; it writes only inside this folder (nero_pipeline/).
#
#   maintainer/collect_from_workspace.sh
#
# Overridable locations:  WS (ROS workspace)  PL (launch folder)  CGP (contact_graspnet_pytorch)  DL (Downloads)
# NOTE: no 'set -e/-u' on purpose: sourcing ROS setup scripts fails under them, and we want to continue past missing files.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"           # .../nero_pipeline
WS="${WS:-$HOME/agx_arm_ws}"; PL="${PL:-$HOME/nero_pipeline_launch}"
CGP="${CGP:-$HOME/contact_graspnet_pytorch}"; DL="${DL:-$HOME/Downloads}"
NEW=0; UPD=0; SAME=0; MISS=0; DIFF=0

collect() {   # collect SRC DEST_RELATIVE_TO_HERE   - the lab copy is the truth: overwrite and report
  local src="$1" rel="$2" dest="$HERE/$2"
  if [ ! -f "$src" ]; then echo "MISSING   $src"; MISS=$((MISS+1)); return; fi
  mkdir -p "$(dirname "$dest")"
  if [ ! -f "$dest" ]; then cp -p "$src" "$dest"; echo "NEW       $rel"; NEW=$((NEW+1))
  elif cmp -s "$src" "$dest"; then echo "same      $rel"; SAME=$((SAME+1))
  else cp -p "$src" "$dest"; echo "UPDATED   $rel   (lab copy differs from what was in the folder; review with: git diff)"; UPD=$((UPD+1)); fi
}
compare() {   # compare SRC REL  - the repo holds a TESTED reference copy; never overwrite it, only report differences
  local src="$1" rel="$2" dest="$HERE/$2"
  if [ ! -f "$src" ]; then echo "absent    $src  (lab machine has no copy; reference version in the folder is used)"; return; fi
  if cmp -s "$src" "$dest"; then echo "same      $rel"; SAME=$((SAME+1))
  elif diff -q <(grep -v '^[[:space:]]*#' "$src" | sed '/^[[:space:]]*$/d') <(grep -v '^[[:space:]]*#' "$dest" | sed '/^[[:space:]]*$/d') >/dev/null; then
    echo "same      $rel   (only comments differ; the folder's version has the explanatory header)"; SAME=$((SAME+1))
  else echo "DIFFERS   $rel   (lab copy: $src - NOT copied; run: diff $src $dest)"; DIFF=$((DIFF+1)); fi
}
first_existing() { for f in "$@"; do [ -f "$f" ] && { echo "$f"; return; }; done; echo "$1"; }

echo "== 1. Files with our final hand edits (copied from the lab machine) =="
collect "$PL/start_pipeline.sh"                 launch/start_pipeline.sh
collect "$PL/run_pick_and_place.sh"             launch/run_pick_and_place.sh
collect "$PL/ckpt.sh"                           launch/ckpt.sh
collect "$PL/scripts/add_table_collision.py"    launch/scripts/add_table_collision.py
collect "$PL/scripts/move_to_pose_90deg.py"     launch/scripts/move_to_pose_90deg.py
collect "$PL/scripts/approach_monitor.py"       launch/scripts/approach_monitor.py
collect "$PL/scripts/check_tcp_in_mtc.py"       launch/scripts/check_tcp_in_mtc.py

echo; echo "== 2. Source files that live in other repositories (read-only reference copies) =="
collect "$WS/src/nero_mtc_task/nero_mtc_task/mtc_pick_and_place.py"      reference_copies/nero_mtc_task/nero_mtc_task/mtc_pick_and_place.py
collect "$WS/src/nero_mtc_task/launch/mtc_pick_and_place.launch.py"      reference_copies/nero_mtc_task/launch/mtc_pick_and_place.launch.py
collect "$WS/src/contact_graspnet_ros/contact_graspnet_ros/contact_graspnet_node.py"  reference_copies/contact_graspnet_ros/contact_graspnet_ros/contact_graspnet_node.py
collect "$WS/src/contact_graspnet_ros/scripts/contact_graspnet_inference.py"           reference_copies/contact_graspnet_ros/scripts/contact_graspnet_inference.py
collect "$WS/src/gsam2_ros/gsam2_ros/gsam2_segmentation_node.py"          reference_copies/gsam2_ros/gsam2_ros/gsam2_segmentation_node.py
collect "$WS/src/gsam2_ros/launch/groundedsam2.launch.py"                 reference_copies/gsam2_ros/launch/groundedsam2.launch.py
collect "$CGP/visualize_grasps_from_saved.py"                             reference_copies/contact_graspnet_pytorch/visualize_grasps_from_saved.py

echo; echo "== 3. Configuration changed in third-party packages + the calibration =="
CFG="$WS/src/agx_arm_ros/src/agx_arm_moveit/config"
collect "$CFG/agx_arm.srdf.xacro"        config_snapshots/agx_arm_moveit/agx_arm.srdf.xacro
collect "$CFG/initial_positions.yaml"    config_snapshots/agx_arm_moveit/initial_positions.yaml
collect "$CFG/joint_limits.yaml"         config_snapshots/agx_arm_moveit/joint_limits.yaml
collect "$HOME/.ros2/easy_handeye2/calibrations/nero_eye_to_hand_calib.calib"  config_snapshots/calibration/nero_eye_to_hand_calib.calib

echo; echo "== 4. Tested reference copies already in this folder: report only, never overwrite =="
for f in stop_arm.sh scripts/launch_arm.sh scripts/launch_arm_real.sh; do compare "$PL/$f" "launch/$f"; done
for f in grasp_watch.py grasp_view.py calib_adjust.awk; do compare "$PL/$f" "launch/tools/$f"; done
compare "$CGP/upsample.py" launch/tools/upsample.py
compare "$(first_existing "$PL/tools/adjust_calib_yaw.py" "$PL/adjust_calib_yaw.py" "$DL/adjust_calib_yaw.py")" launch/tools/adjust_calib_yaw.py

echo; echo "== 5. Package versions (docs/PACKAGE_VERSIONS.md) and local modifications (patches/) =="
PV="$HERE/docs/PACKAGE_VERSIONS.md"; mkdir -p "$HERE/patches" "$HERE/docs"
{
  echo "# Package versions (generated)"; echo
  echo "Generated by \`maintainer/collect_from_workspace.sh\` on $(date -u '+%Y-%m-%d %H:%M UTC')."
  echo "Clone each repository, check out the commit, and (where \"Local changes\" is not 0) apply the patch listed in \`patches/\`."; echo
  echo "| Directory | Remote | Branch | Commit | Local changes | Unpushed commits |"; echo "|---|---|---|---|---|---|"
} > "$PV"
record() {
  local d="${1%/}" name; name="$(basename "$d")"
  if [ -e "$d/.git" ]; then
    local remote branch commit dirty ahead
    ahead="$(git -C "$d" rev-list --count '@{u}..HEAD' 2>/dev/null || echo '?')"
    remote="$(git -C "$d" remote get-url origin 2>/dev/null || echo '(no remote)')"; branch="$(git -C "$d" rev-parse --abbrev-ref HEAD 2>/dev/null)"
    commit="$(git -C "$d" rev-parse --short=12 HEAD 2>/dev/null)"; dirty="$(git -C "$d" status --porcelain 2>/dev/null | wc -l | tr -d ' ')"
    echo "| \`$name\` | $remote | $branch | \`$commit\` | $dirty | $ahead |" >> "$PV"
    [ "$ahead" != "0" ] && [ "$ahead" != "?" ] && echo "UNPUSHED commits in $name: $ahead  <-- git -C $d push"
    if [ "$dirty" != "0" ]; then
      git -C "$d" diff --binary > "$HERE/patches/${name}_local_changes.patch" 2>/dev/null
      git -C "$d" ls-files --others --exclude-standard > "$HERE/patches/${name}_untracked_files.txt" 2>/dev/null
      [ -s "$HERE/patches/${name}_local_changes.patch" ] || rm -f "$HERE/patches/${name}_local_changes.patch"
      [ -s "$HERE/patches/${name}_untracked_files.txt" ] || rm -f "$HERE/patches/${name}_untracked_files.txt"
      echo "uncommitted changes in $name ($dirty entries): patch written to patches/  <-- COMMIT AND PUSH if this is one of OUR packages"
    fi
  else
    echo "| \`$name\` | not a git checkout | - | - | - | - |" >> "$PV"
  fi
}
for d in "$WS"/src/*/; do [ -d "$d" ] && record "$d"; done
for d in "$HOME/near-agilex-manipulation-base" "$CGP"; do [ -d "$d" ] && record "$d"; done
echo "wrote docs/PACKAGE_VERSIONS.md"

echo; echo "== 6. Environment snapshot (environments/) =="
ENVD="$HERE/environments"; mkdir -p "$ENVD"
{ echo "# generated $(date -u '+%Y-%m-%d %H:%M UTC')"; uname -srmo; lsb_release -a 2>/dev/null; echo; dpkg -l 2>/dev/null | grep -i "nvidia-jetpack\|nvidia-l4t-core"
  echo; (nvcc --version || /usr/local/cuda/bin/nvcc --version) 2>/dev/null | tail -2
  echo; echo "--- gs_usb module"; lsmod | grep gs_usb; modinfo gs_usb 2>/dev/null | head -8; stat -c "%n: %s bytes, modified %y" "$HOME"/gs_usb_build/gs_usb.ko 2>/dev/null
  echo; echo "--- ROS"; echo "ROS_DISTRO=${ROS_DISTRO:-}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-}"; } > "$ENVD/system.txt" 2>&1
grep -n "ROS_\|alias STOP\|CUDA\|LD_LIBRARY\|conda initialize" "$HOME/.bashrc" > "$ENVD/bashrc_relevant_lines.txt" 2>/dev/null
dpkg -l 2>/dev/null | awk '/^ii/ && $2 ~ /^ros-humble-/ {print $2, $3}' > "$ENVD/apt_ros_humble.txt"
/usr/bin/python3 -m pip freeze > "$ENVD/system_python_pip.txt" 2>/dev/null
( source /opt/ros/humble/setup.bash 2>/dev/null; [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash" 2>/dev/null
  ros2 pkg list > "$ENVD/ros2_pkg_list.txt" 2>/dev/null
  echo "moveit_task_constructor_core: $(ros2 pkg prefix moveit_task_constructor_core 2>/dev/null)" >> "$ENVD/system.txt" )
CONDA="$(command -v conda || echo "$HOME/miniconda3/bin/conda")"
for e in grounded_sam2 contact_graspnet; do
  if [ -x "$CONDA" ] && "$CONDA" env list 2>/dev/null | grep -q "^$e "; then
    "$CONDA" env export -n "$e" --no-builds > "$ENVD/$e.yml" 2>/dev/null
    "$CONDA" run -n "$e" python -m pip freeze > "$ENVD/${e}_pip.txt" 2>/dev/null
    echo "wrote environments/$e.yml and ${e}_pip.txt"
  else echo "conda env '$e' not found (skipped)"; fi
done
find "$ENVD" -type f -size 0 -delete 2>/dev/null     # do not commit empty files
# The repository is public: drop the conda prefix line and replace the home directory path in the GENERATED text files.
# (Collected source files and patches are not edited here; verify_repo.sh reports any remaining trace of your user name.)
sed -i '/^prefix:/d' "$ENVD"/*.yml 2>/dev/null
for f in "$ENVD"/* "$HERE/docs/PACKAGE_VERSIONS.md"; do [ -f "$f" ] && sed -i -e "s#$HOME#~#g" -e "s#/home/$(id -un)#~#g" -e "s#\b$(id -un)\b#user#g" "$f"; done
echo "removed the home directory path from the generated files"
ls "$ENVD" | sed 's/^/  environments\//'

echo; echo "== Summary =="
echo "new: $NEW   updated: $UPD   unchanged: $SAME   missing on this machine: $MISS   reference copies that differ: $DIFF"
echo "Next: run maintainer/verify_repo.sh, read 'git status' and 'git diff', then follow maintainer/PUSH_CHECKLIST.md"
