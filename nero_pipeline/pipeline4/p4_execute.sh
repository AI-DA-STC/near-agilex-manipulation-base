#!/bin/bash
# PIPELINE 4 - COMMAND 3 of 3: execute what p4_plan.sh found (same grasps, place spot and obstacles, no new picture). Asks EXECUTE first,
# then MoveIt plans ONCE MORE with the same scene and runs that plan right away - hand on STOP, watch the arm.
P4_EXECUTE_VERSION="p4-execute-v2"
echo "p4_execute $P4_EXECUTE_VERSION"
# no conda Python in this script (ROS needs the system Python); the `conda` command stays available for GraspMolmo's worker
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'miniconda3/(bin|envs/[^/]+/bin)$|anaconda3/(bin|envs/[^/]+/bin)$' | paste -sd:)"
command -v conda >/dev/null 2>&1 || { [ -x "$HOME/miniconda3/condabin/conda" ] && export PATH="$PATH:$HOME/miniconda3/condabin"; }
# exactly what `conda deactivate` leaves behind: level 0, no active prefix (a level above 0 without a prefix makes `conda run` crash)
export CONDA_SHLVL=0
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER $(env | sed -n "s/^\(CONDA_PREFIX_[0-9]*\)=.*/\1/p")
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash 2>/dev/null; source "$HOME/agx_arm_ws/install/setup.bash" 2>/dev/null
n=$(/usr/bin/python3 "$D/core/p3_scene.py" show 2>/dev/null | grep -c "p4_wrist_")
echo "MoveIt holds $n wrist box(es) (p4_wrist_*) besides the top camera's boxes and the table."
/usr/bin/python3 - <<'PY'
import json, os, time
p = os.path.expanduser("~/p3_runs/p4_grasp_latest.json")
try:
    d = json.load(open(p)); age = time.time() - d["unix_time"]
    print("gripper opening: %.0f mm (grasp step of %s, %.0f min ago)%s" % (1000 * d["open_width_m"], d["time"], age / 60,
          "  -> TOO OLD: fully open" if age > 1800 else ""))
    for g in d["grasps"]:
        print("  grasp #%d: tilt %.0f -> %.0f deg, opening %.0f mm, lowest fingertip %.0f mm above the table" % (
            g["rank"], g["tilt_from"], g["tilt_to"], 1000 * g["open_m"], g["lowest_tip_mm"]))
except Exception:
    print("no pipeline 4 grasp step result: gripper FULLY open, pipeline 3's grasps")
PY
echo "Hand on STOP. After you type EXECUTE, MoveIt plans once more and runs that plan right away - watch the arm."
exec /usr/bin/python3 "$D/core/p3_go.py" --last --run_script "$D/p4_run.sh"
