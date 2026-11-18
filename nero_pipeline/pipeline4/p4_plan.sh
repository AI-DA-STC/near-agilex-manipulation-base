#!/bin/bash
# PIPELINE 4 - COMMAND 2 of 3: look and plan. The arm goes to its start pose, then: GraspMolmo -> Contact-GraspNet -> pipeline 4's grasp step ->
# place point + top-camera obstacles -> WRIST-camera 3D obstacles -> ONE MoveIt plan with both cameras' obstacles. Nothing else moves.
#   bash ~/nero_pipeline_launch/pipeline4/p4_plan.sh "eggplant" "basket"       (look at the plan in RViz, then Ctrl+C)
#   P4_WRIST=off ...   plan with the top camera's obstacles only (comparison)
P4_PLAN_VERSION="p4-plan-v2"
echo "p4_plan $P4_PLAN_VERSION (P4_WRIST=${P4_WRIST:-on})"
# no conda Python in this script (ROS needs the system Python); the `conda` command stays available for GraspMolmo's worker
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'miniconda3/(bin|envs/[^/]+/bin)$|anaconda3/(bin|envs/[^/]+/bin)$' | paste -sd:)"
command -v conda >/dev/null 2>&1 || { [ -x "$HOME/miniconda3/condabin/conda" ] && export PATH="$PATH:$HOME/miniconda3/condabin"; }
# exactly what `conda deactivate` leaves behind: level 0, no active prefix (a level above 0 without a prefix makes `conda run` crash)
export CONDA_SHLVL=0
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER $(env | sed -n "s/^\(CONDA_PREFIX_[0-9]*\)=.*/\1/p")
if [ -z "${1:-}" ]; then echo 'usage: p4_plan.sh "<what to pick>" ["<where to place>"]'; exit 2; fi
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash 2>/dev/null; source "$HOME/agx_arm_ws/install/setup.bash" 2>/dev/null
ros2 node list 2>/dev/null | grep -qx "/p3_graspmolmo_node" || { echo "STOP: GraspMolmo is not running - start: bash $D/start_pipeline4.sh"; exit 1; }
out=$(timeout 20 ros2 service call /p4/clear_wrist_obstacles std_srvs/srv/Trigger 2>&1)
echo "$out" | grep -q "success=True" || { echo "STOP: the fusion node does not answer - is start_pipeline4.sh running?"; echo "$out" | tail -2; exit 1; }
if pgrep -f "pipeline4/p4_grasp.py" >/dev/null; then echo "grasp step: pipeline 4 (tilt <= the limit set at start, default 20 deg; opened to the object's width)"; else echo "grasp step: pipeline 3's relay copy (P4_GRASP=off)"; fi
echo "previous wrist boxes removed; now: start pose, look, both cameras' obstacles, plan"
export P4_WRIST="${P4_WRIST:-on}"
exec /usr/bin/python3 "$D/core/p3_go.py" --plan --task "$1" --target "${2:-plate}" --run_script "$D/p4_run.sh"
