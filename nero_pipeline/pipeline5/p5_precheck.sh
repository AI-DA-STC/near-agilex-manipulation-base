#!/bin/bash
# shared by p5_plan.sh / p5_go.sh: is pipeline 5 running? then a clean slate for this look (old plan, old wrist boxes) and tidy old run files
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'miniconda3/(bin|envs/[^/]+/bin)$|anaconda3/(bin|envs/[^/]+/bin)$' | paste -sd:)"
command -v conda >/dev/null 2>&1 || { [ -x "$HOME/miniconda3/condabin/conda" ] && export PATH="$PATH:$HOME/miniconda3/condabin"; }
export CONDA_SHLVL=0
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER $(env | sed -n "s/^\(CONDA_PREFIX_[0-9]*\)=.*/\1/p")
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash 2>/dev/null; source "$HOME/agx_arm_ws/install/setup.bash" 2>/dev/null
ros2 node list 2>/dev/null | grep -qx "/p3_graspmolmo_node" || { echo "STOP: GraspMolmo is not running - start: bash $D/start_pipeline5.sh"; exit 1; }
ros2 service list 2>/dev/null | grep -qx "/p5/grasps" || { echo "STOP: pipeline 5's grasp node is not running - is start_pipeline5.sh running?"; exit 1; }
out=$(timeout 20 ros2 service call /p5/clear_wrist_obstacles std_srvs/srv/Trigger 2>&1)
echo "$out" | grep -q "success=True" || { echo "STOP: the fusion node does not answer - is start_pipeline5.sh running?"; echo "$out" | tail -2; exit 1; }
rm -f "$HOME/p3_runs/p5_grasp_latest.json" "$HOME/p3_runs/p5_plan_latest.bin" "$HOME/p3_runs/p5_plan_latest.json"
rm -f /tmp/p5_view/view.png                 # this look's picture starts fresh
/usr/bin/python3 "$D/p5_tidy.py" > /dev/null
