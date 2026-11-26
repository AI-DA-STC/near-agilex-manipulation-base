#!/bin/bash
# PIPELINE 5 - plan again with the SAME look (grasps, place point, obstacles), about 10-30 s; saves the plan. MoveIt's search is random, so a
# failed plan often works the second time.  P5_GO=1: execute it right away if found.
P5_REPLAN_VERSION="p5-replan-v1"
echo "p5_replan $P5_REPLAN_VERSION"
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'miniconda3/(bin|envs/[^/]+/bin)$|anaconda3/(bin|envs/[^/]+/bin)$' | paste -sd:)"
command -v conda >/dev/null 2>&1 || { [ -x "$HOME/miniconda3/condabin/conda" ] && export PATH="$PATH:$HOME/miniconda3/condabin"; }
export CONDA_SHLVL=0
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER $(env | sed -n "s/^\(CONDA_PREFIX_[0-9]*\)=.*/\1/p")
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash 2>/dev/null; source "$HOME/agx_arm_ws/install/setup.bash" 2>/dev/null
export P4_WRIST="${P5_WRIST:-on}" P3_SHOW_PICTURE="${P3_SHOW_PICTURE:-0}" P5_PLAN_EXIT=1
rm -f "$HOME/p3_runs/p5_plan_latest.bin"
/usr/bin/python3 "$D/core/p3_go.py" --plan --last --run_script "$D/p5_run.sh"
[ -f "$HOME/p3_runs/p5_plan_latest.bin" ] || { echo "no plan this time either: look again (p5_plan.sh / p5_go.sh)"; exit 1; }
if [ "${P5_GO:-0}" = 1 ]; then bash "$D/p5_execute.sh"; else echo "PLAN SAVED. To execute it:  bash $D/p5_execute.sh"; fi
