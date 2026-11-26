#!/bin/bash
# PIPELINE 5 - execute the plan p5_plan.sh saved, right away: no re-planning, no typing. Refuses (nothing moves) if the arm moved since planning,
# the plan is old or from an earlier look. Hand on STOP.
P5_EXECUTE_VERSION="p5-execute-v2"   # v2: runs the SAVED plan (no re-plan, no EXECUTE prompt)
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'miniconda3/(bin|envs/[^/]+/bin)$|anaconda3/(bin|envs/[^/]+/bin)$' | paste -sd:)"
command -v conda >/dev/null 2>&1 || { [ -x "$HOME/miniconda3/condabin/conda" ] && export PATH="$PATH:$HOME/miniconda3/condabin"; }
export CONDA_SHLVL=0
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER $(env | sed -n "s/^\(CONDA_PREFIX_[0-9]*\)=.*/\1/p")
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash 2>/dev/null; source "$HOME/agx_arm_ws/install/setup.bash" 2>/dev/null
mkdir -p "$HOME/p3_runs/go_logs"
/usr/bin/python3 "$D/p5_exec_plan.py" 2>&1 | tee "$HOME/p3_runs/go_logs/exec_$(date +%Y%m%d_%H%M%S).txt"
exit ${PIPESTATUS[0]}
