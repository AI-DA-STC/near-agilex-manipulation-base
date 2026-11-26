#!/bin/bash
# PIPELINE 5 - look, plan and EXECUTE in one go (no typing): as soon as a plan exists, the arm runs it. Hand on STOP.
#   bash ~/nero_pipeline_launch/pipeline5/p5_go.sh "eggplant" "the middle of the basket"
P5_GO_VERSION="p5-go-v1"
echo "PIPELINE 5 GO - the arm moves right after planning: hand on STOP"
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'miniconda3/(bin|envs/[^/]+/bin)$|anaconda3/(bin|envs/[^/]+/bin)$' | paste -sd:)"
command -v conda >/dev/null 2>&1 || { [ -x "$HOME/miniconda3/condabin/conda" ] && export PATH="$PATH:$HOME/miniconda3/condabin"; }
export CONDA_SHLVL=0
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER $(env | sed -n "s/^\(CONDA_PREFIX_[0-9]*\)=.*/\1/p")
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash 2>/dev/null; source "$HOME/agx_arm_ws/install/setup.bash" 2>/dev/null
if [ -z "${1:-}" ]; then echo 'usage: p5_go.sh "<what to pick>" ["<where to place>"]'; exit 2; fi
bash "$D/p5_precheck.sh" || exit 1
export P5_WRIST="${P5_WRIST:-on}" P4_WRIST="${P5_WRIST:-on}" P3_SHOW_PICTURE="${P3_SHOW_PICTURE:-0}" P5_PLAN_EXIT=1
/usr/bin/python3 "$D/core/p3_go.py" --plan --task "$1" --target "${2:-plate}" --run_script "$D/p5_run.sh" || { echo "NOT EXECUTED"; exit 1; }
[ -f "$HOME/p3_runs/p5_plan_latest.bin" ] || { echo "NOT EXECUTED (no plan saved)"; exit 1; }
bash "$D/p5_execute.sh"
