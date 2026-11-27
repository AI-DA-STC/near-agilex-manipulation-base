#!/usr/bin/env bash
# MoSAIC-DUO model server on the LAPTOP (p5-start-server-v1). Keep this terminal open; Ctrl+C stops the server.
[ "$(whoami)" = "liyun" ] || echo "NOTE: this is meant for the laptop (liyun@nearadmin-X56xWNx), you are $(whoami)@$(hostname)"
source "$HOME/miniconda3/etc/profile.d/conda.sh" 2>/dev/null || eval "$(conda shell.bash hook)"
conda activate p5_gpu || { echo "STOP: conda env p5_gpu not found"; exit 1; }
USED=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | head -1)
if [ -n "$USED" ] && [ "$USED" -gt 4000 ]; then
  echo "WARNING: ${USED} MiB of GPU memory already in use - is the pi0.5 server running? GraspMolmo + SAM2 need about 16 GB."
  nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv,noheader
fi
ss -ltn 2>/dev/null | grep -q ":8765 " && { echo "STOP: something already listens on port 8765 (an older server?)"; exit 1; }
cd "$(dirname "$0")" && exec python p5_server.py "$@"
