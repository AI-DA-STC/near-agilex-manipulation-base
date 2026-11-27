#!/usr/bin/env python3
"""PIPELINE 5, P5_REMOTE=1: SAM2 on the laptop. p5_grasp_node.py (v6f) starts this instead of p5_sam2_worker.py when
P5_REMOTE=1, in the grounded_sam2 env with the Grounded-SAM-2 folder as working directory. --model/--repo are passed through
to the local worker, which this process becomes if no laptop server answers at start. Standard library only."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import p5_remote_worker as R  # noqa: E402

LOCAL = os.environ.get("P5_SAM2_LOCAL_WORKER", os.path.join(os.path.dirname(HERE), "p5_sam2_worker.py"))
if __name__ == "__main__":
    sys.exit(R.main(["--kind", "sam2", "--local", LOCAL] + sys.argv[1:]))
