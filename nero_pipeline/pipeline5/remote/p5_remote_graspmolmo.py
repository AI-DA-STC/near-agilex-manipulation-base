#!/usr/bin/env python3
"""PIPELINE 5, P5_REMOTE=1: GraspMolmo on the laptop. start_pipeline5.sh gives this script to core/p3_graspmolmo_node.py as its
worker_script, so the node runs it in the graspmolmo conda env exactly like the local worker. If no laptop server answers at
start, it becomes core/p3_graspmolmo_worker.py (the local model) with the same arguments. Standard library only."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import p5_remote_worker as R  # noqa: E402

LOCAL = os.environ.get("P5_GM_LOCAL_WORKER", os.path.join(os.path.dirname(HERE), "core", "p3_graspmolmo_worker.py"))
if __name__ == "__main__":
    sys.exit(R.main(["--kind", "graspmolmo", "--local", LOCAL] + sys.argv[1:]))
