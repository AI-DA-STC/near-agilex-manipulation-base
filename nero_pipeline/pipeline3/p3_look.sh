#!/bin/bash
# PIPELINE 3, STEP 2 (every time the objects or the prompt change): look at the table again and plan. The arm only goes to its start pose.
#   bash ~/nero_pipeline_launch/pipeline3/p3_look.sh "carrot" "plate"
# -> GraspMolmo's grasp + place points, Contact-GraspNet's grasps, the picture (opens), the plan in RViz. Then Ctrl+C, and STEP 3 (p3_do.sh).
if [ -z "${1:-}" ]; then echo 'usage: p3_look.sh "<what to pick>" ["<where to place>", default "plate"]   e.g.  p3_look.sh "carrot" "plate"'; exit 2; fi
exec /usr/bin/python3 "$HOME/nero_pipeline_launch/pipeline3/p3_go.py" --plan --task "$1" --target "${2:-plate}"
