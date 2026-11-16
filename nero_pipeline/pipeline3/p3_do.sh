#!/bin/bash
# PIPELINE 3, STEP 3: EXECUTE what STEP 2 (p3_look.sh) found - the same grasps and place spot, no new picture. Asks EXECUTE on the real arm.
# (MoveIt plans once more right before executing; the objects must not have moved since STEP 2.)
exec /usr/bin/python3 "$HOME/nero_pipeline_launch/pipeline3/p3_go.py" --last
