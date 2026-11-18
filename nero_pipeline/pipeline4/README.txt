PIPELINE 4 - top camera + wrist camera (D405). Independent of pipeline 3 (pipeline4/core = copies of pipeline 3, made at install).

THREE COMMANDS
  1. bash ~/nero_pipeline_launch/pipeline4/start_pipeline4.sh            everything up, checks, READY   (Ctrl+C stops it all)
  2. bash ~/nero_pipeline_launch/pipeline4/p4_plan.sh "eggplant" "basket"   look + both cameras' obstacles + ONE plan   (Ctrl+C after looking)
  3. bash ~/nero_pipeline_launch/pipeline4/p4_execute.sh                  asks EXECUTE, then plans once more and runs it (hand on STOP)
Real arm: arm at HOME before 1, STOP terminal open (STOP typed, no Enter) during 2 and 3.

WHAT PIPELINE 4 ADDS TO PIPELINE 3
  - wrist camera obstacles: 3D boxes at their real height (mid-air things stay mid-air), sink walls kept even where the object leans on them
  - grasps: jaw line level (both fingertips at the same height), tilted at most 20 deg across the jaws (P4_MAX_TILT_DEG; 0 = straight down), finger pads at the object's mid-height, gripper opened to the object's width + 2 cm (P4_OPEN_MARGIN_M)
  - camera offsets from ~/.p4_offsets.yaml (e.g. wrist_offset_z: -0.014)

SWITCHES (set before the command)
  start:  REAL_ARM=0  P4_GRASP=off (pipeline 3's grasps, full opening)  P4_CLOUD_RVIZ=0  P4_MAX_TILT_DEG=0  P4_OPEN_MARGIN_M=0.01
  plan:   P4_WRIST=off (top camera obstacles only)       execute/plan: P4_GRIPPER_WIDTH=off (full opening)

FILES  start_pipeline4.sh p4_plan.sh p4_execute.sh p4_clean.sh | p4_run.sh (generated) core/ (generated) | p4_fuse_node.py p4_wrist_obstacles.py
       p4_grasp.py | p4_cloud_compare.py p4_marker_compare.py (calibration checks) | p4_clouds.rviz | p4_make_core.py (makes core/ and p4_run.sh)
