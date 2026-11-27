# MoSAIC-DUO: Modular Skills for Adaptive Interpretable manipulation Control with Dual-camera Unified Observation

Pipeline 5 of the AgileX Nero manipulation stack (PIPELINE 5 - 8e = 8d + ONE change: the pipeline's arm moves ).

## What pipeline 5 adds to pipeline 4
- Contact-GraspNet replaced by a geometric grasp step for the Nero gripper (from GraspMolmo's point; measured fingers + wrist camera)
- SAM2 segments the object at GraspMolmo's point (pipeline 4's depth region swallowed sink walls)
- Place point moved to where the gripper fits; top obstacle boxes rebuilt around it; smaller release opening
- MoveIt knows the object in hand and the wrist camera; full 8 cm approach; checked path kept free of wrist boxes
- One-command p5_go.sh, saved plans executed without re-planning, CAN check, reliable Ctrl+C, arm 0.5 rad/s
- Three staged pictures in one window, concise terminal with step times, offline replay, ~210 offline checks

Full list, known limitations and all settings: [CHANGES_FROM_PIPELINE4.md](CHANGES_FROM_PIPELINE4.md)

## Commands and settings
```
PIPELINE 5 - pipeline 4 with Contact-GraspNet REPLACED by grasps made from GraspMolmo's point (no training). Tier 1: straight down, tilt <= 20 deg.
Independent: everything it runs is in pipeline5/ (core/ = copies of pipeline 4 v10, see core/COPIED_FROM_PIPELINE4.txt). Pipeline 4 stays the baseline.

COMMANDS
  1. bash ~/nero_pipeline_launch/pipeline5/start_pipeline5.sh                    everything up, checks, READY, the live window "Pipeline 5"
  2. bash ~/nero_pipeline_launch/pipeline5/p5_go.sh "eggplant" "the middle of the basket"   look + plan + EXECUTE right away (no typing)
     or: p5_plan.sh "..." "..."  (look + plan, SAVED)   then   p5_execute.sh  (runs the saved plan: no re-planning, no typing)
     p5_replan.sh: plan again with the same look (cheap retry; P5_GO=1 executes it if found)
  p5_execute.sh refuses (nothing moves) if the arm moved since planning, the plan is >15 min old or from an earlier look.
  Real arm: STOP terminal open (STOP typed, no Enter) during 2. Ctrl+C in 1 stops everything and deletes the live-view pictures.
LIVE WINDOW "Pipeline 5" (p5_view.py): every step with its time; the picture of the look as soon as the grasps exist (object outline, G1..G3,
  rejected grasps, GraspMolmo's points, the place point used); failures in red. Old run folders are trimmed (newest 10 kept, p5_tidy.py).

THE GRASP STEP (/p5/grasps, p5_grasp_node.py + p5_grasp_core.py + p5_gripper.py)
  OBJECT MASK: SAM2 (p5_sam2_worker.py, pipeline 1's grounded_sam2 env) clicked at GraspMolmo's point. GraspMolmo's own region ("everything raised
  within 8 cm of the point") swallows sink walls and neighbours; it is only a fallback and is REFUSED when it fills its search circle.
  GraspMolmo's point -> the object's points (top camera) -> jaw across the narrow side near the point (also: every 15 deg, the long side),
  slide +-1 cm along the object, tilt 0/+-10/+-20 deg across the jaws (jaw line level) -> fingertips 25 % of the object's height above the floor
  it stands on (6-10 mm; table or sink floor, measured 1-3 cm around the object; also 5 / 10 mm higher when the low grasp is blocked;
  the jaws may sit up to 8 mm off-centre - the near finger touches first and slides the object to the middle - for room beside a wall) -> checks: opening (width + 2 cm <= 10 cm), fingers + body vs the object, the neighbours (top camera)
  and the fused cloud (both cameras), also along the 8 cm approach -> best 3 with ONE opening -> MoveIt.
  See the grasps: 2nd RViz (green G1, yellow G2/G3, red rejected); main RViz: add MarkerArray /p5/grasp_markers.
  Every look is saved in ~/p3_runs/p5_snaps/; rerun it offline with other settings:  /usr/bin/python3 ~/nero_pipeline_launch/pipeline5/p5_replay.py

THE PLACE STEP (/p5/place, p5_place_core.py)
  GraspMolmo's place point -> SAM2 (the whole container) -> the nearest floor spot (within 8 cm) where the gripper fits at release, at some yaw
  within 60 deg of the grasp's (MTC's release yaw range): fingers (tapered) + the object between them, and the 165 x 55 mm body, clear of walls
  and things in the basket. -> /p5/mask_target: target_pose, the fusion's landing zone and the picture follow it. The top camera's boxes are
  rebuilt around the NEW point (GraspMolmo's 6 cm landing zone around its own point hid the basket wall next to it). P5_PLACE=off: unchanged.

MOVEIT SIDE
  The object being picked is in MoveIt's scene as "carrot" (4 slice boxes from SAM2's points; MTC attaches it at the grasp), so it is
  collision-checked during the lift, transit and place. The wrist camera + mount + cable is an 8 x 8 x 8 cm box attached to tcp_link (beside the gripper, +x; ~/.p5_gripper.yaml cam_cx/cy/cz/sx/sy/sz),
  in every plan and in the grasp/place checks (P5_CAMERA_BOX=off: without it). The grasp node says once whether the calibrated camera sits in it.
  MTC allows gripper-object contact from before opening (the carrot box is coarser than the object; the grasp step checked the real fingers).
  The wrist boxes leave out the gripper's checked corridor (p5_keepout.py: fingers + body along the 8 cm approach, +3 cm) - the grasp step
  checked it finer; without this the raised sink floor became a box and MTC closed 2 cm above the object. MTC's approach is the full 8 cm
  or no plan (pipeline 5's MTC copy).
CAN (real arm): start_pipeline5.sh checks the can_piper LINK first (up, 1 Mbit/s; one adapter: runs can_activate.sh itself), then that the arm
  publishes /feedback/joint_states after its driver starts (the Nero is silent before that).

SWITCHES (set before start_pipeline5.sh; p5_replay.py reads the same ones)
  P5_MAX_TILT_DEG=20   P5_OPEN_MARGIN_M=0.02   P5_GRASPS_TO_MOVEIT=3   P5_OFFSET_M=0.01   P5_CLOUD_CHECK=on|off   P5_TIP_CLEARANCE=<m> (fixed)
  P5_MASK=sam2|depth   P5_SAM2_MODEL=large|base_plus|small|tiny   P5_SAM2_REPO=<Grounded-SAM-2 folder>   P5_KEEPOUT=on|off   P5_PLACE=on|off   P5_CAMERA_BOX=on|off   P5_OBJECT_BOX=on|off   P5_MIN_CORNER_MM=1   P5_DESCEND_MIN_M=0.01   P5_VERBOSE=1 (full terminal output)   P5_POPUP=off   P5_RAISE_MM=0,5,10   P5_CAN=can_piper
  REAL_ARM=0   P5_CLOUD_RVIZ=0    plan/execute: P5_WRIST=off (top camera obstacles only), P5_GRIPPER_WIDTH=off (fully open)
GRIPPER SIZES: p5_gripper.py (measured); override in ~/.p5_gripper.yaml, e.g.  pad_h: 0.065
OWN FILES: ~/.p5_offsets.yaml (camera offsets), ~/p3_runs/p5_grasp_latest.json, p5_relay_latest.json, p5_snaps/, ~/p5_runs/ (wrist boxes)
SHARED WITH EVERY PIPELINE: ~/nero_pipeline_launch/scripts, the calibrations, ~/.p3_table_z, the ROS workspace, ~/p3_runs/latest (GraspMolmo's run)
TESTS: bash ~/nero_pipeline_launch/pipeline5/tests/run_tests.sh
```

## Speed-ups (27 Nov): laptop GPU + parallel steps
- `P5_REMOTE=1 bash start_pipeline5.sh`: GraspMolmo + SAM2 run on the laptop GPU (`laptop_server/start_server.sh`, cable 10.10.0.1 first, then Wi-Fi); the Jetson's own models are used automatically if the laptop does not answer. GraspMolmo 27.5 s -> 4.3 s (grasp), 17.0 s -> 2.4 s (place).
- `P5_EARLY_WRIST=1`: the wrist-camera boxes start right after /p5/place, alongside pose_transform.
- `P5_MTC_MAX_SOLUTIONS=N`: MTC stops after N solutions (default 100 with two orientations).
- One plan (eggplant -> basket): 143.8 s (Jetson only) -> 69.2 s (laptop models, early wrist boxes, MTC cap 10, arm at start pose).
