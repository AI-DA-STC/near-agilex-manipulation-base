# 06. Tuning reference

Every number you may want to change, where it lives, what it does and how it got its value.
"Last known" is the value at the end of the last working session. **Confirm the current value with `maintainer/verify_repo.sh`.**

After editing `nero_mtc_task/mtc_pick_and_place.py` always rebuild (plain copies in `build/` otherwise hide your edit):

```bash
python3 -m py_compile ~/agx_arm_ws/src/nero_mtc_task/nero_mtc_task/mtc_pick_and_place.py && echo SYNTAX OK
rm -rf ~/agx_arm_ws/build/nero_mtc_task ~/agx_arm_ws/install/nero_mtc_task
cd ~/agx_arm_ws && colcon build --symlink-install --packages-select nero_mtc_task && source install/setup.bash
```

## 1. Constants in `mtc_pick_and_place.py`

### 1.1 Which grasps MTC gets

| Name | Last known | Unit | Meaning and history |
|---|---|---|---|
| `MAX_TILT_DEG` | 55.0 | degrees | Grasps whose approach axis is tilted more than this from straight down are dropped. It is a **cap, not a target**: there is no minimum angle (a 6-degree grasp passes). Do not raise above about 60 for now; steep grasps are the ones that hit the table with the real fingers |
| `MAX_CANDIDATES_IN_TASK` | 6 | count | How many candidates (after the tilt cap and, when present, the sort below) are given to MTC. History: 1 (for a "clean single motion"), then 3, then 6 because a single candidate failed on reachability. More candidates cost planning time |
| `usable.sort(key=...tilt...)` | added 2026-10-08; **required**: `maintainer/verify_repo.sh` fails if it is missing | n/a | Orders the usable candidates **most top-down first** before the cut to `MAX_CANDIDATES_IN_TASK`. Without it the cut keeps the network's best-scoring ones, which may all be steeply tilted. Added 2026-10-08 after a run in which the 6-degree and 12-degree candidates ranked 9th and 10th and never reached MTC. **The operator credits this change for the grasps working** (docs/07 stage 12). `launch/tools/grasp_watch.py` prints whether it is active and the tilts handed over |
| pregrasp variants | only the `flip` variant is offered | n/a | A two-finger gripper can close with the jaws rotated 180 degrees about the approach axis (`flip`). The `orig` variant is commented out to cut the number of solutions. If pregrasp starts failing, re-enable it first |

### 1.2 Converting a Contact-GraspNet pose into a `tcp_link` target (`to_tcp_pose`)

| Name | Last known | Unit | Meaning and history |
|---|---|---|---|
| `CGN_GRIPPER_DEPTH` | 0.1034 | m | Contact-GraspNet reports the pose of the *hand base*; the grasp centre is this far along its approach axis. It is the default of `gripper_depth` in `build_6d_grasp` ("distance from gripper coordinate frame to gripper baseline"; the library computes `pose origin = contact point + (opening/2) * jaw direction - gripper_depth * approach direction`). Do not change unless the library's value differs |
| `OUR_FINGER_REACH` | 0.14 | m | Distance from `tcp_link` to the point between **our** fingertips. The pose is shifted along the approach axis by `CGN_GRIPPER_DEPTH - OUR_FINGER_REACH`. Measured finger length: tips at 0.138 m (from the AgileX finger meshes), so the contact centre is probably nearer 0.13 |
| rotation about local Z by -90 degrees | fixed | n/a | Contact-GraspNet's grasp frame has the jaws on **X**, `tcp_link` has them on **Y**; the 90-degree turn maps one onto the other. Unverified against the network's source on this machine (recalled from the upstream code); the grasp watcher checks the end result |
| `GRASP_OFFSET_X`, `GRASP_OFFSET_Y` | 0.0 | m | World-frame nudge added to **every** grasp (negative X is farther from the arm). Used as an emergency compensation for a calibration error (values -0.02, -0.05, -0.08 were tried). Keep at 0.0 when the calibration is right, otherwise you correct twice. A fixed nudge cannot fix an error that depends on the tilt direction, which is why calibration is the proper fix |

### 1.3 Approach, lift, place, retreat

| Name | Last known | Unit | Meaning and history |
|---|---|---|---|
| `PREGRASP_OFFSET_M` | 0.08 | m | Standoff of the pregrasp pose back along the approach axis. History: 0.08, then 0.15 (too far, no IK), 0.11, back to 0.08 |
| `APPROACH_MIN`, `APPROACH_MAX` | 0.02, 0.08 | m | Straight-line approach distance accepted (min) and attempted (max). Keep max equal to `PREGRASP_OFFSET_M`. (A minimum of 0.07 was suggested during tuning but the script in use has 0.02; the grasp watcher shows how far the arm actually went along the approach.) |
| `LIFT_MIN`, `LIFT_MAX` | 0.02, 0.05 | m | Vertical lift after grasping |
| `RETREAT_MIN`, `RETREAT_MAX` | 0.02, 0.15 | m | Vertical retreat after release |
| `PLACE_Z_CLEARANCE_M` | 0.18 | m | Height of `tcp_link` above the plate surface at release. The fingers extend 0.138 m below `tcp_link`, so 0.18 leaves about 4 cm between fingertips and plate. History: 0.05 (before the frame fix), 0.13, 0.21, 0.18. Too low: `eef in collision ... table_surface`. Too high: the object drops from height |
| `PREPLACE_OFFSET_M` | 0.50 | m | Not used by the current code |
| place cone | +-32 degrees per axis | degrees | `W = math.radians(64)` (full range) in `GenerateRandomPose`; roll and pitch tilt the approach, yaw spins the gripper. Corners reach about 45 degrees. Seed orientation `(1, 0, 0, 0)` (straight down). `gen.set_max_solutions(40)` samples, `ik.max_ik_solutions = 20` each |
| `ik.max_ik_solutions` (pregrasp) | 20 | count | IK solutions kept per pose |

### 1.4 Speed (the single place that sets the executed speed)

| Name | Last known | Unit | Meaning |
|---|---|---|---|
| `ARM_MAX_VEL` | 0.1 | rad/s per arm joint | Cruise speed of every arm segment. 0.1 rad/s is about 6 degrees per second. The whole pick-and-place took about 50 s at this value. History: lowered to 0.1 for the real arm |
| `ARM_MAX_ACC` | 1.0 | rad/s^2 | How gently segments start and stop |
| `GRIPPER_MAX_VEL` | 0.03 | m/s | Gripper opening/closing speed (a full opening takes about 3.4 s) |
| `GRIPPER_MAX_ACC` | 0.3 | m/s^2 | |

How the executed speed is set (and what does **not** set it):

| Knob | Effect on the executed speed |
|---|---|
| `ARM_MAX_VEL`, `ARM_MAX_ACC`, `GRIPPER_*` (`retime_solution`) | **Yes.** Every segment gets its timestamps from these just before it is sent |
| `joint_limits.yaml` velocity values | **No.** OMPL and gripper segments leave the planner with no timestamps at all, so there is nothing for a limit to cap. (It only affects the straight-line approach, and `retime_solution` overrides that timing too) |
| URDF velocity limits | No (they are only the ceiling the planners may use) |
| RViz MotionPlanning panel velocity/acceleration scaling (0.10) | No (only plans made from that panel) |
| MTC planner `max_velocity_scaling_factor` | No (removed; the default is 1.0) |
| Panel replay speed (State Display Time) | Preview only |

## 2. Perception parameters

| Parameter | Where | Last known | Meaning |
|---|---|---|---|
| `source_prompt` | `gsam2_ros` parameter; set in `launch/start_pipeline.sh` | `a green cube.` (earlier `a carrot.`, `a roll of tape.` were tried) | What to pick. Must end with a period. An article ("a ...") worked better than none |
| `target_prompt` | same | `plate.` | The place target |
| `box_threshold` | `gsam2_ros` (default 0.4 in code and launch file) | lowered live during testing (0.25 was used) | Detection confidence cutoff. Do not go below about 0.2: false boxes on other objects |
| `forward_passes` | `contact_graspnet_ros` inference script | 5 | Number of network passes; 1 pass found no grasps on thin objects |
| `z_range` | same | `[0.2, 1.8]` | Depth range kept (metres) |
| camera intrinsics | read from `/topcam/depth/camera_info` | fx = fy = 488.78, cx = 317.81, cy = 216.85 (640x480, aligned depth) | |

Change the prompt live (no restart): `ros2 param set /gsam2_segmentation_node source_prompt "a roll of tape."`.
To make it permanent edit the `source_prompt:=...` argument in `launch/start_pipeline.sh`.
The internal object name in MTC stays `carrot` whatever you pick; that is only a label.

## 3. Configuration files changed in third-party packages (`config_snapshots/agx_arm_moveit/`)

| File | Change | Values |
|---|---|---|
| `config/agx_arm.srdf.xacro` | Named states in the `arm_type == 'nero'` block | `home` (current): joint1..7 = -0.67085, 1.69723, -0.02670, -0.13057, -0.01522, -0.00407, 0.01602. Earlier `home`: -3.49e-05, 1.68793, 1.75e-05, -0.13659, -0.07585, 0.00248, 0.04035. `pose_90deg`: -3.49e-05, -0.00630, -5.24e-05, 1.56736, -0.07589, 0.00257, 0.04032 (gripper values: `home` 0.000377, `pose_90deg` 0.000195). The file also has a separate 6-joint `home` block for other arm types; edit the one that contains `joint7` |
| `config/initial_positions.yaml` | The pose the **simulation** starts in | the `home` values above (updated together with the SRDF) |
| `config/joint_limits.yaml` | Planning limits | joints 1..7 were set to velocity 1.0 and acceleration 2.0. The original values are in `joint_limits.yaml.bak` on the lab machine. No effect on executed speed (see above). Restoring the `.bak` copy is harmless. Whether it was restored is not recorded |

`launch/scripts/move_to_pose_90deg.py` moves to `pose_90deg` with a direct trajectory; the target values are defined inside the script
(not read from the SRDF), so change both if you change the pose. The script's duration is `HOME_MOVE_SECONDS` (default 4 s).

## 4. Shell environment variables

See the table at the end of `docs/05_run_real_arm.md` (`REAL_ARM`, `TABLE_TOP_Z`, `ARM_WAIT`, `JOINT_STATES_TOPIC`,
`HOME_MOVE_SECONDS`, `MTC_EXECUTE`). The grasp watcher also reads `GRASP_VIEW_PY` (the python that has Open3D;
default `~/miniconda3/envs/contact_graspnet/bin/python`).
