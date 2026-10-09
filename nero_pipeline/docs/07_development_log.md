# 07. Development log

What was done, in the order it happened, with what was tried, what failed and what worked. It is written so that a newcomer
can understand **why the system looks the way it does** and avoid repeating dead ends.

Conventions: dates are 2026, local time of the lab (UTC+8). Early dates come from the recorded sessions; a stage's date range
is approximate. "Result" is the state after the stage. The section **Corrections** at the end lists statements that were
made during the project and later turned out to be wrong.

## Timeline at a glance

| Dates | Stage | Result |
|---|---|---|
| Sep 14 to 17 | 0. Hardware bring-up | Arm, CAN, MoveIt, two cameras running; scripted demos (carrot, jacket folding) with fixed poses |
| Sep 17 to 24 | 1. Perception components and the first task | Segmentation, grasp, plate-pose and transform nodes built; hand-eye calibration; first MTC task |
| Sep 24 to 29 | 2. Pipeline scripting and planning bugs | One-command launch scripts; CHOMP crash removed; execute capability found; calibration drift fixed; plans found |
| Sep 29 to 30 | 3. The execution abort (`val=99999`) | Root cause found (untimed trajectories); first successful execution |
| Sep 30 to Oct 1 | 4. RViz panel pitfalls and the gripper-frame bug | `tcp_link` corrected in both robot models; straight-down orientation verified on the arm |
| Oct 1 | 5. Making pick and place plan | Cartesian approach, pregrasp offsets, continuous place cone; first full plan and execution |
| Oct 1 to 5 | 6. Usability and speed | No hang on failure; every segment retimed; one speed constant |
| Oct 5 | 7. Candidate filtering, start pose, new home | Tilt filter, Contact-GraspNet frame conversion, start pose move, home pose edited |
| Oct 6 | 8. Real-arm bring-up | CAN verified; driver working; real-arm launcher; software stop; first slow executions |
| Oct 6 to 7 | 9. Grasps land short; recalibration | Offsets tried; recalibration attempted and rejected; calibration set by hand |
| Oct 7 to 8 | 10. Grasp-quality investigation | Inputs verified good; network behaviour on the sample scenes good; cause found afterwards: candidate selection (stage 12) |
| Oct 8 | 11. Inspection tools, candidate ordering, reach limit | Grasp watcher/viewer; tilt sort; place failures explained by reach |
| Oct 8 | 12. Grasps working | The operator attributes it to sorting candidates most top-down first |

---

## Stage 0. Hardware bring-up (Sep 14 to 17)

**Goal:** control the AgileX Nero from ROS 2 on a Jetson.

**Done**
* ROS 2 Humble, MoveIt 2; cloned `agx_arm_ros` with `git clone -b ros2 --recurse-submodules`.
* **CAN adapter:** `modprobe gs_usb` failed (`Module gs_usb not found in directory /lib/modules/5.15.199-tegra`). We built the driver
  out of tree from the upstream kernel v5.15 source and loaded it with `insmod`. `modprobe` cannot find it because it is not installed into the module tree.
  Procedure: [docs/02](02_install_and_environment.md) section 2.2. The interface is named `can_piper` and bound to a USB port.
* Two cameras integrated: Intel RealSense (arm camera) and Orbbec Dabai DC1 (top camera, the one used later).
* Named states `home` and `pose_90deg` added to the SRDF (`agx_arm.srdf.xacro`, `arm_type == 'nero'` block).
* Scripted pick-and-place demos (carrot, a jacket-folding demo) with fixed, hand-taught poses; GitHub workflow (an umbrella repository with the
  packages as submodules, feature branches).
* First architecture discussion for a perception-based pipeline (segmentation, grasp detection, task planner).

**Dead ends / lessons**
* Initially no frames on `candump`. **Always verify CAN traffic with `candump` before blaming software**; the adapter is bound to a USB *port*, so a different
  port or a re-plugged adapter needs `find_all_can_port.sh` and `can_activate.sh` again.

## Stage 1. Perception components and the first task (Sep 17 to 24)

**Goal:** from "a carrot." to a pose in `base_link`.

**Done**
* **Grounded-SAM2** and **Contact-GraspNet (PyTorch port)** installed in two conda environments with the Jetson-specific PyTorch wheel.
  Fixes that were needed: `LD_LIBRARY_PATH` with CUDA and `cusparselt`, `PYTHONNOUSERSITE=1`, `numpy==1.26.1` after `opencv-python`, `transformers==4.44.2`.
* ROS wrappers written: `gsam2_ros`, `contact_graspnet_ros` (Sep 22), `target_pose_ros`, `pose_transform_ros` (Sep 23).
  * `contact_graspnet_ros` needs `forward_passes=5`: a single pass returned no grasps on thin objects.
  * `pose_transform_ros` needed `import tf2_geometry_msgs` to register the PoseStamped transform.
  * `gsam2_ros`: prompts must end with a period; "No detection" cases were fixed with an article ("a carrot.") and a lower `box_threshold`.
* **Hand-eye calibration (eye on base)** with `easy_handeye2` and ArUco (first a ChArUco board, then a single ArUco marker).
  Problems found and fixed (details in [docs/03](03_calibration.md)):
  * translation values 1000 times too large (`y = -1134`): the marker size had been entered as `50` instead of `0.050` (metres);
  * the result targeted `topcam_color_optical_frame`, which already has a parent in the camera driver's TF, so it was re-expressed relative to `topcam_link`;
  * a single marker needs 15 to 20 samples with varied **rotation**, not just varied position.
* **First MTC task** (`nero_mtc_task`, Sep 23). Design decision: give MTC **all** candidate grasps as an `Alternatives` container instead of testing IK ourselves. Our external IK checks and MTC's own
  stages disagreed about the same pose because a 7-DOF IK solution depends on the starting joint configuration.
* **AnyGrasp** was considered (Sep 17) as a stronger grasp network; its licence was not available, so Contact-GraspNet is used. This is still the plan once the licence arrives.

## Stage 2. Pipeline scripting and planning bugs (Sep 24 to 29)

**Goal:** one command to start everything, one command to run it, plans that succeed.

**Done**
* `start_pipeline.sh` (starts all nodes, per-node logs in `~/nero_pipeline_logs/`), `run_pick_and_place.sh` (four perception services, then MTC),
  `add_table_collision.py` (Sep 25: a box under the arm so planning avoids the table), `ckpt.sh` (save and restore known-good states of these scripts).
* **Reachability sweep** (`reach_sweep`, Sep 25) and a **standalone MTC test** (Sep 28, no perception): showed the task construction itself was sound.
  (The sweep later turned out to rank orientations in a mislabeled gripper frame, see stage 4.)

**Failures and fixes**

| Symptom | Cause | Fix |
|---|---|---|
| `move_group` segfault at planning start | CHOMP: `ChompOptimizer::initialize()` crashed (confirmed with `gdb`) and the plugin loader chose CHOMP even when OMPL was requested | Removed the CHOMP package; forced `planning_pipelines` to OMPL in the MTC launch file |
| Library version errors (`libchomp_motion_planner.so.2.5.9` vs `libmoveit_planning_interface.so.2.5.10`) | Mixed MoveIt versions | `sudo apt full-upgrade` |
| Pilz planner fails to load | `libmoveit_planning_interface.so.2.5.9` missing | Left as is. The code names a Pilz pipeline that falls back to OMPL with a warning |
| Edits to the MTC launch file or script had no effect | `build/` held a plain copy while `src/` was edited | `rm -rf build/<pkg> install/<pkg>` and rebuild ([docs/02](02_install_and_environment.md)) |
| `Failed to connect to execute_task_solution` / silent execution failure | `move_group` was started without `ExecuteTaskSolutionCapability` | Launch with `capabilities:="move_group/ExecuteTaskSolutionCapability"` |
| All IK failed although targets looked reachable | Calibration drift put the cloud about 14 cm too close to the base | Fixed by correcting the calibration (yaw adjusted by hand with `adjust_calib_yaw.py`). **Lesson: check the point-cloud overlay before planning** |
| `add_table_collision.py` failed with an `rclpy` import error | The shell was in conda `base`, whose Python cannot import `rclpy` | `conda deactivate`; call the script with `/usr/bin/python3` |

## Stage 3. The execution abort `val=99999` (Sep 29 to 30)

**Symptom:** planning succeeded ("Found N solutions"), execution printed `val=99999` and the arm did not move.

**What was checked and was fine:** controllers active, the `execute_task_solution` action present, joint states publishing, action names.

**Root cause chain**
1. `task.execute()` in MTC discards the real error and returns the placeholder `99999` (`MoveItErrorCodes::FAILURE`) for any failure
   (`task.cpp`, around line 334).
2. Calling the `execute_task_solution` **action directly** (from `rclpy`) exposed the real code: `-4 = CONTROL_FAILED`.
3. The controller log said: `Time between points 0 and 1 is not strictly increasing, it is 0.000000 and 0.000000 respectively`.
4. A diagnostic print of every sub-trajectory showed that **only the Cartesian approach had timestamps**. Every OMPL segment (connect, lift, transit, retreat) and
   every gripper move had `time_from_start = 0` for all points.
5. A first fix (sanitising timestamps) **had no effect** because a later duplicate line `goal.solution = task.solutions[0].toMsg()` re-fetched a fresh,
   unsanitised copy of the solution and overwrote the fix. Removing the duplicate made it work.

**Result:** `error_code.val = 1` (SUCCESS), the arm model moved (verified from joint states) on 2026-09-30.
**Lessons:** read the real error code; when a fix "does nothing", look for something overwriting it.

## Stage 4. RViz panel pitfalls and the gripper-frame bug (Sep 30 to Oct 1)

**RViz panel**
* The Task Solution Topic must be `/solution` **with the leading slash**, otherwise the panel shows nothing.
* The panel's Execute button sends the raw plan without retiming (explained from its source) and fails like stage 3. Our own execution path is used instead ([docs/01](01_system_overview.md) section 5).
* The MTC process only stays alive after a **successful** plan; after a failed one it now exits (an earlier version hung).

**The gripper-frame bug (the most important bug of the project)**
* **Symptom:** gripper approaches looked sideways; moving the arm by hand-set joint values to a pose that "looked straight down" gave no consistent ground truth; the orientations that had been judged "straight down" from the reachability sweep did not point down on the real geometry.
* **Cause:** from reading the URDF: `tcp_link` was attached to `link7` with **zero offset and zero rotation**. The real gripper is mounted `link7 -> gripper_flange -> gripper_base` with rotation `rpy = (-1.5708, 0, -1.5708)`. So every orientation we requested was applied about axes 90 degrees away from the real approach direction. All earlier orientation work (sweeps, the "best" quaternion) was done in the wrong frame and had to be redone.
* **Fix attempts that failed:** launch arguments `tcp_offset_xyz` / `tcp_offset_rpy` (they do not exist as launch arguments); passing a multi-word value through a shell string and `bash -c` (the quoting broke the list).
* **Fix that worked:** the real argument is a single Python list: `tcp_offset:="[0.037, 0, -0.0235, -1.5708, 0, -1.5708]"`, passed from a dedicated script (`launch_arm.sh`).
* **Second bug:** MTC has **its own copy of the robot model** (generated at start into `/tmp/mtc_moveit_params_*.yaml`) that still had the zero offset. Found by parsing that file; fixed by passing `tcp_offset` to the MTC launch as well; verified with `launch/scripts/check_tcp_in_mtc.py`.
* **Straight-down orientation** was verified on the arm with `launch/tools/find_down_interactive.py` (tries candidate quaternions, you judge by eye): **`(1, 0, 0, 0)`** is straight down in the corrected frame.

**Lessons:** validate frames with a physical test before building on them; a "reachable" orientation is not necessarily the one you meant.

## Stage 5. Making pick and place plan (Oct 1)

| Problem | Finding | Fix / decision |
|---|---|---|
| `eef in collision` at pregrasp | Pregrasp too low / candidate too close to the table | Raised offsets; pregrasp offset tried 0.15 (no IK), 0.11, settled on 0.08 |
| `approach` failed for every candidate with no message | The OMPL pose-goal version of the straight approach gives no diagnostics | Replaced by a **Cartesian straight-line solver** (`core.CartesianPath`, 5 mm steps). It plans and says how far it got. This was the change that finally made planning work |
| Place failed with a single fixed orientation | Fixed orientation too restrictive for 7-DOF IK | Place now samples a **continuous cone** around straight down (`GenerateRandomPose`, +-32 degrees) |
| Single candidate often failed | Reachability depends on the candidate | Candidates 1, then 3, then 6 as `Alternatives` |
| Warning on every run: `Attempting to attach object 'carrot' ... no geometry` | The held object has no collision shape | Left (open item) |

**Result:** the first complete plan and execution in simulation.

## Stage 6. Usability and speed (Oct 1 to 5)

* **Speed.** The real arm moved fast. Several knobs were tried and **none changed the executed speed**: URDF velocity limits, `joint_limits.yaml` (edited to 1.0 rad/s and 2.0 rad/s^2), RViz scaling sliders and the planner's velocity scaling.
  Reason (from the stage 3 diagnostics): OMPL and gripper segments have no timestamps, so there is nothing for a limit to cap; the controller only runs what it is given.
  **Fix:** `retime_solution()` recomputes a trapezoidal timing for every sub-trajectory from four constants before sending; one place sets the speed (`ARM_MAX_VEL = 0.1` rad/s for the real arm; the complete pick and place then takes about 50 s).
* The RViz panel replay (raw plan) is always fast and disagrees with the executed speed. An attempt to make the planner name `ompl` (so the plan carries timing) broke the replay and was **reverted** to the Pilz name that falls back to OMPL.
* `joint_limits.yaml` velocity edits are therefore cosmetic; the original is in `joint_limits.yaml.bak`.

## Stage 7. Candidate filtering, start pose, new home (Oct 5)

* **Candidate filter:** every candidate is printed with its tilt from straight down and `KEEP` or `drop`; cap `MAX_TILT_DEG = 55`; at most 6 candidates go to MTC. Place uses the continuous cone (stage 5).
* **Contact-GraspNet to `tcp_link` conversion (`to_tcp_pose`):** the network's grasp frame has the jaws on **X** and ours on **Y** (rotate -90 degrees about local Z), and its pose is the *hand base*, 0.1034 m behind the grasp centre, while our `tcp_link` is 0.14 m behind our fingertips (shift along the approach axis by the difference).
* **Start pose:** `run_pick_and_place.sh` step 0 moves the arm to `pose_90deg` with a direct trajectory, so every run starts from the same configuration (IK is seed-dependent). The script reads the joint state first and skips the move if the arm is already there.
* **Home pose changed:** a new `home` was captured from the real arm (SRDF and `initial_positions.yaml`; the SRDF has two `home` blocks, the 6-joint one for other arms must not be edited).
* **Table:** the table box height is set in `add_table_collision.py`; the plan was to raise it in small steps toward the true table height. A shell override (`TABLE_TOP_Z`) was prepared for this (stage 8), but it was **not applied to the working copy of the script** until the repository was assembled, so the runs in which grasps worked used the script's built-in height.
* **Detection:** `gsam2_ros` `box_threshold` lowered from 0.4 (0.25 was used) after detections were missed.

## Stage 8. Real-arm bring-up (Oct 6)

1. **CAN check:** `ip -details link` showed the interface up but that says nothing about traffic. Only `candump` shows traffic. After re-plugging, the interface number and USB port changed and `can_activate.sh` had to be re-run.
2. **Read joint states without moving anything:** `start_single_agx_arm.launch.py ... auto_enable:=false control_enabled:=false` starts only the driver (no RViz, no MoveIt). The first attempt died with `Failed to get firmware version`:
   the driver asks the arm for its firmware version (CAN id `0x4AF`) and exits after about 5 s without an answer. The arm answered on a retry (firmware `v111`). `/feedback/joint_states` then carried the real joint values, used to define the new `home`.
3. **Real launcher (`launch_arm_real.sh`).** Iterations and why:

| Attempt | Failure | Resolution |
|---|---|---|
| AgileX's combined `start_single_agx_arm_moveit.launch.py` | Does not forward `capabilities`, so no `execute_task_solution` | Launch the driver and the normal MoveIt demo (with `follow:=true`) separately |
| Wait for feedback with a single `ros2 topic echo --once` | The command exits immediately when the topic does not exist yet, so the launcher aborted before the driver published | Retry loop (up to 20 attempts) |
| `auto_control_gate:=true` | `executable 'agx_arm_control_gate' not found` in the installed `agx_arm_moveit` | Gate dropped: the driver runs with `control_enabled:=true` (control path always open, see [docs/05](05_run_real_arm.md) rule 4) |
| Stale `agx_arm_ctrl_single` processes (three at once) | Earlier launches left drivers running; several drivers on one CAN bus fight | The launcher kills stale drivers on start and on exit |
| Pasting a long heredoc into the terminal | The paste was garbled (a stray command inside an `echo` line) | Create long files in short pastes and verify with a checksum |
| Table step ran before MoveIt existed | MoveIt starts 10 to 20 s after the driver | `ARM_WAIT` makes the pipeline wait before adding the table |

4. **Software stop.** The arm has no physical e-stop, so `stop_arm.sh` was written from the driver source (`/emergency_stop` holds the current angles, `/control_enable false` closes the control path, `cancel_goal` calls stop MoveIt).
   `enable_agx_arm false` (motors off) and `move_home` (all joints to zero) are **not** stops and must not be used as such.
5. **Run switches:** `REAL_ARM`, `JOINT_STATES_TOPIC`, `HOME_MOVE_SECONDS`, `TABLE_TOP_Z`, `ARM_WAIT` ([docs/05](05_run_real_arm.md)). Everything else is identical to simulation.
6. **Incident:** once the pipeline reported "reached" on step 0 while the real arm did not move. Most likely cause (not confirmed): the simulated controller had moved but the control path had been closed by an earlier `STOP` test, and the script was reading the simulated joint state instead of the real one.
   **Rule:** after any STOP restart everything, and read the **real** joint topic (`JOINT_STATES_TOPIC=/feedback/joint_states`).
7. A guard (`call_checked`) was added to `run_pick_and_place.sh` as a precaution after a rerun appeared to replay the previous solution. The suspected cause (not confirmed): a perception step answered `success=False` and the script went on with the previous run's latched poses.

## Stage 9. Grasps land short; recalibration (Oct 6 to 7)

**Symptom:** on the real arm the gripper consistently closed about 5 cm closer to the base than the object, while the place height was fine.

**Tried, in order**
1. `OUR_FINGER_REACH` (distance from `tcp_link` to the fingertips). Rejected: the error was flat along the table, not in height.
2. A flat world-frame nudge `GRASP_OFFSET_X` (negative = farther from the base): -0.02, then -0.05, then -0.08. The gripper moved from about 5 cm short to "right in front of the object" but did not reach it.
   A fixed nudge cannot fix an error that depends on the approach tilt, and it hides calibration errors, so it was **reset to 0** once recalibration started.
3. **Recalibration** (Oct 7) with the procedure in [docs/03](03_calibration.md). The new values were about 18 degrees away in rotation from the previous ones and the point cloud was visibly off. **Rejected.**
4. The previous values were restored and edited by hand. A tool for a combined pitch-and-shift correction was written and tested (`launch/tools/calib_adjust.awk`);
   the final values were set directly: translation (-0.37, -0.03, 0.88), rotation a pure pitch `(0, 0.7071, 0, 0.7071)`.
   A hand-typed quaternion `0.7, 0.7` is not unit length (0.990), so it was normalised.
5. Two operational mistakes cost time and are worth knowing: two calibration publishers running at once (the pipeline's and a manual one) make the cloud jump; and a calibration file is read only when the publisher starts.

**Result:** calibration is still **approximate**; a clean recalibration remains open.

## Stage 10. Grasp-quality investigation (Oct 7 to 8)

The grasps proposed for the object were often "sticks poking into the cube" or on one side of it. The complete experiment record with numbers is in
`docs/10_grasp_quality_investigation.md`. Short version:

* **Verified good:** depth/mask/intrinsics (shapes match, no resizing, cube measured 41 mm tall with 2 mm noise, mask on the object).
* **Verified good:** the network on its own sample scenes (good poses, widths that match the objects).
* **Verified not a problem:** the camera angle (training views include straight top-down), the "thin gripper" appearance (a drawing difference: the sample script draws a fixed 8 cm width, our viewer draws the predicted opening), the object type (a solid cube behaves like the carrot).
* **Measured difference:** points per cm^2 on the object are about 35 for our setup versus about 100 in the sample scenes.
* **Not yet tested:** whether doubling the resolution of the depth fixes it (script ready: `launch/tools/upsample.py`); a geometry-based grasp generator (proposed, not built).
* **Status at the time:** unresolved. Grasps were reported working afterwards, see stage 12.

## Stage 11. Inspection tools, candidate ordering, reach limit (Oct 8)

* **Grasp watcher/viewer** (`launch/tools/grasp_watch.py`, `grasp_view.py`): read-only tools that show all candidates, the ones handed to MTC, and where the gripper really closed.
  An early version of the viewer drew each raw candidate as a 5 cm stick from the hand base, which made grasps look "beside the object"; the grasp centre is 10.3 cm along the stick.
  The viewer now draws the full length with a cross at the grasp centre.
* **Candidate ordering:** the table of tilts showed that 6-degree and 12-degree candidates ranked 9th and 10th by score and never reached MTC (only the first 6 pass). MTC then picks the cheapest solution, not the most vertical. **Fix:** sort the usable candidates most top-down first before the cut (`usable.sort(...)`).
* **Place failure after the pick improved:** `place IK cone (0/560): no IK found` for all 560 samples while pick and lift had 14 solutions. The plate target was at `(-0.583, -0.201)`, 0.617 m from the base axis.
  A kinematic check (forward kinematics built from the URDF numbers, validated against RViz's TF to 5 decimals; random-restart IK with the joint limits) found that a downward-pointing gripper reaches only about 0.58 to 0.60 m at that height.
  **Cause: the plate is out of reach**, not the pick change. Keep the plate **under about 0.55 m** from the base axis. The plate target had also moved about 10 cm outward compared with earlier runs, which may be an effect of the calibration change; this was not resolved (measure the real distance with a ruler).

## Stage 12. Grasps working (Oct 8)

After stage 11 the operator reported that **grasps now work extremely well on the real arm**, and, asked which change made the difference, named
**sorting the candidate grasps most top-down first** before they are handed to MTC (`usable.sort(...)` in `mtc_pick_and_place.py`).

**What that means.** Contact-GraspNet had been producing near-vertical grasps all along (in the table that led to the fix, 6 and 12 degrees from straight down),
but they ranked 9th and 10th by the network's score, and `MAX_CANDIDATES_IN_TASK = 6` hands MTC only the first six. So MTC was choosing among six more steeply tilted
grasps (26 to 49 degrees) and executing the cheapest of those, which on a small flat object seen from above tended to close beside it.
With the sort, the near-vertical grasps are among the six, and the arm uses them.

**Why steep grasps are the fragile ones (geometry, not a measurement).** An error along the approach axis (for example in the finger reach, the hand-base to grasp-centre
distance, or the calibration) moves the gripper sideways by error x sin(tilt) and vertically by error x cos(tilt). For a vertical approach the same error only changes
the height; for a 45-degree approach it is a sideways miss. This also explains why a fixed sideways nudge (stage 9) could never fix it.

**Status of the other changes and tests.** The hand-adjusted calibration (stage 9), the plate within reach and the corrected viewer display (stage 11) were also in place or
recommended, and the 2x upsampling test (stage 10) was prepared but its result was not recorded. The attribution above is the operator's judgement; **no controlled
comparison (sort on versus off with everything else equal) was recorded**. The point-density hypothesis of [docs/10](10_grasp_quality_investigation.md) was therefore
neither needed nor tested.

**Lesson:** when grasps look wrong, first print which candidates MTC actually receives and their tilts (`launch/tools/grasp_watch.py` prints them) before suspecting the network, the camera or the calibration.

---

## Corrections: statements made during the project that were later found wrong

This list is here so nobody relies on them.

| # | Statement | What is true |
|---|---|---|
| 1 | The orientations that the early reachability sweep favoured are "straight down" | They were ranked in the mislabeled gripper frame. The verified straight-down quaternion in the corrected frame is `(1, 0, 0, 0)` |
| 2 | Calibration values were "1000 times too small" | They were 1000 times too **large** (`y = -1134` instead of `-1.134`) |
| 3 | Intrinsics `fx = 476.92, cx = 322.46, cy = 200.18` for the grasp viewer | Those are the **native** 640x400 depth intrinsics. With aligned depth (640x480) the correct values are `fx = fy = 488.78, cx = 317.81, cy = 216.85` (what the node reads from `camera_info`) |
| 4 | "Contact-GraspNet was trained on tilted views, so a top-down camera is the worst case" | The training configuration samples camera elevation 30 to 150 degrees where 90 degrees is **straight top-down**, at 0.9 to 1.3 m. Top-down is inside the training range |
| 5 | "The thin grippers in the viewer show the model is wrong" | The sample script draws every grasp at a fixed 8 cm width; our viewer draws the *predicted* opening, which is narrow for a 4 cm cube |
| 6 | "Use `auto_control_gate:=true` for the real arm" | The helper executable was not installed; the gate was removed |
| 7 | Checking for arm feedback once with `ros2 topic echo --once` | The command exits immediately if the topic does not exist yet; use a retry loop |
| 8 | "Fixed offset in x will correct the grasps" | Only partly: the error depends on the tilt direction; the calibration is the real fix |
| 9 | A mistaken test result claimed a 27 mm error in the upsampling script | It was a flaw in the test (comparing off-centre pixels). The script's 3D positions agree to 0.13 mm (median) |

## State at the end of this log (2026-10-08)

* Working: simulation end to end; real arm executes planned motions at 0.1 rad/s; perception chain; planning with candidates sorted by tilt; grasps land on the object (operator report).
* Open: a controlled on/off comparison of the sort (optional), a clean recalibration, place reach (plate under about 0.55 m), safety hardening (no physical e-stop), reproducible install.
  Details and priorities: `docs/09_known_issues_and_next_steps.md`.
