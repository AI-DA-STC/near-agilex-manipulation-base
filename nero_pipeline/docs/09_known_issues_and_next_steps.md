# 09. Known issues and next steps

Ordered by priority. "Evidence" points to where the supporting facts are. Nothing here is a promise; it is the current
understanding of what blocks a reliable system.

> **Update 2026-10-08 (late):** the operator reports that grasps now land correctly on the real arm. Items 1 and 2 below were written
> before that report; read them as *what to do if grasp accuracy regresses* and as *why it is worth making the fix reproducible*,
> not as open blockers. Safety (item 3) is unchanged.

> **Update 2026-10-14:** perception is fast, a wrong frame label was fixed (grasps and plate had been about 3.3 cm off sideways), grasps are re-centred on the object, and the operator reports that the real arm worked with the corrected grasp, place and table collision
> ([docs/11](11_perception_workers_frames_recentring.md)). Item 2 below now matters more (the calibration was tuned while the frame label was wrong), and item 6 is partly answered (the real table is at z = 0). New open items are in **P4**.

## P0. Highest priority (items 1 and 2 were reported fixed, see the note above; item 3 is open)

### 1. Grasps beside the object (fixed by sorting candidates most top-down first)
* **Observed (before the fix):** the gripper often closes beside the object. In the grasp viewer many proposals are "sticks poking into" the object or on one side;
  a plain solid cube behaves like the carrot, so it is not about the object.
* **Evidence:** [docs/10](10_grasp_quality_investigation.md). Inputs verified good; network good on its sample scenes; difference measured: about 35 points per cm^2 on our objects versus about 100 in the sample scenes.
* **Fix (operator's attribution, [docs/07](07_development_log.md) stage 12):** the candidates handed to MTC are sorted most top-down first. **If it regresses, check first** that `usable.sort(` is in `mtc_pick_and_place.py` and run `launch/tools/grasp_watch.py` to see the tilts MTC receives. **Then try, cheapest first:**
  1. Run the 2x upsampling test (`launch/tools/upsample.py` then the viewer on `/tmp/up_*.npy`). If grasps improve a lot, build the upsampling into `contact_graspnet_ros`.
  2. Run the grasp watcher with execution and check whether the arm follows the generated grasp when it closes (`launch/tools/grasp_watch.py`): separates "bad proposals" from "bad conversion/calibration".
  3. Try a closer or higher-resolution view for grasp detection (for example the arm camera) to test the density hypothesis directly.
  4. Try the other Contact-GraspNet checkpoints (the original repository ships variants trained for clean or noisy depth; the PyTorch port ships one).
  5. Build a **geometry-based grasp generator** for simple objects (cube, roll, mug): centre and orientation from the mask and depth, approach straight down, jaws across the object, published in the same message format. Proposed, not built.
  6. AnyGrasp when the licence arrives.

### 2. Calibration was set by hand
* The calibration in use was set by hand ([docs/03](03_calibration.md)) and works in practice. Nobody can reproduce it from data, and a systematic error would shift every grasp and place pose.
* **Next:** a clean recalibration with the procedure in [docs/03](03_calibration.md), then a **physical touch test**: command the gripper to a point on the table measured with a ruler and compare. Keep the old file as a backup.

### 3. Safety on the real arm
* No physical e-stop. The software stop takes about a second and needs ROS and the driver alive. **A hardware e-stop (or a power switch within reach) is the first thing to add.**
* The table's edges are not modelled; the held object has no collision geometry; the control path is always open (no gate); the arm must be at `home` at launch (the simulated controller starts there).
* **Next:** model the table boundary box; attach a real collision shape to the held object; automate the start check (compare `/control/joint_states` and `/feedback/joint_states`, refuse to continue if they differ).

## P1. Needed for dependable operation

| # | Issue | Evidence | Proposed work |
|---|---|---|---|
| 4 | Place fails when the plate is more than about 0.55 m from the base | [docs/07](07_development_log.md) stage 11: reach limit about 0.58 to 0.60 m for a downward gripper at plate height | Make `run_pick_and_place.sh` check the plate radius and stop with a clear message; document the workspace in the README; consider letting the place cone tilt toward the arm |
| 5 | MTC chooses the cheapest solution, not the best grasp | [docs/01](01_system_overview.md) section 4 | Add a cost term (tilt, score) or restrict the candidate set to a vertical cone with fallback |
| 6 | Table top height is a guess (`TABLE_TOP_Z`) | [docs/05](05_run_real_arm.md) | Measure the real table height in `base_link` and set it; expect more collision failures when it is correct (the tuning was done at -0.10 m) |
| 7 | The firmware check in the driver sometimes fails at start | [docs/07](07_development_log.md) stage 8 | Collect occurrences; if it persists pass a fixed `fw_version` (profiles `v111`, `v112`, `v120`, `v121` for the Nero; the arm reported `v111`) |
| 8 | Controller sync assumption (simulated controller starts at `home`, real arm must match) | [docs/05](05_run_real_arm.md) rule 3 | Automate the check (see item 3) or make the controller start from the real feedback |

## P2. Engineering quality

| # | Issue | Proposed work |
|---|---|---|
| 9 | **Timing is patched, not planned.** OMPL and gripper segments have no timestamps; `retime_solution()` adds them just before sending | Use a proper time-parameterisation stage in MTC (needs a working Pilz or a time-parameterisation plugin) |
| 10 | The RViz panel preview is not the executed motion | Document (done); optionally a replay tool that plays the retimed trajectory |
| 11 | `gs_usb` is not loaded automatically at boot | Install the module into `/lib/modules/$(uname -r)/` and run `depmod`, or add an `insmod` systemd unit. Not tested |
| 12 | Installation is manual | Capture the environment (`environments/`), then a setup script or a container for the perception side |
| 13 | No automated tests | Unit tests for the pure functions (`to_tcp_pose`, candidate selection, retiming) already exist in spirit as the checks in `launch/tools/`; make them part of a CI run |
| 14 | Packages must be pushed as submodules | Commit and push each of our packages, then update the umbrella repository pointers (`maintainer/PUSH_CHECKLIST.md`) |
| 15 | The object is always called `carrot` internally | Rename to a parameter (cosmetic, but confusing for newcomers) |
| 16 | `MODE` switch (`mode.sh`) was proposed but **not** adopted | The run mode is selected by environment variables ([docs/05](05_run_real_arm.md)); remove any mention of `mode.sh` you find |

## P3. Later

* AnyGrasp integration once the licence arrives.
* A second place target type, multiple objects, re-grasp on failure.
* Closed-loop correction with the arm camera.

## P4. Added 2026-10-14

| # | Issue | Proposed work |
|---|---|---|
| 17 | `launch/scripts/add_table_collision.py` builds a box 0.2 m thick, so its top face is 0.095 m above `TABLE_TOP_Z`, with a 2 m x 2 m footprint that overlaps the arm's base; `start_pipeline.sh` still calls it ([docs/11](11_perception_workers_frames_recentring.md) section 4) | Call `add_table_box.py --top_z 0.0` instead (or fix the script); read the live box back with `--show` and record it |
| 18 | The fingers' collision geometry is unverified (the canary named only `gripper_base`) | A canary box that only the fingers can touch (top between the hand-base height and the tip height at the grasp); add finger collision links if they are missing |
| 19 | The cube and the plate are not in the planning scene | Add them as collision objects (the object is already attached under the name `carrot` at pick) |
| 20 | Intermittent `no IK found`; MTC tries one orientation per candidate (`orig` is off on purpose, one gripper direction) | Several roll angles per candidate (a compact object can be grasped at any roll; `ik_probe.py` sweeps them), more than six candidates, or a start pose with more IK freedom. Keep the one-direction policy in any roll set |
| 21 | Contact-GraspNet proposes rim pinches for boxes | A geometric top-down candidate for compact objects (centre and orientation from the mask's top face) as an extra candidate; or another grasp network (AnyGrasp, see item 1) |
| 22 | An unregistered depth image is silently accepted (the mask is resized onto it) | Refuse with a clear message when mask and depth shapes differ, or reproject the mask through the depth-to-colour TF |
| 23 | The closing depth is a constant (0.006 m) tuned for a 4.8 cm box | Derive it from the object's height and the finger geometry; check the fingertip clearance in the node |
| 24 | The hand-eye calibration and the table height were not re-verified numerically after the frame change | A touch test with a ruler (item 2); record the numbers |
| 25 | The silent arm on 2026-10-13 was not diagnosed | Next time record the number of CAN frames in 3 s (`candump can_piper`), the driver log and the power state |
| 26 | A stuck ROS daemon, a camera without registration and stale build copies each cost time | Add the three checks to `start_pipeline.sh` (daemon health, depth `640x480`, which-file check) and refuse to start otherwise |
| 27 | The prompt question at run start is built but not installed | Install `future_work/ask_prompts` ([future_work/README.md](../future_work/README.md)) and use it for a few runs |
| 28 | Re-centring and the 0.006 depth were tested on one 5 cm box only | Test other objects (a mug, a roll of tape) and record closing heights and results |
| 29 | The first move of every session (`home` to `initial`) is a straight line in joint space without collision checking; only a geometric proxy (closest arm parts 0.229 m apart) says it is safe ([docs/11](11_perception_workers_frames_recentring.md) section 5). Joint 4 of `initial` is at its limit unless `--joint4 2.08` was used | Plan that move with MoveIt (or add a via point), and give joint 4 margin; until then do the RViz Plan & Execute move once per session ([docs/12](12_current_run_sequence.md) 4.6) |

## Questions for the supervisor

1. Is the AnyGrasp licence still expected, and when?
2. Can a hardware e-stop be fitted before more real-arm experiments?
3. Is a second, closer camera for grasp detection acceptable (it would test the density hypothesis directly)?
4. Should the plate stay a fixed target, and if so can its position be fixed within reach (under about 0.55 m)?
