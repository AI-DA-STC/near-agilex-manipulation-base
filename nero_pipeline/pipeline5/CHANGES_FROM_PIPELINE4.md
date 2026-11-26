# MoSAIC-DUO (pipeline 5) — what changed from pipeline 4

**MoSAIC-DUO: Modular Skills for Adaptive Interpretable manipulation Control with Dual-camera Unified Observation**

Version described: **8e** (`nero_pipeline/pipeline5/VERSIONS.txt`). Pipeline 4 (v10) stays runnable as the baseline; pipeline 5 is independent of it
(its `core/` holds its own copies, checksummed in `core/COPIED_FROM_PIPELINE4.txt`).

In one line: **GraspMolmo picks where to grab and where to place, SAM2 cuts out the object, our grasp step works out how the Nero gripper should hold
it, and MoveIt Task Constructor plans the motion around the obstacles both cameras see.**

---

## 1. Grasping: Contact-GraspNet replaced by a geometric grasp step for the Nero gripper

| Pipeline 4 | Pipeline 5 |
|---|---|
| Contact-GraspNet (trained for the Franka Panda gripper) proposed grasps; GraspMolmo's point picked one | No grasp network. Grasps are **built from GraspMolmo's point** and the object's measured shape (`p5_grasp_core.py`) |
| Fingertips often planned below the table; pipeline 3 lifted them 12–34 mm, so the hand closed above the object | Fingertips placed **6–10 mm above whatever the object stands on** (table or sink floor, measured around the object) |
| Tilts 25–46°, rolls 15–29°; no check that the grasp fits *our* gripper | Jaw line level, tilt ≤ 20° across the jaws; every candidate checked against the **measured Nero gripper** |
| "No grasps found" after the first plan of a session (cause never found) | Gone (no Contact-GraspNet) |

What the grasp step does, per look:
- jaw across the object's narrow side (plus the long side and every 15°), opening = object width + 2 cm (≤ 10 cm);
- variants: 0/±10/±20° tilt, ±1 cm along the object, 5/10 mm higher, jaws up to 8 mm off-centre (room beside a wall);
- checks: fingers (6 stacked boxes following the measured taper), gripper body and **wrist camera box**, also along the 8 cm approach, against
  the top camera's points (treated as columns: what is seen from above blocks everything below it) and the fused two-camera cloud;
  the pads must cover at least half the object's height;
- the best 3 distinct grasps go to MoveIt with one common opening.

Gripper model (`p5_gripper.py`, measured): finger width 20→55 mm, thickness 7→25 mm, grip face 70 mm, body 165 × 55 mm, camera + mount + cable
8 × 8 × 8 cm beside the gripper (placement checked against the eye-in-hand calibration). Fingertip distance set per machine in
`~/.p5_gripper.yaml` (`tip_z`).

## 2. Object segmentation: SAM2 at GraspMolmo's point

- GraspMolmo only gives a **point**. Pipeline 4 used "every raised pixel within 8 cm of it" as the object — in a sink that swallowed the walls
  (measured objects of 140–250 mm).
- Pipeline 5 runs **SAM2** (pipeline 1's model, its own worker) clicked at that point and takes SAM2's **best-rated** proposal (score ≥ 0.3, not
  the sink/table, not touching the image border). GraspMolmo's rough region remains a fallback and is refused when it is clearly merged.

## 3. Placing: the place point is moved to where the gripper fits

- New step `/p5/place` (`p5_place_core.py`): SAM2 finds the whole container at GraspMolmo's place point; the release point is moved (≤ 8 cm) to
  where the fingers, body and camera fit at a release yaw MTC will use.
- The top camera's obstacle boxes are **rebuilt around the new point**. In pipeline 4 the 6 cm "landing zone" around GraspMolmo's own point hid
  the basket wall right next to it, so the gripper pushed the basket.
- At release the gripper opens only to the object's width + 1 cm (was the full grasp opening, which pushed a finger into the container's edge).
- If lowering into the container is blocked, the gripper stops there and lets go (`P5_DESCEND_MIN_M`, default 1 cm of lowering).

## 4. MoveIt's scene knows more

- The **object in the gripper** is modelled (4 slice boxes from SAM2's points, never below the table) and attached at the grasp, so the lift,
  transit and place are collision-checked with it. (MTC attached an object called "carrot" since pipeline 2, but it never existed.)
- The **wrist camera box** is attached to the gripper.
- The gripper may touch the object model from just before opening (allowed by the finger **link names**; pipeline 3's allowance used a group name
  MTC read as a link and never applied).
- The wrist-camera boxes leave out the gripper's already-checked path (+3 cm): coarse floor boxes no longer stop the approach 2 cm early.
- The approach must cover the full 8 cm or the plan fails (pipeline 4 accepted ≥ 2 cm and could close above the object without saying so).

## 5. Running it

| Pipeline 4 | Pipeline 5 |
|---|---|
| `p4_plan.sh`, then `p4_execute.sh` asked **EXECUTE** and **planned again** before moving | `p5_go.sh "<object>" "<place>"`: look, plan and execute in one go. Or `p5_plan.sh` (saves the plan) then `p5_execute.sh` (runs **that** plan: no re-planning, no typing; refuses if the arm moved, the plan is old or from an earlier look) |
| — | `p5_replan.sh`: plan again with the same look (`P5_GO=1` executes) |
| CAN brought up by hand | `start_pipeline5.sh` checks the CAN link (brings it up if one adapter), then that the arm publishes `/feedback/joint_states` |
| Ctrl+C could hang | Ctrl+C stops everything within 10 s (forced after that) |
| Arm 0.3 rad/s | Arm **0.5 rad/s** per joint (`P5_ARM_MAX_VEL`) |

## 6. Seeing what it does

- **One picture window**, which stays where you drag it, shows three pictures in turn: **1/3 GraspMolmo's pick point**, **2/3 SAM2's object + the
  place point** (GraspMolmo's and the one used), **3/3 the grasps** (G1 green, G2/G3 yellow, rejected red, with what blocked them).
- **Concise terminal**: one line per step with its start time and duration, one `ok` / `FAILED` line per service, failure reasons kept.
  The log file (`~/p3_runs/go_logs/`) keeps every line; `P5_VERBOSE=1` prints everything.
- Every look is saved (`~/p3_runs/p5_snaps/`); `p5_replay.py` reruns the grasp step offline on it and shows the picture.

## 7. Tests

`tests/run_tests.sh`: 9 offline suites (≈ 210 checks, no robot) — the grasp step on synthetic scenes and on a **real sink look** (kept in
`tests/data/`), the place step, the keep-out corridor, the scene objects, the run flow and the terminal/picture display, plus the bundle
integrity (copies unchanged, nothing points back to pipeline 4). Installing runs them.

## 8. Known limitations / next

- Near a container's rim the held object model can block the release (`place IK cone … carrot`, `place descend`): the place step checks the
  gripper but not yet the object's own footprint. Workaround: `P5_OBJECT_BOX=off`. Fix tested offline, not yet in 8e.
- At a jaw angle near 45° a coarse wrist box can still reach into the approach (the +3 cm keep-out margin is only exact along the voxel grid;
  a diagonal-safe 4.5 cm margin is tested offline, not yet in 8e).
- MTC searches for up to 100 solutions and then picks the smoothest; a lower cap (tested earlier) would save 10–25 s.
- Speed: GraspMolmo ~19–30 s per question (pick and place asked separately); wrist obstacles ~14–19 s; MTC ~20–45 s.
- Tier 2/3 grasps (tilts beyond 20°, side grasps), grasp verification with the wrist camera, and a comparison with Contact-GraspNet/GraspGen
  are not done yet.

## Settings (environment, set before the command)

`P5_MAX_TILT_DEG` `P5_OPEN_MARGIN_M` `P5_GRASPS_TO_MOVEIT` `P5_TIP_CLEARANCE` `P5_MASK=sam2|depth` `P5_SAM2_MODEL` `P5_PLACE=on|off`
`P5_CAMERA_BOX` `P5_OBJECT_BOX` `P5_KEEPOUT` `P5_RELEASE_EXTRA_M` `P5_DESCEND_MIN_M` `P5_ARM_MAX_VEL` `P5_POPUP` `P5_VERBOSE`
`P5_CLOUD_RVIZ` `REAL_ARM` — per machine: `~/.p5_gripper.yaml` (`tip_z`, camera box), `~/.p5_offsets.yaml`, `~/.p3_table_z`.
