# 01. System overview

Read this first. It explains what runs, what talks to what, the coordinate frames, and why the
pipeline is built the way it is.

## 1. Hardware

| Item | Detail |
|---|---|
| Arm | AgileX Nero, 7 DOF, with the AgileX parallel gripper (`agx_gripper`). Firmware reported by the driver: `v111` (1.11) |
| Computer | NVIDIA Jetson, Ubuntu 22.04, JetPack 6.2, CUDA 12.6, kernel `5.15.199-tegra` |
| Top camera (`topcam`) | Orbbec Dabai DC1, mounted above the table looking down. This is the camera used by the pipeline |
| Arm camera (`armcam`) | Intel RealSense Depth Camera 405, mounted on the arm. Used early in the project, not used by the current pipeline |
| CAN adapter | USB-CAN adapter `1d50:606f` (Geschwister Schneider / `gs_usb`). Needs a kernel module that we had to build, see [docs/02](02_install_and_environment.md) |
| Network/IDs | `ROS_DOMAIN_ID=7` (set in `~/.bashrc`) |

The arm has **no physical emergency stop**. See `docs/05_run_real_arm.md`.

## 2. Software components

### 2.1 Third-party software (not ours)

| Component | Role |
|---|---|
| ROS 2 Humble | Middleware |
| MoveIt 2 | Motion planning; `move_group` |
| MoveIt Task Constructor (MTC) with Python bindings | Task-level planning, `from moveit.task_constructor import core, stages` |
| `agx_arm_ros` (AgileX) | Arm driver `agx_arm_ctrl`, MoveIt configuration `agx_arm_moveit`, URDF `agx_arm_description`. We modified some configuration files, see `config_snapshots/` and `patches/` |
| `pyAgxArm` (AgileX Python SDK) | Used by the driver to talk CAN |
| `orbbec_camera` | Camera driver for the Dabai DC1 |
| `easy_handeye2`, `aruco_opencv` | Hand-eye calibration and marker detection |
| Grounded-SAM2 | Text-prompted segmentation (conda env `grounded_sam2`) |
| Contact-GraspNet (PyTorch port) | Grasp proposals (conda env `contact_graspnet`, folder `~/contact_graspnet_pytorch`) |

### 2.2 Our packages (own repositories, submodules of the umbrella repository)

| Package | Role | Key interface |
|---|---|---|
| `gsam2_ros` | Wraps Grounded-SAM2 as a ROS node | Service `/gsam2/segment` (`std_srvs/Trigger`); parameters `source_prompt`, `target_prompt`, `box_threshold`; publishes object and plate masks and annotated images |
| `contact_graspnet_ros` | Wraps Contact-GraspNet | Service `/contact_graspnet/detect_grasps` (`Trigger`); publishes `/contact_graspnet/grasps` (`PoseArray`, camera frame, best score first). Saves `/tmp/last_depth.npy` and `/tmp/last_segmap.npy` on every call (our addition, used by `launch/tools/`) |
| `target_pose_ros` | Plate pose from mask and depth (top surface) | Service `/target_pose/extract`; publishes `/target_pose/pose` |
| `pose_transform_ros` | Camera frame to `base_link` with `tf2` | Service `/pose_transform/transform_all`; publishes `/grasp_poses_transformed` (all candidates) and `/target_pose_transformed`, both **latched** (transient local) |
| `nero_mtc_task` | The MTC pick-and-place task | Script `mtc_pick_and_place.py`, started by `launch/run_pick_and_place.sh` through `mtc_pick_and_place.launch.py` |

### 2.3 Environments (why `conda deactivate` appears everywhere)

* **System Python / ROS**: everything that imports `rclpy`, including MTC. Conda's Python (3.14 in the `base`
  environment) cannot import `rclpy`; this single fact caused many failures during the project.
* **Conda `grounded_sam2`** and **`contact_graspnet`**: PyTorch (a Jetson-specific wheel) for the two neural
  networks. The ROS nodes `gsam2_ros` and `contact_graspnet_ros` call the networks as subprocesses in these envs.
* Helper tools that need Open3D or PyTorch say `conda activate contact_graspnet` explicitly.

## 3. Coordinate frames

```
base_link  ---- (hand-eye calibration, TF from easy_handeye2) ----> topcam_link ----> topcam_*_optical_frame
   |
   +-- joint1 ... joint7 --> link7 --(tcp_offset)--> tcp_link
```

* `base_link`: arm base. Targets of the workspace have **negative x** (the workspace is on the
  -x side). "Farther from the arm" means more negative x.
* `topcam_link`: the camera's root frame. The calibration was expressed relative to this frame (not to the optical
  frame) because the camera driver already owns the TF parent of the optical frames; see the log, stage 1.
* `tcp_link` is **the gripper-base frame**, defined by launching MoveIt with
  `tcp_offset:="[0.037, 0, -0.0235, -1.5708, 0, -1.5708]"` (xyz in metres, rpy in radians, relative to `link7`).
  * +Z of `tcp_link` is the **approach direction** (fingers point along +Z).
  * The jaws open and close along the **Y** axis.
  * The finger bodies span z = 0.062 m to 0.138 m (measured from the AgileX Piper finger meshes, whose joint numbers
    match the Nero's); the fingertips are at z = 0.138 m.
  * **Pointing straight down** means `tcp_link` is rotated 180 degrees about X: quaternion `(x, y, z, w) = (1, 0, 0, 0)`.
    This was confirmed visually, see the log, stage 4.
  * This frame had been wrong for a long time (it was attached to `link7` with no rotation, 90 degrees away
    from the real gripper axis). It must be present in **both** the simulation's/`move_group`'s robot model
    **and** MTC's own copy of the robot model. See `docs/08_troubleshooting.md`, "gripper points sideways".

## 4. The MTC task, stage by stage

Implemented in `nero_mtc_task/mtc_pick_and_place.py` (a copy for reading is in `reference_copies/`).

| # | Stage | Solver / method | Notes |
|---|---|---|---|
| 0 | `current` | CurrentState | |
| 1 | `connect to pregrasp` | Connect, OMPL | Free-space path to the pregrasp pose |
| 2 | `pregrasp IK options` | Alternatives of (GeneratePose, ComputeIK) | One entry per grasp candidate. Candidates are filtered by tilt (`MAX_TILT_DEG`), converted to `tcp_link` poses, **sorted most top-down first** (when the `usable.sort` line is present, see [docs/06](06_tuning_reference.md)) and cut to `MAX_CANDIDATES_IN_TASK` |
| 3 | `open gripper` | MoveTo named state `gripper_open` | |
| 4 | `approach` | MoveRelative, **Cartesian straight line** along `tcp_link` +Z (min 0.02 m, max 0.08 m) | The straight-line solver also gives a useful failure message |
| 5 | `allow gripper-object contact` | ModifyPlanningScene | |
| 6 | `close gripper` | MoveTo named state `gripper_close` | |
| 7 | `attach carrot` | ModifyPlanningScene | The object is always called `carrot` internally, whatever you grasp. It has **no collision geometry** (a known limitation) |
| 8 | `lift` | MoveRelative world +Z, OMPL | 0.02 to 0.05 m |
| 9 | `transit` | Connect, OMPL | |
| 10 | `place IK options` | GenerateRandomPose + ComputeIK | A *continuous* cone of orientations around straight down (+-32 degrees about each axis, up to 40 samples, 20 IK solutions each) at the plate position plus `PLACE_Z_CLEARANCE_M` |
| 11 | `detach carrot`, `release gripper` | | |
| 12 | `retreat` | MoveRelative world +Z, OMPL | 0.02 to 0.15 m |

MTC explores every alternative and returns the **cheapest complete solution**; we have not changed its cost terms, and in our runs the cost follows the length of the motion.
It does not prefer the highest-scoring grasp or the most vertical one. The candidate ordering and the tilt cap are
therefore our only levers on which grasp is used, see `docs/06_tuning_reference.md`.

## 5. Execution path

```
task.plan()  ->  solution message  ->  retime_solution()  ->  ExecuteTaskSolution action (move_group)  ->  controller
```

Why we do not call `task.execute()` and do not use the RViz panel's Execute button:

1. `task.execute()` discards the real error code on failure (MTC `task.cpp`, around line 334) and returns the placeholder
   `val=99999` (`MoveItErrorCodes::FAILURE`). We call the action directly to see the real code.
2. The planner output has **no usable timestamps** for OMPL segments and for gripper moves (every point has
   `time_from_start = 0`). The controller rejects such trajectories ("Time between points 0 and 1 is not strictly
   increasing"). `retime_solution()` recomputes timestamps for every segment from one set of speed limits
   (`ARM_MAX_VEL`, `ARM_MAX_ACC`, `GRIPPER_MAX_VEL`, `GRIPPER_MAX_ACC`). This is also the **single place that sets the
   executed speed**.
3. The RViz panel replays and executes the raw plan without this retiming. Its preview is therefore fast, and its Execute
   button does not work reliably.

## 6. Simulation versus real arm

| | Simulation | Real arm |
|---|---|---|
| Selected by | default | `REAL_ARM=1` before `start_pipeline.sh` |
| Arm launcher | `launch/scripts/launch_arm.sh` (MoveIt demo with fake hardware) | `launch/scripts/launch_arm_real.sh` (driver, then the same MoveIt demo with `follow:=true`) |
| Joint state topic used by scripts | `/control/joint_states` (default) | `/feedback/joint_states` (`JOINT_STATES_TOPIC`) |
| Start-pose move (`pose_90deg`) duration | 4 s | 15 s (`HOME_MOVE_SECONDS`) |
| Table collision top face | z = -0.10 m (default) | `TABLE_TOP_Z=-0.03` (set by you) |
| Start condition | arm model starts at `initial_positions.yaml` (`home`) | the physical arm must already be at `home` |

Everything else (cameras, perception nodes, calibration publisher, MTC, retiming) is identical.

## 7. Main design decisions and the reasons

| Decision | Reason |
|---|---|
| Give MTC all candidates as `Alternatives` instead of pre-checking IK ourselves | 7-DOF IK depends on the starting joint configuration. Our external checks and MTC's own stages disagreed about the same pose |
| OMPL only (CHOMP package removed) | CHOMP crashed (segfault at `ChompOptimizer::initialize`) and was selected by the plugin loader even when OMPL was requested. Pilz is also broken on this system (missing library) |
| Straight-line (Cartesian) solver for `approach` | The OMPL pose-goal version failed for every candidate with no message; the Cartesian solver succeeds and reports how far it got |
| Retime every segment ourselves | See section 5 |
| Real arm: driver and MoveIt launched separately (`launch_arm_real.sh`) | AgileX's combined launch file does not forward the `capabilities` argument, so `execute_task_solution` would not exist |
| `ROS` commands with conda deactivated | See section 2.3 |
