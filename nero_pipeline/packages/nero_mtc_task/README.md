# nero_mtc_task

Task planning stage: builds and executes a MoveIt Task Constructor (MTC)
pick-and-place task using the **real, perception-detected** grasp and
target poses - not MTC's built-in automatic pose sampling. Final stage of
the perception + task planning pipeline, consuming
[`pose_transform_ros`](https://github.com/rarlyx/pose_transform_ros)'s
output directly.

## Honesty check before you run this

This is a first-draft adaptation of MoveIt Task Constructor's own official
tutorial (`pickplace.py`), rewritten against **individually verified**
Python binding signatures (checked directly against
`moveit_task_constructor`'s Humble-branch source, not written from memory)
- but it has **not been run end to end** against the real Nero arm or
planning scene. Expect a debugging round, the same as every other package
in this pipeline needed on its first real run.

One specific, deliberate design choice worth knowing before debugging:
this does **not** use MTC's higher-level `Pick`/`Place`/`SimpleGrasp`/
`SimpleUnGrasp` composite classes. Those expect their wrapped pose
generator to expose `pregrasp`/`grasp` properties (which `GenerateGraspPose`
has, since it's designed for automatic sampling) - but we're using
`GeneratePose` instead (a plain, fixed-pose generator, since we already
have one real detected pose, not many candidates to search over), which
does not have those properties. Rather than risk relying on undocumented
behavior from that mismatch, this task is built from the individually
verified lower-level stages instead (`ComputeIK`, `MoveTo`, `MoveRelative`,
`ModifyPlanningScene`, `Connect`, `CurrentState`) - more verbose, but every
piece is confirmed to actually exist with the exact property names used.

## Known limitation: no modeled collision object for the carrot

This pipeline never adds an actual `CollisionObject` for the segmented
carrot into the planning scene (that would need shape/size info this
pipeline doesn't currently extract). The `allowCollisions`/`attachObject`/
`detachObject` calls in this script reference an object named `"carrot"`
that doesn't exist in the scene yet - they'll silently no-op rather than
error. Practically: the gripper will still physically hold the object once
closed, but MTC's planning scene won't "know" about it as an attached
body, so collision checking during transit won't account for the carrot's
own geometry. Adding a real collision object (even a rough placeholder box
sized from the segmentation mask) is a natural next improvement.

## Setup

```bash
cd ~/agx_arm_ws/src
cp -r nero_mtc_task .
cd ~/agx_arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select nero_mtc_task
source install/setup.bash
```

Requires `moveit_task_constructor` already built in your workspace (per
your earlier install).

## Usage

**1. Have MoveIt running** (sim first, always, before real hardware):
```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=agx_gripper
```

**2. Run the full perception chain** (as already established):
```bash
ros2 service call /gsam2/segment std_srvs/srv/Trigger
ros2 service call /contact_graspnet/detect_grasps std_srvs/srv/Trigger
ros2 service call /target_pose/extract std_srvs/srv/Trigger
ros2 service call /pose_transform/transform_all std_srvs/srv/Trigger
```
Confirm the final response shows real, sane-looking positions in
`base_link` before continuing.

**3. Run this task**
```bash
ros2 launch nero_mtc_task mtc_pick_and_place.launch.py
```
This is a **launch file**, not a bare `ros2 run` - MTC's `PipelinePlanner`
and `ComputeIK` stages need the full MoveIt config (kinematics.yaml,
ompl.*, robot_description) on the node to actually solve IK and plan,
exactly like `move_group` needs it. The launch file reuses
`agx_arm_moveit`'s own config builder to provide this (same proven pattern
already used for `gsam2_ros`'s MoveItPy integration) - a bare `ros2 run`
of the script fails with `No kinematics solver instantiated for group
'arm'`.

This reads `/grasp_pose_transformed` and `/target_pose_transformed` once,
builds the MTC task, plans, and (if planning succeeds) executes it.

**If planning fails**, open RViz's **MotionPlanning Tasks** panel (a
separate panel from the regular MotionPlanning one, provided by
`moveit_task_constructor_visualization`) to see which specific stage
failed and why - this is genuinely the intended debugging tool for MTC,
much more informative than the script's own console output alone.

## Tunable constants (top of the script)

- `PREGRASP_OFFSET_M` / `PREPLACE_OFFSET_M` - how far above the real
  grasp/target pose to approach from before the final linear descent
- `APPROACH_MIN/MAX`, `LIFT_MIN/MAX`, `PLACE_MIN/MAX`, `RETREAT_MIN/MAX` -
  distance ranges (meters) for each linear motion stage

These are first-guess defaults (a few centimeters each) - adjust based on
what you actually observe when planning fails or looks physically wrong
in RViz.

## Pipeline (full chain, all repos)

```
gsam2_ros (segment source+target)
    -> contact_graspnet_ros (grasp pose, source only)
    -> target_pose_ros (target pose, position only)
    -> pose_transform_ros (both poses -> base_link, via eye_on_base calibration)
    -> nero_mtc_task (this repo: task planning + execution)
```
