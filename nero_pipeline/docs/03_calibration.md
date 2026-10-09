# 03. Hand-eye calibration

The calibration tells the system where the top camera is relative to the arm. **Every grasp and place target in
`base_link` depends on it**, so a calibration error shows up as the gripper closing "beside" the object even when
the grasp proposal itself is fine.

**Current state: hand-adjusted and working in practice.** The calibration in use was adjusted by hand (rotation and translation edits) after a
recalibration attempt on 2026-10-07 turned out worse than the previous values; the operator reports that grasps now land correctly on the real arm.
It was not produced by a clean recalibration run, so it cannot be reproduced from data. A clean recalibration is a recommended task
([docs/09](09_known_issues_and_next_steps.md)).

## 1. What is calibrated

* Type: `eye_on_base` (the camera is fixed in the room; the marker is on the arm).
* Result: the transform `base_link` to `topcam_link`, stored in
  `~/.ros2/easy_handeye2/calibrations/nero_eye_to_hand_calib.calib` (a copy is in
  `config_snapshots/calibration/`).
* It is published as TF by `ros2 launch easy_handeye2 publish.launch.py name:=nero_eye_to_hand_calib`, which
  `launch/start_pipeline.sh` starts. The camera driver publishes `topcam_link` to the optical frames itself.
* The file is **read only when the publisher starts.** Editing the file changes nothing until you restart the
  publisher (section 4).

File format (values are examples):

```yaml
parameters:
  name: nero_eye_to_hand_calib
  calibration_type: eye_on_base
  robot_base_frame: base_link
  robot_effector_frame: gripper_base
  tracking_base_frame: topcam_link
  tracking_marker_frame: marker_0
  freehand_robot_movement: true
  move_group_namespace: /
  move_group: manipulator
transform:
  translation:
    x: -0.37
    y: -0.03
    z: 0.88
  rotation:
    x: 0.0
    y: 0.7071068
    z: 0.0
    w: 0.7071068
```

## 2. History of the values (so you can judge how much to trust them)

| When | `translation x, y, z` (m) | `rotation x, y, z, w` | Status |
|---|---|---|---|
| Late September | -0.38599903, -0.06282647, 0.90639605 | 0.00860897, 0.70307473, 0.02821910, 0.71050369 | "Final" calibration after a yaw correction by hand; the point cloud matched the arm model in RViz |
| Before 2026-10-07 (the `.bak_before_recal` file) | -0.38599903, -0.06282647, 0.86639605 | 0.00860897, 0.70307473, 0.0, 0.71050369 | The value above after two more small hand edits (z and rotation z) |
| 2026-10-07 recalibration attempt | -0.39739135, -0.01952748, 0.87542579 | 0.01450662, 0.59152443, -0.05742954, 0.80410836 | Rotation about 18 degrees away from the previous one; the point cloud was visibly off. **Rejected** |
| Last value seen in use (2026-10-08) | -0.37, -0.03, 0.88 | 0, 0.7071068, 0, 0.7071068 | Hand-set (pure pitch, quaternion normalised). Approximate |

Independent checks that this chain is mathematically consistent (these do **not** prove the calibration is accurate):
* The transform camera to `base_link` in `pose_transform_ros` was verified by hand calculation to three decimals.
* Depth and color camera intrinsics are identical and correct for the aligned depth (see [docs/02](02_install_and_environment.md) section 2.5).
* The object height measured from depth is right (a 4 cm cube measures 41 mm above the table, noise 2 mm), so the depth is metric.

## 3. How to check the calibration

1. **RViz overlay (main test).** With the arm model following the real arm and the point cloud shown, the arm in the
   cloud and the arm model must coincide. Compare the gripper and the base.
2. **TF values.** `ros2 run tf2_ros tf2_echo base_link topcam_link` (conda deactivated) must print the numbers in the file.
   "Could not find a connection" means no publisher is running.
3. **Physical test.** Measure with a ruler the distance from the arm base axis to the plate and compare with the radius the
   pipeline prints (`sqrt(x^2 + y^2)` of `/target_pose_transformed`):

```bash
conda deactivate
ros2 topic echo /target_pose_transformed --once --field pose.position | awk '/x:/{x=$2} /y:/{y=$2} END{printf "plate radius from the base axis: %.3f m\n", sqrt(x*x+y*y)}'
```

## 4. Adjusting by hand

Always keep a backup first:

```bash
C=~/.ros2/easy_handeye2/calibrations/nero_eye_to_hand_calib.calib
cp $C $C.bak_$(date +%H%M%S)
nano $C                       # edit translation x,y,z (metres)
```

Reload (kill the old publisher first, **two publishers of the same transform make the cloud jump**):

```bash
pkill -f "easy_handeye2 publish.launch.py"; pkill -f handeye_publisher; sleep 2
conda deactivate; source ~/agx_arm_ws/install/setup.bash
ros2 launch easy_handeye2 publish.launch.py name:=nero_eye_to_hand_calib
```

Rules for rotations:
* A quaternion must have length 1. A hand-typed `0.7, 0.7` has length 0.99 and TF may reject larger errors.
  Check and normalise with the commands in `docs/08_troubleshooting.md` ("point cloud vanished after editing the calibration").
* Rotation about the vertical axis: `/usr/bin/python3 ~/nero_pipeline_launch/tools/adjust_calib_yaw.py <degrees>` (a negative number
  turns the other way; it keeps one backup, `<file>.bak`, overwritten on every run). Use `/usr/bin/python3`, conda's Python has no `yaml` module.
* Pitch or any shift combined with a rotation: `launch/tools/calib_adjust.awk`. Usage and the exact sign convention are in
  `launch/tools/README.md`.

## 5. Full recalibration (the procedure that was used)

Needs: the marker taped on the gripper where the top camera can see it; arm and RViz running.
ArUco dictionary `5X5_250`, marker id 0, **marker side length 0.050 m**.

| Terminal | Command |
|---|---|
| 1. Arm with RViz | `ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can_piper arm_type:=nero effector_type:=agx_gripper follow:=true` |
| 2. Top camera | the `ob_camera.launch.py` command in [docs/02](02_install_and_environment.md) section 2.5 |
| 3. Marker detection | `ros2 run aruco_opencv aruco_tracker_autostart --ros-args -p cam_base_topic:=/topcam/color/image_raw -p marker_dict:=5X5_250 -p marker_size:=0.050 -p publish_tf:=true` |
| 4. Calibration window | `ros2 launch easy_handeye2 calibrate.launch.py calibration_type:=eye_on_base name:=nero_eye_to_hand_calib robot_base_frame:=base_link robot_effector_frame:=gripper_base tracking_base_frame:=topcam_link tracking_marker_frame:=marker_0 freehand_robot_movement:=true` |
| 5. See the detection | `ros2 run rqt_image_view rqt_image_view /aruco_tracker/debug` |
| 6. Watch the marker pose settle | `ros2 run tf2_ros tf2_echo topcam_color_optical_frame marker_0` |

All terminals: `conda deactivate; source ~/agx_arm_ws/install/setup.bash` first. Stop the pipeline (and its own camera and
calibration publisher) before starting these, and back up the old `.calib` file.

Procedure: move the arm in RViz (MotionPlanning panel, Plan and Execute); when the marker pose in terminal 6 has stopped
changing for 2 to 3 seconds, click **Take Sample** in the calibration window; take **15 to 20 samples**; vary the **position
across the workspace and rotate the gripper** between samples (a single marker needs rotational variety); click **Save**
before closing anything.

Lessons from earlier attempts:

| Problem seen | Cause | Fix |
|---|---|---|
| Translation values 1000 times too **large** (`y = -1134` instead of `-1.134`) | The marker size was given in the wrong unit (probably `marker_size:=50` instead of `0.050`; the values were exactly 1000 times a plausible camera position) | Re-run with the size in metres |
| Calibration result targeted `topcam_color_optical_frame`, which already has a parent from the camera driver's TF | Two parents for one frame | Express the calibration relative to `topcam_link` instead (`tracking_base_frame:=topcam_link`) |
| Unstable result with few distinct orientations | A single marker is a weak constraint | 15 to 20 samples with varied rotation |
| Point cloud about 14 cm too close to the base, which made all IK fail | Calibration drift | Always check the RViz overlay **before** planning |
| Cloud jumps between two positions | Two publishers of the same transform (the pipeline's and a manually started one) | `pkill` all publishers, start one |
