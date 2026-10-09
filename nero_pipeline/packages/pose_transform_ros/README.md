# pose_transform_ros

Transforms the source grasp pose and target placement pose out of the
camera's optical frame and into `base_link`, using the `eye_on_base`
hand-eye calibration - the missing link both
[`contact_graspnet_ros`](https://github.com/rarlyx/contact_graspnet_ros)
and [`target_pose_ros`](https://github.com/rarlyx/target_pose_ros) flag in
their own READMEs: neither of their outputs is directly usable by MoveIt
without this step.

No custom matrix math here - uses `tf2_ros`'s native `Buffer.transform()`,
since `PoseStamped` is already registered with `tf2_ros`'s transform
dispatch. This is the standard, correct way to do frame transforms in
ROS 2 rather than reimplementing quaternion/translation composition by
hand.

## Pipeline

```
/contact_graspnet/grasps (PoseArray, camera frame)  --\
                                                          +--> tf2 lookup + transform --> /grasp_pose_transformed  (base_link)
/target_pose/pose (PoseStamped, camera frame)  --------/                            --> /target_pose_transformed (base_link)

                    ^
                    |
     requires this transform chain to actually exist at runtime:
     base_link -> ... -> topcam_color_optical_frame  (published by easy_handeye2's
                                                        calibration result)
     topcam_color_optical_frame <-> topcam_depth_optical_frame  (published by the
                                                        camera driver's own TF)
```

## Setup

No conda env, pure tf2_ros - just the package itself.

```bash
cd ~/agx_arm_ws/src
cp -r pose_transform_ros .
cd ~/agx_arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select pose_transform_ros
source install/setup.bash
```

## Prerequisite: the calibration's transform must actually be broadcast

This is the part most likely to need debugging on first run. Before
launching this node, confirm the full TF chain from the camera to
`base_link` genuinely resolves:

```bash
ros2 run tf2_ros tf2_echo base_link topcam_depth_optical_frame
```

If this hangs or errors instead of printing a live transform, the chain is
broken somewhere - check:
1. Is `easy_handeye2`'s **publish** step running (not just the
   calibration itself)? A completed calibration only produces a saved
   result file - something still needs to be actively broadcasting it as
   a static transform at runtime. Check `nero-camera-calibration` for
   how the `eye_on_base` calibration for the top camera was saved, and
   look for the corresponding `easy_handeye2` publish launch invocation.
2. Is the camera driver's own TF (`topcam_color_optical_frame` vs
   `topcam_depth_optical_frame`) actually present? Check with
   `ros2 run tf2_tools view_frames` and inspect the generated tree.
3. Is `robot_state_publisher` running (provides the robot's own
   `base_link -> ...` chain)? This should already be up as part of
   whichever arm launch you're using.

## Usage

Requires `contact_graspnet_ros` and `target_pose_ros` both already run at
least once (so a grasp pose and a target pose exist to transform), and
the TF chain above confirmed working.

**1. Launch**
```bash
ros2 launch pose_transform_ros posetransform.launch.py
```

**2. Run the full chain**
```bash
ros2 service call /gsam2/segment std_srvs/srv/Trigger
ros2 service call /contact_graspnet/detect_grasps std_srvs/srv/Trigger
ros2 service call /target_pose/extract std_srvs/srv/Trigger
ros2 service call /pose_transform/transform_all std_srvs/srv/Trigger
```
Check the response for both poses' positions in `base_link`. Sanity-check
these against what you'd expect physically (roughly where the carrot and
plate actually are relative to the robot's base) before trusting them for
motion planning.

Full poses publish to:
```bash
ros2 topic echo /grasp_pose_transformed
ros2 topic echo /target_pose_transformed
```

These two topics are the actual inputs the task-planning stage (MoveIt
Task Constructor) needs.
