# target_pose_ros

Extracts a placement pose from a segmented **target** object (e.g. a
plate) - the half of the perception pipeline that skips grasp detection
entirely, per the source/target design worked out with the architecture:
the target is never grasped, so its pose is read directly off its own
point cloud instead of going through Contact-GraspNet.

No neural network here - pure pixel + depth + camera-intrinsics geometry.
This is why, unlike [`gsam2_ros`](https://github.com/rarlyx/gsam2_ros) and
[`contact_graspnet_ros`](https://github.com/rarlyx/contact_graspnet_ros),
this package needs **no conda environment at all** - it runs entirely in
plain ROS 2 Python.

## Pipeline

```
gsam2_ros /gsam2/mask_target (2D)  -----\
                                          +--> back-project each target pixel to 3D
/topcam/depth/image_raw   --------------/       (standard pinhole camera math)
/topcam/depth/camera_info -------------/
                                                    |
                                                    v
                                    mean(x,y) + min(z) or mean(z)
                                                    |
                                                    v
                              placement pose (/target_pose/pose)
                              in the CAMERA'S OWN optical frame
```

Same camera-frame limitation as `contact_graspnet_ros`: this pose is not
usable by MoveIt as-is. It needs the same `eye_on_base` extrinsic
transform (see [`nero-camera-calibration`](https://github.com/rarlyx/nero-camera-calibration))
applied before reaching the task planner.

**No meaningful orientation is derived here.** A flat, round-ish target
(a plate) has no natural yaw to extract - the published pose's orientation
is identity. Whichever node consumes this pose (task planning) should
apply its own fixed placing orientation, not rely on this field.

## Setup

No conda env, no model downloads - just the package itself.

```bash
cd ~/agx_arm_ws/src
cp -r target_pose_ros .
cd ~/agx_arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select target_pose_ros
source install/setup.bash
```

## Usage

Requires `gsam2_ros` already running (for the target mask) and the top
camera live with `depth_registration:=true` (see `contact_graspnet_ros`'s
README for why this alignment matters - same requirement here).

**1. Launch**
```bash
ros2 launch target_pose_ros targetpose.launch.py \
  mask_topic:=/gsam2/mask_target \
  depth_topic:=/topcam/depth/image_raw \
  camera_info_topic:=/topcam/depth/camera_info \
  depth_scale_to_meters:=0.001 \
  z_mode:=top_surface
```

**2. Run the pipeline**
```bash
# Segment first (produces both source and target masks)
ros2 service call /gsam2/segment std_srvs/srv/Trigger

# Then extract the target's placement pose
ros2 service call /target_pose/extract std_srvs/srv/Trigger
```
Check the response for the extracted position. Full pose publishes to:
```bash
ros2 topic echo /target_pose/pose
```

## z_mode - which one to use

- **top_surface** (default) - z = the closest point to the camera among
  target pixels. Since the camera looks straight down, "closest" means
  "physically highest" - use this when placing something on top of the
  target (e.g. dropping the carrot onto the plate's surface).
- **centroid** - z = mean depth across all target pixels. Use this if you
  instead want the pose at the target's average height (e.g. placing
  beside it, or the target itself isn't flat).
