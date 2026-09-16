# Camera setup — arm-mounted RealSense + top-view Orbbec

## RealSense (arm-mounted)

```bash
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
```

If unavailable via apt for this Jetson/arm64 combo, build from source:
```bash
cd ~/agx_arm_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
cd ~/agx_arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select realsense2_camera realsense2_camera_msgs realsense2_description
```

Launch (always set `camera_name` explicitly to avoid namespace collisions
if running alongside another camera):
```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=armcam \
  enable_depth:=true \
  enable_color:=true \
  pointcloud__neon_.enable:=true
```

> **Gotcha:** on this specific ARM64/NEON build, the pointcloud enable
> parameter is `pointcloud__neon_.enable`, not the standard
> `pointcloud.enable` used on most other platforms. Confirm the real param
> name any time with:
> ```bash
> ros2 param list /<node_name> | grep -i pointcloud
> ```

## Orbbec (top-view, Dabai DC1 in this setup)

The `v2-main` branch of `OrbbecSDK_ROS2` does **not** support the Dabai
series (legacy/OpenNI-era hardware) — its device enumeration silently
returns zero devices, even as root, even though `lsusb` and the kernel see
the hardware fine. **Use the `main` branch instead:**

```bash
cd ~/agx_arm_ws/src
git clone -b main https://github.com/orbbec/OrbbecSDK_ROS2.git orbbec_camera
cd ~/agx_arm_ws
rosdep install --from-paths src --ignore-src -r -y --skip-keys warehouse_ros_mongo
colcon build --symlink-install --packages-up-to orbbec_camera
source install/setup.bash
```

> Only clone one branch's worth of these packages into `src/` at a time —
> both branches define packages with the identical names
> (`orbbec_camera`, `orbbec_camera_msgs`, `orbbec_description`), and colcon
> will refuse to build with both present.

Install udev rules (needed once, from inside the actual `orbbec_camera`
package folder, not the outer clone folder):
```bash
cd ~/agx_arm_ws/src/orbbec_camera/orbbec_camera
sudo bash scripts/install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```
Unplug/replug the camera after this.

Confirm the SDK actually sees it (should print a serial + USB port, not
nothing):
```bash
ros2 run orbbec_camera list_devices_node
```

### Finding the right stream profiles

Model-specific launch files (`dabai_d1.launch.py`, etc.) may have
incomplete or wrong-for-your-unit default parameters and can hard-crash the
whole node on an unsupported profile instead of gracefully skipping it. Use
the generic launch file with explicit, confirmed-supported profile values
instead:

```bash
ros2 run orbbec_camera list_camera_profile_mode_node
```
This prints every genuinely supported resolution/fps/format combination for
your exact connected unit. Then launch with values taken directly from that
list, e.g.:

```bash
ros2 launch orbbec_camera ob_camera.launch.py \
  camera_name:=topcam \
  enable_depth:=true depth_width:=640 depth_height:=400 depth_fps:=30 depth_format:=Y12 \
  enable_color:=true color_width:=640 color_height:=480 color_fps:=30 color_format:=MJPG \
  enable_ir:=true ir_format:=Y10 \
  enable_point_cloud:=true
```

> **Gotcha:** this device is USB2.0 — running full depth + color
> simultaneously at high fps may exceed bandwidth. Drop `color_fps` first
> if you hit issues.

## Connecting both cameras into the robot's TF tree

Both cameras' link frames are **not connected to `base_link`/`world`
automatically** — running `ros2 run tf2_tools view_frames` will show them
as disconnected root frames until you add static transforms describing
their actual physical mounting position:

```bash
# Arm-mounted camera — relative to whatever link it's bolted to (adjust to actual mount)
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.05 --roll 0 --pitch 0 --yaw 0 \
  --frame-id link7 --child-frame-id armcam_link

# Top-view camera — relative to base_link (adjust to actual mount)
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 1.2 --roll 0 --pitch 1.5708 --yaw 0 \
  --frame-id base_link --child-frame-id topcam_link
```

Without these, PointCloud2 displays in RViz will show nothing (no error,
just silently empty) even if the topics are actively publishing data, and
Octomap won't populate under **MotionPlanning → Scene Geometry**.

For anything beyond rough visualization — actual grasping — replace these
placeholder values with a real calibration via
[`easy_handeye2`](https://github.com/marcoesposito1988/easy_handeye2).

## RViz display setup once both are running

- **Global Options → Fixed Frame** → `base_link` (must be a frame reachable
  from both camera frames via the static transforms above)
- **Add → By display type → PointCloud2** for each camera, Topic set to
  the exact live topic name — check `ros2 topic list | grep points` for
  the real current names, they can shift depending on launch args
- If a PointCloud2 display still shows nothing with topics confirmed live
  and TF confirmed connected, expand the display and try switching
  **Reliability Policy** to **Best Effort** — some camera drivers publish
  with QoS settings that don't match RViz's default subscription
