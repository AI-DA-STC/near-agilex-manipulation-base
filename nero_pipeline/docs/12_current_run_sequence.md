# 12. Current run sequence (simulation, real arm, demo recording)

These are the command blocks that were used on the lab machine on 2026-10-13 and 2026-10-14, collected in one place. They **replace** the sequences in [docs/04](04_run_simulation.md) and [docs/05](05_run_real_arm.md) where those differ
(those documents describe the earlier, slower pipeline; the safety protocol in [docs/05](05_run_real_arm.md) section 1 still applies in full).

**State of these blocks.** Every block was syntax-checked (`bash -n`, and `py_compile` for the embedded Python). The simulation and real-arm sequences were run by the operator (**operator report**: they worked, including a real pick and place);
no timings or measurements of the real-arm run were recorded. The install-from-repo block in section 1 was tested only with stand-in folders.
What each part is based on is explained in [docs/11](11_perception_workers_frames_recentring.md).

**Conventions.** Every block starts with `conda deactivate`: ROS commands must run with conda deactivated. Use a separate terminal for each titled block, in the order shown.

| Part | Use it when |
|---|---|
| [1. Get the code and build](#1-get-the-code-and-build) | A new machine, or the workspace is older than this repository |
| [2. Defaults in `start_pipeline.sh`](#2-defaults-in-start_pipelinesh) | Once, so grasp re-centring, 5 passes and the closing depth are on by default |
| [3. Simulation](#3-simulation-no-arm-needed) | First run, or no arm connected. The camera must be plugged in |
| [4. Real arm](#4-real-arm) | Only after the simulation works and [docs/05](05_run_real_arm.md) section 1 has been read |
| [5. Demo recording](#5-demo-recording) | Showing the segmentation image and the grasp poses on screen |
| [6. Not installed yet](#6-future-work-prompt-question-at-run-start) | The prompt question at the start of each run |

---

## 1. Get the code and build

This **overwrites** the workspace copies of four packages with the versions in this repository, then clears their stale build folders. Do it on a new machine, or when this repository is the newer one.
(On the lab machine the same changes were installed from update bundles; the files are identical.)

```bash
conda deactivate
source /opt/ros/humble/setup.bash
R=~/near-agilex-manipulation-base/nero_pipeline
for p in gsam2_ros contact_graspnet_ros target_pose_ros nero_mtc_task; do
  mkdir -p ~/agx_arm_ws/src/$p
  ( cd $R/packages/$p && tar --exclude=offline_tests --exclude=__pycache__ -cf - . ) | ( cd ~/agx_arm_ws/src/$p && tar -xf - )
  rm -rf ~/agx_arm_ws/build/$p ~/agx_arm_ws/install/$p
done
mkdir -p ~/gsam2_ros_scripts ~/contact_graspnet_ros_scripts
cp $R/packages/gsam2_ros/scripts/*.py ~/gsam2_ros_scripts/
cp $R/packages/contact_graspnet_ros/scripts/*.py ~/contact_graspnet_ros_scripts/
cd ~/agx_arm_ws && colcon build --symlink-install --packages-select gsam2_ros contact_graspnet_ros target_pose_ros nero_mtc_task
source ~/agx_arm_ws/install/setup.bash
```

Check which file ROS really loads for each node (it must say `ok`, not `OLD`): see the block at the end of [docs/11](11_perception_workers_frames_recentring.md) section 1.4.

## 2. Defaults in `start_pipeline.sh`

`launch/start_pipeline.sh` starts the grasp node with `forward_passes:=5` (instead of 1), `recentre_on_object:=true` and `recentre_depth_below_top_m:=0.006`. These edits are idempotent (they do nothing when already present) and keep a backup. Override per run with
`CGN_FORWARD_PASSES`, `CGN_RECENTRE` (`true` or `false`) and `CGN_DEPTH`, for example `CGN_RECENTRE=false ./start_pipeline.sh`.

```bash
F=~/nero_pipeline_launch/start_pipeline.sh
cp "$F" "$F.bak_cgn_$(date +%H%M%S)"
sed -i '/contactgraspnet.launch.py/{/forward_passes/!s|$| forward_passes:=${CGN_FORWARD_PASSES:-5}|}' "$F"
sed -i '/contactgraspnet.launch.py/{/recentre_on_object/!s|$| recentre_on_object:=${CGN_RECENTRE:-true}|}' "$F"
grep -n "contactgraspnet.launch.py" "$F"
bash -n "$F" && echo "start_pipeline.sh syntax OK"
```

```bash
F=~/nero_pipeline_launch/start_pipeline.sh
cp "$F" "$F.bak_depth_$(date +%H%M%S)"
sed -i '/contactgraspnet.launch.py/{/recentre_depth_below_top_m/!s|$| recentre_depth_below_top_m:=${CGN_DEPTH:-0.006}|}' "$F"
grep -n "contactgraspnet.launch.py" "$F"
bash -n "$F" && echo "start_pipeline.sh syntax OK"
grep -c "recentre_depth_below_top_m" ~/agx_arm_ws/src/contact_graspnet_ros/launch/contactgraspnet.launch.py | sed 's/^/launch file knows the argument (must be 2 or more): /'
```

The last command prints a number: it must be 2 or more (the launch file knows the argument).

---

## 3. Simulation (no arm needed)

MoveIt runs with fake hardware, RViz shows the arm, nothing physical moves. The camera and the whole perception chain are the real ones.

**3.1 Clean slate** (also restarts the ROS daemon: a stuck daemon makes the arm launcher report `NO feedback` and RViz never opens, [docs/08](08_troubleshooting.md) section H). It must print `clean`.

```bash
conda deactivate
source ~/agx_arm_ws/install/setup.bash
pkill -9 -f "agx_arm_ctrl_single|move_group|rviz2|orbbec_camera|handeye_publisher|mtc_pick_and_place|grasp_watch|grasp_view|grounded_sam2_worker|contact_graspnet_worker|contact_graspnet_node|gsam2_segmentation_node|target_pose_node|pose_transform"
ros2 daemon stop
sleep 2
ros2 daemon start
unset REAL_ARM TABLE_TOP_Z JOINT_STATES_TOPIC HOME_MOVE_SECONDS ARM_WAIT
pgrep -af "agx_arm_ctrl_single|move_group|rviz2|orbbec_camera|grounded_sam2_worker|contact_graspnet_worker" || echo clean
```

**3.2 Terminal 1: start everything (leave it running).** Wait for `=== All nodes started ===` and for RViz.

```bash
conda deactivate
source ~/agx_arm_ws/install/setup.bash
unset REAL_ARM TABLE_TOP_Z JOINT_STATES_TOPIC HOME_MOVE_SECONDS ARM_WAIT
cd ~/nero_pipeline_launch && ./start_pipeline.sh
```

**3.3 Terminal 2: readiness checks.** Expect two `worker ready` lines, `recentre_on_object` true, `forward_passes` 5, empty `camera_frame` values (= automatic), and depth `640x480` with `fx=488.8`.
If the depth is `640x400` the camera started without registration: unplug and replug its USB cable and start again.

```bash
source ~/.bashrc; conda deactivate
source ~/agx_arm_ws/install/setup.bash
L=~/nero_pipeline_logs/$(ls -t ~/nero_pipeline_logs | head -1)
grep -h "worker ready" $L/gsam2_ros.log $L/contact_graspnet_ros.log | cut -c1-150
ros2 param get /contact_graspnet_node recentre_on_object
ros2 param get /contact_graspnet_node camera_frame
ros2 param get /contact_graspnet_node forward_passes
N=$(ros2 node list 2>/dev/null | grep -i target_pose | head -1); ros2 param get ${N:-/target_pose_node} camera_frame
ros2 topic echo /control/joint_states --once
/usr/bin/python3 ~/nero_pipeline_launch/scripts/add_table_collision.py
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{}" | grep -c table_surface
/usr/bin/python3 - << 'PYEOF'
import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
rclpy.init(); n = Node("kcheck"); got = {}
topics = [("/topcam/depth/image_raw", Image), ("/topcam/depth/camera_info", CameraInfo)]
for name, typ in topics:
    n.create_subscription(typ, name, lambda m, name=name: got.setdefault(name, m), qos_profile_sensor_data)
t0 = time.time()
while len(got) < len(topics) and time.time() - t0 < 15: rclpy.spin_once(n, timeout_sec=0.3)
for name, typ in topics:
    m = got.get(name)
    if m is None: print(name, "NOT RECEIVED"); continue
    print(name, "frame=%s" % m.header.frame_id, "%dx%d" % (m.width, m.height), "" if typ is Image else "fx=%.1f cx=%.1f cy=%.1f" % (m.k[0], m.k[2], m.k[5]))
PYEOF
```

**3.4 Terminal 3: the 3D grasp viewer (leave it running).** It opens its window after the first `pose_transform/transform_all`.

```bash
conda deactivate
source ~/agx_arm_ws/install/setup.bash
export JOINT_STATES_TOPIC=/control/joint_states
W=$(ls ~/nero_pipeline_launch/tools/grasp_watch.py ~/nero_pipeline_launch/grasp_watch.py 2>/dev/null | head -1); echo "using: $W"; /usr/bin/python3 "$W"
```

**3.5 Terminal 4: the perception test.** Expect two `labelled with frame` lines naming the same frame, a `recentred` line, and in the `pregrasp_check.py` table `dx` and `dy` near 0 and `dz` about -2 cm.

```bash
conda deactivate
source ~/agx_arm_ws/install/setup.bash
ros2 service call /gsam2/segment std_srvs/srv/Trigger | grep -o "message=.*" | cut -c1-90
ros2 service call /contact_graspnet/detect_grasps std_srvs/srv/Trigger | grep -o "message=.*" | cut -c1-90
ros2 service call /target_pose/extract std_srvs/srv/Trigger | grep -o "message=.*" | cut -c1-120
ros2 service call /pose_transform/transform_all std_srvs/srv/Trigger | grep -o "message=.*" | cut -c1-170
L=~/nero_pipeline_logs/$(ls -t ~/nero_pipeline_logs | head -1)
grep -h "labelled with frame" $L/contact_graspnet_ros.log $L/target_pose_ros.log | cut -c1-200
grep -h "recentred" $L/contact_graspnet_ros.log | tail -1 | cut -c1-260
T=$(find -L ~/near-agilex-manipulation-base ~/nero_pipeline_launch -maxdepth 5 -name reach_check.py 2>/dev/null | head -1 | xargs -r dirname); echo "tools: ${T:-NOT FOUND}"
/usr/bin/python3 ~/Downloads/pregrasp_check.py --tools "$T"
```

**3.6 Optional: MoveIt plan-only (nothing physical moves).** `Found N solution(s).` means it planned; the node then idles on purpose (Ctrl+C). `Failing stage(s)` means no plan this time; run it again ([docs/11](11_perception_workers_frames_recentring.md) section 6).

```bash
conda deactivate
source ~/agx_arm_ws/install/setup.bash
cd ~/nero_pipeline_launch && ./run_pick_and_place.sh 2>&1 | tee /tmp/last_plan_run.txt | tail -n 40
```

---

## 4. Real arm

**Read [docs/05](05_run_real_arm.md) section 1 first.** The arm has no physical e-stop. Type `STOP` (without Enter) in Terminal 2 before anything moves.
Before the **first** run after this update, measure the table height with a ruler (shoulder joint axis 13.8 cm above the table means z = 0) and check the `FINGER TIPS` block in 4.5.

**4.0 The arm must be talking on the CAN bus** (stop here if it prints 0 frames):

```bash
ip link show can_piper 2>/dev/null | grep -o "state [A-Z]*" || echo "CAN IS DOWN"
timeout 3 candump can_piper | wc -l | sed 's/^/CAN frames in 3 s (0 = the arm is NOT talking, stop here): /'
```

**4.1 Clean slate** (arm powered, at `home`):

```bash
conda deactivate
source ~/agx_arm_ws/install/setup.bash
pkill -9 -f "agx_arm_ctrl_single|move_group|rviz2|orbbec_camera|handeye_publisher|mtc_pick_and_place|grasp_watch|grasp_view|grounded_sam2_worker|contact_graspnet_worker|contact_graspnet_node|gsam2_segmentation_node|target_pose_node|pose_transform"
ros2 daemon stop
sleep 2
ros2 daemon start
pgrep -af "agx_arm_ctrl_single|move_group|rviz2|orbbec_camera|grounded_sam2_worker|contact_graspnet_worker" || echo clean
```

**4.2 Terminal 1: start everything (leave it running).** `unset TABLE_TOP_Z` keeps the built-in table box harmless; it is replaced by the exact one in 4.3.

```bash
conda deactivate
source ~/agx_arm_ws/install/setup.bash
unset TABLE_TOP_Z
export REAL_ARM=1 ARM_WAIT=30
cd ~/nero_pipeline_launch && ./start_pipeline.sh
```

**4.3 Terminal 2: readiness checks and the exact table box.** The `add_table_box.py` output must end with `TOP FACE z = +0.000` for `table_surface`. If it prints `WARNING`, stop. Then type `STOP` without pressing Enter.

```bash
source ~/.bashrc; conda deactivate
source ~/agx_arm_ws/install/setup.bash
L=~/nero_pipeline_logs/$(ls -t ~/nero_pipeline_logs | head -1)
grep -h "worker ready" $L/gsam2_ros.log $L/contact_graspnet_ros.log | cut -c1-150
ros2 param get /contact_graspnet_node recentre_on_object
ros2 param get /contact_graspnet_node recentre_depth_below_top_m
ros2 param get /contact_graspnet_node forward_passes
ros2 topic echo /control/joint_states --once
ros2 topic echo /feedback/joint_states --once
/usr/bin/python3 ~/Downloads/add_table_box.py --top_z 0.0
/usr/bin/python3 - << 'PYEOF'
import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
rclpy.init(); n = Node("kcheck"); got = {}
topics = [("/topcam/depth/image_raw", Image), ("/topcam/depth/camera_info", CameraInfo)]
for name, typ in topics:
    n.create_subscription(typ, name, lambda m, name=name: got.setdefault(name, m), qos_profile_sensor_data)
t0 = time.time()
while len(got) < len(topics) and time.time() - t0 < 15: rclpy.spin_once(n, timeout_sec=0.3)
for name, typ in topics:
    m = got.get(name)
    if m is None: print(name, "NOT RECEIVED"); continue
    print(name, "frame=%s" % m.header.frame_id, "%dx%d" % (m.width, m.height), "" if typ is Image else "fx=%.1f cx=%.1f cy=%.1f" % (m.k[0], m.k[2], m.k[5]))
PYEOF
```

**4.4 Terminal 3: the 3D grasp viewer.**

```bash
conda deactivate
source ~/agx_arm_ws/install/setup.bash
export JOINT_STATES_TOPIC=/feedback/joint_states
W=$(ls ~/nero_pipeline_launch/tools/grasp_watch.py ~/nero_pipeline_launch/grasp_watch.py 2>/dev/null | head -1); echo "using: $W"; /usr/bin/python3 "$W"
```

**4.5 Terminal 4: the perception test (the arm does not move).** In the `FINGER TIPS` block every tip should be at least 0.5 cm above the camera's table; if it prints a WARNING, do not continue.

```bash
conda deactivate
source ~/agx_arm_ws/install/setup.bash
ros2 service call /gsam2/segment std_srvs/srv/Trigger | grep -o "message=.*" | cut -c1-90
ros2 service call /contact_graspnet/detect_grasps std_srvs/srv/Trigger | grep -o "message=.*" | cut -c1-90
ros2 service call /target_pose/extract std_srvs/srv/Trigger | grep -o "message=.*" | cut -c1-120
ros2 service call /pose_transform/transform_all std_srvs/srv/Trigger | grep -o "message=.*" | cut -c1-170
L=~/nero_pipeline_logs/$(ls -t ~/nero_pipeline_logs | head -1)
grep -h "labelled with frame" $L/contact_graspnet_ros.log $L/target_pose_ros.log | cut -c1-200
grep -h "recentred" $L/contact_graspnet_ros.log | tail -1 | cut -c1-260
T=$(find -L ~/near-agilex-manipulation-base ~/nero_pipeline_launch -maxdepth 5 -name reach_check.py 2>/dev/null | head -1 | xargs -r dirname); echo "tools: ${T:-NOT FOUND}"
/usr/bin/python3 ~/Downloads/pregrasp_check.py --tools "$T" --table_z 0.0 --depth_below_top 0.006
```

**4.6 Once, in RViz: home to `initial`.** The lab machine starts every run from `initial` ([docs/11](11_perception_workers_frames_recentring.md) section 5), and step 0 of `run_pick_and_place.sh` moves there in one straight joint-space line **without collision checking** if the arm is elsewhere. So at the start of a session, with the arm at `home`, select the goal state `initial` in the MotionPlanning panel and press Plan & Execute (a collision-checked move); step 0 then finds the arm already there.

**4.7 Plan only.** The arm moves to the start pose first. `Found N solution(s).` means it planned (the node then idles; check the plan in RViz and press Ctrl+C). `Failing stage(s)` means nothing moved; run it again. **Never use the RViz Execute button**: it runs the raw, untimed trajectory.

```bash
conda deactivate
source ~/agx_arm_ws/install/setup.bash
export JOINT_STATES_TOPIC=/feedback/joint_states HOME_MOVE_SECONDS=15
cd ~/nero_pipeline_launch && ./run_pick_and_place.sh 2>&1 | tee /tmp/real_plan_run.txt | tail -n 45
```

**4.8 Execute** (hand near `STOP`, workspace clear). Stop with `STOP` if the fingertips approach the table, the closing point is clearly off the object, or the motion differs from the plan.

```bash
conda deactivate
source ~/agx_arm_ws/install/setup.bash
export JOINT_STATES_TOPIC=/feedback/joint_states HOME_MOVE_SECONDS=15
cd ~/nero_pipeline_launch && MTC_EXECUTE=y ./run_pick_and_place.sh 2>&1 | tee /tmp/real_exec_run.txt
```

**4.9 Before shutting down:** use RViz Plan & Execute to move to `home`, then stop the pipeline (the driver needs the arm at `home` on its next start).

---

## 5. Demo recording

Three windows tell the story: the segmentation window (`launch/tools/demo_view.py`: pick object on the left, place target on the right), the 3D grasp viewer, and RViz. Start the segmentation window **before** the pipeline runs: the node publishes the images once per call.
Do parts 3.1 to 3.4 (simulation) or 4.1 to 4.4 (real arm) first.

**5.1 One-time: can OpenCV open windows?** You want `GTK` or `QT` to say `YES`. If not, use `rqt_image_view` on `/gsam2/annotated_source` and `/gsam2/annotated_target`.

```bash
/usr/bin/python3 -c "import cv2; b = cv2.getBuildInformation(); print('cv2', cv2.__version__); print([l.strip() for l in b.splitlines() if 'GTK' in l or 'QT' in l or 'GUI' in l][:4])"
ros2 pkg list 2>/dev/null | grep -c rqt_image_view | sed 's/^/rqt_image_view installed (1 = yes): /'
```

**5.2 Optional: remove the 30-second limit of Ubuntu's built-in recorder** (GNOME only).

```bash
gsettings set org.gnome.settings-daemon.plugins.media-keys max-screencast-length 0 && echo "screen recorder length limit removed"
```

**5.3 Terminal: the segmentation window (leave it running).** Esc or q in the window closes it.

```bash
conda deactivate
source ~/agx_arm_ws/install/setup.bash
/usr/bin/python3 ~/Downloads/demo_view.py --height 480
```

**5.4 Arrange the windows, start the recorder, then run the whole pipeline** (real arm: hand near `STOP`). On screen: the arm moves to the start pose, the segmentation window fills in, the 3D window opens with the grasps, RViz shows the plan, the arm picks and places.

```bash
conda deactivate
source ~/agx_arm_ws/install/setup.bash
export JOINT_STATES_TOPIC=/feedback/joint_states HOME_MOVE_SECONDS=15
cd ~/nero_pipeline_launch && MTC_EXECUTE=y ./run_pick_and_place.sh 2>&1 | tee /tmp/demo_run.txt
```

**Perception only (no motion)**, for a calm narrated segment:

```bash
conda deactivate
source ~/agx_arm_ws/install/setup.bash
ros2 service call /gsam2/segment std_srvs/srv/Trigger | grep -o "message=.*" | cut -c1-110
ros2 service call /contact_graspnet/detect_grasps std_srvs/srv/Trigger | grep -o "message=.*" | cut -c1-110
ros2 service call /target_pose/extract std_srvs/srv/Trigger | grep -o "message=.*" | cut -c1-110
ros2 service call /pose_transform/transform_all std_srvs/srv/Trigger | grep -o "message=.*" | cut -c1-110
```

The full run re-detects everything, so the grasps and images of the full run differ slightly from this showcase.

---

## 6. Future work: prompt question at run start

`future_work/ask_prompts/` asks for the source (pick) and target (place) prompts at the start of every run (`y` keeps the last; `s:red mug t:blue tray` changes). It was built and tested with stand-ins and has **not been installed or used on the lab machine**.
Install steps and behaviour: [future_work/README.md](../future_work/README.md).
