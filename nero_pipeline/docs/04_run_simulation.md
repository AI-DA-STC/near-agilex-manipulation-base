# 04. Run in simulation

> **Update 2026-10-14:** the commands that work now (persistent workers, exact table box, camera-frame check, demo windows) are in [docs/12](12_current_run_sequence.md) part 3. They replace the sequence below where they differ;
> the background is in [docs/11](11_perception_workers_frames_recentring.md). In simulation use `unset REAL_ARM TABLE_TOP_Z JOINT_STATES_TOPIC HOME_MOVE_SECONDS ARM_WAIT` before starting.

Always do this first. The simulation uses MoveIt's fake hardware: the arm model moves in RViz, nothing physical moves,
and a mistake costs nothing.

## 1. Preconditions

* [docs/02](02_install_and_environment.md) checklist passed for ROS, MTC, the five packages, both conda environments and the top camera.
  (The CAN adapter and the arm are **not** needed in simulation, but the top camera is: perception needs real images.)
* The calibration file exists ([docs/03](03_calibration.md)).
* Nothing from an earlier run is still alive:

```bash
pkill -f agx_arm_ctrl_single; pkill -9 -f move_group; pkill -f demo.launch; pkill -f publish.launch.py; pkill -f handeye_publisher; pkill -f orbbec_camera; pkill -f mtc_pick_and_place
pgrep -af "agx_arm_ctrl_single|move_group|rviz2|orbbec|handeye|mtc_pick" || echo clean
unset REAL_ARM TABLE_TOP_Z JOINT_STATES_TOPIC HOME_MOVE_SECONDS ARM_WAIT
```

## 2. Terminal 1: start everything

```bash
conda deactivate
cd ~/nero_pipeline_launch && ./start_pipeline.sh
```

What `start_pipeline.sh` starts, in order: the arm (`launch/scripts/launch_arm.sh`: MoveIt demo with the
`ExecuteTaskSolutionCapability` and the corrected `tcp_link`), the table collision object
(`scripts/add_table_collision.py`), the top camera, the calibration publisher, `gsam2_ros`, `contact_graspnet_ros`,
`target_pose_ros`, `pose_transform_ros` and a debug TF broadcaster. Each node logs to
`~/nero_pipeline_logs/<timestamp>/<node>.log`.

Wait for `=== All nodes started ===` and for RViz to open. Press Ctrl+C in this terminal to stop everything.

If the line `FAILED to add table collision object` appears, MoveIt was not ready yet: add it again later (step 3).

## 3. Terminal 2: checks

```bash
conda deactivate
ros2 param get /move_group capabilities                                  # move_group/ExecuteTaskSolutionCapability
ros2 topic echo /control/joint_states --once                              # joint values of the simulated arm
/usr/bin/python3 ~/nero_pipeline_launch/scripts/add_table_collision.py    # (re)adds the table box, safe to repeat
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{}" | grep -c table_surface   # a number above 0
ros2 param get /gsam2_segmentation_node source_prompt                     # what object it will look for
```

In RViz check that the **point cloud overlaps the arm model** (calibration check, [docs/03](03_calibration.md)).
The planning scene (the table box) is lost whenever `move_group` restarts, so re-add it after every restart.

## 4. Terminal 3: plan (no motion yet)

```bash
conda deactivate
cd ~/nero_pipeline_launch && ./run_pick_and_place.sh
```

The script prints six steps:

| Step | What happens | Good output |
|---|---|---|
| 0 | `scripts/move_to_pose_90deg.py` moves the arm to the start pose `pose_90deg` (direct joint move, no collision checking). It first reads the joint state and does nothing if the arm is already there | `Reached pose_90deg.` |
| 1 | `/gsam2/segment`: masks for the object and the plate | `Source: '...' ... Target: 'plate' ...` and `success=True` |
| 2 | `/contact_graspnet/detect_grasps` | `Found N grasp(s). Best score=...` |
| 3 | `/target_pose/extract` | `Target pose: position=(...)` |
| 4 | `/pose_transform/transform_all` | `Grasp in base_link: (...)  Target in base_link: (...)` |
| 5 | `nero_mtc_task` builds and plans the task | `Found N solution(s).` and the task tree |

Steps 1 to 4 go through `call_checked`: **if any step answers `success=False` the script stops** ("so the previous
detection is not reused"). Before this guard existed, a failed detection silently reused the previous run's latched poses.

After a successful plan the MTC process stays alive (so RViz can keep its connection) until you press Ctrl+C.
A failed plan exits by itself and prints `Failing stage(s):`, see [docs/08](08_troubleshooting.md).

## 5. Execute

Ctrl+C the waiting plan, then:

```bash
MTC_EXECUTE=y ./run_pick_and_place.sh
```

The script plans again, retimes the solution (`retime_solution`, prints `Retimed solution: total motion time N s`),
sends it to `execute_task_solution` and prints the **real** result:

```
REAL error_code.val = 1                       # 1 = SUCCESS (-4 = CONTROL_FAILED, see docs/08)
Result status (0=UNKNOWN,1=SUCCEEDED,2=CANCELED,3=ABORTED) = 4     # rclpy status 4 = SUCCEEDED
```

## 6. Looking at results in RViz

* Motion Planning Tasks panel: the **Task Solution Topic** must be `/solution` (leading slash).
* Click a numbered solution in the right-hand list to replay it as a ghost. The replay speed is the panel's own
  (State Display Time) and does **not** show the executed speed. Do not use the panel's Execute button.
* Untick the display's **Robot** entry if the fast ghost distracts you; re-tick it to replay.

## 7. Optional: see which grasp was used

`launch/tools/grasp_watch.py` and `launch/tools/grasp_view.py` show all candidate grasps, the ones handed to MTC, and where the gripper
really closed. See `launch/tools/README.md`. In simulation start the watcher with `export JOINT_STATES_TOPIC=/control/joint_states`.

## 8. Plan-only vs execute vs "which object"

| You want | Do |
|---|---|
| A different object | `ros2 param set /gsam2_segmentation_node source_prompt "a roll of tape."` (live; make permanent in `launch/start_pipeline.sh`) |
| Only look at the grasps, no planning | Run the four perception services by hand ([docs/10](10_grasp_quality_investigation.md), section 4) |
| Change speed | `ARM_MAX_VEL` and friends in `mtc_pick_and_place.py` ([docs/06](06_tuning_reference.md)) |
