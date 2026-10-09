# 08. Troubleshooting

Find the symptom, read the cause, apply the fix. Every row was seen on this system. Rows marked **(likely)** are the most probable cause
but were not confirmed. Node logs of the last run: `~/nero_pipeline_logs/<latest timestamp>/<node>.log`
(`ls -t ~/nero_pipeline_logs | head -1`).

## A. Shell and build

| Symptom | Cause | Fix |
|---|---|---|
| `No module named 'rclpy'` / `rclpy._rclpy_pybind11` error mentioning Python 3.14 | The shell is in conda `base`, whose Python is not the ROS Python | `conda deactivate`; run Python tools with `/usr/bin/python3` |
| `No module named 'yaml'` when running a tool with `python3` | Same (conda Python) | `/usr/bin/python3 <tool>` |
| An edit to a launch file or `mtc_pick_and_place.py` has no effect | `build/<pkg>` holds a plain copy and `install/` points at it | `rm -rf ~/agx_arm_ws/build/<pkg> ~/agx_arm_ws/install/<pkg>` then `colcon build --symlink-install --packages-select <pkg>` and `source install/setup.bash` |
| A pasted multi-line command or heredoc misbehaves (a stray command inside a line) | Long terminal pastes can be garbled | Create long files in several short pastes, then check with `bash -n file` and `wc -l` / `md5sum` |

## B. Start-up

| Symptom | Cause | Fix |
|---|---|---|
| `FAILED to add table collision object`, `/apply_planning_scene service not available` | MoveIt was not up yet when the table step ran | Wait for RViz; re-run `TABLE_TOP_Z=... /usr/bin/python3 ~/nero_pipeline_launch/scripts/add_table_collision.py`; use `ARM_WAIT=30` for the real arm. Check that the table is in the scene with the `get_planning_scene` command in the block below the table (the count must be above 0) |
| Real arm: RViz never opens | `launch_arm_real.sh` stopped before starting MoveIt | Read the end of `arm.log` in the newest log folder (command below the table). `NO feedback from the arm` means the driver died or the arm is silent (next rows) |
| Driver: `Failed to get firmware version` then the process dies | The arm did not answer the firmware request (CAN id `0x4AF`) within about 5 s | Check `candump can_piper` shows frames; power-cycle the arm and retry. Diagnostic: run `candump -ta can_piper,4AF:7FF` while launching: requests but no replies means the arm is not answering |
| `candump` shows no frames | Arm off, adapter on a different USB port, interface not up | Check `lsmod` lists `gs_usb` (else `sudo insmod ~/gs_usb_build/gs_usb.ko`); `bash find_all_can_port.sh`; `bash can_activate.sh can_piper 1000000 "<port>"` |
| Several `agx_arm_ctrl_single` processes (`pgrep -af agx_arm_ctrl_single`) | Earlier launches left drivers behind | `pkill -f agx_arm_ctrl_single; sleep 2; pkill -9 -f agx_arm_ctrl_single`. The real launcher now does this itself |
| `executable 'agx_arm_control_gate' not found` | `auto_control_gate:=true` needs a helper that is not installed | Do not use `auto_control_gate` (not used by our launcher) |
| `ros2 topic echo /feedback/joint_states` says "does not appear to be published yet" | The driver is not running (or not yet ready) | Start the driver ([docs/05](05_run_real_arm.md)) and retry after a few seconds |
| `move_group` crashes at the start of planning (segfault) | CHOMP | `sudo apt remove ros-humble-moveit-planners-chomp` |
| Pilz library errors at start | `libmoveit_planning_interface.so.2.5.9` missing | Expected. The code falls back to OMPL with a warning |

Handy commands for this section:

```bash
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{}" | grep -c table_surface      # above 0 = table present
tail -n 40 ~/nero_pipeline_logs/$(ls -t ~/nero_pipeline_logs | head -1)/arm.log                         # why did the arm launcher stop?
lsmod | grep gs_usb                                                                                      # CAN module loaded?
```

## C. Perception

| Symptom | Cause | Fix |
|---|---|---|
| `No detection for prompt '...'` | Wrong wording, object not in view, threshold too high | Use an article and end with a period (`a green cube.`); check the object is fully visible and not under the arm; `ros2 param set /gsam2_segmentation_node box_threshold 0.25`. Look at `/gsam2/annotated_source` (`ros2 run rqt_image_view rqt_image_view /gsam2/annotated_source`) |
| `Mask shape ... != depth shape` in `contact_graspnet_ros.log` | Camera started without `depth_registration:=true` | Use the camera command in [docs/02](02_install_and_environment.md) section 2.5 |
| The run script stops with `!!! <service> FAILED - stopping so the previous detection is not reused` | A perception step answered `success=False` | Read the message printed above that line; fix the cause (detection, view); run again |
| A rerun appears to replay the previous solution | **(likely)** A step failed silently and old latched poses (`/grasp_poses_transformed`, `/target_pose_transformed`) were reused; or an old `mtc_pick_and_place` process is still alive | Make sure the guard above is installed (`grep -c call_checked ~/nero_pipeline_launch/run_pick_and_place.sh`); `pkill -f mtc_pick_and_place`; with the grasp watcher running, a new run must print a fresh `candidates:` line |
| No grasps returned | `forward_passes` too low, object outside `z_range`, object too small | `forward_passes` is 5; check the depth in the mask ([docs/10](10_grasp_quality_investigation.md)) |
| Grasps beside the object | **(likely)** MTC received only steeply tilted candidates (the near-vertical ones rank low by score) | Check that `usable.sort(` is in `mtc_pick_and_place.py` (the verifier requires it) and that you rebuilt the package; run `launch/tools/grasp_watch.py` and read `tilts given to MTC`; the near-vertical ones must come first. See [docs/10](10_grasp_quality_investigation.md) section 7 |

## D. Planning (`Failing stage(s):` and the task tree)

How to read the tree printed after a failed plan: each row is a stage; the number in the middle is how many solutions that stage produced.
The **first row with a non-zero count above a row with 0** is where the chain broke. A stage with a large `0` count on its own row
(for example `place IK cone (0/560)`) failed for every sample.

| Failing line | Meaning | Fix |
|---|---|---|
| `pregrasp IK i-...: eef in collision` | The pregrasp pose touches the table box or the scene | Table box too high (`TABLE_TOP_Z`), pregrasp offset too small, or the point cloud/calibration is off so the object is "inside" the table |
| `pregrasp IK i-...: no IK found` | Pose unreachable | Check the arm model and the point cloud overlap ([docs/03](03_calibration.md)); reduce `MAX_TILT_DEG` or add candidates; check the object is within reach |
| `approach`: no solution | Cartesian line blocked or too short | The message says how far it got; check collisions with the table; keep `APPROACH_MAX` equal to `PREGRASP_OFFSET_M` |
| `place IK cone (0/N): no IK found` | The plate position is unreachable with the gripper pointing down | **Plate too far from the base.** A downward-pointing gripper reaches only about 0.58 to 0.60 m at plate height. Keep the plate under about 0.55 m: `ros2 topic echo /target_pose_transformed --once --field pose.position` and compute `sqrt(x^2+y^2)` |
| `place pose cone: eef in collision` (`table_surface` or the plate) | `PLACE_Z_CLEARANCE_M` too low or the table box too high | Raise the clearance or lower `TABLE_TOP_Z` |
| `transit` has start states but 0 goal states | Nothing on the place side succeeded | Look at the place IK row |
| Planning "succeeds" but looks wrong in RViz | The panel shows the raw plan, not what is executed | Check the printed `Retimed solution` line and the arm itself |

## E. Execution

| Symptom | Cause | Fix |
|---|---|---|
| `Execution failed val=99999` | MTC's `execute()` hides the real code | The script calls the action itself and prints `REAL error_code.val = ...` |
| `error_code.val = -4` (CONTROL_FAILED), controller log `Time between points 0 and 1 is not strictly increasing` | Trajectory points without timestamps (OMPL and gripper segments) | `retime_solution()` in `mtc_pick_and_place.py` (already present). If it reappears, check the retime block still runs **after** the last `goal.solution = ...` line |
| The panel shows nothing | Wrong topic | `Task Solution Topic` = `/solution` with a leading slash |
| The arm is far too fast or slow | The speed constants | `ARM_MAX_VEL` in `mtc_pick_and_place.py` ([docs/06](06_tuning_reference.md)); not RViz scaling, not `joint_limits.yaml` |
| The panel's Execute button fails or is fast | It sends the untimed raw plan | Use `MTC_EXECUTE=y ./run_pick_and_place.sh` |
| Step 0 says "Reached" but the real arm did not move | **(likely)** The control path is closed (after a `STOP`) and/or the script read the simulated joint state | Restart everything with the arm at home and `JOINT_STATES_TOPIC=/feedback/joint_states` ([docs/05](05_run_real_arm.md) section 5) |
| After `STOP` the arm will not move | The control path stays closed | Restart (preferred) or `ros2 service call /control_enable std_srvs/srv/SetBool "{data: true}"` only if the arm is where the simulated controller thinks it is |
| The two joint-state topics disagree at start | The arm was not at `home` when launched | Stop everything, put the arm at `home`, restart |

## F. Gripper frame

| Symptom | Cause | Fix |
|---|---|---|
| The gripper approaches sideways, or fingers point up | `tcp_link` is attached to `link7` without the `tcp_offset` | `tcp_offset:="[0.037, 0, -0.0235, -1.5708, 0, -1.5708]"` must be passed to **both** the MoveIt launch and the MTC launch. `launch/scripts/launch_arm.sh` / `launch_arm_real.sh` do the first; `run_pick_and_place.sh` does the second. Check the MTC copy with `launch/scripts/check_tcp_in_mtc.py` |
| `tcp_offset_xyz` / `tcp_offset_rpy` "not found" | Those are not launch arguments | A single `tcp_offset:="[x, y, z, roll, pitch, yaw]"` |

## G. Point cloud and calibration

| Symptom | Cause | Fix |
|---|---|---|
| Cloud and arm model do not overlap | Calibration error | [docs/03](03_calibration.md) |
| Cloud jumps between two positions | Two calibration publishers | `pkill -f "easy_handeye2 publish.launch.py"; pkill -f handeye_publisher`, start one |
| `tf2_echo base_link topcam_link`: "Could not find a connection ... two or more unconnected trees" | No calibration publisher is running | Start it: `ros2 launch easy_handeye2 publish.launch.py name:=nero_eye_to_hand_calib` |
| Cloud vanished after editing the `.calib` file | A hand-typed quaternion is not unit length, or a typo (missing minus sign or colon) | Check the length and normalise (below), then restart the publisher |
| Editing the file changed nothing | The file is read only when the publisher starts | Restart the publisher |

Check and normalise the calibration quaternion:

```bash
C=~/.ros2/easy_handeye2/calibrations/nero_eye_to_hand_calib.calib
awk '/rotation:/{r=1} r&&$1=="x:"{x=$2} r&&$1=="y:"{y=$2} r&&$1=="z:"{z=$2} r&&$1=="w:"{w=$2} END{printf "quaternion length: %.6f (must be 1.000000)\n", sqrt(x*x+y*y+z*z+w*w)}' $C
cp $C $C.bak_pre_norm
awk 'FNR==NR{ if($1=="rotation:")r=1; if(r&&$1=="x:")x=$2; if(r&&$1=="y:")y=$2; if(r&&$1=="z:")z=$2; if(r&&$1=="w:")w=$2; next }
FNR==1{n=sqrt(x*x+y*y+z*z+w*w)}
{ if($1=="rotation:")q=1;
  if(q&&$1=="x:"){printf "    x: %.12f\n",x/n;next} if(q&&$1=="y:"){printf "    y: %.12f\n",y/n;next}
  if(q&&$1=="z:"){printf "    z: %.12f\n",z/n;next} if(q&&$1=="w:"){printf "    w: %.12f\n",w/n;next}
  print }' $C $C > $C.new && mv $C.new $C
```
