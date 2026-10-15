# 05. Run the real arm

**Read section 1 completely before the arm moves.** The simulation steps in [docs/04](04_run_simulation.md) must already work.

## 1. Safety protocol

This arm has **no physical emergency stop**. What exists instead:

| Measure | Details |
|---|---|
| Software stop | `launch/stop_arm.sh` (alias `STOP`). Closes the driver's control path, commands the arm to hold its current joint angles, cancels running trajectories. It is **a hold, not a power cut**, takes about a second to act, and works only while ROS and the driver respond |
| Last resort 1 | `pkill -9 -f agx_arm_ctrl_single` stops all commands from reaching the arm (nothing in the driver code disables the motors on exit; what your arm does then was not tested) |
| Last resort 2 | Cut the arm's power. **It may drop** (not tested whether the joints hold without power). Keep the area clear |
| Low speed | `ARM_MAX_VEL = 0.1` rad/s per joint (about 6 degrees per second), so a stop within 2 seconds limits further travel to roughly 11 degrees. **Do not raise it until the whole chain is trusted** |

Rules:

1. Open a **STOP terminal** before anything runs: `source ~/.bashrc; conda deactivate`, type `STOP` and do **not** press Enter. Keep the window visible while the arm moves.
2. **Plan only first** (no `MTC_EXECUTE=y`), look at the plan in RViz, then execute.
3. The **physical arm must be at the `home` pose when you launch.** The simulated controller inside MoveIt starts at `home`
   (`initial_positions.yaml`) and the real arm follows it. If they differ, the arm moves toward the simulated pose as soon as
   the driver's control path is open. Start of every session: put the arm at `home`; end of every session: plan to `home` in RViz.
   (This is our understanding of how AgileX's `follow` setup behaves; it was **not verified in AgileX's source**, so treat it as a precaution that has been followed on every run.)
4. The control path is **always open** (we run the driver with `control_enabled:=true`; AgileX's `auto_control_gate` option was
   dropped because its helper executable `agx_arm_control_gate` was missing from the installed package). Consequence: the
   arm follows MoveIt's controller continuously.
5. **After any STOP, restart everything** (section 5). The simulated controller keeps going after the real arm stops, so
   they no longer agree.
6. Step 0 of `run_pick_and_place.sh` is a **direct joint move without collision checking** (about 100 degrees on two joints
   from `home` to `pose_90deg`). Clear the workspace first. It is slowed by `HOME_MOVE_SECONDS=15`.
7. The table collision box only prevents going **through** the table. The table's **edges** are not modelled, so the arm can swing
   past them. Never run unattended.
8. The grasped object has **no collision geometry** (MTC prints `Attempting to attach object 'carrot' ... no geometry specified`
   on every run), so nothing avoids the held object.
9. Never use the RViz panel's Execute button on the real arm (it sends the raw plan without the retiming).

## 2. Hardware preparation

1. Arm powered, indicator on, workspace clear.
2. CAN interface up (full procedure and the `gs_usb` module in [docs/02](02_install_and_environment.md) section 2.2):

```bash
lsmod | grep gs_usb || sudo insmod ~/gs_usb_build/gs_usb.ko
cd ~/agx_arm_ws/src/agx_arm_ros/scripts
bash find_all_can_port.sh
bash can_activate.sh can_piper 1000000 "<usb_port>"
timeout 3 candump can_piper | head -3        # frames must appear
```

3. If `candump` shows nothing: arm power or cable, wrong adapter port (re-run `find_all_can_port.sh`), or a replugged adapter
   (the interface number changes; `can_activate.sh` fixes it).
4. Arm at `home`.

## 3. Start (four terminals)

> **Update 2026-10-14:** the sequence that was used on the real arm after the perception changes is in [docs/12](12_current_run_sequence.md) part 4. It differs from the one below in four ways:
> `unset TABLE_TOP_Z` and then the **exact table box** (`launch/scripts/add_table_box.py --top_z 0.0`) instead of the built-in box, whose top face is not where its comment says ([docs/11](11_perception_workers_frames_recentring.md) section 4);
> a readiness check for the two persistent workers; a check that the camera's depth is registered (depth `640x480`); and the grasp test with the `FINGER TIPS` check before anything moves.
> The safety protocol in section 1 applies unchanged.

**Terminal 1: launch everything**

```bash
pkill -f agx_arm_ctrl_single; pkill -9 -f move_group; pkill -f demo.launch; pkill -f publish.launch.py; pkill -f handeye_publisher; pkill -f orbbec_camera; pkill -f mtc_pick_and_place; pkill -f grasp_watch; pkill -f grasp_view
pgrep -af "agx_arm_ctrl_single|move_group|rviz2|orbbec|handeye|mtc_pick|grasp_" || echo clean

conda deactivate
export REAL_ARM=1 ARM_WAIT=30          # (add TABLE_TOP_Z=<metres> only if you want a table height other than the built-in -0.10)
cd ~/nero_pipeline_launch && ./start_pipeline.sh
```

Wait for `=== All nodes started ===` **and** for RViz (MoveIt starts about 10 to 20 seconds after the driver; `ARM_WAIT=30`
makes the pipeline wait for it before adding the table). Always clear stale driver processes first: two drivers on one CAN bus fight each other.

**Terminal 2: checks and the STOP command (keep open)**

```bash
source ~/.bashrc; conda deactivate
ros2 topic echo /control/joint_states --once        # simulated controller
ros2 topic echo /feedback/joint_states --once       # real arm
/usr/bin/python3 ~/nero_pipeline_launch/scripts/add_table_collision.py
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{}" | grep -c table_surface
```

The two joint-state outputs **must agree to about 0.01 rad**; the last number must be above 0; the point cloud must overlap the arm
model in RViz. If any check fails, stop. Then type `STOP` without pressing Enter.

**Terminal 3 (optional): grasp watcher**

```bash
conda deactivate
source ~/agx_arm_ws/install/setup.bash
export JOINT_STATES_TOPIC=/feedback/joint_states
/usr/bin/python3 ~/nero_pipeline_launch/tools/grasp_watch.py
```

**Terminal 4: run**

```bash
conda deactivate
export JOINT_STATES_TOPIC=/feedback/joint_states HOME_MOVE_SECONDS=15
cd ~/nero_pipeline_launch && ./run_pick_and_place.sh                  # plan only
```

Check the plate is within reach before executing (it must be under about 0.55 m from the base, [docs/09](09_known_issues_and_next_steps.md)):

```bash
ros2 topic echo /target_pose_transformed --once --field pose.position | awk '/x:/{x=$2} /y:/{y=$2} END{printf "plate radius from the base axis: %.3f m (keep it under about 0.55)\n", sqrt(x*x+y*y)}'
```

Then (Ctrl+C the waiting plan first):

```bash
MTC_EXECUTE=y ./run_pick_and_place.sh
```

## 4. What the real launch does and why it is built this way

`launch/scripts/launch_arm_real.sh`:

1. kills stale drivers, starts the driver `agx_arm_ctrl start_single_agx_arm.launch.py can_port:=can_piper arm_type:=nero effector_type:=agx_gripper control_enabled:=true`,
2. waits (up to 20 attempts) until `/feedback/joint_states` publishes. (`ros2 topic echo` exits immediately if the topic does not exist
   yet, so it is retried in a loop; an earlier version checked once and aborted too early),
3. starts MoveIt: `agx_arm_moveit demo.launch.py ... follow:=true capabilities:=... tcp_offset:=...` (the same launch file as the simulation),
4. stops the other process if either dies.

Why not AgileX's combined `start_single_agx_arm_moveit.launch.py`: it does not forward the `capabilities` argument, so
`execute_task_solution` would not exist. It does forward `tcp_offset`.

The driver reads the firmware version from the arm at start (CAN id `0x4AF`) and **exits if the arm does not answer within about 5
seconds** (`Failed to get firmware version`). This happened on 2026-10-06 and cleared after re-trying; a power-cycle of the arm is the first thing to try.
`auto_enable` (default true) switches the motors on at start so the arm holds its pose; it does not command any motion.

## 5. After a STOP, an error, or at the end of the session

Return the arm to `home` before restarting anything:
* If MoveIt is still running and healthy: RViz, Goal State `home`, arm group, Plan and Execute.
* Otherwise move it with a small, slow, supervised command and then restart.

Shutdown:

```bash
# Ctrl+C in terminals 3 and 1, then:
pkill -f agx_arm_ctrl_single; pkill -9 -f move_group; pkill -f demo.launch; pkill -f publish.launch.py; pkill -f handeye_publisher; pkill -f orbbec_camera
unset REAL_ARM TABLE_TOP_Z JOINT_STATES_TOPIC HOME_MOVE_SECONDS ARM_WAIT
```

To reopen the control path after a STOP without restarting (only if the arm is still where the simulated controller thinks it is):

```bash
ros2 service call /control_enable std_srvs/srv/SetBool "{data: true}"
```

## 6. Environment variables used by the scripts

| Variable | Set where | Meaning | Real-arm value |
|---|---|---|---|
| `REAL_ARM` | before `start_pipeline.sh` | `1` selects `launch_arm_real.sh` | `1` |
| `TABLE_TOP_Z` | before `start_pipeline.sh` and when re-running `add_table_collision.py` | Height of the table box's top face in `base_link`. **Unset = the script's built-in height (-0.10).** The override was added late, so the runs in which the grasps worked used the built-in height | Optional. `-0.03` is a suggested starting point for a higher (more realistic) table, **not validated**; raise it in small steps and expect more collision failures |
| `ARM_WAIT` | before `start_pipeline.sh` | Seconds to wait after launching the arm before the table step | `30` |
| `JOINT_STATES_TOPIC` | before `run_pick_and_place.sh` and the watcher | Joint state topic read by `move_to_pose_90deg.py` | `/feedback/joint_states` |
| `HOME_MOVE_SECONDS` | before `run_pick_and_place.sh` | Duration of the step-0 move | `15` |
| `MTC_EXECUTE` | before `run_pick_and_place.sh` | `y` executes the plan; anything else plans only | `y` for execution |
| `START_POSE` | before `run_pick_and_place.sh` | Start pose for step 0: `initial` or `pose_90deg`. **The lab machine runs `initial`** (operator report 2026-10-14), so the pipeline starts from it every time. Needs the updated `move_to_pose_90deg.py` | `initial` |
| `CGN_FORWARD_PASSES`, `CGN_RECENTRE`, `CGN_DEPTH` | before `start_pipeline.sh` | Override the grasp node's launch defaults: 5 passes, re-centring `true`, closing depth 0.006 m ([docs/06](06_tuning_reference.md) section 5) | unset |
| `ASK_PROMPTS` | before `run_pick_and_place.sh` | Only with the future-work prompt question (not installed): `0` skips it, `force` asks even without a terminal | n/a |
