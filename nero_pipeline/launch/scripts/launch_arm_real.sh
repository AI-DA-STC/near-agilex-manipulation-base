#!/bin/bash
# Real arm: AgileX driver + the SAME MoveIt demo.launch.py as the simulation (see launch_arm.sh).
# Used by start_pipeline.sh when REAL_ARM=1.
#
#  1. starts the driver (agx_arm_ctrl_single) which talks to the arm over CAN (can_piper)
#  2. waits until /feedback/joint_states is publishing (the driver died or the arm is silent otherwise)
#  3. starts MoveIt with follow:=true (plans from the real arm's feedback)
#  4. if either process dies, stops the other one (and any stale driver) so nothing keeps running
#
# SAFETY: the driver's control path is OPEN (control_enabled:=true), i.e. the arm follows MoveIt's
# controller from the moment this runs. The arm must physically be at the 'home' pose when you start.
# See docs/05_run_real_arm.md.
export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
source ~/agx_arm_ws/install/setup.bash
pkill -f agx_arm_ctrl_single 2>/dev/null; sleep 1
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  can_port:=can_piper arm_type:=nero effector_type:=agx_gripper \
  control_enabled:=true &
DRIVER=$!
MOVEIT=""
cleanup() {
  trap - INT TERM EXIT
  kill -TERM $MOVEIT $DRIVER 2>/dev/null
  pkill -f agx_arm_ctrl_single 2>/dev/null
  wait 2>/dev/null
}
trap cleanup INT TERM EXIT
echo "[launch_arm_real] waiting for the arm's feedback..."
READY=0
for i in $(seq 1 20); do
  if timeout 5 ros2 topic echo /feedback/joint_states --once >/dev/null 2>&1; then READY=1; break; fi
  if ! kill -0 $DRIVER 2>/dev/null; then echo "[launch_arm_real] driver process died"; break; fi
  sleep 1
done
if [ "$READY" != 1 ]; then
  echo "[launch_arm_real] NO feedback from the arm - not starting MoveIt."
  exit 1
fi
echo "[launch_arm_real] feedback OK - starting MoveIt"
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=nero effector_type:=agx_gripper \
  follow:=true \
  capabilities:="move_group/ExecuteTaskSolutionCapability" \
  tcp_offset:="[0.037, 0, -0.0235, -1.5708, 0, -1.5708]" &
MOVEIT=$!
wait -n $DRIVER $MOVEIT
