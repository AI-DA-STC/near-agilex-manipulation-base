#!/bin/bash
# FREEZE the arm where it is (software stop - this arm has NO physical e-stop).
#   /control_enable false -> the driver ignores further motion commands (it stays closed until you reopen it)
#   /emergency_stop       -> the driver commands the arm to hold its CURRENT joint angles (not a power cut)
#   cancel_goal calls      -> cancel any running trajectory / MTC execution so MoveIt stops streaming
# Reopen the control path afterwards with:
#   ros2 service call /control_enable std_srvs/srv/SetBool "{data: true}"
# After a STOP, restart the pipeline with the arm back at 'home' (docs/05_run_real_arm.md).
# Install an alias:  echo "alias STOP='~/nero_pipeline_launch/stop_arm.sh'" >> ~/.bashrc
# FREEZE the arm where it is. All calls run in parallel; each gives up after 8 s so nothing can hang.
source /opt/ros/humble/setup.bash 2>/dev/null
timeout 8 ros2 service call /control_enable std_srvs/srv/SetBool "{data: false}" &
timeout 8 ros2 service call /emergency_stop std_srvs/srv/Empty &
timeout 8 ros2 service call /arm_controller/follow_joint_trajectory/_action/cancel_goal action_msgs/srv/CancelGoal "{}" &
timeout 8 ros2 service call /execute_task_solution/_action/cancel_goal action_msgs/srv/CancelGoal "{}" &
timeout 8 ros2 service call /execute_trajectory/_action/cancel_goal action_msgs/srv/CancelGoal "{}" &
wait
echo "STOP sent. Check the arm is holding: ros2 topic echo /feedback/joint_states --once"
