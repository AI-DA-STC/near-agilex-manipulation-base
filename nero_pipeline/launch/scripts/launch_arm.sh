#!/bin/bash
# Simulation arm: MoveIt demo (fake ros2_control hardware) + RViz.
# Used by start_pipeline.sh when REAL_ARM is not set to 1.
#
#  tcp_offset        [x, y, z, roll, pitch, yaw] of tcp_link relative to link7. These numbers are the
#                    gripper_base frame (link7 -> gripper_flange -> gripper_base in the URDF), so that
#                    tcp_link's +Z axis is the real approach axis of the gripper. This is a SINGLE launch
#                    argument holding a Python list; "tcp_offset_xyz"/"tcp_offset_rpy" are NOT launch arguments.
#  capabilities      loads move_group's ExecuteTaskSolution action server. Without it MTC cannot execute.
exec ros2 launch agx_arm_moveit demo.launch.py \
    arm_type:=nero \
    effector_type:=agx_gripper \
    capabilities:="move_group/ExecuteTaskSolutionCapability" \
    tcp_offset:="[0.037, 0, -0.0235, -1.5708, 0, -1.5708]"
