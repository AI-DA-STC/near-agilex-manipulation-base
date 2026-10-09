#!/usr/bin/env python3
"""
Ensures the arm is at the known 'pose_90deg' SRDF state before task
planning starts - checks current /joint_states first, and only sends a
move command if it isn't already close enough, so this is safe to run
every single time without unnecessary motion.

Uses a direct FollowJointTrajectory action goal (same proven pattern as
agx_carrot_demo/agx_jacket_demo earlier in this project) - NOT OMPL/MTC
planning, since the target joint values are already known exactly. This
is deterministic: same starting point every time, every run.

Run with plain system Python (rclpy only, no conda env needed):
    conda deactivate
    python3 move_to_pose_90deg.py
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# pose_90deg's known joint values (from the custom SRDF state defined
# early in this project).
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
POSE_90DEG = [-3.49e-05, -0.0063, -5.24e-05, 1.5674, -0.0759, 0.0026, 0.0403]

TOLERANCE_RAD = 0.02  # ~1.1 degrees - close enough to skip moving
MOVE_DURATION_SEC = float(os.environ.get("HOME_MOVE_SECONDS", "4.0"))  # how long the trajectory takes to execute


class PoseNormalizer(Node):
    def __init__(self):
        super().__init__("move_to_pose_90deg")
        self.current_positions = None
        self.create_subscription(JointState, os.environ.get("JOINT_STATES_TOPIC", "/control/joint_states"), self._joint_state_cb, 10)
        self.action_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )

    def _joint_state_cb(self, msg):
        # Only keep the arm's 7 joints, in our known order - /joint_states
        # can report joints in any order and can include the gripper too.
        try:
            self.current_positions = [
                msg.position[msg.name.index(j)] for j in JOINT_NAMES
            ]
        except ValueError:
            pass  # not all joints present yet in this message

    def wait_for_joint_state(self, timeout_sec=10.0):
        start = time.time()
        while self.current_positions is None:
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - start > timeout_sec:
                return False
        return True

    def is_already_at_pose_90deg(self):
        return all(
            abs(cur - target) < TOLERANCE_RAD
            for cur, target in zip(self.current_positions, POSE_90DEG)
        )

    def move_to_pose_90deg(self):
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/arm_controller/follow_joint_trajectory not available")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = POSE_90DEG
        point.time_from_start.sec = int(MOVE_DURATION_SEC)
        point.time_from_start.nanosec = int((MOVE_DURATION_SEC % 1) * 1e9)
        goal.trajectory.points = [point]

        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=MOVE_DURATION_SEC + 5.0)
        return result_future.result() is not None


def main():
    rclpy.init()
    node = PoseNormalizer()

    if not node.wait_for_joint_state():
        print("Timed out waiting for /joint_states - is the arm running?")
        sys.exit(1)

    if node.is_already_at_pose_90deg():
        print("Already at pose_90deg - no movement needed.")
    else:
        print(f"Not at pose_90deg (current: {node.current_positions}) - moving there now...")
        if node.move_to_pose_90deg():
            print("Reached pose_90deg.")
        else:
            print("FAILED to reach pose_90deg - check the arm_controller is running.")
            sys.exit(1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
