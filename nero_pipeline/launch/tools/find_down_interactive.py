#!/usr/bin/env python3
"""
Tries candidate orientations ONE AT A TIME via real execution, pausing
after each for YOUR OWN visual check in RViz. Type 'y' if the gripper
is genuinely pointing straight down, 'n' to move on to the next
candidate. On 'y', prints the exact quaternion and stops immediately.

Run with plain system Python (conda deactivate first):
    python3 find_down_interactive.py
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

TEST_POSITION = (-0.490, -0.103, 0.30)

CANDIDATES = [
    ("identity",    (0.0, 0.0, 0.0, 1.0)),
    ("flip_x",      (1.0, 0.0, 0.0, 0.0)),
    ("flip_y",      (0.0, 1.0, 0.0, 0.0)),
    ("flip_z",      (0.0, 0.0, 1.0, 0.0)),
    ("rot_y_neg90", (0.0, -0.7071, 0.0, 0.7071)),
    ("rot_y_pos90", (0.0, 0.7071, 0.0, 0.7071)),
    ("rot_x_neg90", (-0.7071, 0.0, 0.0, 0.7071)),
    ("rot_x_pos90", (0.7071, 0.0, 0.0, 0.7071)),
    ("rot_z_neg90", (0.0, 0.0, -0.7071, 0.7071)),
    ("rot_z_pos90", (0.0, 0.0, 0.7071, 0.7071)),
]


class Tester(Node):
    def __init__(self):
        super().__init__("find_down_interactive")
        self.client = ActionClient(self, MoveGroup, "move_action")

    def try_orientation(self, name, quat):
        print(f"\n--- Trying '{name}': quaternion {quat} ---")
        if not self.client.wait_for_server(timeout_sec=10.0):
            print("move_action server not available!")
            return False

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = TEST_POSITION
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quat

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 8.0

        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "tcp_link"
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose.pose)
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "tcp_link"
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            print(f"'{name}': REJECTED - skipping")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
        result = result_future.result()
        if result.result.error_code.val != 1:
            print(f"'{name}': FAILED to execute (error_code={result.result.error_code.val}) - skipping")
            return False

        time.sleep(0.5)
        return True


def main():
    rclpy.init()
    node = Tester()

    for name, quat in CANDIDATES:
        reached = node.try_orientation(name, quat)
        if not reached:
            continue  # automatically skip ones that failed to even execute

        answer = input(f">>> Is '{name}' pointing STRAIGHT DOWN? [y/n]: ").strip().lower()
        if answer == "y":
            print(f"\n{'='*50}")
            print(f"FOUND IT: '{name}'")
            print(f"Quaternion (x, y, z, w) = {quat}")
            print(f"{'='*50}")
            break
    else:
        print("\nNone of the candidates were confirmed as pointing down.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
