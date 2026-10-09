#!/usr/bin/env python3
"""
Tests several candidate orientations by commanding the REAL arm (via
MoveGroup pose-goal planning, executed) and checking gripper_link1's
OWN resulting orientation afterward - the actual finger geometry, not
tcp_link's theoretical frame. This sidesteps any risk of composition-
math errors in deriving the relationship analytically.

Run with plain system Python (conda deactivate first):
    python3 find_real_down_orientation.py
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
import tf2_ros

TEST_POSITION = (-0.490, -0.103, 0.30)  # safely elevated, known-reachable area

CANDIDATES = [
    ("identity",   (0.0, 0.0, 0.0, 1.0)),
    ("flip_x",     (1.0, 0.0, 0.0, 0.0)),
    ("flip_y",     (0.0, 1.0, 0.0, 0.0)),
    ("flip_z",     (0.0, 0.0, 1.0, 0.0)),
    ("rot_y_neg90",(0.0, -0.7071, 0.0, 0.7071)),
    ("rot_y_pos90",(0.0, 0.7071, 0.0, 0.7071)),
    ("rot_x_neg90",(-0.7071, 0.0, 0.0, 0.7071)),
    ("rot_x_pos90",(0.7071, 0.0, 0.0, 0.7071)),
]


class Tester(Node):
    def __init__(self):
        super().__init__("find_real_down_orientation")
        self.client = ActionClient(self, MoveGroup, "move_action")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def try_orientation(self, name, quat):
        print(f"\n--- Trying '{name}': quaternion {quat} ---")
        if not self.client.wait_for_server(timeout_sec=10.0):
            print("move_action server not available!")
            return

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
            print(f"'{name}': REJECTED")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
        result = result_future.result()
        if result.result.error_code.val != 1:
            print(f"'{name}': FAILED (error_code={result.result.error_code.val})")
            return

        time.sleep(1.0)  # let TF settle
        try:
            t = self.tf_buffer.lookup_transform("base_link", "gripper_link1", rclpy.time.Time())
            q = t.transform.rotation
            # Compute local Z axis in world frame directly from quaternion
            zx = 2 * (q.x * q.z + q.w * q.y)
            zy = 2 * (q.y * q.z - q.w * q.x)
            zz = 1 - 2 * (q.x * q.x + q.y * q.y)
            print(f"'{name}': gripper_link1's local Z axis in world frame = ({zx:.3f}, {zy:.3f}, {zz:.3f})")
            if zz < -0.9:
                print(f"    *** THIS ONE POINTS DOWN! ***")
        except Exception as e:
            print(f"Could not get gripper_link1 transform: {e}")


def main():
    rclpy.init()
    node = Tester()
    for name, quat in CANDIDATES:
        node.try_orientation(name, quat)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
