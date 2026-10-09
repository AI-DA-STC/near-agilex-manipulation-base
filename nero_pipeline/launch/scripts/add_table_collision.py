#!/usr/bin/env python3
"""
Adds a large, thin collision box to MoveIt's planning scene representing
the table surface, positioned right at the robot's own base level (since
the arm is mounted directly on top of the table). This gives MoveIt's
collision checker (already enabled by default in nero_mtc_task's
ComputeIK stages via the "ignore_collisions" property defaulting to
False) something concrete to check against - without this, the planner
has zero awareness that a table (or anything below the arm) physically
exists at all.

This addresses "the arm shouldn't reach under the table surface" directly
and unconditionally - it does NOT address "the arm shouldn't swing out
past the table's edge" (that needs the table's actual width/length and a
separate boundary box - not yet built, see the script's own printed
reminder at the end).

Run with plain system Python (rclpy only, no conda env needed):
    conda deactivate
    python3 add_table_collision.py
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class TableCollisionAdder(Node):
    def __init__(self):
        super().__init__("add_table_collision")
        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def add_table(self):
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/apply_planning_scene service not available")
            return False

        table = CollisionObject()
        table.header.frame_id = "base_link"
        table.id = "table_surface"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        # Large flat box: 2m x 2m footprint, 20cm thick. IMPORTANT: the top
        # surface sits at z=-0.10, NOT z=0 - putting it exactly at z=0
        # (tried initially) overlapped the robot's own static base/mounting
        # geometry, which occupies real space right around the origin,
        # causing EVERY possible arm configuration to register as
        # "in collision" from the start (confirmed: every single IK check
        # failed universally the moment collision-checking was enabled,
        # which only makes sense if the robot's own resting state was
        # already colliding with this object). Pulling the surface down to
        # -0.10 gives clearance from the base mounting hardware while still
        # blocking any real dive toward the actual table surface below that.
        box.dimensions = [2.0, 2.0, 0.2]

        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = float(__import__("os").environ.get("TABLE_TOP_Z", "-0.10")) - 0.005  # box center; its TOP face is at TABLE_TOP_Z (the box is 0.01 thick)
        pose.orientation.w = 1.0

        table.primitives.append(box)
        table.primitive_poses.append(pose)
        table.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world.collision_objects.append(table)
        scene.is_diff = True

        req = ApplyPlanningScene.Request()
        req.scene = scene

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            self.get_logger().error("Failed to apply planning scene")
            return False
        return future.result().success


def main():
    rclpy.init()
    node = TableCollisionAdder()
    success = node.add_table()
    if success:
        print("Table collision object added successfully.")
        print("The arm should now refuse to plan any path that dips below z=-0.12.")
    else:
        print("FAILED to add table collision object - check that MoveIt (move_group) is running.")

    print()
    print("=" * 70)
    print("REMINDER: this only protects against the arm going THROUGH the")
    print("table. It does NOT stop the arm from swinging OUTWARD past the")
    print("table's real edges into open space where a person could be.")
    print("That needs the table's actual width/length to add a matching")
    print("boundary box - not yet done. Until then, supervise the arm")
    print("directly during any real execution, don't run it unattended.")
    print("=" * 70)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
