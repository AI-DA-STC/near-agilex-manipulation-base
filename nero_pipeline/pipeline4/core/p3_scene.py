#!/usr/bin/env python3
"""
p3_scene.py - what MoveIt's planning scene REALLY holds, and clearing pipeline 3's obstacle boxes - through MoveIt's services, which answer with
success or failure (a plain message on /planning_scene can be lost when it is sent before the connection to move_group is made).

    conda deactivate; source ~/agx_arm_ws/install/setup.bash
    /usr/bin/python3 ~/nero_pipeline_launch/pipeline3/p3_scene.py show     # every collision object MoveIt holds (read-only)
    /usr/bin/python3 ~/nero_pipeline_launch/pipeline3/p3_scene.py clear    # remove pipeline 3's obstacle boxes (p3_obst_*), then confirm

The table (table_surface) is never touched. The next p3_look.sh rebuilds the obstacle boxes from a fresh picture.
"""
import sys
import time

P3_SCENE_VERSION = "p3-scene-v2"
PREFIX = "p3_obst_"


def describe_object(co):
    """'p3_obst_0  BOX 0.08 x 0.09 x 0.059 m  centre (-0.222, -0.205, +0.019)' - centre = object pose + shape pose (MoveIt 2 reports both)."""
    if not co.primitives:
        return "%-14s (no primitive shapes)" % co.id
    p, q = co.primitive_poses[0].position, co.pose.position
    d = list(co.primitives[0].dimensions)
    return "%-14s BOX %s m  centre (%+.3f, %+.3f, %+.3f)" % (co.id, " x ".join("%.2f" % v for v in d[:3]), q.x + p.x, q.y + p.y, q.z + p.z)


class Scene:
    """Thin wrapper around move_group's /get_planning_scene and /apply_planning_scene (needs rclpy initialised)."""

    def __init__(self, node, timeout=10.0):
        from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
        self.node, self.timeout = node, timeout
        self.get = node.create_client(GetPlanningScene, "/get_planning_scene")
        self.apply = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.GetPlanningScene, self.ApplyPlanningScene = GetPlanningScene, ApplyPlanningScene

    def _call(self, client, req, what):
        import rclpy
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError("MoveIt's %s service is not available - is pipeline 3 (move_group) running?" % what)
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=self.timeout)
        if not fut.done() or fut.result() is None:
            raise RuntimeError("no answer from MoveIt's %s service within %.0f s" % (what, self.timeout))
        return fut.result()

    def objects(self):
        """-> list of CollisionObject MoveIt holds now"""
        from moveit_msgs.msg import PlanningSceneComponents
        req = self.GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        return list(self._call(self.get, req, "/get_planning_scene").scene.world.collision_objects)

    def obstacle_ids(self):
        return sorted((co.id for co in self.objects() if co.id.startswith(PREFIX)), key=lambda s: int(s[len(PREFIX):]) if s[len(PREFIX):].isdigit() else 0)

    def remove(self, ids):
        """Remove these ids (they should exist); True if MoveIt reports success."""
        return self.replace(ids, [])

    def replace(self, remove_ids, boxes):
        """ONE confirmed change: remove remove_ids and add boxes (dicts with "centre" and "size", base_link) as p3_obst_0.. -> MoveIt's success."""
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import CollisionObject, PlanningScene
        from shape_msgs.msg import SolidPrimitive
        ps = PlanningScene()
        ps.is_diff = True
        for i in remove_ids:
            co = CollisionObject()
            co.id, co.header.frame_id, co.operation = i, "base_link", CollisionObject.REMOVE
            ps.world.collision_objects.append(co)
        for k, b in enumerate(boxes):
            co = CollisionObject()
            co.id, co.header.frame_id, co.operation = PREFIX + str(k), "base_link", CollisionObject.ADD
            co.pose.orientation.w = 1.0
            co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in b["size"]])]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = (float(v) for v in b["centre"])
            pose.orientation.w = 1.0
            co.primitive_poses = [pose]
            ps.world.collision_objects.append(co)
        req = self.ApplyPlanningScene.Request()
        req.scene = ps
        return bool(self._call(self.apply, req, "/apply_planning_scene").success)

    def set_obstacles(self, boxes):
        """v2: make MoveIt hold EXACTLY these boxes (p3_obst_0 .. p3_obst_<n-1>) and nothing else of pipeline 3's, then read back to confirm.
        Removes every p3_obst_* box MoveIt really holds (stale ones too), adds the new ones in the same change. -> (confirmed, message)"""
        before = self.obstacle_ids()
        ok = self.replace(before, boxes)
        after = self.obstacle_ids()
        want = [PREFIX + str(k) for k in range(len(boxes))]
        if ok and after == want:
            return True, "MoveIt confirmed %d obstacle box(es)%s" % (len(boxes), "" if not before else " (replaced %d from before)" % len(before))
        return False, "MoveIt NOT confirmed: it %s the change and holds %s, expected %s" % (
            "accepted" if ok else "REFUSED", ", ".join(after) or "none", ", ".join(want) or "none")


class AsyncScene(Scene):
    """v2: the same, usable from INSIDE a ROS service callback: its own small node is spun by its own executor in a background thread, so
    waiting for MoveIt's answer cannot block the caller's executor (which would deadlock)."""

    def __init__(self, timeout=10.0):
        import threading
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        node = rclpy.create_node("p3_scene_client")
        super().__init__(node, timeout)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def _call(self, client, req, what):
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError("MoveIt's %s service is not available - is move_group running?" % what)
        fut = client.call_async(req)
        t_end = time.time() + self.timeout
        while not fut.done():
            if time.time() > t_end:
                raise RuntimeError("no answer from MoveIt's %s service within %.0f s" % (what, self.timeout))
            time.sleep(0.01)
        if fut.result() is None:
            raise RuntimeError("MoveIt's %s service returned nothing" % what)
        return fut.result()


def clear(scene, out=print):
    """Remove every p3_obst_* box, then read the scene back. -> 0 if none is left."""
    ids = scene.obstacle_ids()
    if not ids:
        out("no pipeline 3 obstacle boxes in MoveIt's planning scene - nothing to clear")
        return 0
    ok = scene.remove(ids)
    time.sleep(0.3)
    left = scene.obstacle_ids()
    if ok and not left:
        out("CLEARED %d obstacle box(es) (%s) - confirmed: MoveIt holds none now" % (len(ids), ", ".join(ids)))
        return 0
    out("NOT CLEARED: MoveIt %s the removal and still holds %s" % ("accepted" if ok else "REFUSED", ", ".join(left) or "none"))
    return 1


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    if not args or args[0] not in ("show", "clear"):
        print("usage: p3_scene.py show | clear")
        return 2
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node("p3_scene")
    try:
        sc = Scene(node)
        print("p3_scene %s" % P3_SCENE_VERSION)
        if args[0] == "clear":
            return clear(sc)
        objs = sc.objects()
        print("MoveIt's planning scene holds %d collision object(s):" % len(objs))
        for co in sorted(objs, key=lambda c: (not c.id.startswith(PREFIX), c.id)):
            print("  " + describe_object(co))
        print("pipeline 3 obstacle boxes: %d" % sum(1 for co in objs if co.id.startswith(PREFIX)))
        return 0
    except RuntimeError as e:
        print("STOP: %s" % e)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
