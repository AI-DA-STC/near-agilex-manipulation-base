#!/usr/bin/env python3
"""
add_table_box.py - show, set or remove MoveIt's 'table_surface' collision box, with the top face EXACTLY where you say. (system python, MoveIt must be running)

  --show                       print every collision object in MoveIt's scene: size, centre, TOP FACE height and footprint (changes nothing)
  --top_z 0.0                  replace 'table_surface' by a box whose top face is exactly at z = 0.0 m in base_link
        [--x_min -1.1 --x_max -0.20 --y_half 0.7 --thickness 0.05]    the footprint: only IN FRONT of the arm (x from x_min to x_max), so the box
                                                                      cannot overlap the arm's own base (a 2 m x 2 m box centred on the base did, which
                                                                      is why raising it used to make every plan fail)
  --remove                     delete the box (to test without a table)

Why this exists: add_table_collision.py builds a box 0.2 m thick centred at TABLE_TOP_Z - 0.005, so its real top face is 9.5 cm ABOVE TABLE_TOP_Z, not at it.
"""
import argparse, sys

DEFAULT_ID = "table_surface"


def box_for(top_z, x_min, x_max, y_half, thickness):
    """-> (dims [dx, dy, dz], centre (x, y, z)) of a box whose TOP face is exactly at top_z"""
    dims = [x_max - x_min, 2.0 * y_half, thickness]
    return dims, ((x_min + x_max) / 2.0, 0.0, top_z - thickness / 2.0)


def overlaps_base(x_min, x_max, y_half, radius=0.20):
    """True if the footprint reaches the area around the arm's base (|x|, |y| < radius)"""
    return x_min < radius and x_max > -radius and y_half > 0.0


def describe(obj_id, ptype, dims, pos):
    if ptype == 1:                                                    # SolidPrimitive.BOX
        cx, cy, cz = pos
        return ("%-14s BOX  size %.2f x %.2f x %.3f m   centre (%+.3f, %+.3f, %+.3f)   TOP FACE z = %+.3f m   footprint x [%+.2f, %+.2f]  y [%+.2f, %+.2f]" %
                (obj_id, dims[0], dims[1], dims[2], cx, cy, cz, cz + dims[2] / 2.0, cx - dims[0] / 2, cx + dims[0] / 2, cy - dims[1] / 2, cy + dims[1] / 2))
    return "%-14s primitive type %d  dims %s  centre (%+.3f, %+.3f, %+.3f)" % ((obj_id, ptype, [round(d, 3) for d in dims]) + tuple(pos))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--show", action="store_true"); ap.add_argument("--remove", action="store_true")
    ap.add_argument("--top_z", type=float, help="height of the table's top face in base_link (m)")
    ap.add_argument("--x_min", type=float, default=-1.10); ap.add_argument("--x_max", type=float, default=-0.20)
    ap.add_argument("--y_half", type=float, default=0.70); ap.add_argument("--thickness", type=float, default=0.05)
    ap.add_argument("--id", default=DEFAULT_ID)
    a = ap.parse_args(argv)
    if a.top_z is None and not (a.show or a.remove):
        ap.error("give --top_z VALUE, or --show, or --remove")
    if a.top_z is not None and a.x_min >= a.x_max:
        ap.error("--x_min must be smaller than --x_max")

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
    from shape_msgs.msg import SolidPrimitive

    rclpy.init()
    node = Node("add_table_box")
    apply_c = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    get_c = node.create_client(GetPlanningScene, "/get_planning_scene")
    if not apply_c.wait_for_service(timeout_sec=10.0) or not get_c.wait_for_service(timeout_sec=10.0):
        print("MoveIt's planning scene services are not available - is move_group running (RViz open)?"); rclpy.shutdown(); return 2

    def apply(co):
        scene = PlanningScene(); scene.is_diff = True; scene.world.collision_objects.append(co)
        req = ApplyPlanningScene.Request(); req.scene = scene
        f = apply_c.call_async(req); rclpy.spin_until_future_complete(node, f, timeout_sec=5.0)
        return f.result() is not None and f.result().success

    def show():
        req = GetPlanningScene.Request(); req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        f = get_c.call_async(req); rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
        if f.result() is None:
            print("could not read the planning scene"); return
        objs = f.result().scene.world.collision_objects
        print("collision objects in MoveIt's scene: %d" % len(objs))
        for o in objs:
            for prim, pose in zip(o.primitives, o.primitive_poses):
                print("  " + describe(o.id, prim.type, list(prim.dimensions), (pose.position.x, pose.position.y, pose.position.z)))
        return objs

    rc = 0
    if a.top_z is not None or a.remove:
        rm = CollisionObject(); rm.id = a.id; rm.header.frame_id = "base_link"; rm.operation = CollisionObject.REMOVE
        apply(rm)                                                         # always start clean so the new box REPLACES the old one
    if a.top_z is not None:
        dims, (cx, cy, cz) = box_for(a.top_z, a.x_min, a.x_max, a.y_half, a.thickness)
        if overlaps_base(a.x_min, a.x_max, a.y_half):
            print("WARNING: the box footprint reaches the area around the arm's base (|x|,|y| < 0.20 m). It may collide with the arm's own base and make every plan fail.")
        co = CollisionObject(); co.id = a.id; co.header.frame_id = "base_link"; co.operation = CollisionObject.ADD
        box = SolidPrimitive(); box.type = SolidPrimitive.BOX; box.dimensions = list(dims)
        pose = Pose(); pose.position.x, pose.position.y, pose.position.z = cx, cy, cz; pose.orientation.w = 1.0
        co.primitives.append(box); co.primitive_poses.append(pose)
        ok = apply(co); print("table box %s (top face requested at z = %+.3f m)" % ("applied" if ok else "FAILED to apply", a.top_z)); rc = 0 if ok else 1
    if a.remove:
        print("table box removed")
    print("--- what MoveIt holds now:")
    show()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
