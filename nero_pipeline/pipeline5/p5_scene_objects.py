#!/usr/bin/env python3
"""p5_scene_objects.py - PIPELINE 5: what MoveIt did not know about (builders tested offline; apply() needs ROS).

  carrot      the object being picked, as N_SLICES boxes along its length (from SAM2's object points, so a fat end and a thin neck each get their
              own width), resting just above what it stands on. MTC already attaches/detaches an object called "carrot" (pipeline 3's name) -
              until now it did not exist, so the object in the gripper was never collision-checked during the lift, transit and place.
              Replaced every look; one still attached from an interrupted run is detached first.
  p5_wristcam the wrist camera + mount + cable: an 8 x 8 x 8 cm box beside the gripper (tcp_link +x, p5_gripper.py CAM_*), ATTACHED to
              tcp_link, so every plan checks it. Re-attached every look (a changed box takes effect without restarting MoveIt).
"""
import math

import numpy as np

P5_SCENE_OBJECTS_VERSION = "p5-scene-objects-v2k"   # 7k: the object model stays 3 mm above the table surface |   # v2: camera box from p5_gripper (beside the gripper); re-attached every look
OBJECT_ID, CAMERA_ID = "carrot", "p5_wristcam"
N_SLICES = 4
MARGIN = 0.003
BOTTOM_GAP = 0.002
ATTACH_LINK = "tcp_link"
TOUCH_LINKS = ["tcp_link", "gripper_base", "gripper_link1", "gripper_link2", "gripper_flange", "flange_link", "link7", "link6"]


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


TABLE_GAP = 0.003            # 7k: the model never reaches below the table surface + this (it was attached INTO the table: 'table_surface colliding')


def object_slices(O, floor_z, n=N_SLICES, margin=MARGIN, table_z=None):
    """Object points (base_link) -> list of boxes {"centre", "size", "yaw"}: n slices along the object's long axis, each as wide and as tall as
    the object is THERE (2-98 % of its points), from just above the floor up. [] if too few points."""
    O = np.asarray(O, float).reshape(-1, 3)
    O = O[O[:, 2] > floor_z + 0.004]
    if len(O) < 20:
        return []
    c = O[:, :2].mean(0)
    d = O[:, :2] - c
    w, v = np.linalg.eigh(d.T @ d / len(d))
    major = v[:, 1]
    yaw = math.atan2(major[1], major[0])
    minor = np.array([-major[1], major[0]])
    u, s = d @ major, d @ minor
    lo, hi = np.percentile(u, [2, 98])
    edges = np.linspace(lo, hi, n + 1)
    out = []
    for k in range(n):
        sel = (u >= edges[k]) & (u <= edges[k + 1])
        if sel.sum() < 5:
            continue
        s0, s1 = np.percentile(s[sel], [2, 98])
        top = float(np.percentile(O[sel, 2], 98))
        z0 = floor_z + BOTTOM_GAP
        if table_z is not None:
            z0 = max(z0, table_z + TABLE_GAP)                     # 7k: the measured floor can come out a few mm below MoveIt's table surface
        if top + margin - z0 < 0.003:                             # a slice with (almost) nothing above that: no box
            continue
        um, sm = 0.5 * (edges[k] + edges[k + 1]), 0.5 * (s0 + s1)
        xy = c + um * major + sm * minor
        out.append({"centre": [float(xy[0]), float(xy[1]), 0.5 * (z0 + top + margin)],
                    "size": [float(edges[k + 1] - edges[k] + 2 * margin), float(s1 - s0 + 2 * margin), float(top + margin - z0)], "yaw": yaw})
    return out


def camera_box(geom):
    """The camera + mount + cable in tcp_link: {"centre", "size"} (p5_gripper's CAM_*: beside the gripper, tcp_link +x)."""
    return {"centre": [geom["CAM_CX"], geom["CAM_CY"], geom["CAM_CZ"]], "size": [geom["CAM_SX"], geom["CAM_SY"], geom["CAM_SZ"]]}


def inside_box(p, box, grow=0.0):
    c, s = np.asarray(box["centre"]), np.asarray(box["size"])
    return bool(np.all(np.abs(np.asarray(p) - c) <= s / 2 + grow))


def apply(scene, carrot, cam, attach_link=ATTACH_LINK, touch_links=TOUCH_LINKS):
    """scene = a p3_scene.Scene (its node, /get_planning_scene, /apply_planning_scene). -> note. Never raises for MoveIt refusals (says so)."""
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, PlanningSceneComponents
    from shape_msgs.msg import SolidPrimitive

    def box_object(oid, frame, boxes):
        co = CollisionObject()
        co.id, co.header.frame_id, co.operation = oid, frame, CollisionObject.ADD
        co.pose.orientation.w = 1.0
        for b in boxes:
            co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in b["size"]]))
            p = Pose()
            p.position.x, p.position.y, p.position.z = (float(v) for v in b["centre"])
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = yaw_quat(b.get("yaw", 0.0))
            co.primitive_poses.append(p)
        return co

    req = scene.GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    attached = {a.object.id for a in scene._call(scene.get, req, "/get_planning_scene").scene.robot_state.attached_collision_objects}
    notes = []
    old = [i for i in (OBJECT_ID, CAMERA_ID) if i in attached]    # a carrot left attached by an interrupted run; the camera box (re-attached
    if old:                                                       # every look, so a changed box takes effect without restarting MoveIt)
        ps = PlanningScene()
        ps.is_diff = ps.robot_state.is_diff = True
        for i in old:
            a = AttachedCollisionObject()
            a.link_name, a.object.id, a.object.operation = attach_link, i, CollisionObject.REMOVE
            ps.robot_state.attached_collision_objects.append(a)
            if i == CAMERA_ID:                                    # REMOVE of an attached object leaves it in the world: remove it there too
                co = CollisionObject()
                co.id, co.header.frame_id, co.operation = i, "base_link", CollisionObject.REMOVE
                ps.world.collision_objects.append(co)
        r = scene.ApplyPlanningScene.Request()
        r.scene = ps
        ok = scene._call(scene.apply, r, "/apply_planning_scene").success
        notes.append(("removed first: " if ok else "NOT removed: ") + ", ".join(old))
        attached -= set(old)
    ps = PlanningScene()
    ps.is_diff = ps.robot_state.is_diff = True
    if carrot:
        ps.world.collision_objects.append(box_object(OBJECT_ID, "base_link", carrot))
    if cam is not None and CAMERA_ID not in attached:
        a = AttachedCollisionObject()
        a.link_name, a.object = attach_link, box_object(CAMERA_ID, attach_link, [cam])
        a.touch_links = list(touch_links)
        ps.robot_state.attached_collision_objects.append(a)
    if not ps.world.collision_objects and not ps.robot_state.attached_collision_objects:
        return "; ".join(notes + ["nothing to add"])
    r = scene.ApplyPlanningScene.Request()
    r.scene = ps
    ok = scene._call(scene.apply, r, "/apply_planning_scene").success
    what = ([("object in hand = %d slice box(es)" % len(carrot))] if carrot else []) + (["wrist camera box attached"] if cam is not None and CAMERA_ID not in attached else [])
    notes.append(("MoveIt now knows: " if ok else "MoveIt REFUSED: ") + ", ".join(what))
    return "; ".join(notes)
