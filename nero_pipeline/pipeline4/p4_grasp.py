#!/usr/bin/env python3
"""
p4_grasp.py - PIPELINE 4 GRASP STEP: replaces pipeline 3's grasp relay (p3_jaw_fix.py) while it runs - same input
(/contact_graspnet/grasps, camera frame) and output (/p3/grasps, camera frame, read by pose_transform). Started by start_pipeline4.sh (P4_GRASP=off there uses pipeline 3's relay copy instead).

For every Contact-GraspNet grasp, in its own order (best first):
  1. jaw turn, exactly as pipeline 3 (-90 deg about the approach: Contact-GraspNet's jaws are on X, the Nero's on Y)
  2. STRAIGHTEN: the jaw line is made LEVEL (its compass direction kept, so the fingers still close across the object the way Contact-GraspNet
     chose; both fingertips at the same height, roll 0), and the approach may tilt only ACROSS the jaws, by at most P4_MAX_TILT_DEG (default 20)
  3. DEPTH: the finger pads go to the object's MID-HEIGHT (from the height GraspMolmo measured), at Contact-GraspNet's x/y;
     if the lower fingertip would then be closer than pipeline 3's clearance to the table, the grasp is raised (straight up: it stays aligned)
  4. WIDTH: the object's width across the jaws (top-camera points of GraspMolmo's object region) + P4_OPEN_MARGIN_M, within 3-10 cm.
     MTC opens the gripper ONCE, so the widest of the grasps passed on is used; pipeline 4's MTC copy reads it from ~/p3_runs/p4_grasp_latest.json.
The first P3_GRASPS_TO_MOVEIT (default 3) distinct grasps go on. Report: ~/p3_runs/relay_latest.json (same format, so pipeline 3's picture shows it)
and ~/p3_runs/p4_grasp_latest.json (with the opening width).
"""
import json
import math
import os
import sys
import time

import numpy as np

P4_GRASP_VERSION = "p4-grasp-v4"   # v4: the jaw line is always LEVEL (roll 0): tilt only across the jaws, so both fingertips are at the same height
# v3:   # v3: default tilt limit 20 deg (was 0 = straight down)
# v2:   # v2: pipeline 3 parts from pipeline4/core; started by start_pipeline4.sh
# v1:

MAX_TILT_DEG = float(os.environ.get("P4_MAX_TILT_DEG", "20"))         # grasps tilted more are turned to this; 0 = straight down
OPEN_MARGIN_M = float(os.environ.get("P4_OPEN_MARGIN_M", "0.02"))      # added to the measured width
OPEN_MIN_M, OPEN_MAX_M = 0.03, 0.10                                    # the Nero gripper: 0.10 = fully open
TIP_REACH_M = 0.138                                                    # = pipeline 3: tcp_link -> fingertips along the approach
PAD_DEPTH_M = float(os.environ.get("P4_PAD_DEPTH_M", "0.123"))         # tcp_link -> middle of the finger pads (about 15 mm above the tips)
CGN_CONTACT_DEPTH_M = 0.1034                                           # Contact-GraspNet's gripper: its pose origin -> contact
REPORT = os.path.expanduser("~/p3_runs/p4_grasp_latest.json")
RELAY_REPORT = os.path.expanduser("~/p3_runs/relay_latest.json")
DOWN = np.array([0.0, 0.0, -1.0])


# ---------------------------------------------------------------- pure geometry (tested offline)
def quat_to_R(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 1e-12 else 0.0
    return np.array([[1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
                     [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
                     [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def R_to_quat(R):
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return tuple(q / np.linalg.norm(q))


def tilt_deg(R):
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.asarray(R)[:, 2] @ DOWN)))))


def straighten(R, max_tilt_deg=MAX_TILT_DEG):
    """v4: level jaw line (Y) with the same compass direction; the approach (Z) keeps only its tilt ACROSS the jaws, limited to max_tilt_deg.
    -> new R (columns X, Y, Z, right-handed) or None if the jaw line is nearly vertical (no compass direction to keep)."""
    R = np.asarray(R, dtype=float)
    z, y = R[:, 2], R[:, 1]
    yl = np.array([y[0], y[1], 0.0])
    if np.linalg.norm(yl) < 0.3:
        return None
    yl /= np.linalg.norm(yl)
    xl = np.cross(yl, DOWN)                           # level, across the jaws
    a = math.atan2(float(z @ xl), float(z @ DOWN))    # the approach's tilt across the jaws (signed)
    lim = math.radians(max(0.0, max_tilt_deg))
    a = max(-lim, min(lim, a))
    z2 = math.cos(a) * DOWN + math.sin(a) * xl
    return np.column_stack([np.cross(yl, z2), yl, z2])


def roll_deg(R):
    """How far the jaw line (Y) is from level (0 = both fingertips at the same height)."""
    return math.degrees(math.asin(min(1.0, abs(float(np.asarray(R)[2, 1])))))


def object_width(points, jaw, centre=None):
    """Extent of the object's points along the jaw line (2nd-98th percentile, so stray pixels do not count). None if too few points."""
    if points is None or len(points) < 20:
        return None
    j = np.asarray(jaw, dtype=float)
    j = j / np.linalg.norm(j)
    s = (np.asarray(points) - (np.zeros(3) if centre is None else centre)) @ j
    lo, hi = np.percentile(s, [2, 98])
    return float(hi - lo)


def opening(width, margin=OPEN_MARGIN_M):
    """Gripper opening for an object this wide: width + margin, within the gripper's 3-10 cm. None width -> fully open."""
    if width is None:
        return OPEN_MAX_M
    return float(min(OPEN_MAX_M, max(OPEN_MIN_M, width + margin)))


def lowest_tip_z(tcp, R, open_w):
    R = np.asarray(R)
    tip = np.asarray(tcp) + TIP_REACH_M * R[:, 2]
    half = 0.5 * open_w * R[:, 1]
    return float(min((tip + half)[2], (tip - half)[2]))


def plan_grasp(o_b, R_b, table_z, clearance, height, obj_pts, max_tilt_deg=MAX_TILT_DEG, margin=OPEN_MARGIN_M):
    """One Contact-GraspNet grasp (base_link, after the jaw turn) -> dict with the new tcp pose, opening and notes, or None if unusable."""
    R_b = np.asarray(R_b, dtype=float)
    o_b = np.asarray(o_b, dtype=float)
    t0 = tilt_deg(R_b)
    R2 = straighten(R_b, max_tilt_deg)
    if R2 is None:
        return None
    contact = o_b + CGN_CONTACT_DEPTH_M * R_b[:, 2]              # where Contact-GraspNet meant the fingers to close
    pad = contact.copy()
    if height is not None and height > 0:                         # pads at the object's mid-height
        pad[2] = table_z + 0.5 * height
    w = object_width(obj_pts, R2[:, 1])
    ow = opening(w, margin)
    tcp = pad - PAD_DEPTH_M * R2[:, 2]
    low = lowest_tip_z(tcp, R2, ow) - table_z
    raised = 0.0
    if low < clearance:
        raised = clearance - low
        tcp = tcp + np.array([0.0, 0.0, raised])
        low = clearance
    return {"tcp": tcp, "R": R2, "tilt_from": t0, "tilt_to": tilt_deg(R2), "roll_from": roll_deg(R_b), "roll_to": roll_deg(R2), "width": w, "open": ow,
            "lowest_tip_mm": 1000 * low, "raised_mm": 1000 * raised, "pad_z": float(pad[2] + raised)}


def distinct(plans, min_dist=0.01, min_angle_deg=10.0):
    """Drop grasps that became (nearly) the same after straightening: within min_dist and jaw lines within min_angle_deg."""
    out = []
    for p in plans:
        same = False
        for q in out:
            if np.linalg.norm(p["tcp"] - q["tcp"]) < min_dist:
                c = abs(float(p["R"][:, 1] @ q["R"][:, 1]))
                if math.degrees(math.acos(min(1.0, c))) < min_angle_deg:
                    same = True
                    break
        if not same:
            out.append(p)
    return out


# ---------------------------------------------------------------- ROS node
def main():
    import rclpy
    from geometry_msgs.msg import PointStamped, Pose, PoseArray
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    import tf2_ros

    here = os.path.dirname(os.path.abspath(__file__))
    sys.path[:0] = [here, os.path.join(here, "core")]          # v2: pipeline 4's own copies
    import p3_jaw_fix as J                        # pipeline 3's jaw turn, clearance rule, table height, number of grasps (not modified)
    import p4_wrist_obstacles as W                # image decoding, mask -> 3D

    class P4Grasp(Node):
        def __init__(self):
            super().__init__("p4_grasp")
            self.pub = self.create_publisher(PoseArray, "/p3/grasps", 10)
            self.height, self.last = None, {}
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(PointStamped, "/graspmolmo/object_height", lambda m: setattr(self, "height", (m.point.x, time.time())), latched)
            self.create_subscription(Image, "/graspmolmo/mask_source", lambda m: self.last.update(mask=m, mask_t=time.time()), 10)
            self.create_subscription(Image, "/topcam/depth/image_raw", lambda m: self.last.__setitem__("depth", m), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, "/topcam/depth/camera_info", lambda m: self.last.__setitem__("info", m), qos_profile_sensor_data)
            self.tf_buf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
            self.create_subscription(PoseArray, "/contact_graspnet/grasps", self.cb, 10)
            self.get_logger().info("%s (instead of %s): straighten to <= %.0f deg, pads at mid-height, opening = width + %.0f mm (%.0f-%.0f mm); "
                                   "table z %+.1f mm; up to %d grasp(s) to MoveIt" % (
                                       P4_GRASP_VERSION, J.P3_JAW_FIX_VERSION, MAX_TILT_DEG, 1000 * OPEN_MARGIN_M, 1000 * OPEN_MIN_M,
                                       1000 * OPEN_MAX_M, 1000 * J.TABLE_Z, J.GRASPS_TO_MOVEIT))

        def object_points(self, R_bc, t_bc):
            m, d, info = self.last.get("mask"), self.last.get("depth"), self.last.get("info")
            if m is None or d is None or info is None or time.time() - self.last.get("mask_t", 0) > J.HEIGHT_MAX_AGE_S:
                return None
            mask = W.image_to_array(m, np.uint8)
            depth = W.depth_to_m(d)
            if mask.shape != depth.shape:
                return None
            pts = W.mask_points_base(mask, depth, np.array(info.k, dtype=float).reshape(3, 3), R_bc, t_bc)
            return pts[pts[:, 2] > J.TABLE_Z + 0.005]               # the object, not the table around it

        def cb(self, msg):
            h, t_h = self.height if self.height else (None, None)
            clearance, why = J.clearance_from_height(h, None if t_h is None else time.time() - t_h)
            height = h if (h is not None and h > 0 and t_h is not None and time.time() - t_h < J.HEIGHT_MAX_AGE_S) else None
            try:
                tr = self.tf_buf.lookup_transform("base_link", msg.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
            except Exception as e:
                self.get_logger().error("no TF base_link -> %s (%s): passing NOTHING on (pipeline 3 would fall back; pipeline 4 does not guess)"
                                        % (msg.header.frame_id, e))
                return
            q, t = tr.transform.rotation, tr.transform.translation
            R_bc, t_bc = quat_to_R(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])
            obj = self.object_points(R_bc, t_bc)
            plans, report = [], []
            for rank, p in enumerate(msg.poses, start=1):
                qq = J.turn_about_local_z((p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
                R_b = R_bc @ quat_to_R(*qq)
                o_b = R_bc @ np.array([p.position.x, p.position.y, p.position.z]) + t_bc
                g = plan_grasp(o_b, R_b, J.TABLE_Z, clearance, height, obj)
                r = {"rank": rank, "tilt_deg": round(tilt_deg(R_b), 1), "roll_deg": round(roll_deg(R_b), 1)}
                if g is None:
                    r.update(result="rejected: its jaw line is along the approach, cannot be straightened", lowest_finger_mm=None)
                else:
                    g["rank"] = rank
                    r["lowest_finger_mm"] = round(g["lowest_tip_mm"], 1)
                    plans.append(g)
                report.append(r)
            plans = distinct(plans)[:max(1, J.GRASPS_TO_MOVEIT)]
            out = PoseArray(); out.header = msg.header
            for g in plans:
                o_c = R_bc.T @ (g["tcp"] - t_bc)
                qc = R_to_quat(R_bc.T @ g["R"])
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = (float(v) for v in o_c)
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qc
                out.poses.append(pose)
                report[g["rank"] - 1]["result"] = ("PASSED to MoveIt: Contact-GraspNet #%d tilt %.0f -> %.0f deg, jaw roll %.0f -> 0 deg (level), pads at z %+.3f m, "
                                                   "object %s wide -> opening %.0f mm%s" % (
                                                       g["rank"], g["tilt_from"], g["tilt_to"], g["roll_from"], g["pad_z"],
                                                       "?" if g["width"] is None else "%.0f mm" % (1000 * g["width"]), 1000 * g["open"],
                                                       "" if g["raised_mm"] < 0.5 else ", raised %.0f mm for the fingertip clearance" % g["raised_mm"]))
            for r in report:
                r.setdefault("result", "not passed (a near-duplicate after straightening, or beyond the first %d)" % J.GRASPS_TO_MOVEIT)
            open_w = max([g["open"] for g in plans], default=OPEN_MAX_M)
            self.pub.publish(out)
            stamp = time.strftime("%Y-%m-%d %H:%M:%S")
            try:
                os.makedirs(os.path.dirname(REPORT), exist_ok=True)
                json.dump({"time": stamp, "cgn_grasps": len(msg.poses), "clearance": why + " [pipeline 4 grasp step]", "grasps": report},
                          open(RELAY_REPORT, "w"), indent=1)
                json.dump({"version": P4_GRASP_VERSION, "time": stamp, "unix_time": time.time(), "open_width_m": open_w,
                           "max_tilt_deg": MAX_TILT_DEG, "margin_m": OPEN_MARGIN_M, "object_points": 0 if obj is None else int(len(obj)),
                           "grasps": [{"rank": g["rank"], "tilt_from": round(g["tilt_from"], 1), "tilt_to": round(g["tilt_to"], 1),
                                       "roll_from": round(g["roll_from"], 1), "roll_to": round(g["roll_to"], 1),
                                       "width_m": g["width"], "open_m": g["open"], "pad_z": g["pad_z"], "raised_mm": round(g["raised_mm"], 1),
                                       "lowest_tip_mm": round(g["lowest_tip_mm"], 1)} for g in plans]}, open(REPORT, "w"), indent=1)
            except Exception as e:
                self.get_logger().warn("could not write the report: %s" % e)
            self.get_logger().info("Contact-GraspNet gave %d grasp(s); %s; passed to MoveIt %d; gripper opening %.0f mm%s" % (
                len(msg.poses), why, len(plans), 1000 * open_w, "" if obj is not None else " (no object region: FULLY open)"))
            for r in report[:12]:
                self.get_logger().info("  #%d tilt %.0f deg: %s" % (r["rank"], r["tilt_deg"], r["result"]))

    rclpy.init()
    import rclpy.duration
    import rclpy.time
    node = P4Grasp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
