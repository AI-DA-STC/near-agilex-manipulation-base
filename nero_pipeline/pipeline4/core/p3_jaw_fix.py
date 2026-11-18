#!/usr/bin/env python3
"""
p3_jaw_fix.py - PIPELINE 3: make the real fingers close along the line Contact-GraspNet intended.

Why: Contact-GraspNet's grasp frame has the jaws (the closing line) on its X axis; the Nero's tcp_link has them on Y (docs/01). Pipeline 1's MTC
has the conversion for this (to_tcp_pose: -90 deg about the local Z axis) but runs with APPLY_TCP_CONVERSION = False - the raw grasp pose IS the
tcp_link target - so the real arm closes 90 deg turned from the intended grasp. Invisible on a cube (a quarter turn still closes on two opposite
faces), wrong on anything elongated (an eggplant gets grasped along its length instead of across it).

This relay sits between Contact-GraspNet and pose_transform (start_pipeline3.sh launches pose_transform with grasps_topic:=/p3/grasps):
    /contact_graspnet/grasps (PoseArray, camera frame)  ->  every orientation turned -90 deg about its own Z (the approach)  ->  /p3/grasps
Position, approach direction and tilt are unchanged, so heights, re-centring and MTC's 55-deg rule behave exactly as in pipeline 1.
Pipeline 1 itself is not touched. P3_JAW_FIX=0 when starting start_pipeline3.sh leaves the relay out (the old behaviour).
"""
import math
import sys

P3_JAW_FIX_VERSION = "p3-jawfix-v8"
# v8: DEEPEN (P3_DEEPEN, default on): a safe grasp whose lower fingertip is higher than the clearance (+2 mm) is moved DEEPER along its own
# approach until that fingertip is at the clearance - the pads then hold the object's widest part instead of its top. Same spot, angle and
# finger direction as Contact-GraspNet's grasp; at most DEEPEN_MAX_M. P3_DEEPEN=0 = Contact-GraspNet's depth as it is.
# v7: CONTACT-GRASPNET'S OWN RANKING. Its worker sorts the grasps by its confidence score (best first); the relay keeps that order and only
# drops grasps that fail a real check: tilted more than 55 deg (MoveIt's limit) or a FINGER that would come too close to the table - BOTH
# fingertips are checked (p3_fingers.py: a rolled grasp has one finger lower than the centre line). The first P3_GRASPS_TO_MOVEIT (default 3)
# safe ones go to MoveIt in Contact-GraspNet's order (MoveIt tries the first, the others only if it cannot plan it). Contact-GraspNet's poses are
# NOT changed - except, only if NONE of its grasps clears the table, the best ones are raised (height only) until both fingertips clear.
# What was passed / rejected and why: ~/p3_runs/relay_latest.json (the picture step prints it).
HEIGHT_MAX_AGE_S = 300.0   # v6: an object height older than this is not from the current run -> the maximum clearance is used
# v3: TIP LIFT (pipeline 2's P2_TIP_LIFT, now on by default): a grasp whose lowest fingertip would come closer than TIP_MIN_M to the table is raised
# ONLY in height (base_link z) until it does not - same x, y, angle and jaw direction. A low object (carrot) gets its good low grasp a few mm higher
# instead of being stopped. P3_TIP_LIFT=0 switches it off (then p3_picture's check stops such runs, as before).
FINGERTIP_REACH = 0.138                                          # = p3_picture.py: grasp origin -> lowest fingertip, along the approach

MAX_TILT_DEG = 55.0       # = pipeline 1's MTC rule
# v2: how many grasps go on to MoveIt. 1 (default) = only the most top-down one - exactly the G1 the picture highlights, so the picture and the
# real arm cannot disagree (with several, MTC executes whichever plan is cheapest). P3_GRASPS_TO_MOVEIT=5 = several, as before.
import os as _os
GRASPS_TO_MOVEIT = int(_os.environ.get("P3_GRASPS_TO_MOVEIT", "3"))
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import p3_tablez as _tz                                            # v4: the real table height, shared by all pipeline 3 tools
TABLE_Z, TABLE_Z_SOURCE = _tz.table_z()
TIP_MIN_M = _tz.tip_min()[0] + 0.001        # the MAXIMUM clearance (+1 mm, so a lifted grasp passes p3_picture's check)
LIFT_EXTRA_M = 0.001                         # v6: the lift aims 1 mm above the clearance the check demands
TIP_LIFT = _os.environ.get("P3_TIP_LIFT", "1") != "0"
DEEPEN = _os.environ.get("P3_DEEPEN", "1") != "0"
DEEPEN_MAX_M = 0.04
TURN_DEG = -90.0          # = pipeline 1's to_tcp_pose: "Rotate -90 deg about local Z so tcp Y lands on the grasp's X"


def quat_mul(a, b):
    """Hamilton product of (x, y, z, w) quaternions: a * b (b applied in a's local frame)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def turn_about_local_z(q, deg=TURN_DEG):
    h = math.radians(deg) / 2.0
    return quat_mul(q, (0.0, 0.0, math.sin(h), math.cos(h)))


def tilt_deg_cam(q):
    """Angle between the grasp's approach (its Z axis) and straight down. The camera looks straight down (capture: optical axis (0, 0, -1) in
    base_link), so in the camera frame 'straight down' is +Z - the same order as MTC's tilt in base_link."""
    x, y, z, w = q
    az = 1.0 - 2.0 * (x * x + y * y)                 # Z component of the rotated Z axis
    return math.degrees(math.acos(max(-1.0, min(1.0, az))))


def quat_to_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 1e-12 else 0.0
    return [[1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
            [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
            [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]]


def lift_needed(origin_cam, quat_cam, R_bc, t_bc, clearance_m=None):
    """How far (m, along base_link +z) a grasp must be raised so its lowest fingertip is clearance_m above the table (default TIP_MIN_M,
    the maximum); 0 if it already is.
    R_bc, t_bc: the camera frame in base_link (rotation rows, translation)."""
    Rg = quat_to_matrix(*quat_cam)
    a_cam = [Rg[0][2], Rg[1][2], Rg[2][2]]                       # approach (grasp Z) in the camera frame
    tip_cam = [origin_cam[i] + FINGERTIP_REACH * a_cam[i] for i in range(3)]
    tip_z = sum(R_bc[2][j] * tip_cam[j] for j in range(3)) + t_bc[2]
    c = TIP_MIN_M if clearance_m is None else clearance_m
    return max(0.0, (TABLE_Z + c) - tip_z)


def clearance_from_height(height_msg_point_x, age_s):
    """v6: the fingertip clearance for this run (m, incl. LIFT_EXTRA_M) and why - from the object height the GraspMolmo node published.
    A missing, unknown (-1) or old height gives the maximum (the safe side)."""
    if height_msg_point_x is None or age_s is None or age_s > HEIGHT_MAX_AGE_S or height_msg_point_x <= 0:
        c, why = _tz.clearance_for(None)
        return c + LIFT_EXTRA_M, why + ("" if height_msg_point_x is None else "; the published height is %s" % (
            "unknown" if height_msg_point_x <= 0 else "%.0f s old" % age_s))
    c, why = _tz.clearance_for(height_msg_point_x)
    return c + LIFT_EXTRA_M, why


def raise_in_base(origin_cam, dz, R_bc):
    """The grasp origin moved dz along base_link +z, expressed in the camera frame (R_bc^T applied to (0, 0, dz))."""
    return [origin_cam[i] + R_bc[2][i] * dz for i in range(3)]


def rank_and_check(grasps_cam, R_bc, t_bc, table_z, clearance, n=None, tip_lift=None):
    """v7. grasps_cam: [(origin xyz, quaternion xyzw AFTER the jaw turn)] in the camera frame, in CONTACT-GRASPNET'S ORDER (best first).
    R_bc, t_bc: the camera in base_link. -> (passed [(origin_cam, quat, cgn_rank, note)], report [one dict per grasp]).
    Keeps Contact-GraspNet's order; drops tilt > 55 deg and grasps whose LOWER fingertip is closer than `clearance` to the table."""
    import numpy as _np
    import p3_fingers
    n = GRASPS_TO_MOVEIT if n is None else n
    tip_lift = TIP_LIFT if tip_lift is None else tip_lift
    R_bc, t_bc = _np.asarray(R_bc, dtype=float), _np.asarray(t_bc, dtype=float)
    report, safe, tilt_ok = [], [], []
    for rank, (o, q) in enumerate(grasps_cam, start=1):
        Rb = R_bc @ _np.asarray(quat_to_matrix(*q))
        ob = R_bc @ _np.asarray(o, dtype=float) + t_bc
        tilt = float(_np.degrees(_np.arccos(max(-1.0, min(1.0, -Rb[2, 2])))))
        low = p3_fingers.lowest_tip_z(ob, Rb) - table_z
        r = {"rank": rank, "tilt_deg": round(tilt, 1), "roll_deg": round(p3_fingers.roll_deg(Rb), 1), "lowest_finger_mm": round(1000 * low, 1)}
        if tilt > MAX_TILT_DEG:
            r["result"] = "rejected: tilted %.0f deg (> %.0f)" % (tilt, MAX_TILT_DEG)
        elif low < clearance - 1e-9:
            r["result"] = "rejected: a fingertip %.1f mm above the table (needs %.1f)" % (1000 * low, 1000 * clearance)
            tilt_ok.append((o, q, rank, low))
        else:
            r["result"] = "safe"
            note = "Contact-GraspNet #%d, unchanged" % rank
            down = -Rb[2, 2]                                  # how much of a step along the approach goes DOWN (1 = straight down)
            if DEEPEN and down > 0.2 and low > clearance + 0.002:
                d = min(DEEPEN_MAX_M, (low - clearance) / down)       # along the approach, so the lower fingertip ends at the clearance
                a_cam = _np.asarray(quat_to_matrix(*q))[:, 2]           # the approach in the camera frame
                o = tuple(_np.asarray(o, dtype=float) + d * a_cam)
                note = "Contact-GraspNet #%d, %.0f mm deeper along its approach (lower fingertip %.0f -> %.0f mm above the table)" % (
                    rank, 1000 * d, 1000 * low, 1000 * (low - d * down))
            safe.append((o, q, rank, note))
        report.append(r)
    passed = safe[:max(1, n)]
    if not passed and tilt_ok and tip_lift:               # none of its grasps clears the table: raise its best ones (height only), and say so
        for o, q, rank, low in tilt_ok[:max(1, n)]:
            dz = (clearance - low) + LIFT_EXTRA_M
            passed.append((raise_in_base(list(o), dz, R_bc.tolist()), q, rank, "Contact-GraspNet #%d, raised %.1f mm (none of its grasps cleared the table)" % (rank, 1000 * dz)))
    for o, q, rank, note in passed:
        report[rank - 1]["result"] = "PASSED to MoveIt: " + note
    return passed, report


def select(quats, n=None):
    """Indices of the grasps to pass on: tilt <= 55 deg, most top-down first, at most n. None kept -> all (MTC then reports there is nothing to plan)."""
    n = GRASPS_TO_MOVEIT if n is None else n
    t = [(tilt_deg_cam(q), i) for i, q in enumerate(quats)]
    ok = sorted([(d, i) for d, i in t if d <= MAX_TILT_DEG])
    return [i for _, i in ok[:max(1, n)]] if ok else [i for _, i in t]


def main():
    import rclpy
    from geometry_msgs.msg import PoseArray
    from rclpy.node import Node

    class JawFix(Node):
        def __init__(self):
            super().__init__("p3_jaw_fix")
            self.pub = self.create_publisher(PoseArray, "/p3/grasps", 10)
            from geometry_msgs.msg import PointStamped
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            self.height = None                                  # (object height m, receive time) from the GraspMolmo node
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(PointStamped, "/graspmolmo/object_height", lambda m: setattr(self, "height", (m.point.x, __import__("time").time())), latched)
            import tf2_ros
            self.tf_buf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
            self.create_subscription(PoseArray, "/contact_graspnet/grasps", self.cb, 10)
            self.get_logger().info("%s: /contact_graspnet/grasps -> each grasp turned %+.0f deg about its approach axis -> /p3/grasps; table z %+.1f mm (%s)"
                                   % (P3_JAW_FIX_VERSION, TURN_DEG, 1000 * TABLE_Z, TABLE_Z_SOURCE) + "; fingertips kept >= %.1f mm above it" % (1000 * TIP_MIN_M))

        def cb(self, msg):
            out = PoseArray()
            out.header = msg.header
            turned = []
            for p in msg.poses:
                q = turn_about_local_z((p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
                turned.append(p)
            import time as _time
            h, t_h = self.height if self.height else (None, None)
            clearance, why = clearance_from_height(h, None if t_h is None else _time.time() - t_h)
            try:                                               # v7: Contact-GraspNet's order, both-finger table check
                import rclpy.time
                import rclpy.duration
                tr = self.tf_buf.lookup_transform("base_link", msg.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
                q, t = tr.transform.rotation, tr.transform.translation
                R_bc, t_bc = quat_to_matrix(q.x, q.y, q.z, q.w), [t.x, t.y, t.z]
                passed, report = rank_and_check([((p.position.x, p.position.y, p.position.z),
                                                  (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)) for p in turned],
                                                R_bc, t_bc, TABLE_Z, clearance - LIFT_EXTRA_M)
                out.poses = []
                for o, qq, rank, note in passed:
                    pp = turned[rank - 1]
                    pp.position.x, pp.position.y, pp.position.z = o
                    out.poses.append(pp)
                self._write_report(report, why, len(turned))
                self.pub.publish(out)
                self.get_logger().info("Contact-GraspNet gave %d grasp(s); fingertip clearance %s; passed to MoveIt: %s" % (
                    len(turned), why, ", ".join(n for _, _, _, n in passed) or "NONE (all tilted > 55 deg)"))
                for r in report[:12]:
                    self.get_logger().info("  #%d tilt %.0f deg, roll %.0f deg, lower fingertip %+.1f mm: %s" % (
                        r["rank"], r["tilt_deg"], r["roll_deg"], r["lowest_finger_mm"], r["result"]))
                return
            except Exception as e:
                self.get_logger().warn("no camera pose in TF (%s): falling back to the old selection (least tilted, centre-line lift); "
                                       "p3_picture's fingertip check still guards the run" % e)
            keep = select([(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w) for p in turned])
            out.poses = [turned[i] for i in keep]
            if TIP_LIFT:
                import time as _time
                h, t_h = self.height if self.height else (None, None)
                clearance, why = clearance_from_height(h, None if t_h is None else _time.time() - t_h)
                self.get_logger().info("fingertip clearance: %s" % why)
                try:
                    import rclpy.time
                    import rclpy.duration
                    tr = self.tf_buf.lookup_transform("base_link", msg.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
                    q, t = tr.transform.rotation, tr.transform.translation
                    R_bc, t_bc = quat_to_matrix(q.x, q.y, q.z, q.w), [t.x, t.y, t.z]
                    for p in out.poses:
                        o = [p.position.x, p.position.y, p.position.z]
                        dz = lift_needed(o, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w), R_bc, t_bc, clearance)
                        if dz > 0:
                            p.position.x, p.position.y, p.position.z = raise_in_base(o, dz, R_bc)
                            self.get_logger().info("TIP LIFT: grasp raised %.1f mm (height only) so its lowest fingertip is %.1f mm above the table"
                                                   % (1000 * dz, 1000 * clearance))
                except Exception as e:
                    self.get_logger().warn("TIP LIFT skipped: no camera pose in TF (%s); p3_picture's fingertip check still guards the run" % e)
            self.pub.publish(out)
            self.get_logger().info("relayed %d of %d grasp(s) (the most top-down, tilt %s deg), jaws turned %+.0f deg" % (
                len(out.poses), len(turned), ", ".join("%.0f" % tilt_deg_cam((turned[i].orientation.x, turned[i].orientation.y,
                                                                                 turned[i].orientation.z, turned[i].orientation.w)) for i in keep), TURN_DEG))

    def _write_report(self, report, why, n):
        import json
        import time as _time
        try:
            _os.makedirs(_os.path.expanduser("~/p3_runs"), exist_ok=True)
            with open(_os.path.expanduser("~/p3_runs/relay_latest.json"), "w") as f:
                json.dump({"time": _time.strftime("%Y-%m-%d %H:%M:%S"), "cgn_grasps": n, "clearance": why, "grasps": report}, f, indent=1)
        except OSError:
            pass
    JawFix._write_report = _write_report

    rclpy.init()
    node = JawFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
