#!/usr/bin/env python3
"""
p3_objects.py - every object on the table as the CAMERA sees it (through the current calibration), with the centre of its TOP face in base_link.
Read-only: moves nothing, changes nothing. For the sideways calibration check: steer the gripper (pointing straight down) to an object's centre
as printed here, then see how far the real object's centre is from the gripper's centre.

    conda deactivate; source ~/agx_arm_ws/install/setup.bash
    /usr/bin/python3 ~/nero_pipeline_launch/pipeline3/p3_objects.py

Objects = everything at least 2.5 cm above the real table (p3_tablez), the robot left out (as for the obstacle boxes). The centre is the middle
of the object's top face (its points within 1 cm of its top), so the sides the camera sees at an angle do not shift it.
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p3_obstacles  # noqa: E402
import p3_tablez  # noqa: E402

P3_OBJECTS_VERSION = "p3-objects-v1"
TOP_BAND_M = 0.01


def objects_from_points(P, table_z):
    """Obstacle points (base_link) -> objects [{"centre_xy", "top_z", "height", "size_xy", "points"}], grouped like the obstacle boxes."""
    out = []
    for b in p3_obstacles.group_boxes(P, table_z):
        cx, cy, _ = b["centre"]; sx, sy, _ = b["size"]
        inside = (np.abs(P[:, 0] - cx) <= sx / 2) & (np.abs(P[:, 1] - cy) <= sy / 2)
        Q = P[inside]
        top = float(np.percentile(Q[:, 2], 95))
        T = Q[Q[:, 2] >= top - TOP_BAND_M]
        lo, hi = T[:, :2].min(axis=0), T[:, :2].max(axis=0)
        top_med = float(np.median(T[:, 2]))                       # the top face's own height (a high percentile reads ~3 mm too tall in noise)
        out.append({"centre_xy": [float((lo[0] + hi[0]) / 2), float((lo[1] + hi[1]) / 2)], "top_z": top_med, "height": top_med - table_z,
                    "size_xy": [float(hi[0] - lo[0]), float(hi[1] - lo[1])], "points": int(len(Q))})
    return out


def read_frame(timeout=15.0):
    """One depth frame + intrinsics + the camera pose in base_link + the robot's own TF frame positions."""
    import rclpy
    import tf2_ros
    import yaml
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    rclpy.init()
    node = Node("p3_objects")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    got = {}
    node.create_subscription(Image, "/topcam/depth/image_raw", lambda m: got.__setitem__("d", m), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, "/topcam/depth/camera_info", lambda m: got.__setitem__("i", m), qos_profile_sensor_data)
    t_end = time.time() + timeout
    cam = None
    while time.time() < t_end and (len(got) < 2 or cam is None):
        rclpy.spin_once(node, timeout_sec=0.1)
        if "i" in got and cam is None:
            try:
                tr = buf.lookup_transform("base_link", got["i"].header.frame_id, rclpy.time.Time())
                q, p = tr.transform.rotation, tr.transform.translation
                x, y, z, w = q.x, q.y, q.z, q.w
                R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                              [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                              [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
                cam = (R, np.array([p.x, p.y, p.z]))
            except Exception:
                pass
    robot = []
    try:
        for f in (yaml.safe_load(buf.all_frames_as_yaml()) or {}):
            if any(s in f for s in ("topcam", "camera", "base_link", "world")):
                continue
            try:
                t = buf.lookup_transform("base_link", f, rclpy.time.Time()).transform.translation
                robot.append((t.x, t.y, t.z))
            except Exception:
                pass
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()
    if len(got) < 2:
        raise SystemExit("STOP: no depth frame / camera_info on /topcam/depth/* - is the camera running?")
    if cam is None:
        raise SystemExit("STOP: no camera pose in TF - is the hand-eye publisher running?")
    m = got["d"]
    d = CvBridge().imgmsg_to_cv2(m, desired_encoding="passthrough").astype(np.float64) * (0.001 if m.encoding in ("16UC1", "mono16") else 1.0)
    return d, np.array(got["i"].k).reshape(3, 3), cam, robot


def main():
    print("p3_objects %s - objects on the table as the CAMERA sees them (base_link, m)" % P3_OBJECTS_VERSION, flush=True)
    depth, K, (R, t), robot = read_frame()
    tz, tzs = p3_tablez.table_z()
    P, rows, cols = p3_obstacles.points_in_base(depth, K, R, t)
    keep = p3_obstacles.obstacle_points(P, rows, cols, tz, robot_points=robot)
    objs = objects_from_points(P[keep], tz)
    if not objs:
        print("no objects found (nothing at least 2.5 cm above the table at %+.1f mm)" % (1000 * tz))
        return 1
    print("table at %+.1f mm (%s); %d object(s), biggest first:" % (1000 * tz, tzs, len(objs)))
    for k, o in enumerate(objs, 1):
        print("  %d.  top-face centre  x=%+.3f  y=%+.3f   %.1f cm tall, top face %.1f x %.1f cm" % (
            k, o["centre_xy"][0], o["centre_xy"][1], 100 * o["height"], 100 * o["size_xy"][0], 100 * o["size_xy"][1]))
    print("Steer the gripper (pointing straight down) to an object's x, y above; then measure how far the REAL object's centre is from the gripper's.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
