#!/usr/bin/env python3
"""p4_marker_compare: where is the ArUco marker in base_link, seen by each camera?
Listens to /aruco_detections (both trackers publish there; the camera is told apart by header.frame_id),
transforms every marker-0 pose to base_link with TF, averages for a few seconds and prints:
  per camera: x y z (z = height above base_link z=0, the table top), marker tilt from horizontal, spread
  wrist minus top: the disagreement between the two cameras (what makes the clouds blend or not)
Read-only. Run with conda deactivated:  /usr/bin/python3 p4_marker_compare.py [--seconds 4] [--id 0]
"""
import argparse, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from aruco_opencv_msgs.msg import ArucoDetection

VERSION = "p4-marker-compare-v1"
BASE = "base_link"


def qmat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def to_base(tf, p, q):
    """tf: base<-camera transform; p, q: marker position and quaternion in the camera frame.
    Returns marker position in base and the marker's normal (its Z axis) in base."""
    tt, tr = tf.transform.translation, tf.transform.rotation
    Rb = qmat(tr.x, tr.y, tr.z, tr.w)
    pos = Rb @ np.array(p) + np.array([tt.x, tt.y, tt.z])
    normal = Rb @ qmat(*q)[:, 2]
    return pos, normal


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--id", type=int, default=0)
    a = ap.parse_args()
    print(VERSION)
    rclpy.init()
    n = Node("p4_marker_compare")
    buf = Buffer(); TransformListener(buf, n)
    data = {}

    def cb(m):
        for mk in m.markers:
            if mk.marker_id != a.id:
                continue
            try:
                tf = buf.lookup_transform(BASE, m.header.frame_id, rclpy.time.Time())
            except Exception:
                return
            p = mk.pose.position; o = mk.pose.orientation
            pos, nrm = to_base(tf, (p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
            data.setdefault(m.header.frame_id, []).append((pos, nrm, p.z))

    n.create_subscription(ArucoDetection, "/aruco_detections", cb, 10)
    t0 = time.time()
    while time.time() - t0 < a.seconds:
        rclpy.spin_once(n, timeout_sec=0.05)
    if not data:
        print("NO detections of marker %d transformed to %s (is a tracker running, is TF connected?)" % (a.id, BASE)); return
    mean = {}
    for fr, v in sorted(data.items()):
        P = np.array([x[0] for x in v]); N = np.array([x[1] for x in v])
        p = P.mean(0); nm = N.mean(0); nm /= np.linalg.norm(nm)
        tilt = math.degrees(math.acos(min(1.0, abs(nm[2]))))
        cam = "WRIST" if "armcam" in fr else ("TOP" if "topcam" in fr else fr)
        mean[cam] = p
        print("%-5s (%s, %d samples, camera %.2f m away)" % (cam, fr, len(v), np.mean([x[2] for x in v])))
        print("      marker x=%+.4f y=%+.4f z=%+.4f m   tilt %.1f deg   spread %.1f mm"
              % (p[0], p[1], p[2], tilt, 1000 * P.std(0).max()))
    if "WRIST" in mean and "TOP" in mean:
        d = mean["WRIST"] - mean["TOP"]
        print("WRIST - TOP: dx=%+.1f dy=%+.1f dz=%+.1f mm   (horizontal %.1f mm, total %.1f mm)"
              % (1000 * d[0], 1000 * d[1], 1000 * d[2], 1000 * math.hypot(d[0], d[1]), 1000 * np.linalg.norm(d)))


if __name__ == "__main__":
    main()
