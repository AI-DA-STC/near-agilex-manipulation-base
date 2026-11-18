#!/usr/bin/env python3
"""
p3_grasp_watch.py - READ-ONLY. Shows EVERY grasp Contact-GraspNet generates, in 3D, on the point cloud, in its own Open3D window.

Run it in its own terminal (conda OFF, workspace sourced) while pipeline 3 is running; it starts the viewer itself:
    conda deactivate; source ~/agx_arm_ws/install/setup.bash
    /usr/bin/python3 ~/nero_pipeline_launch/pipeline3/p3_grasp_watch.py

Every time Contact-GraspNet publishes its grasps (/contact_graspnet/grasps, during p3_look.sh), it saves a snapshot in /tmp/p3_view/scene.npz:
the colour point cloud of that moment and all grasps in base_link, plus the grasp(s) the relay passed on to MoveIt (/p3/grasps). The viewer
(p3_grasp_view.py, in the contact_graspnet conda env, which has Open3D) reloads it by itself. Ctrl+C here stops both.
"""
import os
import signal
import subprocess
import sys
import time

import numpy as np

P3_GRASP_WATCH_VERSION = "p3-graspwatch-v1"
OUT_DIR = "/tmp/p3_view"
OUT = os.path.join(OUT_DIR, "scene.npz")
VIEWER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "p3_grasp_view.py")
CONDA_ENV = os.environ.get("P3_VIEW_ENV", "contact_graspnet")
MAX_POINTS = 120000


def quat_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 1e-12 else 0.0
    return np.array([[1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
                     [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
                     [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def poses_to_base(poses, T_base_cam):
    """[(position xyz, quaternion xyzw)] in the camera frame -> N x 4 x 4 in base_link."""
    out = []
    for p, q in poses:
        G = np.eye(4)
        G[:3, :3] = quat_matrix(*q)
        G[:3, 3] = p
        out.append(T_base_cam @ G)
    return np.array(out).reshape(-1, 4, 4)


def cloud_in_base(depth_m, K, T_base_cam, rgb=None, max_points=MAX_POINTS, seed=0):
    """Depth image -> (points N x 3 in base_link, colours N x 3 in 0..1)."""
    H, W = depth_m.shape
    vs, us = np.mgrid[0:H, 0:W]
    ok = np.isfinite(depth_m) & (depth_m > 0)
    z = depth_m[ok]
    cam = np.stack([(us[ok] - K[0, 2]) * z / K[0, 0], (vs[ok] - K[1, 2]) * z / K[1, 1], z], axis=1)
    base = cam @ T_base_cam[:3, :3].T + T_base_cam[:3, 3]
    col = (rgb[ok][:, ::-1] / 255.0) if rgb is not None and rgb.shape[:2] == depth_m.shape else np.full((len(base), 3), 0.6)
    if len(base) > max_points:
        idx = np.random.default_rng(seed).choice(len(base), max_points, replace=False)
        base, col = base[idx], col[idx]
    return base.astype(np.float32), col.astype(np.float32)


def save(path, points, colours, grasps, chosen, note):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp.npz"
    np.savez_compressed(tmp, points=points, colours=colours, grasps=grasps, chosen=chosen, note=np.array(note), time=np.array(time.time()))
    os.replace(tmp, path)


def main():
    import rclpy
    import tf2_ros
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseArray
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = Node("p3_grasp_watch")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    bridge = CvBridge()
    latest = {}
    state = {"grasps": np.zeros((0, 4, 4)), "chosen": np.zeros((0, 4, 4)), "points": None, "colours": None, "dirty": False}

    def T_base(frame):
        tr = buf.lookup_transform("base_link", frame, rclpy.time.Time())
        q, p = tr.transform.rotation, tr.transform.translation
        T = np.eye(4)
        T[:3, :3] = quat_matrix(q.x, q.y, q.z, q.w)
        T[:3, 3] = [p.x, p.y, p.z]
        return T

    def to_list(msg):
        return [((p.position.x, p.position.y, p.position.z), (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)) for p in msg.poses]

    def on_grasps(msg):                                     # every Contact-GraspNet result: a new snapshot of the scene
        try:
            T = T_base(msg.header.frame_id)
            state["grasps"] = poses_to_base(to_list(msg), T)
            state["chosen"] = np.zeros((0, 4, 4))
            if "d" in latest and "i" in latest:
                d = bridge.imgmsg_to_cv2(latest["d"], desired_encoding="passthrough").astype(np.float64)
                d *= 0.001 if latest["d"].encoding in ("16UC1", "mono16") else 1.0
                rgb = bridge.imgmsg_to_cv2(latest["c"], desired_encoding="bgr8") if "c" in latest else None
                K = np.array(latest["i"].k).reshape(3, 3)
                state["points"], state["colours"] = cloud_in_base(d, K, T_base(latest["i"].header.frame_id), rgb)
            state["dirty"] = True
            print("[%s] Contact-GraspNet: %d grasp(s) -> viewer" % (time.strftime("%H:%M:%S"), len(state["grasps"])), flush=True)
        except Exception as e:
            print("could not take the snapshot: %s" % e, flush=True)

    def on_chosen(msg):                                     # what the relay passed on to MoveIt (jaws already turned)
        try:
            state["chosen"] = poses_to_base(to_list(msg), T_base(msg.header.frame_id))
            state["dirty"] = True
        except Exception as e:
            print("could not read /p3/grasps: %s" % e, flush=True)

    node.create_subscription(PoseArray, "/contact_graspnet/grasps", on_grasps, 10)
    node.create_subscription(PoseArray, "/p3/grasps", on_chosen, 10)
    node.create_subscription(Image, "/topcam/depth/image_raw", lambda m: latest.__setitem__("d", m), qos_profile_sensor_data)
    node.create_subscription(Image, "/topcam/color/image_raw", lambda m: latest.__setitem__("c", m), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, "/topcam/depth/camera_info", lambda m: latest.__setitem__("i", m), qos_profile_sensor_data)

    viewer = subprocess.Popen(["conda", "run", "-n", CONDA_ENV, "--no-capture-output", "python", "-u", VIEWER, OUT], start_new_session=True)
    print("p3_grasp_watch %s: waiting for Contact-GraspNet (run p3_look.sh). The 3D viewer (%s env) opens and refreshes after every run." % (
        P3_GRASP_WATCH_VERSION, CONDA_ENV), flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if state["dirty"] and state["points"] is not None:
                time.sleep(0.5)                              # the relay's /p3/grasps follows Contact-GraspNet within a moment
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=0.05)
                save(OUT, state["points"], state["colours"], state["grasps"], state["chosen"],
                     "%d Contact-GraspNet grasp(s), %d passed on to MoveIt" % (len(state["grasps"]), len(state["chosen"])))
                state["dirty"] = False
    except KeyboardInterrupt:
        pass
    finally:
        try:
            os.killpg(viewer.pid, signal.SIGTERM)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
