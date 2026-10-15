#!/usr/bin/env python3
"""
grasp_watch.py - READ-ONLY. Shows which grasp the arm really uses.

Run in its own terminal BEFORE the pipeline (conda OFF, workspace sourced):
    export JOINT_STATES_TOPIC=/feedback/joint_states      # real arm   (sim: /control/joint_states)
    python3 grasp_watch.py

 * listens to /grasp_poses_transformed (the candidates) and converts them with the SAME functions
   as mtc_pick_and_place.py (it imports them), so you see exactly the poses MTC is given
 * when the gripper closes it reads where tcp_link really is, finds the nearest candidate,
   prints the result and writes /tmp/grasp_debug.json
 * opens the Open3D viewer (grasp_view.py) which shows candidates, the used one, and the real pose
"""
import inspect, json, math, os, subprocess, sys, time
import numpy as np

JSON_PATH = "/tmp/grasp_debug.json"


def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose_to_T(p):
    T = np.eye(4)
    T[:3, :3] = quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def tf_to_T(ts):
    t, q = ts.transform.translation, ts.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def select_candidates(poses, M, sort_by_tilt=False, convert=False):
    """Same filtering/(sorting)/slicing/(conversion) as build_and_run_task in mtc_pick_and_place.py.

    convert: whether the script applies to_tcp_pose() to the candidates. The script only does so when APPLY_TCP_CONVERSION is True
    (older versions never called it: the raw Contact-GraspNet pose was used directly as the tcp_link target)."""
    usable = [p for p in poses if M.tilt_from_down_deg(p.orientation) <= M.MAX_TILT_DEG]
    if sort_by_tilt:
        usable.sort(key=lambda q: M.tilt_from_down_deg(q.orientation))      # most top-down first
    usable = usable[:M.MAX_CANDIDATES_IN_TASK]
    return [M.to_tcp_pose(p) for p in usable] if convert else usable


def release_report(grasp_T, release_T, M):
    """How far did tcp_link rise between the grasp and the release? Returns (text, rise_mm, expected_mm or None).
    With the automatic release height the rise should equal plate thickness + gap (for vertical grasp and release)."""
    rise = 1000 * float(release_T[2, 3] - grasp_T[2, 3])
    mode = getattr(M, "PLACE_HEIGHT_MODE", None)
    if mode == "auto":
        exp = 1000 * (getattr(M, "PLATE_THICKNESS_M", 0.0) + getattr(M, "PLACE_RELEASE_GAP_M", 0.0))
        return ("RELEASE: tcp rose %.0f mm between grasp and release; the automatic height expects about %.0f mm "
                "(plate thickness + gap) for a vertical grasp and release; difference %+.0f mm. Tilt at release adds to the drop." % (rise, exp, rise - exp)), rise, exp
    return "RELEASE: tcp rose %.0f mm between grasp and release (fixed height mode)." % rise, rise, None


def match_used(T_act, cands):
    """Nearest candidate to the pose the gripper really closed at."""
    best = None
    for i, Tc in enumerate(cands):
        d = float(np.linalg.norm(T_act[:3, 3] - Tc[:3, 3]))
        axis = math.degrees(math.acos(float(np.clip(T_act[:3, 2] @ Tc[:3, 2], -1, 1))))
        Rrel = Tc[:3, :3].T @ T_act[:3, :3]
        rec = dict(index=i, dist_mm=1000 * d, axis_deg=axis,
                   roll_deg=math.degrees(math.atan2(Rrel[1, 0], Rrel[0, 0])),
                   along_mm=1000 * float((T_act[:3, 3] - Tc[:3, 3]) @ Tc[:3, 2]))
        if best is None or rec["dist_mm"] < best["dist_mm"]:
            best = rec
    return best


def verdict(m):
    ok = m["dist_mm"] < 20 and m["axis_deg"] < 5
    roll = abs(m["roll_deg"])
    variant = "same jaw direction" if roll < 90 else "jaw direction flipped 180 deg"
    return ok, variant


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    from geometry_msgs.msg import PoseArray
    from sensor_msgs.msg import JointState, CameraInfo
    import tf2_ros
    from nero_mtc_task import mtc_pick_and_place as M

    topic = os.environ.get("JOINT_STATES_TOPIC", "/feedback/joint_states")
    latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)

    class Watch(Node):
        def __init__(self):
            super().__init__("grasp_watch")
            self.buf = tf2_ros.Buffer(); self.lis = tf2_ros.TransformListener(self.buf, self)
            self.raw, self.cands, self.K, self.cam_frame = [], [], None, None
            self.gripper_open = None; self.viewer = None; self.grasp_T = None
            self.create_subscription(PoseArray, "/grasp_poses_transformed", self.on_poses, latched)
            self.create_subscription(CameraInfo, "/topcam/depth/camera_info", self.on_info, 10)
            self.create_subscription(JointState, topic, self.on_js, 20)
            self.get_logger().info("watching %s - run the pipeline now (candidates appear after detection)" % topic)

        def on_info(self, m):
            self.K, self.cam_frame = list(m.k), m.header.frame_id

        def T_base_cam(self):
            if not self.cam_frame:
                return None
            try:
                return tf_to_T(self.buf.lookup_transform("base_link", self.cam_frame, rclpy.time.Time())).tolist()
            except Exception:
                return None

        def write(self, used=None, actual=None, match=None):
            json.dump(dict(time=time.time(), K=self.K, cam_frame=self.cam_frame, T_base_cam=self.T_base_cam(),
                           raw=[pose_to_T(p).tolist() for p in self.raw],
                           cands=[pose_to_T(p).tolist() for p in self.cands],
                           used=used, actual=None if actual is None else actual.tolist(), match=match),
                      open(JSON_PATH, "w"))

        def on_poses(self, msg):
            self.raw = list(msg.poses)
            self.sort_by_tilt = "usable.sort(" in inspect.getsource(M)        # follow whatever the MTC file does
            self.convert = bool(getattr(M, "APPLY_TCP_CONVERSION", False))     # older script versions never converted
            self.cands = select_candidates(self.raw, M, self.sort_by_tilt, self.convert)
            self.write()
            self.get_logger().info("candidates: %d raw -> %d usable tcp targets given to MTC (tilt<=%.0f deg, %s)" %
                                   (len(self.raw), len(self.cands), M.MAX_TILT_DEG,
                                    ("most top-down first" if self.sort_by_tilt else "best score first") +
                                    (", converted with to_tcp_pose" if self.convert else ", raw Contact-GraspNet poses used as tcp targets")))
            self.get_logger().info("tilts given to MTC (deg): %s" % [round(M.tilt_from_down_deg(p.orientation)) for p in self.cands])
            self.open_viewer()

        def open_viewer(self):
            if self.viewer and self.viewer.poll() is None:
                return
            py = os.environ.get("GRASP_VIEW_PY", os.path.expanduser("~/miniconda3/envs/contact_graspnet/bin/python"))
            script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "grasp_view.py")
            if not os.path.exists(py):
                self.get_logger().warn("viewer python not found (%s). Run by hand: conda activate contact_graspnet; python %s" % (py, script)); return
            self.viewer = subprocess.Popen([py, script, JSON_PATH])

        def on_js(self, msg):
            if "gripper" not in msg.name:
                return
            g = msg.position[msg.name.index("gripper")]
            is_open = g > 0.07
            if self.gripper_open is None:
                self.gripper_open = is_open; return
            if self.gripper_open and not is_open:                       # open -> closing = THE GRASP
                try:
                    T = tf_to_T(self.buf.lookup_transform("base_link", "tcp_link", rclpy.time.Time()))
                except Exception as e:
                    self.get_logger().warn("no TF base_link->tcp_link: %s" % e); self.gripper_open = is_open; return
                if not self.cands:
                    self.get_logger().warn("gripper closed but no candidates were received")
                else:
                    self.grasp_T = T
                    m = match_used(T, [pose_to_T(p) for p in self.cands])
                    ok, variant = verdict(m)
                    self.write(used=m["index"], actual=T, match=m)
                    self.get_logger().info(
                        "GRASP at x=%.3f y=%.3f z=%.3f | nearest candidate #%d: %.0f mm away, approach axis off by %.1f deg, "
                        "%.0f mm along the approach (negative = stopped short), %s -> %s" %
                        (T[0, 3], T[1, 3], T[2, 3], m["index"], m["dist_mm"], m["axis_deg"], m["along_mm"], variant,
                         "ARM FOLLOWED THE GENERATED GRASP" if ok else "DOES NOT MATCH ANY CANDIDATE"))
            elif (not self.gripper_open) and is_open and self.grasp_T is not None:   # closed -> open = THE RELEASE
                try:
                    Tr = tf_to_T(self.buf.lookup_transform("base_link", "tcp_link", rclpy.time.Time()))
                    text, _rise, _exp = release_report(self.grasp_T, Tr, M)
                    self.get_logger().info(text + " | release at x=%.3f y=%.3f z=%.3f" % (Tr[0, 3], Tr[1, 3], Tr[2, 3]))
                except Exception as e:
                    self.get_logger().warn("no TF at release: %s" % e)
                self.grasp_T = None
            self.gripper_open = is_open

    rclpy.init(); n = Watch()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
