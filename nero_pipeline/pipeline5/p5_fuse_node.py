#!/usr/bin/env python3
"""p4_fuse_node: live fusion of the top camera (Orbbec) and the wrist camera (D405) in base_link.

For each camera: take the latest cloud -> keep the camera's useful range -> move it into base_link with TF
(at the cloud's own timestamp, else the latest transform) -> add that camera's offset correction ->
crop to the workspace -> remove the arm's own body -> merge both into voxels.
Publishes /p5/fused_points (base_link, colour = which camera saw the voxel):
  blue = top only, red = wrist only, magenta = both.
It moves nothing. v2 adds two services that change MoveIt's planning scene (boxes named p4_wrist_*, never pipeline 3's p3_obst_*):
  /p5/wrist_obstacles        wrist camera (last few clouds) -> 3D obstacle boxes, with the object being picked and the place's landing zone
                             left out (from GraspMolmo's masks of the latest look, which this node caches) -> MoveIt, confirmed
  /p5/clear_wrist_obstacles  remove every p4_wrist_* box
Keep this node running BEFORE p4_look.sh: GraspMolmo's masks are published only once per look.

Run with conda deactivated:
  /usr/bin/python3 p4_fuse_node.py
Camera offsets: read from ~/.p5_offsets.yaml (PIPELINE 5; keys wrist_offset_x/y/z, top_offset_x/y/z, in metres) if it exists;
a ROS parameter overrides it, e.g.  /usr/bin/python3 p4_fuse_node.py --ros-args -p top_offset_z:=-0.010
"""
import numpy as np

VERSION = "p5-fuse-v2"   # v2: waits for THIS look's place region (P5_EARLY_WRIST=1), wrist timing in the message
# p5-fuse-v1:   # PIPELINE 5 copy of p4-fuse-v5: own names /p5/..., p5_wrist_* boxes, ~/.p5_offsets.yaml, ~/p5_runs
# p4-fuse-v5:   # v5: pipeline 3 parts from pipeline4/core (independent of pipeline 3)
# v4:   # v4: default camera offsets from ~/.p4_offsets.yaml (ROS parameters still override)
# v3: camera messages never wait behind the slow fusion cycle (top cloud went stale); cheaper arm filter; cloud ages in the log
# v2: /p5/wrist_obstacles and /p5/clear_wrist_obstacles (wrist boxes into MoveIt)
BASE = "base_link"
TOP_ONLY, WRIST_ONLY, BOTH = 1, 2, 3
COLOURS = {TOP_ONLY: (0, 120, 255), WRIST_ONLY: (255, 40, 40), BOTH: (230, 0, 230)}


# ---------- pure numpy helpers (tested offline) ----------

def cloud_xyz(msg, stride=1):
    """PointCloud2 -> (N,3) float64 of the finite, non-zero points (every `stride`-th point)."""
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                   "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
    a = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)[::max(1, stride)]
    p = np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64)
    return p[np.isfinite(p).all(1) & (np.abs(p).sum(1) > 1e-6)]


def qmat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def range_filter(p_cam, zmin, zmax):
    """Keep points whose optical-frame depth (z) is in [zmin, zmax]."""
    return p_cam[(p_cam[:, 2] >= zmin) & (p_cam[:, 2] <= zmax)]


def to_base(p_cam, R, t, offset=(0.0, 0.0, 0.0)):
    return p_cam @ np.asarray(R).T + np.asarray(t) + np.asarray(offset)


def crop(p, box):
    """box = (xmin, xmax, ymin, ymax, zmin, zmax) in base_link."""
    x0, x1, y0, y1, z0, z1 = box
    m = (p[:, 0] >= x0) & (p[:, 0] <= x1) & (p[:, 1] >= y0) & (p[:, 1] <= y1) & (p[:, 2] >= z0) & (p[:, 2] <= z1)
    return p[m]


def self_filter(p, chain, radius):
    """Remove points within `radius` of the arm, modelled as capsules between consecutive frame origins."""
    if len(p) == 0 or len(chain) < 2:
        return p
    c = np.asarray(chain, dtype=np.float64)
    near = np.all((p >= c.min(0) - radius) & (p <= c.max(0) + radius), axis=1)   # v3: only these can be on the arm
    if not near.any():
        return p
    keep = np.ones(len(p), bool)
    idx = np.nonzero(near)[0]
    q = p[idx]
    kq = np.ones(len(q), bool)
    for a, b in zip(chain[:-1], chain[1:]):
        a, b = np.asarray(a), np.asarray(b)
        ab = b - a
        L2 = float(ab @ ab)
        s = np.zeros(len(q)) if L2 < 1e-12 else np.clip((q - a) @ ab / L2, 0.0, 1.0)
        d = np.linalg.norm(q - (a + s[:, None] * ab), axis=1)
        kq &= d > radius
    keep[idx] = kq
    return p[keep]


def fuse_voxels(p_top, p_wrist, voxel):
    """Merge both clouds into voxel centres. Returns (centres (M,3), source code per voxel)."""
    parts, src = [], []
    for p, code in ((p_top, TOP_ONLY), (p_wrist, WRIST_ONLY)):
        if len(p):
            parts.append(np.floor(p / voxel).astype(np.int64)); src.append(np.full(len(p), code, np.int8))
    if not parts:
        return np.zeros((0, 3)), np.zeros(0, np.int8)
    keys = np.concatenate(parts); codes = np.concatenate(src)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.ravel()
    flags = np.zeros(len(uniq), np.int8)
    np.bitwise_or.at(flags, inv, codes)
    # PIPELINE 5: each voxel's point is the MEAN of the camera points in it (it lies on the surface), not the cell's centre (up to 5 mm off);
    # the grasp step's 3 mm margin against these points is then a real 3 mm
    pts = np.concatenate([p for p in (p_top, p_wrist) if len(p)])
    sums = np.zeros((len(uniq), 3))
    np.add.at(sums, inv, pts)
    return sums / np.bincount(inv, minlength=len(uniq)).astype(float)[:, None], flags


def pack_xyzrgb(points, flags):
    """(M,3) + source codes -> bytes for a PointCloud2 with fields x, y, z, rgb (16 bytes per point)."""
    rec = np.zeros(len(points), dtype=np.dtype({"names": ["x", "y", "z", "rgb"],
                                                 "formats": ["<f4", "<f4", "<f4", "<u4"],
                                                 "offsets": [0, 4, 8, 12], "itemsize": 16}))
    rec["x"], rec["y"], rec["z"] = points[:, 0], points[:, 1], points[:, 2]
    rgb = np.zeros(len(points), np.uint32)
    for code, (r, g, b) in COLOURS.items():
        rgb[flags == code] = (r << 16) | (g << 8) | b
    rec["rgb"] = rgb
    return rec.tobytes()


# ---------- ROS node ----------

NOT_ROBOT_FRAMES = ("topcam", "camera", "base_link", "world", "map", "odom", "marker")   # TF frames that are not parts of the arm
P4_PREFIX = "p5_wrist_"   # PIPELINE 5: own box names


OFFSETS_FILE = "~/.p5_offsets.yaml"   # PIPELINE 5


def load_offsets(path=OFFSETS_FILE):
    """{"wrist_offset_z": -0.014, ...} from the offsets file, {} if missing or unreadable (unknown keys ignored)."""
    import os
    import yaml
    try:
        d = yaml.safe_load(open(os.path.expanduser(path))) or {}
    except Exception:
        return {}
    keys = ["%s_offset_%s" % (c, a) for c in ("wrist", "top") for a in "xyz"]
    return {k: float(d[k]) for k in keys if k in d}


def main():
    import json
    import os
    import sys
    import threading
    import time
    from collections import deque
    import yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener

    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, here)
    sys.path.insert(0, os.path.join(here, "core"))           # v5: pipeline 4's own copies
    import p5_wrist_obstacles as W
    import p5_keepout                                        # PIPELINE 5: the gripper's checked corridor
    import p3_tablez
    import p3_scene

    class P4Scene(p3_scene.AsyncScene):
        """pipeline 3's confirmed scene client, for the p4_wrist_* boxes only (pipeline 3's p3_obst_* boxes are never touched).
        Own node name (p4_scene_client), so it never clashes with the GraspMolmo node's p3_scene_client."""

        def __init__(self, timeout=10.0):
            from rclpy.executors import SingleThreadedExecutor
            node = rclpy.create_node("p5_scene_client")
            p3_scene.Scene.__init__(self, node, timeout)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(node)
            threading.Thread(target=self.executor.spin, daemon=True).start()

        def obstacle_ids(self):
            return sorted(co.id for co in self.objects() if co.id.startswith(P4_PREFIX))

        def replace(self, remove_ids, boxes):
            from geometry_msgs.msg import Pose
            from moveit_msgs.msg import CollisionObject, PlanningScene
            from shape_msgs.msg import SolidPrimitive
            ps = PlanningScene(); ps.is_diff = True
            for i in remove_ids:
                co = CollisionObject(); co.id, co.header.frame_id, co.operation = i, "base_link", CollisionObject.REMOVE
                ps.world.collision_objects.append(co)
            for k, b in enumerate(boxes):
                co = CollisionObject(); co.id, co.header.frame_id, co.operation = P4_PREFIX + "%02d" % k, "base_link", CollisionObject.ADD
                co.pose.orientation.w = 1.0
                co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in b["size"]])]
                pose = Pose(); pose.position.x, pose.position.y, pose.position.z = (float(v) for v in b["centre"]); pose.orientation.w = 1.0
                co.primitive_poses = [pose]
                ps.world.collision_objects.append(co)
            req = self.ApplyPlanningScene.Request(); req.scene = ps
            return bool(self._call(self.apply, req, "/apply_planning_scene").success)

        def set_obstacles(self, boxes):
            before = self.obstacle_ids()
            ok = self.replace(before, boxes)
            after = self.obstacle_ids()
            want = sorted(P4_PREFIX + "%02d" % k for k in range(len(boxes)))
            if ok and after == want:
                return True, "MoveIt confirmed %d wrist box(es)%s" % (len(boxes), "" if not before else " (replaced %d from before)" % len(before))
            return False, "MoveIt NOT confirmed: it %s the change and holds %d p4_wrist box(es), expected %d" % (
                "accepted" if ok else "REFUSED", len(after), len(want))

    class Fuse(Node):
        def __init__(self):
            super().__init__("p5_fuse")
            P = self.declare_parameter
            self.top_topic = P("top_topic", "/topcam/depth/points").value
            self.wrist_topic = P("wrist_topic", "/camera/armcam/depth/color/points").value
            off = load_offsets()
            self.top_off = [P("top_offset_%s" % k, off.get("top_offset_%s" % k, 0.0)).value for k in "xyz"]
            self.wrist_off = [P("wrist_offset_%s" % k, off.get("wrist_offset_%s" % k, 0.0)).value for k in "xyz"]
            self.top_range = (P("top_min_m", 0.30).value, P("top_max_m", 1.20).value)
            self.wrist_range = (P("wrist_min_m", 0.07).value, P("wrist_max_m", 0.50).value)
            self.box = tuple(P(k, v).value for k, v in (("x_min", -0.90), ("x_max", 0.10), ("y_min", -0.60),
                                                         ("y_max", 0.60), ("z_min", -0.03), ("z_max", 0.80)))
            self.voxel = P("voxel_m", 0.01).value
            self.stride = int(P("stride", 2).value)
            self.self_radius = P("self_radius_m", 0.07).value
            self.link_radius = P("link_radius_m", 0.10).value          # like pipeline 3's LINK_RADIUS, around every robot TF frame
            self.chain = [s for s in P("arm_frames", "base_link,link1,link2,link3,link4,link5,link6,link7,tcp_link").value.split(",") if s]
            self.n_wrist = int(P("wrist_clouds", 5).value)              # clouds combined for the wrist obstacles (arm must be still)
            self.depth_scale = P("depth_scale_to_meters", 0.001).value
            self.buf = Buffer(); self.tfl = TransformListener(self.buf, self, spin_thread=True)
            self.last, self.wrist_q, self.lock = {}, deque(maxlen=self.n_wrist), threading.Lock()
            # v3: receiving messages (only stores them) runs in its own group, in parallel with the slow fusion cycle and the services,
            # so the newest cloud is always the one fused
            self.cb_in, self.cb_timer, self.cb_srv = ReentrantCallbackGroup(), MutuallyExclusiveCallbackGroup(), MutuallyExclusiveCallbackGroup()
            self.create_subscription(PointCloud2, self.top_topic, lambda m: self.last.__setitem__("top", m), qos_profile_sensor_data,
                                     callback_group=self.cb_in)
            self.create_subscription(PointCloud2, self.wrist_topic, self._wrist_cb, qos_profile_sensor_data, callback_group=self.cb_in)
            for key, topic, typ, qos in (("depth", P("top_depth_topic", "/topcam/depth/image_raw").value, Image, qos_profile_sensor_data),
                                         ("info", P("top_info_topic", "/topcam/depth/camera_info").value, CameraInfo, qos_profile_sensor_data),
                                         ("mask_source", "/p5/mask_source", Image, 10),   # PIPELINE 5: the mask the grasp step used (SAM2)
                                         ("mask_target", "/p5/mask_target", Image, 10)):   # PIPELINE 5: the place point after /p5/place
                self.create_subscription(typ, topic, lambda m, k=key: self._keep(k, m), qos, callback_group=self.cb_in)
            self.pub = self.create_publisher(PointCloud2, "/p5/fused_points", 1)
            self.create_timer(1.0 / P("rate_hz", 2.0).value, self.tick, callback_group=self.cb_timer)
            self.create_service(Trigger, "/p5/wrist_obstacles", self._wrist_obstacles_cb, callback_group=self.cb_srv)
            self.create_service(Trigger, "/p5/clear_wrist_obstacles", self._clear_cb, callback_group=self.cb_srv)
            self.tick_s = 0.0
            self.scene = P4Scene()
            self.stats_t = time.time(); self.fallback = {"top": 0, "wrist": 0}
            self.get_logger().info("offsets file %s: %s" % (OFFSETS_FILE, off if off else "none (all offsets 0 unless set as parameters)"))
            self.get_logger().info("%s + %s  top=%s  wrist=%s  top_offset=%s  wrist_offset=%s  voxel=%.3f; services /p5/wrist_obstacles, "
                                   "/p5/clear_wrist_obstacles" % (VERSION, W.P4_WRIST_OBSTACLES_VERSION, self.top_topic, self.wrist_topic,
                                                                  self.top_off, self.wrist_off, self.voxel))

        def _keep(self, key, m):
            self.last[key] = m
            if key.startswith("mask_"):
                self.last[key + "_time"] = time.time()
                self.get_logger().info("cached GraspMolmo's %s (%dx%d) for the wrist obstacles" % (key, m.width, m.height))

        def _wrist_cb(self, m):
            self.last["wrist"] = m
            with self.lock:
                self.wrist_q.append(m)

        def lookup(self, frame, stamp, key):
            try:
                if stamp is not None and self.buf.can_transform(BASE, frame, Time.from_msg(stamp)):
                    return self.buf.lookup_transform(BASE, frame, Time.from_msg(stamp))
                if key:
                    self.fallback[key] += 1
                return self.buf.lookup_transform(BASE, frame, Time())
            except Exception:
                return None

        def arm_chain(self):
            out = []
            for f in self.chain:
                tf = self.lookup(f, None, None)
                if tf is not None:
                    v = tf.transform.translation; out.append((v.x, v.y, v.z))
            return out

        def robot_points(self):
            try:
                frames = yaml.safe_load(self.buf.all_frames_as_yaml()) or {}
            except Exception:
                frames = {}
            pts = []
            for f in frames:
                if any(s in f for s in NOT_ROBOT_FRAMES):
                    continue
                tf = self.lookup(f, None, None)
                if tf is not None:
                    v = tf.transform.translation; pts.append((v.x, v.y, v.z))
            return pts

        def cloud_in_base(self, m, key, rng, off):
            tf = self.lookup(m.header.frame_id, m.header.stamp, key)
            if tf is None:
                return None
            t, r = tf.transform.translation, tf.transform.rotation
            p = range_filter(cloud_xyz(m, self.stride), *rng)
            return crop(to_base(p, qmat(r.x, r.y, r.z, r.w), (t.x, t.y, t.z), off), self.box)

        def one(self, key, rng, off):
            m = self.last.get(key)
            if m is None:
                return np.zeros((0, 3)), "no cloud"
            p = self.cloud_in_base(m, key, rng, off)
            if p is None:
                return np.zeros((0, 3)), "no TF %s->%s" % (BASE, m.header.frame_id)
            return p, "ok"

        def age(self, key):
            m = self.last.get(key)
            if m is None:
                return float("nan")
            return (self.get_clock().now().nanoseconds - (m.header.stamp.sec * 1e9 + m.header.stamp.nanosec)) / 1e9

        def tick(self):
            t_start = time.time()
            chain = self.arm_chain()
            pt, st = self.one("top", self.top_range, self.top_off)
            pw, sw = self.one("wrist", self.wrist_range, self.wrist_off)
            pt, pw = self_filter(pt, chain, self.self_radius), self_filter(pw, chain, self.self_radius)
            cen, flags = fuse_voxels(pt, pw, self.voxel)
            msg = PointCloud2()
            msg.header.frame_id = BASE; msg.header.stamp = self.get_clock().now().to_msg()
            msg.height, msg.width = 1, len(cen)
            msg.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32 if n != "rgb" else PointField.UINT32, count=1)
                          for n, o in (("x", 0), ("y", 4), ("z", 8), ("rgb", 12))]
            msg.is_bigendian = False; msg.point_step = 16; msg.row_step = 16 * len(cen); msg.is_dense = True
            msg.data = pack_xyzrgb(cen, flags)
            self.pub.publish(msg)
            self.tick_s = 0.8 * self.tick_s + 0.2 * (time.time() - t_start) if self.tick_s else time.time() - t_start
            if time.time() - self.stats_t > 5:
                self.stats_t = time.time()
                self.get_logger().info("voxels: top-only %d, wrist-only %d, both %d | top %s (cloud %.1f s old), wrist %s (%.1f s old) | cycle %.2f s | "
                                       "arm frames %d/%d | latest-TF fallbacks top %d wrist %d"
                                       % ((flags == TOP_ONLY).sum(), (flags == WRIST_ONLY).sum(), (flags == BOTH).sum(), st, self.age("top"),
                                          sw, self.age("wrist"), self.tick_s, len(chain), len(self.chain), self.fallback["top"], self.fallback["wrist"]))

        # ------------------------------------------------ services
        def _wrist_obstacles_cb(self, req, res):
            try:
                res.success, res.message = self.wrist_obstacles()
            except Exception as e:
                res.success, res.message = False, "wrist obstacles FAILED: %s: %s" % (type(e).__name__, e)
            self.get_logger().info(res.message)
            return res

        def _clear_cb(self, req, res):
            try:
                res.success, res.message = self.scene.set_obstacles([])
            except Exception as e:
                res.success, res.message = False, "clear FAILED: %s: %s" % (type(e).__name__, e)
            self.get_logger().info(res.message)
            return res

        def wrist_obstacles(self):
            for k, why in (("mask_source", "GraspMolmo's object region"), ("mask_target", "GraspMolmo's place region"),
                           ("depth", "the top camera's depth image"), ("info", "the top camera's camera_info")):
                if k not in self.last:
                    return False, ("REFUSED: no %s yet. Keep this node running and run p4_look.sh (GraspMolmo publishes its masks once per look)"
                                   % why if k.startswith("mask") else "REFUSED: no %s yet - is the top camera running?" % why)
            t0 = time.time()   # PIPELINE 5 speed (p5-fuse-v2): THIS look's place region must be here (P5_EARLY_WRIST=1 calls right after /p5/place)
            while self.last.get("mask_target_time", 0.0) < self.last.get("mask_source_time", 0.0) and time.time() - t0 < 5.0:
                time.sleep(0.05)
            if self.last.get("mask_target_time", 0.0) < self.last.get("mask_source_time", 0.0):
                return False, ("REFUSED: the place region (/p5/mask_target) is older than this look's object region - "
                               "/p5/place has not published this look's place yet")
            t_wait = time.time() - t0
            with self.lock:
                clouds = list(self.wrist_q)
            if not clouds:
                return False, "REFUSED: no wrist cloud - run p4_wrist_up.sh"
            depth = W.depth_to_m(self.last["depth"], self.depth_scale)
            info = self.last["info"]
            K = np.array(info.k, dtype=float).reshape(3, 3)
            ms, mt = W.image_to_array(self.last["mask_source"], np.uint8), W.image_to_array(self.last["mask_target"], np.uint8)
            if ms.shape != depth.shape or mt.shape != depth.shape:
                return False, "REFUSED: mask %s / %s and depth %s differ in size" % (ms.shape, mt.shape, depth.shape)
            tf = self.lookup(info.header.frame_id, None, None)
            if tf is None:
                return False, "REFUSED: no TF base_link -> %s (top camera calibration not published?)" % info.header.frame_id
            t, r = tf.transform.translation, tf.transform.rotation
            R, tt = qmat(r.x, r.y, r.z, r.w), np.array([t.x, t.y, t.z]) + np.array(self.top_off)
            tz = p3_tablez.table_z()[0]
            tbox = W.target_box(W.mask_points_base(ms, depth, K, R, tt), tz)
            place = W.place_point(W.mask_points_base(mt, depth, K, R, tt))
            if tbox is None:
                return False, "REFUSED: the object region has no valid top-camera depth, so it cannot be left out of the wrist obstacles"
            chain, robot = self.arm_chain(), self.robot_points()
            parts, n_tf_missing = [], 0
            for m in clouds:
                p = self.cloud_in_base(m, "wrist", self.wrist_range, self.wrist_off)
                if p is None:
                    n_tf_missing += 1
                    continue
                p = self_filter(p, chain, self.self_radius)
                for rp in robot:
                    if len(p):
                        p = p[np.linalg.norm(p - np.asarray(rp), axis=1) > self.link_radius]
                parts.append(p)
            if not parts:
                return False, "REFUSED: no TF for any wrist cloud (is p4_wrist_up.sh's calibration publisher running?)"
            Pw = np.concatenate(parts)
            Pw, ko_note = p5_keepout.apply_latest(Pw, self.last.get("mask_source_time"))   # PIPELINE 5: corridor left out
            t_clouds = time.time()
            self.get_logger().info(ko_note)
            boxes, note = W.wrist_boxes(Pw, tz, tbox, place)
            t_boxes = time.time()
            note["keepout"] = ko_note
            ok, scene_note = self.scene.set_obstacles(boxes)
            t_scene = time.time()
            age = time.time() - min(self.last["mask_source_time"], self.last["mask_target_time"])
            msg = ("[wrist %.1f s: wait %.1f, clouds %.1f, boxes %.1f, MoveIt %.1f] " % (t_scene - t0, t_wait, t_clouds - t0 - t_wait, t_boxes - t_clouds, t_scene - t_boxes)) + "%s; %s; %s; masks from the look %.0f s ago; %d wrist cloud(s), table z %+.1f mm" % (
                W.describe(boxes, note), ko_note, scene_note, age, len(parts), 1000 * tz)
            self._save(boxes, note, tbox, place, tz, msg)
            return ok, msg

        def _save(self, boxes, note, tbox, place, tz, msg):
            try:
                d = os.path.expanduser("~/p5_runs/" + time.strftime("%Y%m%d_%H%M%S"))
                os.makedirs(d, exist_ok=True)
                json.dump({"version": VERSION, "obstacles_version": W.P4_WRIST_OBSTACLES_VERSION, "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                           "boxes": boxes, "note": note, "target_box": [list(map(float, tbox[0])), list(map(float, tbox[1]))],
                           "place_xyz": None if place is None else list(map(float, place)), "table_z": tz, "message": msg,
                           "wrist_offset": self.wrist_off, "top_offset": self.top_off}, open(os.path.join(d, "wrist_obstacles.json"), "w"), indent=1)
                latest = os.path.expanduser("~/p5_runs/latest")
                if os.path.islink(latest):
                    os.remove(latest)
                if not os.path.exists(latest):
                    os.symlink(d, latest)
            except Exception as e:
                self.get_logger().warn("could not save the run: %s" % e)

    rclpy.init()
    node = Fuse()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
