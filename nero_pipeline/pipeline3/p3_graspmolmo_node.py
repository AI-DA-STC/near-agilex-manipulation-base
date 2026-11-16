#!/usr/bin/env python3
"""
PIPELINE 3 (GraspMolmo + Contact-GraspNet): the replacement for gsam2_ros in pipeline 1's chain. Run with the system python (ROS sourced,
conda deactivated); start_pipeline3.sh starts it.

Two services, called in this order by run_pick_and_place3.sh:

/graspmolmo/segment (std_srvs/Trigger)  - about 25 s
  1. removes the previous run's obstacle boxes from MoveIt's planning scene (they are rebuilt every run)
  2. takes the latest colour + registered depth frame (refuses unregistered 640x400 depth, docs/11 section 2.5)
  3. asks GraspMolmo the GRASP question ("Point to the grasp that would accomplish the following task: pick up the <source_prompt>")
  4. publishes the object region under the grasp point on /graspmolmo/mask_source -> Contact-GraspNet can start right away
  5. starts, in the background: the PLACE question ("Point to the center of the <target_prompt>.") and the OBSTACLE boxes (p3_obstacles.py)
     - these run while Contact-GraspNet works
/graspmolmo/place (std_srvs/Trigger)    - waits for step 5 if it is not finished yet (usually it is)
  (v4) step 4 also publishes the object's measured HEIGHT on /graspmolmo/object_height (point.x, m; -1 = unknown) - the tip lift uses it for
     the fingertip clearance: 40% of the object's height, between the stored minimum (4 mm) and maximum (1.5 cm), see p3_tablez.py
  6. publishes the plate disc on /graspmolmo/mask_target (-> target_pose_ros) and the obstacle boxes into MoveIt's planning scene
     (base_link boxes "p3_obst_<n>", visible in RViz's MotionPlanning display), and saves the run in ~/p3_runs/<time>/ (and ~/p3_runs/latest)

P3_OBSTACLES=0 (when starting start_pipeline3.sh) switches the obstacle boxes off.
v5: the boxes are changed through MoveIt's /apply_planning_scene service and READ BACK (/get_planning_scene): every run removes every
p3_obst_* box MoveIt really holds (stale ones too) and confirms the result. Only if the service does not answer, the old /planning_scene
message is sent instead - and the run says the boxes are NOT confirmed.
"""
import json
import os
import sys
import tempfile
import threading
import time

import cv2
import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p3_obstacles  # noqa: E402
import p3_scene  # noqa: E402
import p3_tablez  # noqa: E402
from p3_geometry import annotate, lift, object_mask, place_mask  # noqa: E402
from p3_jobs import BackgroundJob, JobTimeout  # noqa: E402
from worker_client import Worker, WorkerError  # noqa: E402

GRASP_PREFIX = "Point to the grasp that would accomplish the following task: "
P3_NODE_VERSION = "p3-node-v8"   # v6: the obstacle step has a hard time limit and logs each stage; it can no longer stall the place step
RUNS_DIR = os.path.expanduser("~/p3_runs")
OBSTACLE_ID = "p3_obst_%d"
NOT_ROBOT_FRAMES = ("topcam", "camera", "base_link", "world", "map", "odom")   # TF frames that are not parts of the arm


def quat_to_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class GraspMolmoNode(Node):
    def __init__(self):
        super().__init__("p3_graspmolmo_node")
        P = self.declare_parameter
        P("color_topic", "/topcam/color/image_raw")
        P("depth_topic", "/topcam/depth/image_raw")
        P("camera_info_topic", "/topcam/depth/camera_info")
        P("source_prompt", "green cube")
        P("target_prompt", "plate")
        P("conda_env_name", "graspmolmo")
        P("worker_script", os.path.join(os.path.dirname(os.path.abspath(__file__)), "p3_graspmolmo_worker.py"))
        P("dtype", os.environ.get("P3_GM_DTYPE", "bf16"))      # bf16 (default) or fp32
        P("worker_extra_args", "")                             # "--fake" for tests without the model
        P("worker_timeout_sec", 180.0)
        P("max_new_tokens", 256)
        P("depth_scale_to_meters", 0.001)
        P("object_radius_px", 45)
        P("above_table_m", 0.008)
        P("place_disc_px", 6)
        P("obstacles", os.environ.get("P3_OBSTACLES", "1") != "0")
        self.g = lambda n: self.get_parameter(n).value

        self.bridge = CvBridge()
        self.latest = {}
        self.create_subscription(Image, self.g("color_topic"), lambda m: self.latest.__setitem__("color", m), qos_profile_sensor_data)
        self.create_subscription(Image, self.g("depth_topic"), lambda m: self.latest.__setitem__("depth", m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.g("camera_info_topic"), lambda m: self.latest.__setitem__("info", m), qos_profile_sensor_data)
        self.pub_src = self.create_publisher(Image, "/graspmolmo/mask_source", 10)
        self.pub_tgt = self.create_publisher(Image, "/graspmolmo/mask_target", 10)
        self.pub_ann = self.create_publisher(Image, "/graspmolmo/annotated", 10)
        self.pub_pt = self.create_publisher(PointStamped, "/graspmolmo/grasp_point", 10)
        self.pub_scene = self.create_publisher(PlanningScene, "/planning_scene", 10)   # move_group applies diffs published here
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_height = self.create_publisher(PointStamped, "/graspmolmo/object_height", latched)   # v4: for the tip lift
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.create_service(Trigger, "graspmolmo/segment", self._segment_cb)
        self.create_service(Trigger, "graspmolmo/place", self._place_cb)

        self.n_obstacles = 0          # how many p3_obst_<n> boxes this node last asked for
        self.scene = p3_scene.AsyncScene(timeout=8.0)   # v5: confirmed changes through MoveIt's services (own executor thread)
        self.scene_note = ""
        self.job = None               # the background place question + obstacles of the current run
        self.ctx = None               # what /graspmolmo/place needs from /graspmolmo/segment

        extra = [a for a in str(self.g("worker_extra_args")).split() if a] + ["--dtype", str(self.g("dtype"))]
        self.worker = Worker(["conda", "run", "-n", self.g("conda_env_name"), "--no-capture-output",
                              "python", "-u", os.path.expanduser(self.g("worker_script"))] + extra,
                             log=lambda m: self.get_logger().info(m), startup_timeout=400.0)
        threading.Thread(target=self._warm, daemon=True).start()   # load GraspMolmo in the background right away
        tz, tzs = p3_tablez.table_z()
        self.get_logger().info(f"{P3_NODE_VERSION} ready. source_prompt='{self.g('source_prompt')}' target_prompt='{self.g('target_prompt')}'; "
                               f"obstacles {'ON' if self.g('obstacles') else 'OFF'}; table at {1000 * tz:+.1f} mm ({tzs}). "
                               "GraspMolmo takes about a minute to load.")

    # ------------------------------------------------------------------ services
    def _warm(self):
        try:
            self.worker.start()
        except WorkerError as e:
            self.get_logger().error(f"GraspMolmo worker start-up failed: {e}")

    def _answer(self, res, fn):
        try:
            res.success, res.message = fn()
        except WorkerError as e:
            res.success, res.message = False, f"GraspMolmo worker problem: {e}"
        except Exception as e:  # never let a service die silently
            res.success, res.message = False, f"{type(e).__name__}: {e}"
        # v7: separate lines for info and error - rclpy raises "Logger severity cannot be changed between calls" when ONE line logs at two
        # levels, and that exception killed the whole node the first time a step failed after one had succeeded
        if res.success:
            self.get_logger().info(res.message)
        else:
            self.get_logger().error(res.message)
        return res

    def _segment_cb(self, req, res):
        return self._answer(res, self._segment)

    def _place_cb(self, req, res):
        return self._answer(res, self._place)

    # ------------------------------------------------------------------ step 1: the grasp
    def _segment(self):
        if self.job is not None and not self.job.done():          # a previous run's place question still busy: let it finish (one worker)
            try:
                self.job.wait(float(self.g("worker_timeout_sec")))
            except Exception:
                pass
        self.job, self.ctx = None, None
        self._set_obstacles([])                                    # reset: EVERY p3_obst_* box MoveIt holds goes; this run's come with /graspmolmo/place
        reset_note = self.scene_note

        missing = [k for k in ("color", "depth", "info") if k not in self.latest]
        if missing:
            return False, f"no camera data yet ({', '.join(missing)}) - is the camera running?"
        cmsg, dmsg, info = self.latest["color"], self.latest["depth"], self.latest["info"]
        bgr = self.bridge.imgmsg_to_cv2(cmsg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(dmsg, desired_encoding="passthrough")
        depth_m = (np.nan_to_num(depth.astype(np.float32)) if dmsg.encoding == "32FC1"
                   else depth.astype(np.float32) * float(self.g("depth_scale_to_meters")))
        if depth_m.shape[:2] != bgr.shape[:2]:
            return False, (f"REFUSED: depth {depth_m.shape[1]}x{depth_m.shape[0]} != colour {bgr.shape[1]}x{bgr.shape[0]}: "
                           "camera started without depth registration; replug its USB cable and restart")
        K = np.array(info.k, dtype=float).reshape(3, 3)
        hdr = dmsg.header
        hdr.frame_id = info.header.frame_id or hdr.frame_id       # registered depth lives in the colour optical frame
        src, tgt = self.g("source_prompt"), self.g("target_prompt")

        img_path = os.path.join(tempfile.mkdtemp(prefix="p3_req_"), "rgb.png")   # kept for the background place question
        cv2.imwrite(img_path, bgr)
        g = self._ask(img_path, "grasp", f"{GRASP_PREFIX}pick up the {src}")
        if g["uv"] is None:
            return False, f"GraspMolmo gave no grasp point for '{src}'. Its answer: {g['answer'][:250]!r}"
        guv = g["uv"]
        omask, omsg = object_mask(depth_m, guv, int(self.g("object_radius_px")), above_table_m=float(self.g("above_table_m")))
        if omask is None:
            return False, f"grasp point ({guv[0]:.0f}, {guv[1]:.0f}): {omsg}"

        msg = self.bridge.cv2_to_imgmsg(omask, encoding="mono8"); msg.header = hdr
        self.pub_src.publish(msg)                                  # -> Contact-GraspNet can start now
        xyz = lift(guv, depth_m, K)
        if xyz is not None:
            pt = PointStamped(); pt.header = hdr
            pt.point.x, pt.point.y, pt.point.z = xyz
            self.pub_pt.publish(pt)

        cam_in_base, robot_points = self._camera_and_robot(hdr.frame_id)
        height = None
        if cam_in_base is not None:                                # v4: the object's height above the real table, for the fingertip clearance
            height = p3_obstacles.object_height(depth_m, K, cam_in_base[0], cam_in_base[1], omask > 0, p3_tablez.table_z()[0])
        hmsg = PointStamped(); hmsg.header.frame_id = "base_link"; hmsg.header.stamp = self.get_clock().now().to_msg()
        hmsg.point.x = float(height) if height is not None else -1.0
        self.pub_height.publish(hmsg)
        clearance, clearance_why = p3_tablez.clearance_for(height)
        self.ctx = {"bgr": bgr, "depth": depth_m, "K": K, "hdr": hdr, "src": src, "tgt": tgt, "guv": guv, "omask": omask, "omsg": omsg,
                    "grasp": g, "grasp_xyz_cam": xyz, "cam_in_base": cam_in_base, "robot_points": robot_points,
                    "object_height": height, "clearance": clearance, "clearance_why": clearance_why}
        self.job = BackgroundJob(self._place_and_obstacles, img_path, self.ctx)
        return True, (f"grasp px ({guv[0]:.0f},{guv[1]:.0f}) {omsg}; GraspMolmo {g['seconds']:.1f} s; fingertip clearance {clearance_why}; "
                      f"obstacle reset: {reset_note}; "
                      "the place question and the obstacles now run in the background (collected by /graspmolmo/place)")

    # ------------------------------------------------------------------ step 2 (background): the place point and the obstacles
    def _place_and_obstacles(self, img_path, c):
        try:
            return self._place_and_obstacles_inner(img_path, c)
        finally:
            for f in (img_path, os.path.dirname(img_path)):      # the temporary image of this run
                try:
                    os.remove(f) if os.path.isfile(f) else os.rmdir(f)
                except OSError:
                    pass

    def _place_and_obstacles_inner(self, img_path, c):
        t0 = time.time()
        p = self._ask(img_path, "place", f"Point to the center of the {c['tgt']}.")
        if p["uv"] is None:
            raise RuntimeError(f"GraspMolmo gave no place point for '{c['tgt']}'. Its answer: {p['answer'][:250]!r}")
        pmask, pmsg = place_mask(c["depth"], p["uv"], int(self.g("place_disc_px")))
        if pmask is None:
            raise RuntimeError(pmsg)
        out = {"place": p, "pmask": pmask, "pmsg": pmsg, "boxes": [], "obstacles_note": "obstacles OFF (P3_OBSTACLES=0)"}
        if self.g("obstacles"):
            if c["cam_in_base"] is None:
                out["obstacles_note"] = "obstacles SKIPPED: no camera pose in TF (is the hand-eye publisher running?)"
            else:
                R, t = c["cam_in_base"]
                place_cam = lift(p["uv"], c["depth"], c["K"])
                place_base = None if place_cam is None else (R @ np.array(place_cam) + t).tolist()
                tz = p3_tablez.table_z()[0]
                t_ob = time.time()
                self.get_logger().info("place answer received; computing the obstacles ...")
                oj = BackgroundJob(p3_obstacles.obstacle_boxes, c["depth"], c["K"], R, t, tz, target_mask=c["omask"] > 0,
                                   place_xyz=place_base, robot_points=c["robot_points"])
                try:
                    out["boxes"] = oj.wait(float(os.environ.get("P3_OBSTACLE_TIMEOUT_S", "5")))
                    out["obstacles_note"] = p3_obstacles.describe(out["boxes"]) + " (computed in %.1f s)" % (time.time() - t_ob)
                except JobTimeout:
                    out["obstacles_note"] = "obstacles SKIPPED: computing them took longer than %s s (the run continues without them)" % os.environ.get("P3_OBSTACLE_TIMEOUT_S", "5")
                except Exception as e:
                    out["obstacles_note"] = "obstacles SKIPPED: %s: %s (the run continues without them)" % (type(e).__name__, e)
                self.get_logger().info(out["obstacles_note"])
        out["seconds"] = time.time() - t0
        return out

    # ------------------------------------------------------------------ step 3: publish the place point and the obstacles
    def _place(self):
        if self.job is None or self.ctx is None:
            return False, "call /graspmolmo/segment first (it asks the grasp question and starts the place question)"
        t_wait = time.time()
        try:
            r = self.job.wait(float(self.g("worker_timeout_sec")))
        except JobTimeout as e:
            return False, f"the place question is {e}"
        except Exception as e:                                     # the background step failed: report it, publish nothing
            return False, f"place step failed: {e}"
        waited = time.time() - t_wait
        c = self.ctx
        msg = self.bridge.cv2_to_imgmsg(r["pmask"], encoding="mono8"); msg.header = c["hdr"]
        self.pub_tgt.publish(msg)                                  # -> target_pose_ros
        self.get_logger().info("publishing the place disc; sending %d obstacle box(es) to MoveIt ..." % len(r["boxes"]))
        self._set_obstacles(r["boxes"])                            # -> MoveIt's planning scene (confirmed)
        self.get_logger().info("obstacles: " + self.scene_note)
        ann = annotate(c["bgr"], c["omask"], c["guv"], r["place"]["uv"])
        amsg = self.bridge.cv2_to_imgmsg(ann, encoding="bgr8"); amsg.header = c["hdr"]
        self.pub_ann.publish(amsg)
        saved = self._save_run(c["bgr"], ann, {
            "version": P3_NODE_VERSION, "time": time.strftime("%Y-%m-%d %H:%M:%S"), "source_prompt": c["src"], "target_prompt": c["tgt"],
            "dtype": str(self.g("dtype")), "grasp_px": c["guv"], "place_px": r["place"]["uv"], "grasp_point_cam": c["grasp_xyz_cam"],
            "object_region": c["omsg"], "place_region": r["pmsg"], "grasp_answer": c["grasp"]["answer"], "place_answer": r["place"]["answer"],
            "grasp_seconds": c["grasp"]["seconds"], "place_seconds": r["place"]["seconds"], "frame": c["hdr"].frame_id,
            "K": c["K"].tolist(), "image_size_wh": [int(c["bgr"].shape[1]), int(c["bgr"].shape[0])],
            "obstacles": r["boxes"], "obstacles_note": r["obstacles_note"], "scene_note": self.scene_note,
            "object_height_m": c["object_height"], "fingertip_clearance_m": c["clearance"], "fingertip_clearance_why": c["clearance_why"]})
        return True, (f"place px ({r['place']['uv'][0]:.0f},{r['place']['uv'][1]:.0f}) {r['pmsg']}; GraspMolmo {r['place']['seconds']:.1f} s "
                      f"(ran alongside Contact-GraspNet; waited {waited:.1f} s for it here); {r['obstacles_note']}; {self.scene_note}; picture {saved}/annotated.png")

    # ------------------------------------------------------------------ helpers
    def _ask(self, img_path, name, text):
        reply = self.worker.request({"cmd": "point", "image": img_path, "max_new_tokens": int(self.g("max_new_tokens")),
                                     "queries": [{"name": name, "text": text}]}, timeout=float(self.g("worker_timeout_sec")))
        if not reply.get("ok"):
            raise WorkerError(f"GraspMolmo error: {reply.get('error')}")
        return reply["results"][0]

    def _camera_and_robot(self, camera_frame):
        """(camera pose in base_link as (R, t) or None, positions of the robot's own TF frames in base_link) - for the obstacle filter."""
        cam = None
        try:
            tr = self.tf_buf.lookup_transform("base_link", camera_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
            q, p = tr.transform.rotation, tr.transform.translation
            cam = (quat_to_matrix(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z]))
        except Exception as e:
            self.get_logger().warn(f"no camera pose in TF ({e}): obstacles will be skipped this run")
        points = []
        try:
            frames = yaml.safe_load(self.tf_buf.all_frames_as_yaml()) or {}
        except Exception:
            frames = {}
        for f in frames:
            if any(s in f for s in NOT_ROBOT_FRAMES):
                continue
            try:
                p = self.tf_buf.lookup_transform("base_link", f, rclpy.time.Time()).transform.translation
                points.append((p.x, p.y, p.z))
            except Exception:
                continue
        return cam, points

    def _set_obstacles(self, boxes):
        """v5: make MoveIt hold exactly these boxes, CONFIRMED through its services (see p3_scene.set_obstacles). Falls back to the plain
        /planning_scene message only if the services do not answer - then self.scene_note says the boxes are NOT confirmed."""
        try:
            ok, note = self.scene.set_obstacles(boxes)
            self.n_obstacles = len(boxes)
            self.scene_note = note
            if not ok:
                self.get_logger().error(note)
            return ok
        except Exception as e:
            self.scene_note = "MoveIt's scene services did not answer (%s): boxes sent as a plain message, NOT confirmed" % e
            self.get_logger().warn(self.scene_note)
            self._publish_obstacles(boxes)
            return False

    def _publish_obstacles(self, boxes):
        """The old way (v3/v4): one diff message on /planning_scene - not confirmed, so only a fallback."""
        ps = PlanningScene()
        ps.is_diff = True
        for k in range(len(boxes), self.n_obstacles):
            co = CollisionObject()
            co.id, co.header.frame_id, co.operation = OBSTACLE_ID % k, "base_link", CollisionObject.REMOVE
            ps.world.collision_objects.append(co)
        for k, b in enumerate(boxes):
            co = CollisionObject()
            co.id, co.header.frame_id, co.operation = OBSTACLE_ID % k, "base_link", CollisionObject.ADD
            co.pose.orientation.w = 1.0
            co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in b["size"]])]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = (float(v) for v in b["centre"])
            pose.orientation.w = 1.0
            co.primitive_poses = [pose]
            ps.world.collision_objects.append(co)
        if ps.world.collision_objects:
            self.pub_scene.publish(ps)
        self.n_obstacles = len(boxes)

    def _save_run(self, bgr, annotated, info):
        """Picture + answers of this run in ~/p3_runs/<time>/ (and ~/p3_runs/latest), so a run can be checked afterwards without RViz."""
        d = os.path.join(RUNS_DIR, time.strftime("%Y%m%d_%H%M%S"))
        try:
            os.makedirs(d, exist_ok=True)
            cv2.imwrite(os.path.join(d, "rgb.png"), bgr)
            cv2.imwrite(os.path.join(d, "annotated.png"), annotated)
            with open(os.path.join(d, "answers.json"), "w") as f:
                json.dump(info, f, indent=1, default=lambda v: v.tolist() if hasattr(v, "tolist") else str(v))   # numpy numbers and arrays -> plain values
            latest = os.path.join(RUNS_DIR, "latest")
            if os.path.islink(latest) or os.path.exists(latest):
                os.remove(latest)
            os.symlink(d, latest)
        except OSError as e:
            self.get_logger().warn(f"could not save the run to {d}: {e}")
        return d


def main():
    rclpy.init()
    node = GraspMolmoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.worker.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
