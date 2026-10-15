#!/usr/bin/env python3
"""
Bridges ROS2 topics to the Contact-GraspNet inference script running in the
`contact_graspnet` conda env. Consumes the depth image, camera intrinsics,
and the segmentation mask already produced by gsam2_ros, and publishes
resulting 6-DoF grasp poses.

Runs in the normal ROS2 system Python - never imports torch directly.

Exposes:
    /contact_graspnet/detect_grasps (std_srvs/Trigger)

Publishes:
    /contact_graspnet/grasps (geometry_msgs/PoseArray, in the camera's
    optical frame - a separate TF transform from hand-eye calibration is
    still needed to bring these into base_link before handing to MoveIt)
"""

import json
import os
import subprocess
import tempfile
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

from contact_graspnet_ros.recentre import object_top_middle, recentre_grasps
from contact_graspnet_ros.worker_client import SpoolWorker, WorkerError


class ContactGraspNetNode(Node):
    def __init__(self):
        super().__init__("contact_graspnet_node")

        self.declare_parameter("depth_topic", "/topcam/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/topcam/depth/camera_info")
        self.declare_parameter("mask_topic", "/gsam2/mask_source")
        # Frame the published grasp poses are labelled with. "" (default) = AUTO: use the frame_id of the depth camera_info message, i.e. the frame the
        # depth data really lives in. With depth_registration the depth is aligned to the COLOUR image, so a fixed "topcam_depth_optical_frame" label
        # shifted every grasp by the gap between the two sensors. Set a name here only to force a frame.
        self.declare_parameter("camera_frame", "")
        self._camera_frame_info = None
        self._frame_logged = None
        self.declare_parameter("conda_env_name", "contact_graspnet")
        self.declare_parameter(
            "inference_script",
            os.path.expanduser("~/contact_graspnet_ros_scripts/contact_graspnet_inference.py"),
        )
        self.declare_parameter(
            "ckpt_dir",
            os.path.expanduser("~/contact_graspnet_pytorch/checkpoints/contact_graspnet"),
        )
        self.declare_parameter("depth_scale_to_meters", 0.001)  # set to 1.0 if depth already in meters
        # Persistent worker: builds the network and loads model.pt ONCE instead of on every call. use_worker=false (live, with
        # `ros2 param set /contact_graspnet_node use_worker false`) gives the old one-process-per-call behaviour.
        self.declare_parameter("use_worker", True)
        self.declare_parameter(
            "worker_script",
            os.path.expanduser("~/contact_graspnet_ros_scripts/contact_graspnet_worker.py"),
        )
        self.declare_parameter("worker_timeout_sec", 120.0)
        # forward_passes is fixed when the worker starts (it sets the network batch size): changing it needs a node restart.
        self.declare_parameter("forward_passes", 1)
        # Worker only: when a prediction comes back with 0 grasps, predict again up to this many more times (cheap: model already loaded).
        self.declare_parameter("retries_if_empty", 2)
        # Optional re-centring (off by default). For a small box Contact-GraspNet often proposes narrow "pinch the rim" grasps whose closing centre is at
        # the cube's EDGE. With recentre_on_object every grasp keeps its orientation but its closing centre is moved onto the middle of the object's top
        # face, recentre_depth_below_top_m below the top surface; grasps whose centre is farther than recentre_max_offset_m sideways from that middle are
        # dropped. Read on every call, no restart needed:  ros2 param set /contact_graspnet_node recentre_on_object true
        self.declare_parameter("recentre_on_object", False)
        self.declare_parameter("recentre_depth_below_top_m", 0.02)
        self.declare_parameter("recentre_max_offset_m", 0.05)

        self.bridge = CvBridge()
        self._latest_depth = None
        self._latest_mask = None
        self._camera_K = None

        depth_topic = self.get_parameter("depth_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        mask_topic = self.get_parameter("mask_topic").value

        self.create_subscription(Image, depth_topic, self._depth_cb, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(Image, mask_topic, self._mask_cb, 10)

        self.create_service(Trigger, "contact_graspnet/detect_grasps", self._detect_cb)
        self.grasps_pub = self.create_publisher(PoseArray, "/contact_graspnet/grasps", 10)

        self._worker = None
        if self.get_parameter("use_worker").value:
            worker_script = os.path.expanduser(self.get_parameter("worker_script").value)
            if os.path.exists(worker_script):
                self._worker = SpoolWorker(
                    ["conda", "run", "-n", self.get_parameter("conda_env_name").value, "--no-capture-output",
                     "python", "-u", worker_script,
                     "--ckpt_dir", os.path.expanduser(self.get_parameter("ckpt_dir").value),
                     "--forward_passes", str(self.get_parameter("forward_passes").value)],
                    log=lambda m: self.get_logger().info(m),
                )
                threading.Thread(target=self._warm_worker, daemon=True).start()     # load the model in the background right away
            else:
                self.get_logger().warn(f"use_worker is on but {worker_script} does not exist - using one process per call")

        self.get_logger().info(
            f"Ready. Waiting on depth='{depth_topic}', camera_info='{camera_info_topic}', "
            f"mask='{mask_topic}'. Call /contact_graspnet/detect_grasps (std_srvs/Trigger) "
            f"once a segmentation mask has been produced by gsam2_ros."
        )

    def _depth_cb(self, msg):
        # Common depth encodings: 16UC1 (millimeters) or 32FC1 (meters)
        self._latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def _camera_info_cb(self, msg):
        self._camera_K = np.array(msg.k).reshape(3, 3)
        self._camera_frame_info = msg.header.frame_id or None

    def _grasp_frame(self):
        """frame to label the grasps with: explicit parameter, else the depth camera_info frame, else the old default"""
        frame = self.get_parameter("camera_frame").value or self._camera_frame_info or "topcam_depth_optical_frame"
        if frame != self._frame_logged:
            self.get_logger().info(f"grasp poses are labelled with frame '{frame}' "
                                   f"({'set by parameter' if self.get_parameter('camera_frame').value else 'taken from the depth camera_info'})")
            self._frame_logged = frame
        return frame

    def _mask_cb(self, msg):
        self._latest_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

    def _recentre(self, result_data, depth_m, segmap, K):
        top = object_top_middle(depth_m, segmap > 0, K)
        if top is None:
            self.get_logger().warn("recentre_on_object: could not find the object's top face - grasps left as they are")
            return result_data
        middle, n_top, ext = top
        below = float(self.get_parameter("recentre_depth_below_top_m").value)
        max_off = float(self.get_parameter("recentre_max_offset_m").value)
        grasps, dropped = recentre_grasps(result_data["grasps"], middle, below, max_off)
        self.get_logger().info(
            f"recentred {len(grasps)} grasp(s) onto the top-face middle ({n_top} points, {100 * ext[0]:.1f} x {100 * ext[1]:.1f} cm), "
            f"{100 * below:.1f} cm below the top; dropped {dropped} whose closing centre was more than {100 * max_off:.1f} cm sideways from it"
        )
        if not grasps:
            return {"detected": False, "num_grasps": 0, "grasps": [],
                    "error": f"No grasps left after re-centring: none closed within {100 * max_off:.0f} cm of the object's middle"}
        out = dict(result_data)
        out["grasps"] = grasps
        out["num_grasps"] = len(grasps)
        return out

    def _warm_worker(self):
        try:
            self._worker.start()
        except WorkerError as e:
            self.get_logger().error(f"worker start-up failed: {e} (calls fall back to one process per call)")

    def _run_inference(self, depth_path, segmap_path, output_path, K, K_flat_str, conda_env_name, inference_script, ckpt_dir):
        """Returns (result_data, None) or (None, error_message). Persistent worker first; any worker problem falls back to the old
        one-process-per-call script (whose behaviour and messages are unchanged)."""
        t0 = time.time()
        if self.get_parameter("use_worker").value and self._worker is not None:
            try:
                reply = self._worker.request(
                    dict(cmd="infer", depth_npy=depth_path, segmap_npy=segmap_path,
                         K=[float(K[0, 0]), 0.0, float(K[0, 2]), 0.0, float(K[1, 1]), float(K[1, 2]), 0.0, 0.0, 1.0],
                         z_range=[0.2, 1.8], local_regions=True, filter_grasps=True, skip_border_objects=False,
                         attempts=1 + max(0, int(self.get_parameter("retries_if_empty").value))),
                    timeout=self.get_parameter("worker_timeout_sec").value,
                )
                if reply.get("ok"):
                    self.get_logger().info(
                        f"Contact-GraspNet took {time.time() - t0:.1f} s (persistent worker; {reply.get('seconds')} s inside it; "
                        f"prediction attempts used: {reply.get('attempts')})"
                    )
                    return reply["result"], None
                self.get_logger().error("the worker reported an error, falling back: " + str(reply.get("error", "?"))[-400:])
            except WorkerError as e:
                self.get_logger().error(f"persistent worker failed ({e}); falling back to one process per call")
        return self._run_inference_subprocess(
            depth_path, segmap_path, output_path, K_flat_str, conda_env_name, inference_script, ckpt_dir, t0
        )

    def _run_inference_subprocess(self, depth_path, segmap_path, output_path, K_flat_str, conda_env_name, inference_script, ckpt_dir, t0):
        cmd = [
            "conda", "run", "-n", conda_env_name, "--no-capture-output",
            "python", inference_script,
            "--depth_npy", depth_path,
            "--segmap_npy", segmap_path,
            "--K", K_flat_str,
            "--ckpt_dir", ckpt_dir,
            "--output_json", output_path,
            "--forward_passes", str(self.get_parameter("forward_passes").value),
        ]

        self.get_logger().info("Running Contact-GraspNet inference...")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=180)

        # Always surface stdout - it contains the model's real diagnostic
        # output (point cloud shape, grasps generated per object, etc.)
        # which is otherwise silently swallowed even on "success" (exit 0,
        # zero grasps found is not a crash).
        if result.stdout.strip():
            self.get_logger().info(f"Inference stdout:\n{result.stdout}")

        if result.returncode != 0:
            self.get_logger().error(f"Inference failed:\n{result.stderr}")
            return None, f"Inference script failed: {result.stderr[-500:]}"

        if not os.path.exists(output_path):
            return None, "Inference ran but produced no output json"

        with open(output_path) as f:
            result_data = json.load(f)
        self.get_logger().info(f"Contact-GraspNet took {time.time() - t0:.1f} s (one process per call)")
        return result_data, None

    def _detect_cb(self, request, response):
        if self._latest_depth is None:
            response.success = False
            response.message = "No depth frame received yet"
            return response
        if self._camera_K is None:
            response.success = False
            response.message = "No camera_info received yet"
            return response
        if self._latest_mask is None:
            response.success = False
            response.message = "No segmentation mask yet - call gsam2/segment first"
            return response

        conda_env_name = self.get_parameter("conda_env_name").value
        inference_script = os.path.expanduser(self.get_parameter("inference_script").value)
        ckpt_dir = os.path.expanduser(self.get_parameter("ckpt_dir").value)
        depth_scale = self.get_parameter("depth_scale_to_meters").value

        depth_m = self._latest_depth.astype(np.float64) * depth_scale
        mask = self._latest_mask
        if mask.shape != depth_m.shape:
            self.get_logger().warn(
                f"Mask shape {mask.shape} != depth shape {depth_m.shape}. "
                f"Resizing mask to match - this assumes matching aspect ratio/FOV; "
                f"for correct results, enable depth_registration:=true on the camera "
                f"launch so depth and color share the same resolution and alignment."
            )
            import cv2
            mask = cv2.resize(
                mask, (depth_m.shape[1], depth_m.shape[0]), interpolation=cv2.INTER_NEAREST
            )
        segmap = (mask > 0).astype(np.int32)

        K = self._camera_K
        K_flat_str = str([K[0, 0], 0, K[0, 2], 0, K[1, 1], K[1, 2], 0, 0, 1])

        with tempfile.TemporaryDirectory() as tmpdir:
            depth_path = os.path.join(tmpdir, "depth.npy")
            segmap_path = os.path.join(tmpdir, "segmap.npy")
            output_path = os.path.join(tmpdir, "grasps.json")
            np.save(depth_path, depth_m)
            np.save("/tmp/last_depth.npy", depth_m)
            np.save(segmap_path, segmap)
            np.save("/tmp/last_segmap.npy", segmap)

            result_data, error = self._run_inference(
                depth_path, segmap_path, output_path, K, K_flat_str, conda_env_name, inference_script, ckpt_dir
            )
            if error:
                response.success = False
                response.message = error
                return response

        if result_data.get("detected", False) and self.get_parameter("recentre_on_object").value:
            result_data = self._recentre(result_data, depth_m, segmap, K)

        if not result_data.get("detected", False):
            response.success = False
            response.message = result_data.get("error", "No grasps found")
            return response

        pose_array = PoseArray()
        pose_array.header.frame_id = self._grasp_frame()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        for grasp in result_data["grasps"]:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = grasp["position"]
            qx, qy, qz, qw = grasp["quaternion"]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
            pose_array.poses.append(pose)
        self.grasps_pub.publish(pose_array)

        best = result_data["grasps"][0]
        response.success = True
        response.message = (
            f"Found {result_data['num_grasps']} grasp(s). "
            f"Best score={best['score']:.3f} at position={best['position']}"
        )
        return response


def main():
    rclpy.init()
    node = ContactGraspNetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if getattr(node, "_worker", None) is not None:
            node._worker.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
