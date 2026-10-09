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

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_srvs.srv import Trigger
from cv_bridge import CvBridge


class ContactGraspNetNode(Node):
    def __init__(self):
        super().__init__("contact_graspnet_node")

        self.declare_parameter("depth_topic", "/topcam/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/topcam/depth/camera_info")
        self.declare_parameter("mask_topic", "/gsam2/mask_source")
        self.declare_parameter("camera_frame", "topcam_depth_optical_frame")
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

    def _mask_cb(self, msg):
        self._latest_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

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

            cmd = [
                "conda", "run", "-n", conda_env_name, "--no-capture-output",
                "python", inference_script,
                "--depth_npy", depth_path,
                "--segmap_npy", segmap_path,
                "--K", K_flat_str,
                "--ckpt_dir", ckpt_dir,
                "--output_json", output_path,
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
                response.success = False
                response.message = f"Inference script failed: {result.stderr[-500:]}"
                return response

            if not os.path.exists(output_path):
                response.success = False
                response.message = "Inference ran but produced no output json"
                return response

            with open(output_path) as f:
                result_data = json.load(f)

        if not result_data.get("detected", False):
            response.success = False
            response.message = result_data.get("error", "No grasps found")
            return response

        pose_array = PoseArray()
        pose_array.header.frame_id = self.get_parameter("camera_frame").value
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
