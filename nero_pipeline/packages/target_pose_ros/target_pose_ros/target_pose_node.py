#!/usr/bin/env python3
"""
Extracts a single placement pose from the target's 2D segmentation mask
(gsam2_ros's /gsam2/mask_target), using depth + camera intrinsics to lift
it into 3D - pure geometry, no neural network involved. This is the
"target pose extraction" stage from the architecture: the target (e.g. a
plate) is never grasped, so it never goes through Contact-GraspNet; its
pose is read directly off its own point cloud instead.

Because this stage has no ML dependency, it runs entirely in plain ROS 2
Python - no conda env, no subprocess bridging, unlike gsam2_ros and
contact_graspnet_ros.

Exposes:
    /target_pose/extract (std_srvs/Trigger)

Publishes:
    /target_pose/pose (geometry_msgs/PoseStamped, in the camera's own
    optical frame - same camera-frame limitation as contact_graspnet_ros:
    a separate TF transform via the eye_on_base calibration is still
    needed before this pose is usable by MoveIt)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from cv_bridge import CvBridge


class TargetPoseNode(Node):
    def __init__(self):
        super().__init__("target_pose_node")

        self.declare_parameter("mask_topic", "/gsam2/mask_target")
        self.declare_parameter("depth_topic", "/topcam/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/topcam/depth/camera_info")
        self.declare_parameter("camera_frame", "topcam_depth_optical_frame")
        self.declare_parameter("depth_scale_to_meters", 0.001)  # 0.001 for 16UC1 mm, 1.0 if already meters
        self.declare_parameter(
            "z_mode", "top_surface",
            # "top_surface": z = closest point to camera among target pixels
            #                (the highest physical point, since a top-down
            #                camera means "closer" = "higher") - use this
            #                for placing something ON TOP of the target.
            # "centroid": z = mean depth of all target pixels - use this
            #                if you want the pose at the target's average
            #                height instead (e.g. placing beside/into it).
        )

        self.bridge = CvBridge()
        self._latest_mask = None
        self._latest_depth = None
        self._camera_K = None

        self.create_subscription(
            Image, self.get_parameter("mask_topic").value, self._mask_cb, 10
        )
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value, self._depth_cb, 10
        )
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value, self._camera_info_cb, 10
        )

        self.create_service(Trigger, "target_pose/extract", self._extract_cb)
        self.pose_pub = self.create_publisher(PoseStamped, "/target_pose/pose", 10)

        self.get_logger().info(
            "Ready. Call /target_pose/extract (std_srvs/Trigger) once "
            "gsam2_ros has produced a target mask."
        )

    def _mask_cb(self, msg):
        self._latest_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

    def _depth_cb(self, msg):
        self._latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def _camera_info_cb(self, msg):
        self._camera_K = np.array(msg.k).reshape(3, 3)

    def _extract_cb(self, request, response):
        if self._latest_mask is None:
            response.success = False
            response.message = "No target mask yet - call gsam2/segment first"
            return response
        if self._latest_depth is None:
            response.success = False
            response.message = "No depth frame received yet"
            return response
        if self._camera_K is None:
            response.success = False
            response.message = "No camera_info received yet"
            return response

        mask = self._latest_mask
        depth = self._latest_depth
        K = self._camera_K
        depth_scale = self.get_parameter("depth_scale_to_meters").value
        z_mode = self.get_parameter("z_mode").value

        if mask.shape != depth.shape:
            response.success = False
            response.message = (
                f"Mask shape {mask.shape} != depth shape {depth.shape} - "
                f"enable depth_registration:=true on the camera launch"
            )
            return response

        ys, xs = np.where(mask > 0)
        if len(xs) == 0:
            response.success = False
            response.message = "Target mask is empty (no detected pixels)"
            return response

        depth_m = depth[ys, xs].astype(np.float64) * depth_scale
        valid = depth_m > 0  # drop zero/invalid depth readings
        if not np.any(valid):
            response.success = False
            response.message = "All target pixels have invalid (zero) depth"
            return response
        xs, ys, depth_m = xs[valid], ys[valid], depth_m[valid]

        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        # Back-project every target pixel to a 3D point in the camera's
        # optical frame (standard pinhole projection, inverted).
        X = (xs - cx) * depth_m / fx
        Y = (ys - cy) * depth_m / fy
        Z = depth_m

        position_x = float(np.mean(X))
        position_y = float(np.mean(Y))

        if z_mode == "top_surface":
            position_z = float(np.min(Z))  # closest to camera = highest physical point
        else:
            position_z = float(np.mean(Z))

        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter("camera_frame").value
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = position_x
        pose.pose.position.y = position_y
        pose.pose.position.z = position_z
        # No meaningful orientation is derived from the target's geometry
        # here (e.g. a round plate has no natural yaw) - identity
        # orientation in the camera frame. Downstream (task planning)
        # should apply whatever fixed placing orientation the task needs,
        # not rely on this pose's orientation field.
        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)

        response.success = True
        response.message = (
            f"Target pose: position=({position_x:.3f}, {position_y:.3f}, "
            f"{position_z:.3f}) in '{pose.header.frame_id}', "
            f"from {len(xs)} pixels, z_mode='{z_mode}'"
        )
        return response


def main():
    rclpy.init()
    node = TargetPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
