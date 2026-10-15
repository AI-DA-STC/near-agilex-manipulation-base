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

from target_pose_ros.plate_geometry import extract_target, describe


class TargetPoseNode(Node):
    def __init__(self):
        super().__init__("target_pose_node")

        self.declare_parameter("mask_topic", "/gsam2/mask_target")
        self.declare_parameter("depth_topic", "/topcam/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/topcam/depth/camera_info")
        # Frame the plate pose is labelled with. "" (default) = AUTO: the frame_id of the depth camera_info message, i.e. the frame the depth data really
        # lives in. With depth_registration the depth is aligned to the COLOUR image, so a fixed "topcam_depth_optical_frame" label put the plate about
        # 3 cm sideways of where it is (the gap between the two sensors). Set a name here only to force a frame.
        self.declare_parameter("camera_frame", "")
        self._camera_frame_info = None
        self._frame_logged = None
        self.declare_parameter("depth_scale_to_meters", 0.001)  # 0.001 for 16UC1 mm, 1.0 if already meters
        self.declare_parameter(
            "z_mode", "floor",
            # "top_surface": z = closest point to camera among target pixels
            #                (the highest physical point, since a top-down
            #                camera means "closer" = "higher") - use this
            #                for placing something ON TOP of the target.
            # "centroid": z = mean depth of all target pixels - use this
            #                if you want the pose at the target's average
            #                height instead (e.g. placing beside/into it).
            # "floor":       (NEW, default) z = MEDIAN depth of the pixels near the
            #                centre = the plate's inner surface, where an object
            #                will actually rest. Immune to the rim and to a few
            #                stray pixels (the old "top_surface" took the single
            #                closest pixel).
        )
        # How the plate's centre is found (see plate_geometry.py): "hull" (default), "mean" (old), "circle", "bbox".
        self.declare_parameter("center_mode", "hull")
        self.declare_parameter("center_fraction", 0.5)   # "floor" uses the pixels within this fraction of the plate radius around the centre

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
        self._camera_frame_info = msg.header.frame_id or None

    def _pose_frame(self):
        """frame to label the plate pose with: explicit parameter, else the depth camera_info frame, else the old default"""
        frame = self.get_parameter("camera_frame").value or self._camera_frame_info or "topcam_depth_optical_frame"
        if frame != self._frame_logged:
            self.get_logger().info(f"plate pose is labelled with frame '{frame}' "
                                   f"({'set by parameter' if self.get_parameter('camera_frame').value else 'taken from the depth camera_info'})")
            self._frame_logged = frame
        return frame

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

        try:
            r = extract_target(
                mask, depth.astype(np.float64) * depth_scale, K,
                z_mode=z_mode,
                center_mode=self.get_parameter("center_mode").value,
                center_fraction=self.get_parameter("center_fraction").value,
            )
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response

        pose = PoseStamped()
        pose.header.frame_id = self._pose_frame()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = r["x"]
        pose.pose.position.y = r["y"]
        pose.pose.position.z = r["z"]
        # No meaningful orientation is derived from the target's geometry
        # here (e.g. a round plate has no natural yaw) - identity
        # orientation in the camera frame. Downstream (task planning)
        # should apply whatever fixed placing orientation the task needs,
        # not rely on this pose's orientation field.
        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)

        response.success = True
        response.message = f"Target pose: {describe(r)} in '{pose.header.frame_id}'"
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
