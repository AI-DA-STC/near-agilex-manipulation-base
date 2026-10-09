#!/usr/bin/env python3
"""
Transforms the grasp pose (from contact_graspnet_ros) and the target pose
(from target_pose_ros) out of the camera's optical frame and into
`base_link`, using the existing eye_on_base hand-eye calibration (see
nero-camera-calibration) - this is the missing link both of those repos'
READMEs flag: their poses are in the camera frame and not directly usable
by MoveIt.

Uses tf2_ros's Buffer.transform() directly rather than hand-rolled matrix
math - PoseStamped is natively registered with tf2_ros's transform
dispatch, so this is the standard, correct way to do this rather than
reimplementing quaternion/translation composition ourselves.

Requires the calibration's static transform to actually be broadcast at
runtime - i.e. easy_handeye2's publish node (or equivalent) must be
running, publishing base_link -> topcam_color_optical_frame (or wherever
the calibration solved to), AND the camera driver's own TF chain
(topcam_color_optical_frame <-> topcam_depth_optical_frame, since our
poses are published in the depth optical frame by contact_graspnet_ros /
target_pose_ros, while the calibration was done against the color optical
frame - the camera driver's own robot_state_publisher output normally
covers this link, but it's worth confirming rather than assuming).

Exposes:
    /pose_transform/transform_all (std_srvs/Trigger)

Publishes:
    /grasp_pose_transformed  (geometry_msgs/PoseStamped, frame_id=base_link)
    /target_pose_transformed (geometry_msgs/PoseStamped, frame_id=base_link)
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseArray, PoseStamped
from std_srvs.srv import Trigger

import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs  # noqa: F401 - import needed for its side effect: registers
                          # PoseStamped with tf2_ros's Buffer.transform() dispatch.

# transient_local: matches how /tf_static works - the publisher keeps its
# last message and hands it immediately to any NEW subscriber, even one
# that starts after the message was originally published. Without this,
# a fresh process (like nero_mtc_task, started fresh each run) can never
# receive a pose published before it existed - this is a structural fix,
# not a timing workaround.
LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class PoseTransformNode(Node):
    def __init__(self):
        super().__init__("pose_transform_node")

        self.declare_parameter("grasps_topic", "/contact_graspnet/grasps")
        self.declare_parameter("target_pose_topic", "/target_pose/pose")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 2.0)

        self._latest_grasps = None
        self._latest_target_pose = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PoseArray, self.get_parameter("grasps_topic").value, self._grasps_cb, 10
        )
        self.create_subscription(
            PoseStamped, self.get_parameter("target_pose_topic").value, self._target_cb, 10
        )

        self.create_service(Trigger, "pose_transform/transform_all", self._transform_cb)
        self.grasp_pub = self.create_publisher(PoseStamped, "/grasp_pose_transformed", LATCHED_QOS)
        self.target_pub = self.create_publisher(PoseStamped, "/target_pose_transformed", LATCHED_QOS)
        # ALL candidates, not just the best-scored one - the top-scored
        # grasp's orientation can be kinematically awkward even when the
        # position is genuinely reachable (confirmed via reach_sweep.py
        # testing), so downstream task planning needs alternatives to try,
        # not just a single fixed pose.
        self.grasps_all_pub = self.create_publisher(PoseArray, "/grasp_poses_transformed", LATCHED_QOS)

        self.get_logger().info(
            "Ready. Call /pose_transform/transform_all (std_srvs/Trigger) once "
            "both a grasp pose and a target pose have been produced upstream."
        )

    def _grasps_cb(self, msg):
        self._latest_grasps = msg

    def _target_cb(self, msg):
        self._latest_target_pose = msg

    def _transform_one(self, pose_stamped, target_frame, timeout):
        try:
            return self.tf_buffer.transform(pose_stamped, target_frame, timeout=timeout), None
        except TransformException as e:
            return None, str(e)

    def _transform_cb(self, request, response):
        if self._latest_grasps is None or len(self._latest_grasps.poses) == 0:
            response.success = False
            response.message = "No grasp poses received yet - run contact_graspnet_ros first"
            return response
        if self._latest_target_pose is None:
            response.success = False
            response.message = "No target pose received yet - run target_pose_ros first"
            return response

        target_frame = self.get_parameter("target_frame").value
        timeout = Duration(seconds=self.get_parameter("tf_timeout_sec").value)

        # Best (highest-scoring) grasp is poses[0] - contact_graspnet_ros
        # sorts its output best-first before publishing.
        grasp_in = PoseStamped()
        grasp_in.header = self._latest_grasps.header
        grasp_in.pose = self._latest_grasps.poses[0]

        grasp_out, grasp_err = self._transform_one(grasp_in, target_frame, timeout)
        if grasp_out is None:
            response.success = False
            response.message = f"Failed to transform grasp pose into '{target_frame}': {grasp_err}"
            return response

        # Also transform every other candidate, so task planning has
        # alternatives to try if the top-scored one's orientation turns
        # out to be kinematically awkward at that position.
        all_transformed = []
        for pose in self._latest_grasps.poses:
            candidate_in = PoseStamped()
            candidate_in.header = self._latest_grasps.header
            candidate_in.pose = pose
            candidate_out, _ = self._transform_one(candidate_in, target_frame, timeout)
            if candidate_out is not None:
                all_transformed.append(candidate_out.pose)

        grasps_array = PoseArray()
        grasps_array.header.frame_id = target_frame
        grasps_array.header.stamp = self.get_clock().now().to_msg()
        grasps_array.poses = all_transformed
        self.grasps_all_pub.publish(grasps_array)

        target_out, target_err = self._transform_one(
            self._latest_target_pose, target_frame, timeout
        )
        if target_out is None:
            response.success = False
            response.message = f"Failed to transform target pose into '{target_frame}': {target_err}"
            return response

        self.grasp_pub.publish(grasp_out)
        self.target_pub.publish(target_out)

        gp = grasp_out.pose.position
        tp = target_out.pose.position
        response.success = True
        response.message = (
            f"Grasp in {target_frame}: ({gp.x:.3f}, {gp.y:.3f}, {gp.z:.3f}). "
            f"Target in {target_frame}: ({tp.x:.3f}, {tp.y:.3f}, {tp.z:.3f})."
        )
        return response


def main():
    rclpy.init()
    node = PoseTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
