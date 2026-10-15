#!/usr/bin/env python3
"""
Bridges ROS2 topics to the standalone Grounded-SAM-2 inference script
running in the `grounded_sam2` conda env (see scripts/grounded_sam2_inference.py).

Runs in the normal ROS2 system Python - never imports torch/transformers/etc
directly, to keep the heavy perception dependencies fully isolated. Calls
the conda env's own python interpreter as a subprocess, passing the current
camera frame as a temp file and reading the result back the same way.

Segments TWO objects per trigger - a "source" (the object to pick up) and
a "target" (where it gets placed relative to), each its own text prompt,
run as two independent inference calls against the same captured frame.
Only the source mask is meant to be fed to a grasp detector
(contact_graspnet_ros) - the target's pose is read directly off its own
point cloud downstream, no grasp planning involved for it.

Exposes:
    /gsam2/segment (std_srvs/Trigger) - segments both source_prompt and
    target_prompt (ROS parameters) against the current camera frame.

Publishes:
    /gsam2/mask_source, /gsam2/annotated_source
    /gsam2/mask_target, /gsam2/annotated_target
    (all sensor_msgs/Image; masks are mono8, 255 = detected object)
"""

import json
import os
import subprocess
import tempfile
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

from gsam2_ros.worker_client import Gsam2Worker, WorkerError


class GSam2SegmentationNode(Node):
    def __init__(self):
        super().__init__("gsam2_segmentation_node")

        self.declare_parameter("camera_topic", "/topcam/color/image_raw")
        self.declare_parameter("source_prompt", "carrot.")
        self.declare_parameter("target_prompt", "plate.")
        self.declare_parameter("conda_env_name", "grounded_sam2")
        self.declare_parameter(
            "gsam2_repo_dir",
            os.path.expanduser("~/near-agilex-manipulation-base/external/Grounded-SAM-2"),
        )
        self.declare_parameter(
            "inference_script",
            os.path.expanduser("~/gsam2_ros_scripts/grounded_sam2_inference.py"),
        )
        self.declare_parameter("box_threshold", 0.25)
        self.declare_parameter("text_threshold", 0.3)
        # Persistent worker: loads the models ONCE instead of once per prompt per call (about 17 of every 20 seconds). Set use_worker
        # to false (ros2 param set /gsam2_segmentation_node use_worker false) to get the old one-process-per-prompt behaviour.
        self.declare_parameter("use_worker", True)
        self.declare_parameter(
            "worker_script",
            os.path.expanduser("~/gsam2_ros_scripts/grounded_sam2_worker.py"),
        )
        self.declare_parameter("worker_timeout_sec", 120.0)

        self.bridge = CvBridge()
        self._latest_frame = None

        camera_topic = self.get_parameter("camera_topic").value
        self.create_subscription(Image, camera_topic, self._image_cb, 10)

        self.create_service(Trigger, "gsam2/segment", self._segment_cb)

        self.mask_source_pub = self.create_publisher(Image, "/gsam2/mask_source", 10)
        self.annotated_source_pub = self.create_publisher(Image, "/gsam2/annotated_source", 10)
        self.mask_target_pub = self.create_publisher(Image, "/gsam2/mask_target", 10)
        self.annotated_target_pub = self.create_publisher(Image, "/gsam2/annotated_target", 10)

        self._worker = None
        if self.get_parameter("use_worker").value:
            worker_script = os.path.expanduser(self.get_parameter("worker_script").value)
            if os.path.exists(worker_script):
                self._worker = Gsam2Worker(
                    ["conda", "run", "-n", self.get_parameter("conda_env_name").value, "--no-capture-output",
                     "python", "-u", worker_script],
                    cwd=os.path.expanduser(self.get_parameter("gsam2_repo_dir").value),
                    log=lambda m: self.get_logger().info(m),
                )
                threading.Thread(target=self._warm_worker, daemon=True).start()     # load the models in the background right away
            else:
                self.get_logger().warn(f"use_worker is on but {worker_script} does not exist - using one process per prompt")

        self.get_logger().info(
            f"Ready. Subscribed to '{camera_topic}'. "
            f"Call /gsam2/segment (std_srvs/Trigger) to segment both "
            f"source_prompt='{self.get_parameter('source_prompt').value}' and "
            f"target_prompt='{self.get_parameter('target_prompt').value}'."
        )

    def _warm_worker(self):
        try:
            self._worker.start()
        except WorkerError as e:
            self.get_logger().error(f"worker start-up failed: {e} (calls fall back to one process per prompt)")

    def _image_cb(self, msg):
        self._latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _run_segmentation(self, prompt, frame):
        """
        Runs one Grounded-SAM-2 inference call for a single prompt against
        the given frame. Returns (success, message, mask_cv2, annotated_cv2,
        result_data) - mask/annotated/result_data are None on failure.
        """
        conda_env_name = self.get_parameter("conda_env_name").value
        gsam2_repo_dir = os.path.expanduser(self.get_parameter("gsam2_repo_dir").value)
        inference_script = os.path.expanduser(self.get_parameter("inference_script").value)
        box_threshold = self.get_parameter("box_threshold").value
        text_threshold = self.get_parameter("text_threshold").value

        with tempfile.TemporaryDirectory() as tmpdir:
            image_path = os.path.join(tmpdir, "input.jpg")
            cv2.imwrite(image_path, frame)

            cmd = [
                "conda", "run", "-n", conda_env_name, "--no-capture-output",
                "python", inference_script,
                "--image", image_path,
                "--text_prompt", prompt,
                "--output_dir", tmpdir,
                "--box_threshold", str(box_threshold),
                "--text_threshold", str(text_threshold),
            ]

            self.get_logger().info(f"Running inference: prompt='{prompt}'")
            result = subprocess.run(
                cmd, cwd=gsam2_repo_dir, capture_output=True, text=True, timeout=120
            )

            if result.returncode != 0:
                self.get_logger().error(f"Inference failed for '{prompt}':\n{result.stderr}")
                return False, f"Inference script failed: {result.stderr[-500:]}", None, None, None

            result_json_path = os.path.join(tmpdir, "result.json")
            if not os.path.exists(result_json_path):
                return False, "Inference ran but produced no result.json", None, None, None

            with open(result_json_path) as f:
                result_data = json.load(f)

            if not result_data.get("detected", False):
                return False, f"No detection for prompt '{prompt}'", None, None, None

            mask = cv2.imread(os.path.join(tmpdir, "mask.png"), cv2.IMREAD_GRAYSCALE)
            annotated = cv2.imread(os.path.join(tmpdir, "annotated.jpg"))
            return True, "ok", mask, annotated, result_data

    def _collect(self, prompt, res, sub_dir):
        """Turn one worker result into the tuple _run_segmentation returns."""
        if not res.get("detected", False):
            return False, f"No detection for prompt '{prompt}'", None, None, None
        mask = cv2.imread(os.path.join(sub_dir, "mask.png"), cv2.IMREAD_GRAYSCALE)
        annotated = cv2.imread(os.path.join(sub_dir, "annotated.jpg"))
        if mask is None or annotated is None:
            return False, "the worker reported a detection but its files could not be read", None, None, None
        return True, "ok", mask, annotated, res

    def _segment_both(self, source_prompt, target_prompt, frame):
        """Returns (source_result, target_result), each shaped like _run_segmentation's return value.
        Uses the persistent worker when enabled; on any worker problem it falls back to the old one-process-per-prompt path."""
        t0 = time.time()
        if self.get_parameter("use_worker").value and self._worker is not None:
            try:
                with tempfile.TemporaryDirectory() as tmpdir:
                    image_path = os.path.join(tmpdir, "input.jpg")
                    cv2.imwrite(image_path, frame)
                    reply = self._worker.request(
                        dict(cmd="segment", image=image_path, prompts=[source_prompt, target_prompt], output_dir=tmpdir,
                             box_threshold=self.get_parameter("box_threshold").value,
                             text_threshold=self.get_parameter("text_threshold").value),
                        timeout=self.get_parameter("worker_timeout_sec").value,
                    )
                    if reply.get("ok"):
                        res = reply["results"]
                        out = (self._collect(source_prompt, res[0], os.path.join(tmpdir, "0")),
                               self._collect(target_prompt, res[1], os.path.join(tmpdir, "1")))
                        self.get_logger().info(f"segmentation took {time.time() - t0:.1f} s "
                                               f"(persistent worker; {reply.get('seconds')} s inside it)")
                        return out
                    self.get_logger().error("the worker reported an error, falling back: " + str(reply.get("error", "?"))[-400:])
            except WorkerError as e:
                self.get_logger().error(f"persistent worker failed ({e}); falling back to one process per prompt")
        src = self._run_segmentation(source_prompt, frame)
        if not src[0]:                                          # as before: do not bother with the target if the source failed
            return src, (False, "not run because the source failed", None, None, None)
        tgt = self._run_segmentation(target_prompt, frame)
        self.get_logger().info(f"segmentation took {time.time() - t0:.1f} s (one process per prompt)")
        return src, tgt

    def _segment_cb(self, request, response):
        if self._latest_frame is None:
            response.success = False
            response.message = "No camera frame received yet"
            return response

        source_prompt = self.get_parameter("source_prompt").value
        target_prompt = self.get_parameter("target_prompt").value
        # Use one shared frame for both, so source and target come from the
        # exact same instant rather than two frames grabbed moments apart.
        frame = self._latest_frame.copy()

        (src_ok, src_msg, src_mask, src_annotated, src_data), (tgt_ok, tgt_msg, tgt_mask, tgt_annotated, tgt_data) = \
            self._segment_both(source_prompt, target_prompt, frame)
        if not src_ok:
            response.success = False
            response.message = f"Source ('{source_prompt}') failed: {src_msg}"
            return response

        if not tgt_ok:
            response.success = False
            response.message = f"Target ('{target_prompt}') failed: {tgt_msg}"
            return response

        self.mask_source_pub.publish(self.bridge.cv2_to_imgmsg(src_mask, encoding="mono8"))
        self.annotated_source_pub.publish(self.bridge.cv2_to_imgmsg(src_annotated, encoding="bgr8"))
        self.mask_target_pub.publish(self.bridge.cv2_to_imgmsg(tgt_mask, encoding="mono8"))
        self.annotated_target_pub.publish(self.bridge.cv2_to_imgmsg(tgt_annotated, encoding="bgr8"))

        response.success = True
        response.message = (
            f"Source: '{src_data['label']}' (box={src_data['box_score']:.3f}, "
            f"mask={src_data['mask_score']:.3f}). "
            f"Target: '{tgt_data['label']}' (box={tgt_data['box_score']:.3f}, "
            f"mask={tgt_data['mask_score']:.3f})."
        )
        return response


def main():
    rclpy.init()
    node = GSam2SegmentationNode()
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
