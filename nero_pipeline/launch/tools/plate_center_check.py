#!/usr/bin/env python3
"""
plate_center_check.py - WHERE does the pipeline think the plate centre is? One picture answers it.

Run with the pipeline up and after the segmentation step (conda OFF, workspace sourced):
    source ~/agx_arm_ws/install/setup.bash
    /usr/bin/python3 ~/nero_pipeline_launch/tools/plate_center_check.py
It grabs the plate mask (/gsam2/mask_target), the colour image and the depth image, runs the same geometry as target_pose_ros, and writes
/tmp/plate_center.png:
    green outline = the plate mask            red X = 'mean' (the ORIGINAL centre: average of all mask pixels)
    blue +  = 'hull'   (default now)          yellow O = 'circle'        cyan square = 'bbox'
Open it (eog /tmp/plate_center.png) and look:
  * the outline covers the whole plate and the markers sit at its middle  -> the perception is fine; an offset on the table comes from the
    calibration or from how the object sits in the gripper (use PLACE_OFFSET_X / PLACE_OFFSET_Y in mtc_pick_and_place.py);
  * the outline misses part of the plate, or leaks onto the table / another object  -> the mask is the problem (try another prompt such as
    'a white plate.', lower/raise box_threshold, or clear the plate's surroundings).
"""
import sys, time
import numpy as np
import cv2


def draw_overlay(color_bgr, mask, r):
    """Pure drawing function (also used by the tests): returns a BGR image."""
    img = color_bgr.copy()
    cnts, _ = cv2.findContours((mask > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, cnts, -1, (0, 255, 0), 2)
    def pt(p): return (int(round(p[0])), int(round(p[1])))
    cv2.drawMarker(img, pt(r["mean_px"]), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 28, 3)
    cv2.drawMarker(img, pt(r["hull_px"]), (255, 80, 0), cv2.MARKER_CROSS, 34, 3)
    cv2.circle(img, pt(r["circle_px"]), 14, (0, 255, 255), 3)
    u, v = pt(r["bbox_px"]); cv2.rectangle(img, (u - 10, v - 10), (u + 10, v + 10), (255, 255, 0), 2)
    lines = ["red X mean (original)   blue + hull (default)", "yellow O circle   cyan [] bbox",
             "plate ~%.0f cm across, %d mask pixels" % (100 * r["diameter_m"], r["n"])]
    for i, t in enumerate(lines):
        cv2.putText(img, t, (10, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(img, t, (10, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    return img


def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo
    from cv_bridge import CvBridge
    from target_pose_ros.plate_geometry import extract_target, describe

    rclpy.init(); node = Node("plate_center_check"); br = CvBridge(); got = {}
    node.create_subscription(Image, "/gsam2/mask_target", lambda m: got.__setitem__("mask", br.imgmsg_to_cv2(m, "mono8")), 1)
    node.create_subscription(Image, "/topcam/color/image_raw", lambda m: got.__setitem__("color", br.imgmsg_to_cv2(m, "bgr8")), 1)
    node.create_subscription(Image, "/topcam/depth/image_raw", lambda m: got.__setitem__("depth", br.imgmsg_to_cv2(m, "passthrough")), 1)
    node.create_subscription(CameraInfo, "/topcam/depth/camera_info", lambda m: got.__setitem__("K", np.array(m.k).reshape(3, 3)), 1)
    t0 = time.time()
    while len(got) < 4 and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.3)
    missing = [k for k in ("mask", "color", "depth", "K") if k not in got]
    if missing:
        sys.exit("did not receive: %s. Run the segmentation step first (ros2 service call /gsam2/segment std_srvs/srv/Trigger) and keep the camera running." % missing)
    r = extract_target(got["mask"], got["depth"].astype(np.float64) * 0.001, got["K"], "floor", "hull")
    cv2.imwrite("/tmp/plate_center.png", draw_overlay(got["color"], got["mask"], r))
    print(describe(r)); print("\nsaved /tmp/plate_center.png   (open it: eog /tmp/plate_center.png)")


if __name__ == "__main__":
    main()
