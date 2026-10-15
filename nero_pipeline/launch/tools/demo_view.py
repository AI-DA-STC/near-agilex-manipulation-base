#!/usr/bin/env python3
"""
demo_view.py - one window with the Grounded-SAM-2 results, for demos and recordings.
Shows the PICK object (left) and the PLACE target (right) with their boxes and masks the moment /gsam2/segment finishes, and keeps them on screen until the next call.
Start it BEFORE the pipeline runs: the node publishes the images once per call, so a viewer that starts later misses them.
    /usr/bin/python3 demo_view.py [--scale 1.0] [--height 480]        close with Esc or q
Subscribes: /gsam2/annotated_source and /gsam2/annotated_target (bgr8, published by gsam2_ros).
"""
import argparse, sys, time
import numpy as np
import cv2


def panel(img, title, height, note=None):
    """one labelled panel of the given height (aspect kept); a grey placeholder when there is no image yet"""
    if img is None:
        w = int(height * 4 / 3); out = np.full((height, w, 3), 70, np.uint8)
        cv2.putText(out, "waiting for /gsam2/segment ...", (20, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2, cv2.LINE_AA)
    else:
        h, w = img.shape[:2]; out = cv2.resize(img, (max(1, int(w * height / h)), height), interpolation=cv2.INTER_AREA)
    bar = np.full((44, out.shape[1], 3), 30, np.uint8)
    cv2.putText(bar, title + (("   " + note) if note else ""), (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2, cv2.LINE_AA)
    return np.vstack([bar, out])


def compose(src, tgt, height=480, src_note=None, tgt_note=None):
    a, b = panel(src, "PICK: source object", height, src_note), panel(tgt, "PLACE: target", height, tgt_note)
    gap = np.full((a.shape[0], 8, 3), 20, np.uint8)
    return np.hstack([a, gap, b])


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scale", type=float, default=1.0, help="window size factor")
    ap.add_argument("--height", type=int, default=480, help="height of each image panel in pixels")
    ap.add_argument("--src_topic", default="/gsam2/annotated_source"); ap.add_argument("--tgt_topic", default="/gsam2/annotated_target")
    ap.add_argument("--frames", type=int, default=0, help="(testing) stop after this many loop iterations")
    a = ap.parse_args(argv)
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    rclpy.init(); node = Node("demo_view"); bridge = CvBridge(); state = {"src": None, "tgt": None, "src_t": None, "tgt_t": None, "dirty": True}

    def cb(key):
        def f(msg):
            state[key] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"); state[key + "_t"] = time.strftime("%H:%M:%S"); state["dirty"] = True
        return f
    node.create_subscription(Image, a.src_topic, cb("src"), 10); node.create_subscription(Image, a.tgt_topic, cb("tgt"), 10)
    win = "Grounded-SAM-2 segmentation"
    try:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.imshow(win, compose(None, None, a.height)); cv2.waitKey(1); state["dirty"] = False      # placeholder is on screen already
    except cv2.error as e:
        print("This OpenCV cannot open windows (%s).\nUse rqt_image_view instead:\n  ros2 run rqt_image_view rqt_image_view /gsam2/annotated_source\n  ros2 run rqt_image_view rqt_image_view /gsam2/annotated_target" % str(e).strip().splitlines()[0][:120])
        rclpy.shutdown(); return 2
    n = 0
    print("window open - waiting for /gsam2/segment (Esc or q in the window closes it)")
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if state["dirty"]:
            img = compose(state["src"], state["tgt"], a.height, ("(" + state["src_t"] + ")") if state["src_t"] else None, ("(" + state["tgt_t"] + ")") if state["tgt_t"] else None)
            if a.scale != 1.0:
                img = cv2.resize(img, None, fx=a.scale, fy=a.scale, interpolation=cv2.INTER_AREA)
            cv2.imshow(win, img); state["dirty"] = False
        k = cv2.waitKey(30) & 0xFF
        if k in (27, ord("q")): break
        n += 1
        if a.frames and n >= a.frames: break
    cv2.destroyAllWindows(); rclpy.shutdown(); return 0


if __name__ == "__main__":
    sys.exit(main())
