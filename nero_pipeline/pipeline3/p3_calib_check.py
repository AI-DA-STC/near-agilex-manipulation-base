#!/usr/bin/env python3
"""
p3_calib_check.py - how far off is the hand-eye calibration in HEIGHT? Measured, not by eye.

Clear the table in front of the arm (objects are ignored by the fit, but a clear table is the cleanest), camera and hand-eye publisher running
(start_pipeline3.sh is enough), conda deactivated, ROS sourced:

    /usr/bin/python3 ~/nero_pipeline_launch/pipeline3/p3_calib_check.py               # MEASURE only (changes nothing)
    /usr/bin/python3 ~/nero_pipeline_launch/pipeline3/p3_calib_check.py --fix-height  # measure, then correct the camera HEIGHT in the calibration

It averages 10 depth frames, puts every pixel in base_link with the CURRENT calibration (TF), and fits a plane to the table in the work area
(robust: cubes, the plate, cables are ignored). The real table top is z = 0 by definition, so:
    the plane's height in the middle of the work area = the calibration's height error;
    the plane's slope = a tilt error (the calibration's rotation), reported but NOT changed here.
--fix-height backs the calibration file up and lowers (or raises) the camera by exactly that height error - the same for every point, so
x and y are not touched. The hand-eye publisher reads the file only when it starts: restart start_pipeline3.sh, then measure again.
"""
import argparse
import math
import os
import shutil
import sys
import time

import numpy as np

P3_CALIB_CHECK_VERSION = "p3-calibcheck-v4"
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p3_tablez                                                      # noqa: E402  v2: the REAL table height (robot on a plate -> table below 0)
MAX_FIX_TILT_DEG = 3.0          # a bigger tilt is not a calibration slip: --fix refuses
CALIB_PATH = os.path.expanduser("~/.ros2/easy_handeye2/calibrations/nero_eye_to_hand_calib.calib")
WORK_X = (-0.75, -0.20)          # the work area in front of the arm (base_link, m)
WORK_Y = (-0.40, 0.40)
CENTRE = (-0.45, 0.0)
Z_WINDOW = 0.04                  # only points within +-4 cm of z = 0 can be table
INLIER_M = 0.004                 # a point within 4 mm of the plane counts as table
GOOD_MM = 2.0                    # |height error| <= 2 mm: nothing to fix
TILT_WARN_DEG = 0.5


def fit_table_plane(pts, iters=400, seed=0):
    """Robust plane z = a*x + b*y + c through the table points (RANSAC on 3 points, then least squares on the inliers).
    -> (a, b, c, inlier_mask, rms_m)"""
    rng = np.random.default_rng(seed)
    P = np.asarray(pts, dtype=np.float64)
    A = np.c_[P[:, 0], P[:, 1], np.ones(len(P))]
    best = None
    for _ in range(iters):
        idx = rng.choice(len(P), 3, replace=False)
        try:
            coef = np.linalg.solve(A[idx], P[idx, 2])
        except np.linalg.LinAlgError:
            continue
        inl = np.abs(A @ coef - P[:, 2]) < INLIER_M
        if best is None or inl.sum() > best.sum():
            best = inl
    coef, *_ = np.linalg.lstsq(A[best], P[best, 2], rcond=None)
    inl = np.abs(A @ coef - P[:, 2]) < INLIER_M
    coef, *_ = np.linalg.lstsq(A[inl], P[inl, 2], rcond=None)
    rms = float(np.sqrt(np.mean((A[inl] @ coef - P[inl, 2]) ** 2)))
    return float(coef[0]), float(coef[1]), float(coef[2]), inl, rms


def describe(a, b, c, n_in, n_all, rms, table_z=0.0, source=""):
    """-> (lines, height_error_m, tilt_deg); the height error is measured against the REAL table height table_z"""
    seen = a * CENTRE[0] + b * CENTRE[1] + c
    h = seen - table_z
    tilt = math.degrees(math.atan(math.hypot(a, b)))
    corners = [(x, y, a * x + b * y + c - table_z) for x in WORK_X for y in WORK_Y]
    L = ["table plane in base_link (work area x %.2f..%.2f, y %.2f..%.2f; %d of %d points are table, fit noise %.1f mm):"
         % (WORK_X[0], WORK_X[1], WORK_Y[0], WORK_Y[1], n_in, n_all, 1000 * rms),
         "  real table height: %+.1f mm (%s);  the camera sees it at %+.1f mm" % (1000 * table_z, source, 1000 * seen),
         "  HEIGHT ERROR in the middle of the work area (x %.2f, y %.2f): %+.1f mm" % (CENTRE[0], CENTRE[1], 1000 * h),
         "  TILT: %.2f deg  (slope %+.1f mm per 10 cm along x, %+.1f mm per 10 cm along y)" % (tilt, 100 * a, 100 * b),
         "  error at the corners: " + "   ".join("(%.2f, %+.2f): %+.1f mm" % (x, y, 1000 * z) for x, y, z in corners)]
    if abs(h) * 1000 <= GOOD_MM:
        L.append("HEIGHT OK: within %.0f mm - nothing to fix." % GOOD_MM)
    else:
        L.append("HEIGHT OFF by %+.1f mm: the camera sees the table %s than it is." % (1000 * h, "higher" if h > 0 else "lower"))
    if tilt > TILT_WARN_DEG:
        L.append("TILT %.2f deg is more than %.1f deg: the calibration's ROTATION is off too (a height fix alone would leave the far side off by up to %.1f mm)."
                 % (tilt, TILT_WARN_DEG, 1000 * max(abs(z - h) for _, _, z in corners)))
    if abs(h) * 1000 > GOOD_MM or tilt > TILT_WARN_DEG:
        L.append("-> --fix corrects BOTH: it levels the table (rotation about the work area's middle) and moves it to the real table height.")
    return L, h, tilt


def rot_to_quat(R):
    R = np.asarray(R, dtype=np.float64)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        q = [(R[2, 1] - R[1, 2]) / S, (R[0, 2] - R[2, 0]) / S, (R[1, 0] - R[0, 1]) / S, 0.25 * S]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [0.25 * S, (R[0, 1] + R[1, 0]) / S, (R[0, 2] + R[2, 0]) / S, (R[2, 1] - R[1, 2]) / S]
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 1] + R[1, 0]) / S, 0.25 * S, (R[1, 2] + R[2, 1]) / S, (R[0, 2] - R[2, 0]) / S]
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[0, 2] + R[2, 0]) / S, (R[1, 2] + R[2, 1]) / S, 0.25 * S, (R[1, 0] - R[0, 1]) / S]
    q = np.array(q)
    return q / np.linalg.norm(q)


def quat_to_rot(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def level_and_height(a, b, c, table_z, R, t):
    """The corrected calibration (R', t') for a camera pose p_base = R p_cam + t that sees the table as z = a x + b y + c:
    rotate the whole view about the work area's middle point on the table so the table becomes level, then move it to table_z."""
    n = np.array([-a, -b, 1.0]); n /= np.linalg.norm(n)
    z = np.array([0.0, 0.0, 1.0])
    k = np.cross(n, z); s_ = np.linalg.norm(k); cth = float(n @ z)
    if s_ < 1e-12:
        Rf = np.eye(3)
    else:
        k /= s_
        Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        Rf = np.eye(3) + s_ * Kx + (1 - cth) * (Kx @ Kx)               # Rodrigues: turns n onto +z
    cpt = np.array([CENTRE[0], CENTRE[1], a * CENTRE[0] + b * CENTRE[1] + c])
    R2 = Rf @ np.asarray(R)
    t2 = Rf @ (np.asarray(t) - cpt) + cpt
    t2[2] -= cpt[2] - table_z
    return R2, t2


def fix_full(a, b, c, table_z, path=CALIB_PATH, now=None):
    """Level + height fix written into the calibration file (backup first). -> (backup, old (R, t), new (R, t))"""
    import yaml
    content = _load_checked(path)
    tr = content["transform"]
    t = np.array([float(tr["translation"][k]) for k in "xyz"])
    q = tr["rotation"]
    R = quat_to_rot(float(q["x"]), float(q["y"]), float(q["z"]), float(q["w"]))
    R2, t2 = level_and_height(a, b, c, table_z, R, t)
    q2 = rot_to_quat(R2)
    backup = path + ".bak_fix_" + time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
    shutil.copy(path, backup)
    for i, k in enumerate("xyz"):
        tr["translation"][k] = float(t2[i])
    for i, k in enumerate("xyzw"):
        tr["rotation"][k] = float(q2[i])
    with open(path, "w") as f:
        yaml.safe_dump(content, f, default_flow_style=False)
    return backup, (R, t), (R2, t2)


def _load_checked(path):
    import yaml
    with open(path) as f:
        content = yaml.safe_load(f)
    par = content.get("parameters", {})
    base = par.get("robot_base_frame")
    ctype = str(par.get("calibration_type", ""))
    if base not in (None, "base_link") or ("eye_on_base" not in ctype and "eye_to_hand" not in ctype and ctype):
        raise SystemExit("STOP: this calibration is not camera-in-base_link (robot_base_frame=%r, calibration_type=%r): not changed" % (base, ctype))
    return content


def fix_height(height_error_m, path=CALIB_PATH, now=None):
    """Lower the camera in the calibration by height_error_m (translation z of base_link -> camera). Backs the file up first. -> (backup, old_z, new_z)"""
    import yaml
    with open(path) as f:
        content = yaml.safe_load(f)
    par = content.get("parameters", {})
    base = par.get("robot_base_frame")
    ctype = str(par.get("calibration_type", ""))
    if base not in (None, "base_link") or ("eye_on_base" not in ctype and "eye_to_hand" not in ctype and ctype):
        raise SystemExit("STOP: this calibration is not camera-in-base_link (robot_base_frame=%r, calibration_type=%r): not changed" % (base, ctype))
    t = content["transform"]["translation"]
    old = float(t["z"])
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
    backup = path + ".bak_height_" + stamp
    shutil.copy(path, backup)
    t["z"] = old - float(height_error_m)
    with open(path, "w") as f:
        yaml.safe_dump(content, f, default_flow_style=False)
    return backup, old, float(t["z"])


def measure(n_frames=10, timeout=20.0):
    """Points of the work area in base_link through the CURRENT calibration (TF). -> (Nx3 array, frame id)"""
    import rclpy
    import tf2_ros
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    rclpy.init()
    node = Node("p3_calib_check")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    bridge = CvBridge()
    frames, info = [], {}
    node.create_subscription(Image, "/topcam/depth/image_raw", lambda m: frames.append(m) if len(frames) < n_frames else None, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, "/topcam/depth/camera_info", lambda m: info.setdefault("m", m), qos_profile_sensor_data)
    t_end = time.time() + timeout
    while time.time() < t_end and (len(frames) < n_frames or "m" not in info):
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(frames) < n_frames or "m" not in info:
        raise SystemExit("STOP: no depth frames / camera_info on /topcam/depth/* - is the camera running?")
    ci = info["m"]
    frame = ci.header.frame_id
    T = None
    while time.time() < t_end + 5 and T is None:
        try:
            tr = buf.lookup_transform("base_link", frame, rclpy.time.Time())
            q, p = tr.transform.rotation, tr.transform.translation
            x, y, z, w = q.x, q.y, q.z, q.w
            R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            T = (R, np.array([p.x, p.y, p.z]))
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if T is None:
        raise SystemExit("STOP: no TF base_link -> %s: is the hand-eye publisher running?" % frame)
    stack = []
    for m in frames:
        d = bridge.imgmsg_to_cv2(m, desired_encoding="passthrough").astype(np.float64)
        stack.append(d * (0.001 if m.encoding in ("16UC1", "mono16") else 1.0))
    depth = np.median(np.where(np.array(stack) > 0, np.array(stack), np.nan), axis=0)
    H, W = depth.shape
    if (W, H) != (int(ci.width), int(ci.height)) or (W, H) != (640, 480):
        raise SystemExit("STOP: depth is %dx%d - the camera came up WITHOUT registration (docs/11 2.5): replug the camera and restart" % (W, H))
    K = np.array(ci.k).reshape(3, 3)
    vs, us = np.mgrid[0:H, 0:W]
    ok = np.isfinite(depth) & (depth > 0)
    z = depth[ok]
    cam = np.stack([(us[ok] - K[0, 2]) * z / K[0, 0], (vs[ok] - K[1, 2]) * z / K[1, 1], z], axis=1)
    base = cam @ T[0].T + T[1]
    sel = (base[:, 0] > WORK_X[0]) & (base[:, 0] < WORK_X[1]) & (base[:, 1] > WORK_Y[0]) & (base[:, 1] < WORK_Y[1]) & (np.abs(base[:, 2]) < Z_WINDOW)
    return base[sel], frame


def main(argv=None):
    ap = argparse.ArgumentParser(description="Measure the calibration's height error from the table plane; --fix-height corrects it.")
    ap.add_argument("--fix-height", action="store_true", help="correct only the camera height in the calibration file (backup first)")
    ap.add_argument("--fix", action="store_true", help="correct tilt AND height in the calibration file (backup first)")
    ap.add_argument("--set-tip-min", type=float, metavar="M", help="store the MAXIMUM fingertip clearance above the table (m, for tall objects), e.g. 0.015")
    ap.add_argument("--set-tip-floor", type=float, metavar="M", help="store the MINIMUM fingertip clearance above the table (m, for thin objects like a pen), e.g. 0.004")
    ap.add_argument("--set-table-z", type=float, metavar="M", help="store the REAL table height in base_link (m), e.g. -0.0127 for a 12.7 mm plate under the robot")
    a = ap.parse_args(argv)
    print("p3_calib_check %s - calibration file: %s" % (P3_CALIB_CHECK_VERSION, CALIB_PATH), flush=True)
    if a.set_tip_min is not None or a.set_tip_floor is not None:
        if a.set_tip_min is not None:
            v = p3_tablez.set_tip_min(a.set_tip_min)
            print("MAXIMUM fingertip clearance stored: %.1f mm in %s" % (1000 * v, p3_tablez.TIP_MIN_FILE))
        if a.set_tip_floor is not None:
            v = p3_tablez.set_tip_floor(a.set_tip_floor)
            print("MINIMUM fingertip clearance stored: %.1f mm in %s" % (1000 * v, p3_tablez.TIP_FLOOR_FILE))
        for name, h in (("a 1 cm pen", 0.010), ("a 3 cm carrot", 0.030), ("a 5 cm cube", 0.050)):
            print("  now %-14s -> %s" % (name, p3_tablez.clearance_for(h)[1]))
        print("Used by the tip lift (restart start_pipeline3.sh) and the fingertip check.")
        return 0
    if a.set_table_z is not None:
        v = p3_tablez.set_table_z(a.set_table_z)
        print("REAL TABLE HEIGHT stored: %+.1f mm in %s - used by MoveIt's table box (p3_ready.sh), the fingertip check and the tip lift." % (1000 * v, p3_tablez.TABLE_Z_FILE))
        print("Restart start_pipeline3.sh (the tip lift reads it at start) and run p3_ready.sh (it places the table box).")
        return 0
    table_z, source = p3_tablez.table_z()
    pts, frame = measure()
    if len(pts) < 2000:
        print("STOP: only %d points near z = 0 in the work area - is the camera looking at the table, and is the calibration roughly right?" % len(pts))
        return 1
    a_, b_, c_, inl, rms = fit_table_plane(pts)
    lines, h, tilt = describe(a_, b_, c_, int(inl.sum()), len(pts), rms, table_z, source)
    print("\n".join(lines), flush=True)
    if a.fix:
        if tilt > MAX_FIX_TILT_DEG:
            print("Nothing changed: a %.1f deg tilt is too large for a calibration slip - is the camera loose? Send me this output." % tilt)
            return 1
        if abs(h) * 1000 <= GOOD_MM and tilt <= TILT_WARN_DEG:
            print("Nothing changed: height and tilt are already fine.")
            return 0
        backup, old, new = fix_full(a_, b_, c_, table_z)
        print("\nCALIBRATION CHANGED (tilt %.2f deg levelled, height %+.1f mm corrected). Camera position %s -> %s. Backup: %s" % (
            tilt, -1000 * h, np.round(old[1], 4).tolist(), np.round(new[1], 4).tolist(), backup))
        print("Now: restart the hand-eye publisher (Terminal 1's start_pipeline3.sh, or Terminal B) - it reads the file at start - and measure again:")
        print("  /usr/bin/python3 %s" % os.path.abspath(__file__))
        print("To undo:  cp %s %s   (and restart)" % (backup, CALIB_PATH))
        return 0
    if not a.fix_height:
        if abs(h) * 1000 > GOOD_MM or tilt > TILT_WARN_DEG:
            print("\nTo correct it:  /usr/bin/python3 %s --fix    (then restart the hand-eye publisher and measure again)" % os.path.abspath(__file__))
        return 0
    if abs(h) * 1000 <= GOOD_MM:
        print("Nothing changed: the height is already within %.0f mm." % GOOD_MM)
        return 0
    if tilt > TILT_WARN_DEG:
        print("Nothing changed: the tilt (%.2f deg) needs a look first - send me this output." % tilt)
        return 1
    backup, old, new = fix_height(h)                                   # h is already measured against the real table height
    print("\nCALIBRATION CHANGED: camera height (translation z) %.4f -> %.4f m (%+.1f mm). Backup: %s" % (old, new, -1000 * h, backup))
    print("Now: Ctrl+C in Terminal 1, start start_pipeline3.sh again (the hand-eye publisher reads the file at start), then measure again:")
    print("  /usr/bin/python3 %s" % os.path.abspath(__file__))
    print("To undo:  cp %s %s   (and restart)" % (backup, CALIB_PATH))
    return 0


if __name__ == "__main__":
    sys.exit(main())
