#!/usr/bin/env python3
"""p4_cloud_compare: do the two point clouds agree on the table?
Takes one cloud from each camera, moves it into base_link with TF, keeps the table points in a circle
around (--cx, --cy), fits a plane to each, and prints per camera: table height at the centre, table tilt,
fit noise; then wrist minus top. This is the number behind "the clouds blend in RViz".
Read-only. Run with conda deactivated:
  /usr/bin/python3 p4_cloud_compare.py                 (centre = where the wrist camera sees the table)
  /usr/bin/python3 p4_cloud_compare.py --cx -0.39 --cy 0.015 --radius 0.10
"""
import argparse, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

VERSION = "p4-cloud-compare-v4"   # v4: table height from pipeline4/core
# v3:   # v3: also shows the heights after the offsets in ~/.p4_offsets.yaml
# v2:   # v2: centre found from the wrist view; compared with the REAL table (p3_tablez); suggests offsets
BASE = "base_link"
CLOUDS = {"TOP": "/topcam/depth/points", "WRIST": "/camera/armcam/depth/color/points"}
TABLE_BAND = 0.06   # keep points within +-6 cm of base_link z=0 (drops objects taller than that)


def real_table_z():
    """pipeline 3's measured table height (p3_tablez), or None."""
    import os, sys
    sys.path.insert(0, os.path.expanduser("~/nero_pipeline_launch/pipeline4/core"))
    try:
        import p3_tablez
        z, src = p3_tablez.table_z()
        return z, src
    except Exception:
        return None, "p3_tablez not found"


def cloud_xyz(msg):
    """PointCloud2 -> (N,3) float array of the finite, non-zero points."""
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                   "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
    a = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)
    p = np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64)
    return p[np.isfinite(p).all(1) & (np.abs(p).sum(1) > 1e-6)]


def qmat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def fit_table(p, cx, cy, radius):
    """Robust plane z = a x + b y + c on points within radius of (cx, cy) and near z=0.
    Returns (n_points, height at centre, tilt deg, rms mm) or None."""
    m = (np.hypot(p[:, 0] - cx, p[:, 1] - cy) < radius) & (np.abs(p[:, 2]) < TABLE_BAND)
    q = p[m]
    if len(q) < 200:
        return None
    keep = np.ones(len(q), bool)
    for tol in (0.015, 0.008, 0.004):
        A = np.c_[q[keep, 0], q[keep, 1], np.ones(keep.sum())]
        coef, *_ = np.linalg.lstsq(A, q[keep, 2], rcond=None)
        r = q[:, 2] - (coef[0] * q[:, 0] + coef[1] * q[:, 1] + coef[2])
        keep = np.abs(r) < tol
        if keep.sum() < 100:
            return None
    a, b, c = coef
    return int(keep.sum()), a * cx + b * cy + c, math.degrees(math.atan(math.hypot(a, b))), 1000 * r[keep].std()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cx", type=float, default=None, help="default: middle of the table area the wrist camera sees")
    ap.add_argument("--cy", type=float, default=None)
    ap.add_argument("--radius", type=float, default=0.10)
    a = ap.parse_args()
    print(VERSION)
    rclpy.init()
    n = Node("p4_cloud_compare")
    buf = Buffer(); TransformListener(buf, n)
    got = {}
    for name, topic in CLOUDS.items():
        n.create_subscription(PointCloud2, topic, lambda m, k=name: got.setdefault(k, m), qos_profile_sensor_data)
    t0 = time.time()
    while len(got) < len(CLOUDS) and time.time() - t0 < 8:
        rclpy.spin_once(n, timeout_sec=0.1)
    for _ in range(20):                       # let TF fill
        rclpy.spin_once(n, timeout_sec=0.05)
    res, base_pts = {}, {}
    for name in CLOUDS:
        if name not in got:
            print("%-5s no cloud received on %s" % (name, CLOUDS[name])); continue
        m = got[name]
        try:
            tf = buf.lookup_transform(BASE, m.header.frame_id, rclpy.time.Time())
        except Exception as e:
            print("%-5s no TF %s -> %s (%s)" % (name, BASE, m.header.frame_id, str(e).splitlines()[0][:80])); continue
        t, r = tf.transform.translation, tf.transform.rotation
        base_pts[name] = cloud_xyz(m) @ qmat(r.x, r.y, r.z, r.w).T + np.array([t.x, t.y, t.z])
    if a.cx is None or a.cy is None:
        w = base_pts.get("WRIST")
        w = None if w is None else w[np.abs(w[:, 2]) < TABLE_BAND]
        if w is None or len(w) < 500:
            print("the wrist camera sees no table here: point it at the table (20-50 cm away), or give --cx --cy"); return
        a.cx, a.cy = float(np.median(w[:, 0])), float(np.median(w[:, 1]))
    tz, tz_src = real_table_z()
    print("comparing within %.2f m of (%.3f, %.3f); real table %s" % (a.radius, a.cx, a.cy,
          "%+.1f mm (%s)" % (1000 * tz, tz_src) if tz is not None else "unknown (%s)" % tz_src))
    for name in CLOUDS:
        if name not in base_pts:
            continue
        p = base_pts[name]
        f = fit_table(p, a.cx, a.cy, a.radius)
        if f is None:
            print("%-5s too few table points within %.2f m of (%.2f, %.2f)" % (name, a.radius, a.cx, a.cy)); continue
        res[name] = f
        err = "" if tz is None else "   (%+.1f mm vs the real table)" % (1000 * (f[1] - tz))
        print("%-5s %6d table points   height at centre %+6.1f mm   tilt %4.1f deg   noise %.1f mm%s"
              % (name, f[0], 1000 * f[1], f[2], f[3], err))
    if len(res) == 2:
        print("WRIST - TOP: height %+.1f mm   (raw cameras)" % (1000 * (res["WRIST"][1] - res["TOP"][1])))
    import os, yaml
    try:
        off = yaml.safe_load(open(os.path.expanduser("~/.p4_offsets.yaml"))) or {}
    except Exception:
        off = {}
    if off:
        print("AFTER the offsets in ~/.p4_offsets.yaml (what the fusion uses):")
        fixed = {}
        for name, key in (("WRIST", "wrist_offset_z"), ("TOP", "top_offset_z")):
            if name in res:
                fixed[name] = res[name][1] + float(off.get(key, 0.0))
                print("  %-5s height %+6.1f mm%s" % (name, 1000 * fixed[name], "" if tz is None else "   (%+.1f mm vs the real table)" % (1000 * (fixed[name] - tz))))
        if len(fixed) == 2:
            print("  WRIST - TOP: %+.1f mm" % (1000 * (fixed["WRIST"] - fixed["TOP"])))
    if tz is not None:
        for name, key in (("WRIST", "wrist_offset_z"), ("TOP", "top_offset_z")):
            if name in res:
                e = res[name][1] - tz
                print("%-5s %s" % (name, "OK (within 3 mm of the real table)" if abs(e) <= 0.003 else
                                   "is %.1f mm too %s -> fusion parameter  -p %s:=%.4f" % (1000 * abs(e), "high" if e > 0 else "low", key, -e)))


if __name__ == "__main__":
    main()
