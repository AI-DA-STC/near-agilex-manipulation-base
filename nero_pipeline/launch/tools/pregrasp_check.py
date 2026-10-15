#!/usr/bin/env python3
"""
pregrasp_check.py - offline "why did MTC say 'pregrasp IK ... no IK found'?"  Moves nothing, needs no ROS (system python3 with numpy + scipy is fine).

Run it right after a detection while grasp_watch.py is running (it writes /tmp/grasp_debug.json), e.g. after run_pick_and_place.sh:
    /usr/bin/python3 pregrasp_check.py
It reads /tmp/grasp_debug.json (candidates in base_link), /tmp/last_depth.npy and /tmp/last_segmap.npy and prints, for every tcp target that
was handed to MTC:
   tilt            degrees from straight down
   closing offset  where that grasp closes (hand base + 0.1034 m along the approach axis) relative to the cube's centre, in base_link, cm
                   (dx, dy sideways; dz height of the closing point relative to the cube's top; > ~2 cm sideways = beside the cube)
   height          closing point above the table (cm)
   pregrasp        the pre-grasp position MTC has to reach: grasp position minus PREGRASP_OFFSET_M along the approach axis
   reach           kinematic check of the pregrasp pose and of the grasp pose with the existing reach_check.py (joint limits only, NO collisions;
                   it allows the gripper axis anywhere within the candidate's tilt + 5 degrees, so it is optimistic)
How to read it: pregrasp NOT REACHABLE -> the arm cannot get there (move the cube / pick another spot); pregrasp reachable but MTC still failed ->
collision (table, cube, own links) or the exact orientation is out of the wrist's range.
"""
import argparse, json, math, os, re, sys, time
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--json", default="/tmp/grasp_debug.json"); ap.add_argument("--depth", default="/tmp/last_depth.npy"); ap.add_argument("--segmap", default="/tmp/last_segmap.npy")
ap.add_argument("--tools", default=os.path.expanduser("~/nero_pipeline_launch/tools"), help="folder with reach_check.py and nero_fk.py")
ap.add_argument("--mtc", default=os.path.expanduser("~/agx_arm_ws/src/nero_mtc_task/nero_mtc_task/mtc_pick_and_place.py"))
ap.add_argument("--depth_below_top", type=float, default=0.02, help="recentre_depth_below_top_m currently used by the grasp node (m)")
ap.add_argument("--table_z", type=float, default=-0.03, help="table top height in base_link (TABLE_TOP_Z of start_pipeline.sh)")
a = ap.parse_args()

FINGER_TIP = 0.138          # tcp (hand base) -> fingertip along the approach axis (grasp_view docs: the fingers span 0.062 .. 0.138 m)
CLOSE_DEPTH = 0.1034          # CGN: grasp centre is this far along the approach axis from the hand base
pre = 0.08
try:
    pre = float(re.search(r"^PREGRASP_OFFSET_M\s*=\s*([0-9.]+)", open(a.mtc).read(), re.M).group(1))
except Exception:
    print("(could not read PREGRASP_OFFSET_M from %s - using %.2f)" % (a.mtc, pre))

if not os.path.exists(a.json):
    sys.exit("%s does not exist: start grasp_watch.py (it writes it after every detection), then run a detection or run_pick_and_place.sh, then run this again." % a.json)
js = json.load(open(a.json)); cands = [np.array(T) for T in js.get("cands", [])]
print("json is %.0f s old, %d raw candidates, %d tcp targets handed to MTC, pregrasp offset %.3f m, table top z %.3f m" %
      (time.time() - js.get("time", time.time()), len(js.get("raw", [])), len(cands), pre, a.table_z))
if not cands:
    sys.exit("no candidates in the json - run a detection with grasp_watch.py running first")

centre = ztop = None
if js.get("T_base_cam") and js.get("K") and os.path.exists(a.depth) and os.path.exists(a.segmap):
    d = np.load(a.depth).astype(float); s = np.load(a.segmap) > 0; K = js["K"]; fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    v, u = np.nonzero(s); z = d[v, u]; ok = np.isfinite(z) & (z > 0.2) & (z < 1.6)
    if ok.any():
        P = np.c_[(u[ok] - cx) * z[ok] / fx, (v[ok] - cy) * z[ok] / fy, z[ok]]
        T = np.array(js["T_base_cam"]); B = (T[:3, :3] @ P.T).T + T[:3, 3]
        centre = np.median(B, axis=0); ztop = float(np.percentile(B[:, 2], 98)); zbot = float(np.percentile(B[:, 2], 2))
        print("cube (%d points) centre in base_link: (%.3f, %.3f, %.3f) m, radius from the base axis %.3f m, z from %.3f to %.3f m (%.1f cm above table)" %
              (ok.sum(), centre[0], centre[1], centre[2], math.hypot(centre[0], centre[1]), zbot, ztop, 100 * (ztop - a.table_z)))
else:
    print("(no depth/mask/camera transform available - cube position not shown)")

# ---- where does the CAMERA put the table? (pixels in a ring around the mask, in base_link) versus the table box MoveIt uses
table_cloud = None
if centre is not None:
    def _dil(mk, r):
        out = mk.copy(); H, W = out.shape
        for _ in range(r):
            p = np.pad(out, 1); out = np.any([p[dy:dy + H, dx:dx + W] for dy in range(3) for dx in range(3)], axis=0)
        return out
    ring = (_dil(s, 14) & ~_dil(s, 7)) & np.isfinite(d) & (d > 0.2) & (d < 1.6)
    if ring.sum() > 200:
        rv, ru = np.nonzero(ring); rz = d[rv, ru]
        PR = np.c_[(ru - cx) * rz / fx, (rv - cy) * rz / fy, rz]; BR = (T[:3, :3] @ PR.T).T + T[:3, 3]
        table_cloud = float(np.median(BR[:, 2]))
        diff = 100 * (table_cloud - a.table_z)
        print("table as seen by the camera around the cube: z = %+.3f m in base_link (%d points); the table box used by MoveIt is at z = %+.3f m (--table_z)" % (table_cloud, ring.sum(), a.table_z))
        if abs(diff) > 1.5:
            print("  -> MISMATCH of %.1f cm: the cloud puts the table %.1f cm %s than the box. One of them is wrong. If the box (physical table) is right, every closing point is %.1f cm too %s on the real arm."
                  % (abs(diff), abs(diff), "higher" if diff > 0 else "lower", abs(diff), "high" if diff > 0 else "low"))
        else:
            print("  -> they agree to within 1.5 cm.")

R = None
try:
    sys.path.insert(0, a.tools); import reach_check as R
except Exception as e:
    print("(reach check unavailable: %s - needs scipy and %s/reach_check.py)" % (e, a.tools))

rows = []
tips = []
print("\n #  tilt | closing point vs cube (cm)       | height  | pregrasp xyz (m)           radius | reach: pregrasp   grasp")
n_pre = 0
for i, T in enumerate(cands):
    p = T[:3, 3]; ax = T[:3, 2]; tilt = math.degrees(math.acos(float(np.clip(-ax[2], -1, 1))))
    close = p + CLOSE_DEPTH * ax; pg = p - pre * ax
    tips.append((i, tilt, float(p[2] + FINGER_TIP * ax[2])))
    off = "unknown" if centre is None else "dx %+5.1f dy %+5.1f  %+5.1f vs top" % (100 * (close[0] - centre[0]), 100 * (close[1] - centre[1]), 100 * (close[2] - ztop))
    side = "" if centre is None else ("  BESIDE" if math.hypot(close[0] - centre[0], close[1] - centre[1]) > 0.02 else "")
    if centre is not None:
        rows.append((100 * (close[0] - centre[0]), 100 * (close[1] - centre[1]), 100 * (close[2] - ztop), tilt))
    r_pre = r_gr = "n/a"
    if R is not None:
        cone = tilt + 5.0
        e1, _ = R.solve(pg, cone, starts=150); e2, _ = R.solve(p, cone, starts=150)
        def verdict(e):
            return "REACHABLE" if e < 0.003 else ("NOT (closest %.0f mm)" % (1000 * e) if e < 1e3 else "NOT (none within tilt)")
        r_pre, r_gr = verdict(e1), verdict(e2); n_pre += e1 < 0.003
    print("%2d %4.0f | %-32s| %6.1f  | (%+.3f, %+.3f, %+.3f)   %.3f | %-17s %s%s" % (i, tilt, off, 100 * (close[2] - a.table_z), pg[0], pg[1], pg[2], math.hypot(pg[0], pg[1]), r_pre, r_gr, side))
if R is not None:
    print("\n%d of %d pregrasp poses are kinematically reachable (optimistic: no collisions)." % (n_pre, len(cands)))
if tips:
    print("\nFINGER TIPS vs THE TABLE  (tip = hand base + %.3f m along the approach axis; negative = the tip would be BELOW the table surface)" % FINGER_TIP)
    print(" #  tilt | above the camera's table | above MoveIt's table box (z = %+.3f)" % a.table_z)
    for i, tilt, tz in tips:
        cam = "%+6.1f cm" % (100 * (tz - table_cloud)) if table_cloud is not None else "n/a"
        print("%2d %4.0f | %-25s | %+6.1f cm" % (i, tilt, cam, 100 * (tz - a.table_z)))
    if table_cloud is not None:
        worst = min(tz for _, _, tz in tips) - table_cloud
        if worst < 0.005:
            new = max(0.0, a.depth_below_top - (0.005 - worst))
            print("  -> WARNING: the lowest fingertip is %.1f cm %s the table the camera sees. If that table is real, the fingers hit it. MoveIt cannot warn you because its\n"
                  "     table box is %.1f cm %s than the camera's table. To keep every tip at least 0.5 cm above it, close less deep: recentre_depth_below_top_m <= %.3f (now %.3f):\n"
                  "     ros2 param set /contact_graspnet_node recentre_depth_below_top_m %.3f"
                  % (abs(100 * worst), "below" if worst < 0 else "above", abs(100 * (table_cloud - a.table_z)), "lower" if table_cloud < 0 or table_cloud > a.table_z else "higher", new, a.depth_below_top, new))
        else:
            print("  -> every fingertip clears the camera's table by at least 0.5 cm.")
if rows:
    A = np.array(rows)
    print("\nIS THE OFFSET A FIXED SHIFT OR SCATTER?  closing point minus cube centre, in base_link (cm; the centre is the middle of the visible points, good to about 1 cm):")
    def summ(sel, label):
        if sel.sum() == 0:
            return
        mu, sd = A[sel, :3].mean(0), A[sel, :3].std(0)
        print("  %-32s n=%2d  mean dx %+5.1f dy %+5.1f dz %+5.1f   spread dx %4.1f dy %4.1f dz %4.1f   horizontal shift %.1f cm" %
              (label, sel.sum(), mu[0], mu[1], mu[2], sd[0], sd[1], sd[2], math.hypot(mu[0], mu[1])))
    summ(np.ones(len(A), bool), "all candidates")
    summ(A[:, 3] <= 15, "near-vertical (tilt <= 15 deg)")
    summ(A[:, 3] > 15, "tilted (tilt > 15 deg)")
    if js.get("T_base_cam"):
        Rbc = np.array(js["T_base_cam"])[:3, :3]; vc = (Rbc.T @ (A[:, :3] / 100.0).T).T * 100.0; m = vc.mean(0)
        print("  same mean shift in the camera frame (x right, y down, z forward): (%+.1f, %+.1f, %+.1f) cm" % (m[0], m[1], m[2]))
    print("  Reading: spread much smaller than the mean = a FIXED shift (a bias); spread as big as the mean = scatter. If near-vertical and tilted\n"
          "  candidates show the same shift it is a plain translation; if only the tilted ones are shifted it is an error along the approach axis.\n"
          "  NB: this compares grasps with the cube inside the SAME point cloud, so hand-eye calibration cannot change it.")
