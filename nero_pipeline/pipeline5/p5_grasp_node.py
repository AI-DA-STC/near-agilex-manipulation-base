#!/usr/bin/env python3
"""
p5_grasp_node.py - PIPELINE 5 GRASP STEP (replaces Contact-GraspNet + pipeline 4's grasp step). Run with the system Python (ROS sourced, conda off);
start_pipeline5.sh starts it.

Service /p5/grasps (std_srvs/Trigger), called by p5_run.sh right after /graspmolmo/segment:
  1. takes THIS look's GraspMolmo result: /graspmolmo/grasp_point (3D) and /graspmolmo/mask_source (the object region), same frame/stamp,
     never the same one twice (a missing or old result -> refused, nothing published)
  2. the OBJECT MASK: SAM2 (pipeline 1's model, its own worker in the grounded_sam2 env) prompted with GraspMolmo's point - GraspMolmo's own region
     is only "everything raised within 8 cm of the point" and swallows sink walls and neighbours. If SAM2 is off/fails/implausible, GraspMolmo's
     region is used, but REFUSED when it fills its 8 cm search circle (merged with its surroundings). The mask used goes out on /p5/mask_source
     (the fusion node leaves exactly that object out of the wrist boxes).
     top-camera depth -> the object's points and the points around it (base_link); /p5/fused_points (both cameras) for obstacles
  3. p5_grasp_core.generate(): grasps for the Nero gripper (p5_gripper.py), tier 1 (straight down, tilt <= 20 deg across the jaws)
  4. publishes the best few on /p5/grasp_poses (PoseArray, base_link; pose_transform passes them to MTC in this order),
     the finger/body boxes on /p5/grasp_markers (RViz: green G1, yellow G2/G3, red = rejected), and writes
       ~/p3_runs/p5_grasp_latest.json   opening (read by pipeline 5's MTC copy), clearance (read by pipeline 5's picture), grasps,
                                        the gripper's checked corridor (read by the fusion node: no wrist boxes in it, p5_keepout.py)
       ~/p3_runs/p5_relay_latest.json   the candidates in the picture's report format
       ~/p3_runs/p5_snaps/<time>.npz    everything generate() used -> p5_replay.py reruns it offline (last 50 kept)
Service /p5/place (called by p5_run.sh right after /graspmolmo/place): GraspMolmo's place point -> SAM2 (the whole container) -> the nearest spot
  where the gripper fits at release (p5_place_core.py) -> /p5/mask_target (target_pose, the fusion node's landing zone and the picture follow it);
  the top camera's obstacle boxes are REBUILT around the new point (GraspMolmo's landing zone around its own point hid the wall next to it).
  P5_PLACE=off: GraspMolmo's point unchanged (still republished on /p5/mask_target).
Switches (set before start_pipeline5.sh): P5_MAX_TILT_DEG (20), P5_OPEN_MARGIN_M (0.02), P5_GRASPS_TO_MOVEIT (3), P5_OFFSET_M (0.01),
  P5_CLOUD_CHECK (on; off = no fused-cloud check), P5_TIP_CLEARANCE (m; unset = 25 % of the height, 6-10 mm),
  P5_MASK (sam2; depth = GraspMolmo's region only), P5_SAM2_MODEL (large | base_plus | small | tiny), P5_SAM2_REPO (Grounded-SAM-2 folder).
"""
import glob
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, "core")]
import p5_grasp_core as GC  # noqa: E402
import p5_gripper  # noqa: E402
import p5_keepout  # noqa: E402
import p5_place_core as PC  # noqa: E402
import p5_scene_objects as SO  # noqa: E402

P5_NODE_VERSION = "p5-grasp-node-v6e"   # 8d: three pictures in the same window: GraspMolmo, SAM2 + place, grasps | 8c: the picture window is reused (stays where dragged) | 8a: one picture pops up per look (after the place step; at the grasp step if it fails) |   # v6b (7f): SAM2 = its BEST-rated usable proposal - the ONLY change from 7e
# v6: v6: SAM2: the WHOLE object (largest usable proposal, score >= 0.3), not a tiny piece
# v5: v5: the object in hand (carrot) and the wrist camera box go into MoveIt's scene
# v4: v4: /p5/place - the place point moved to where the gripper fits, top boxes rebuilt around it
# v3: v3: writes the gripper's checked corridor (p5_keepout) for the wrist boxes
# v2: v2: SAM2 mask from GraspMolmo's point; depth region only as fallback, refused when merged
# v1: GraspMolmo's depth region
RUNS = os.path.expanduser("~/p3_runs")
REPORT = os.path.join(RUNS, "p5_grasp_latest.json")
RELAY_REPORT = os.path.join(RUNS, "p5_relay_latest.json")
SNAPS = os.path.join(RUNS, "p5_snaps")
KEEP_SNAPS = 50
GM_RADIUS_PX = 45            # = the GraspMolmo node's object_radius_px (its region never reaches farther from the point)
SAM_MAX_AREA_PX = 30000      # ~ 30 x 30 cm at the table: anything bigger is the sink / the table, not the object


# ---------------------------------------------------------------- pure helpers (tested offline)
def env_cfg(env=None):
    """Core settings from the environment -> (cfg dict, fixed clearance or None, cloud check on?)."""
    e = os.environ if env is None else env
    cfg = {"max_tilt_deg": float(e.get("P5_MAX_TILT_DEG", "20")), "open_margin": float(e.get("P5_OPEN_MARGIN_M", "0.02")),
           "n_out": max(1, int(e.get("P5_GRASPS_TO_MOVEIT", "3"))), "offset_m": float(e.get("P5_OFFSET_M", "0.01"))}
    if e.get("P5_MIN_CORNER_MM"):
        cfg["min_tip_corner"] = float(e["P5_MIN_CORNER_MM"]) / 1000.0
    if e.get("P5_RAISE_MM"):
        cfg["raise_steps"] = tuple(float(v) / 1000.0 for v in e["P5_RAISE_MM"].split(",") if v.strip())
    clr = e.get("P5_TIP_CLEARANCE", "").strip()
    return cfg, (float(clr) if clr else None), e.get("P5_CLOUD_CHECK", "on") != "off"


def quat_to_R(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 1e-12 else 0.0
    return np.array([[1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
                     [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
                     [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def R_to_quat(R):
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return tuple(float(v) for v in q / np.linalg.norm(q))


def pixels_to_base(us, vs, depth_m, K, R_bc, t_bc):
    """Pixel lists + depth (m) -> base_link points of the valid ones."""
    z = depth_m[vs, us].astype(np.float64)
    ok = np.isfinite(z) & (z > 0.05)
    us, vs, z = us[ok], vs[ok], z[ok]
    cam = np.stack([(us - K[0, 2]) * z / K[0, 0], (vs - K[1, 2]) * z / K[1, 1], z], axis=1)
    return cam @ np.asarray(R_bc).T + np.asarray(t_bc)


def gather_points(mask, depth_m, K, R_bc, t_bc, uv, radius_px=110, stride=2):
    """-> (object points, scene points around the object) in base_link. Scene = every stride-th pixel within radius_px of the grasp pixel
    (110 px ~ 19 cm at the table) plus every object pixel."""
    h, w = depth_m.shape
    vs, us = np.nonzero(np.asarray(mask) > 0)
    obj = pixels_to_base(us, vs, depth_m, K, R_bc, t_bc)
    u0, v0 = int(round(uv[0])), int(round(uv[1]))
    yy, xx = np.mgrid[max(0, v0 - radius_px):min(h, v0 + radius_px + 1):stride, max(0, u0 - radius_px):min(w, u0 + radius_px + 1):stride]
    sel = (xx - u0) ** 2 + (yy - v0) ** 2 <= radius_px ** 2
    scene = pixels_to_base(xx[sel].ravel(), yy[sel].ravel(), depth_m, K, R_bc, t_bc)
    return obj, np.vstack([scene, obj]) if len(obj) else scene


def project(p_cam, K):
    return (K[0, 0] * p_cam[0] / p_cam[2] + K[0, 2], K[1, 1] * p_cam[1] / p_cam[2] + K[1, 2])


def pick_result(results, last_served_stamp):
    """results: {stamp: {"mask":..., "point":...}} -> (stamp, entry) of the NEWEST complete one not served before, or (None, why)."""
    full = sorted(k for k, v in results.items() if "mask" in v and "point" in v)
    if not full:
        return None, "no GraspMolmo result with both a mask and a 3D point (no depth at its point?) - run the look again"
    k = full[-1]
    if last_served_stamp is not None and k <= last_served_stamp:
        return None, "GraspMolmo's latest result was already used - this needs a NEW look (/graspmolmo/segment first)"
    return k, results[k]


def region_merged(mask, uv, radius_px=GM_RADIUS_PX, frac_max=0.25):
    """GraspMolmo's depth region cannot reach farther than radius_px from its point. If much of its outline lies ON that circle, it was cut off
    there: merged with its surroundings (sink walls, a neighbour) or bigger than the circle. -> (merged?, fraction of the outline on the circle)."""
    m = np.asarray(mask) > 0
    if m.sum() < 10:
        return False, 0.0
    inner = m.copy()
    inner[1:-1, 1:-1] = m[1:-1, 1:-1] & m[:-2, 1:-1] & m[2:, 1:-1] & m[1:-1, :-2] & m[1:-1, 2:]
    vs, us = np.nonzero(m & ~inner)
    if len(us) == 0:
        return False, 0.0
    on = np.hypot(us - uv[0], vs - uv[1]) >= radius_px - 2.5
    f = float(on.mean())
    return f > frac_max, f


def choose_sam_mask(masks, scores, uv, max_area_px=SAM_MAX_AREA_PX, min_area_px=30, min_score=0.3):
    """SAM2's proposals -> (index, note): the WHOLE object - the LARGEST one that contains GraspMolmo's point (3x3 px), scores >= min_score, is not
    too big (the sink / table) or too small and does not hug the image border. v6: was the highest score, which picked a 178 px piece (score 0.03)
    of a 700 px eggplant. (None, why) if none qualifies."""
    u, v = int(round(uv[0])), int(round(uv[1]))
    ok, why = [], []
    for i, (m, sc) in enumerate(zip(masks, scores)):
        m = np.asarray(m) > 0
        h, w = m.shape
        area = int(m.sum())
        if not m[max(0, v - 1):v + 2, max(0, u - 1):u + 2].any():
            why.append("#%d misses the point" % i)
        elif area < min_area_px:
            why.append("#%d too small (%d px)" % (i, area))
        elif area > max_area_px:
            why.append("#%d too big (%d px: the sink/table?)" % (i, area))
        elif sc < min_score:
            why.append("#%d score %.2f < %.2f" % (i, sc, min_score))
        elif (m[0].sum() + m[-1].sum() + m[:, 0].sum() + m[:, -1].sum()) > 20:
            why.append("#%d touches the image border" % i)
        else:
            ok.append((area, sc, i))
    if not ok:
        return None, "no usable SAM2 proposal (%s)" % "; ".join(why)
    # 7f: the one SAM2 RATES BEST (a near tie within 0.05: the larger). 7e took the LARGEST usable one: that picked eggplant + sink ledge (82 mm)
    # and a sink-sized region (200+ mm). The tiny-piece problem it was meant to fix is the score >= min_score rule above.
    best = max(sc for _, sc, _ in ok)
    area, sc, i = max((a, s_, j) for a, s_, j in ok if s_ >= best - 0.05)
    return i, "SAM2 proposal #%d (best rated), score %.2f, %d px%s" % (i, sc, area, ("; skipped: " + "; ".join(why)) if why else "")


def choose_container_mask(masks, scores, uv, max_area_px=60000, min_score=0.5):
    """SAM2's proposals at the PLACE point -> (index, note): the LARGEST usable one (the whole basket, not a slat of it)."""
    u, v = int(round(uv[0])), int(round(uv[1]))
    ok, why = [], []
    for i, (m, sc) in enumerate(zip(masks, scores)):
        m = np.asarray(m) > 0
        area = int(m.sum())
        if not m[max(0, v - 1):v + 2, max(0, u - 1):u + 2].any():
            why.append("#%d misses the point" % i)
        elif area > max_area_px:
            why.append("#%d too big (%d px)" % (i, area))
        elif sc < min_score:
            why.append("#%d score %.2f" % (i, sc))
        elif (m[0].sum() + m[-1].sum() + m[:, 0].sum() + m[:, -1].sum()) > 20:
            why.append("#%d touches the image border" % i)
        else:
            ok.append((area, sc, i))
    if not ok:
        return None, "no usable SAM2 proposal for the container (%s)" % "; ".join(why)
    area, sc, i = max(ok)
    return i, "SAM2 container #%d, %d px, score %.2f" % (i, area, sc)


def image_bgr(msg):
    """sensor_msgs/Image (rgb8 / bgr8) -> HxWx3 BGR uint8, or None."""
    if msg.encoding not in ("rgb8", "bgr8"):
        return None
    a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return a[:, :, ::-1].copy() if msg.encoding == "rgb8" else a.copy()


VIEW_DIR = os.environ.get("P5_VIEW_DIR", "/tmp/p5_view")   # start_pipeline5.sh deletes it on Ctrl+C


def to_px(P, K, R_bc, t_bc):
    """base_link points -> pixels (and depth) of the camera whose pose in base_link is (R_bc, t_bc)."""
    cam = (np.asarray(P, float).reshape(-1, 3) - t_bc) @ np.asarray(R_bc)
    z = np.maximum(cam[:, 2], 1e-6)
    return np.stack([K[0, 0] * cam[:, 0] / z + K[0, 2], K[1, 1] * cam[:, 1] / z + K[1, 2]], 1), cam[:, 2]


def render_view(bgr, K, R_bc, t_bc, geom, mask=None, res=None, gpoint=None, lines=(), place=None, max_rejected=15):
    """The live view picture: the camera image with the object's outline, the grasps (G1 green, G2/G3 yellow, rejected red) as the fingers'
    outlines, GraspMolmo's points (cyan) and the place point used (magenta +), and text lines on a black strip on top. -> BGR image."""
    import cv2
    img = bgr.copy()
    if mask is not None:
        cs, _ = cv2.findContours((np.asarray(mask) > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, cs, -1, (80, 255, 80), 1)

    def fingers(c, col, th):
        bx = p5_gripper.boxes(geom, c.get("open") or c.get("own_open") or geom["OPEN_MAX"])
        for name, (lo, hi) in bx.items():
            if not name.startswith("finger"):
                continue
            corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]) @ np.asarray(c["R"]).T + c["tcp"]
            uv, _ = to_px(corners, K, R_bc, t_bc)
            cv2.polylines(img, [cv2.convexHull(uv.astype(np.int32))], True, col, th)

    if res is not None:
        sel = res.get("selected", [])
        rej = [c for c in res.get("candidates", []) if not c.get("ok") and "tcp" in c]
        for c in rej[:max_rejected]:
            fingers(c, (60, 60, 200), 1)
        hp = [p for c in rej for p in c.get("hit_pts", [])]
        if hp:                                                    # what blocked the fingers: red dots
            uv, _ = to_px(np.asarray(hp), K, R_bc, t_bc)
            for u, v in uv[:: max(1, len(uv) // 400)]:
                cv2.circle(img, (int(u), int(v)), 2, (0, 0, 255), -1)
        for c in sorted(sel, key=lambda x: -x["rank"]):
            fingers(c, (40, 220, 40) if c["rank"] == 1 else (0, 210, 240), 2 if c["rank"] == 1 else 1)
            uv, _ = to_px([c["tcp"] + np.asarray(c["R"])[:, 2] * geom["TIP_Z"]], K, R_bc, t_bc)
            cv2.putText(img, "G%d" % c["rank"], (int(uv[0][0]) + 8, int(uv[0][1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (40, 220, 40) if c["rank"] == 1 else (0, 210, 240), 1)
    if gpoint is not None:
        uv, _ = to_px([gpoint], K, R_bc, t_bc)
        cv2.circle(img, (int(uv[0][0]), int(uv[0][1])), 7, (255, 230, 0), 2)
    if place is not None:
        if place.get("container") is not None:
            cs, _ = cv2.findContours((np.asarray(place["container"]) > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(img, cs, -1, (200, 80, 200), 1)
        u, v = place["uv0"]
        cv2.line(img, (int(u) - 7, int(v) - 7), (int(u) + 7, int(v) + 7), (255, 230, 0), 2)
        cv2.line(img, (int(u) - 7, int(v) + 7), (int(u) + 7, int(v) - 7), (255, 230, 0), 2)
        if place.get("uv") is not None:
            u, v = place["uv"]
            cv2.line(img, (int(u) - 12, int(v)), (int(u) + 12, int(v)), (255, 0, 255), 2)
            cv2.line(img, (int(u), int(v) - 12), (int(u), int(v) + 12), (255, 0, 255), 2)
            cv2.putText(img, "PLACE", (int(u) + 10, int(v) + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)
    if res is not None:
        lines = list(lines) + ["green G1 / yellow G2,G3 / red rejected = fingers, red dots = what blocked them | cyan = GraspMolmo's points | magenta + = place"]
    strip = np.zeros((18 * len(lines) + 8, img.shape[1], 3), np.uint8)
    for i, t in enumerate(lines):
        col = (80, 80, 255) if t.startswith(("NO GRASP", "REFUSED", "WARNING")) else (255, 255, 255)
        cv2.putText(strip, t[:110], (6, 16 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
    return np.vstack([strip, img])


def write_view(img, name="view.png", view_dir=None):
    """Atomically (re)write the live view picture the 'Pipeline 5' window shows."""
    import cv2
    d = view_dir or VIEW_DIR
    os.makedirs(d, exist_ok=True)
    tmp = os.path.join(d, "." + name)
    cv2.imwrite(tmp + ".png", img)
    os.replace(tmp + ".png", os.path.join(d, name))


def popup(src, state, view_dir=None, opener=None):
    """8a/8c: ONE picture window for the looks. The first look opens it; later looks REPLACE the picture inside that same window (the image viewer
    reloads it in place), so it stays where it was dragged. If it was closed, a new one opens. P5_POPUP=off: none. -> the path shown, or None."""
    import shutil
    import subprocess
    if os.environ.get("P5_POPUP", "on") == "off" or not os.path.exists(src):
        return None
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return None
    d = view_dir or VIEW_DIR
    dst = os.path.join(d, "look.png")
    shutil.copyfile(src, dst)                                    # in place (not a rename): the open viewer sees a change and reloads
    proc = state.get("proc")
    alive = proc is not None and (getattr(proc, "poll", lambda: None)() is None)
    if alive:
        return dst
    if opener is None:
        viewer = next((v for v in ("eog", "xdg-open") if shutil.which(v)), None)
        if viewer is None:
            return None
        opener = lambda path: subprocess.Popen([viewer, path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    state["proc"] = opener(dst)
    return dst


def build_markers(res, gpoint, geom, max_rejected=12):
    """-> list of plain dicts (ns, id, type cube/sphere/text, position, quat, scale, rgba, text) in base_link, for RViz."""
    out = []
    sel = res.get("selected", [])
    rejected = [c for c in res.get("candidates", []) if not c.get("ok") and "tcp" in c][:max_rejected]
    sel_ids = {id(s) for s in sel}
    passed = [c for c in res.get("candidates", []) if c.get("ok") and id(c) not in sel_ids][:max_rejected]
    groups = [("G%d" % s["rank"], s, (0.1, 0.9, 0.2, 0.85) if s["rank"] == 1 else (1.0, 0.85, 0.1, 0.6), True) for s in sel]
    groups += [("passed", c, (0.3, 0.6, 1.0, 0.15), False) for c in passed]
    groups += [("rejected", c, (1.0, 0.15, 0.1, 0.18), False) for c in rejected]
    open_w = res.get("info", {}).get("open_width") or geom["OPEN_MAX"]
    for gi, (ns, c, rgba, with_body) in enumerate(groups):
        bx = p5_gripper.boxes(geom, c.get("open") or c.get("own_open") or open_w)
        q = R_to_quat(c["R"])
        for name, (lo, hi) in bx.items():
            if name == "body" and not with_body:
                continue
            lo, hi = np.asarray(lo), np.asarray(hi)
            centre = c["tcp"] + c["R"] @ (0.5 * (lo + hi))
            a = rgba if name != "body" else (rgba[0], rgba[1], rgba[2], 0.25)
            out.append({"ns": ns, "id": len(out), "type": "cube", "position": centre.tolist(), "quat": q, "scale": (hi - lo).tolist(), "rgba": a})
        if with_body:
            out.append({"ns": ns, "id": len(out), "type": "text", "position": (c["tcp"] + np.array([0, 0, 0.03])).tolist(), "quat": (0, 0, 0, 1),
                        "scale": [0, 0, 0.02], "rgba": (1, 1, 1, 1),
                        "text": "%s jaw %.0f tilt %+.0f open %.0f mm" % (ns, c["jaw_deg"], c["tilt_deg"], 1000 * (c.get("open") or 0))})
    if gpoint is not None:
        out.append({"ns": "graspmolmo_point", "id": len(out), "type": "sphere", "position": list(map(float, gpoint)), "quat": (0, 0, 0, 1),
                    "scale": [0.012, 0.012, 0.012], "rgba": (0.0, 1.0, 1.0, 1.0)})
    return out


def reports(res, clearance_note, stamp_str, now):
    """-> (p5_grasp_latest dict, p5_relay_latest dict). The first is what pipeline 5's MTC copy, picture and p5_execute.sh read."""
    info = res["info"]
    sel = res["selected"]
    g_rep = {"version": P5_NODE_VERSION, "core_version": GC.P5_GRASP_CORE_VERSION, "gripper_version": p5_gripper.P5_GRIPPER_VERSION,
             "time": stamp_str, "unix_time": now, "ok": res["ok"], "why": res["why"],
             "open_width_m": info.get("open_width"), "clearance_m": info.get("clearance"), "clearance_why": clearance_note,
             "object_height_m": info.get("height"), "floor_z": info.get("floor_z"), "floor_note": info.get("floor_note"),
             "max_tilt_deg": info["cfg"]["max_tilt_deg"], "margin_m": info["cfg"]["open_margin"],
             "grasps": [{"rank": s["rank"], "jaw_deg": round(s["jaw_deg"], 1), "tilt_from": round(s["tilt_deg"], 1), "tilt_to": round(s["tilt_deg"], 1),
                         "offset_mm": round(s["offset_mm"], 1), "width_m": s["width"], "open_m": s["open"], "lowest_tip_mm": round(s["tip_mm"], 1),
                         "free_mm": round(1000 * min(s["clear_m"], 0.05), 1), "cost": round(s["cost"], 3),
                         "terms": {k: round(v, 3) for k, v in s["terms"].items()}} for s in sel]}
    sel_ids = {id(s) for s in sel}
    order = sel + [c for c in res["candidates"] if id(c) not in sel_ids]
    r_rep = {"time": stamp_str, "cgn_grasps": len(res["candidates"]), "clearance": clearance_note, "grasps": []}
    for i, c in enumerate(order, 1):
        if id(c) in sel_ids:
            result = "PASSED to MoveIt as G%d: jaw %.0f deg, offset %+.0f mm, object %.0f mm -> opening %.0f mm" % (
                c["rank"], c["jaw_deg"], c["offset_mm"], 1000 * c["width"], 1000 * c["open"])
        elif c.get("ok"):
            result = "passed the checks, not sent (%s)" % c.get("note", "lower score or a near-duplicate")
        else:
            result = "rejected: jaw %.0f deg, offset %+.0f mm: %s" % (c["jaw_deg"], c["offset_mm"], c["why"])
        r_rep["grasps"].append({"rank": i, "tilt_deg": c.get("tilt_deg", 0.0), "roll_deg": 0.0,
                                "lowest_finger_mm": c.get("tip_mm", float("nan")) if c.get("ok") else float("nan"), "result": result})
    return g_rep, r_rep


def save_snapshot(path, obj, scene, fused, gpoint, table_z, clearance, cfg, geom, extra=None):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.savez_compressed(path, obj=np.asarray(obj, np.float32), scene=np.asarray(scene, np.float32),
                        fused=np.asarray(fused if fused is not None else np.zeros((0, 3)), np.float32), gpoint=np.asarray(gpoint, float),
                        table_z=float(table_z), clearance=np.nan if clearance is None else float(clearance),
                        meta=json.dumps({"cfg": cfg, "geom": geom, "extra": extra or {}, "node": P5_NODE_VERSION}))
    snaps = sorted(glob.glob(os.path.join(os.path.dirname(path), "p5_*.npz")))
    for old in snaps[:-KEEP_SNAPS]:
        try:
            os.remove(old)
        except OSError:
            pass


def load_snapshot(path):
    d = np.load(path, allow_pickle=False)
    meta = json.loads(str(d["meta"]))
    clr = float(d["clearance"])
    return {"obj": d["obj"].astype(float), "scene": d["scene"].astype(float), "fused": d["fused"].astype(float), "gpoint": d["gpoint"],
            "table_z": float(d["table_z"]), "clearance": None if math.isnan(clr) else clr, "cfg": meta["cfg"], "geom": meta["geom"],
            "extra": meta.get("extra", {})}


# ---------------------------------------------------------------- ROS node
def main():
    import rclpy
    import rclpy.duration
    import rclpy.time
    from geometry_msgs.msg import PointStamped, Pose, PoseArray
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker, MarkerArray
    import tf2_ros

    import tempfile
    import threading
    import cv2
    import p3_tablez
    from worker_client import Worker, WorkerError
    import p5_fuse_node as FU                 # cloud_xyz (pure numpy)
    import p5_wrist_obstacles as W            # image decoding

    class P5Grasp(Node):
        def __init__(self):
            super().__init__("p5_grasp")
            self.cfg, self.fixed_clearance, self.cloud_check = env_cfg()
            self.geom, self.geom_note = p5_gripper.load()
            self.results, self.last, self.last_served = {}, {}, None
            self.mask_mode = os.environ.get("P5_MASK", "sam2")
            self.sam, self.sam_state = None, "off (P5_MASK=%s)" % self.mask_mode
            if self.mask_mode == "sam2":
                repo = os.path.expanduser(os.environ.get("P5_SAM2_REPO", "~/near-agilex-manipulation-base/external/Grounded-SAM-2"))
                model = os.environ.get("P5_SAM2_MODEL", "large")
                self.sam = Worker(["conda", "run", "-n", os.environ.get("P5_SAM2_ENV", "grounded_sam2"), "--no-capture-output", "python", "-u",
                                   os.path.join(HERE, "p5_sam2_worker.py"), "--model", model, "--repo", repo],
                                  cwd=repo, log=lambda m: self.get_logger().info(m), startup_timeout=300.0)
                self.sam_state = "loading (SAM2.1 %s)" % model

                def warm():
                    try:
                        self.sam.start()
                        self.sam_state = "ready (SAM2.1 %s)" % model
                        self.get_logger().info("sam2 worker ready: %s" % self.sam.ready_info)
                    except WorkerError as e:
                        self.sam_state = "FAILED to start: %s" % e
                        self.get_logger().error("sam2 worker start-up failed: %s - GraspMolmo's region will be used (merged ones refused)" % e)
                threading.Thread(target=warm, daemon=True).start()
            cb_in, cb_srv = ReentrantCallbackGroup(), MutuallyExclusiveCallbackGroup()
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(Image, "/graspmolmo/mask_source", lambda m: self._result("mask", m), 10, callback_group=cb_in)
            self.create_subscription(PointStamped, "/graspmolmo/grasp_point", lambda m: self._result("point", m), 10, callback_group=cb_in)
            self.create_subscription(Image, "/graspmolmo/mask_target", lambda m: self._result("target", m), 10, callback_group=cb_in)
            self.create_subscription(Image, "/topcam/depth/image_raw", lambda m: self.last.__setitem__("depth", m), qos_profile_sensor_data,
                                     callback_group=cb_in)
            self.create_subscription(Image, "/topcam/color/image_raw", lambda m: self.last.__setitem__("color", m), qos_profile_sensor_data,
                                     callback_group=cb_in)
            self.create_subscription(CameraInfo, "/topcam/depth/camera_info", lambda m: self.last.__setitem__("info", m), qos_profile_sensor_data,
                                     callback_group=cb_in)
            self.create_subscription(PointCloud2, "/p5/fused_points", lambda m: self.last.update(fused=m, fused_t=time.time()), 1,
                                     callback_group=cb_in)
            self.pub = self.create_publisher(PoseArray, "/p5/grasp_poses", 10)
            self.pub_mk = self.create_publisher(MarkerArray, "/p5/grasp_markers", latched)
            self.pub_mask = self.create_publisher(Image, "/p5/mask_source", 10)
            self.pub_tgt = self.create_publisher(Image, "/p5/mask_target", 10)
            self.place_mode = os.environ.get("P5_PLACE", "on")
            self.popup_state = {}                                        # 8a: the picture window of the latest look
            self.look = {}                    # what /p5/grasps used, for /p5/place
            self.scene = None
            self.tf_buf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
            self.create_service(Trigger, "/p5/grasps", self._srv, callback_group=cb_srv)
            self.create_service(Trigger, "/p5/place", self._srv_place, callback_group=cb_srv)
            self.get_logger().info("%s + %s: object mask %s; tilt <= %.0f deg, margin %.0f mm, %d grasp(s) to MoveIt, fused-cloud check %s, clearance %s; gripper: %s" % (
                P5_NODE_VERSION, GC.P5_GRASP_CORE_VERSION, self.sam_state, self.cfg["max_tilt_deg"], 1000 * self.cfg["open_margin"], self.cfg["n_out"],
                "ON" if self.cloud_check else "OFF", "fixed %.1f mm" % (1000 * self.fixed_clearance) if self.fixed_clearance else "25 %% of height, 6-10 mm",
                self.geom_note))

        def _result(self, kind, m):
            k = (m.header.stamp.sec, m.header.stamp.nanosec)
            self.results.setdefault(k, {})[kind] = m
            for old in sorted(self.results)[:-5]:
                del self.results[old]

        def _srv(self, req, res):
            try:
                res.success, res.message = self.run()
            except Exception as e:  # a service must never die silently
                res.success, res.message = False, "%s: %s" % (type(e).__name__, e)
            # separate lines per level: rclpy raises "Logger severity cannot be changed between calls" if ONE line logs at two levels
            if res.success:
                self.get_logger().info(res.message)
            else:
                self.get_logger().error(res.message)
            return res

        def _srv_place(self, req, res):
            try:
                res.success, res.message = self.run_place()
            except Exception as e:  # a service must never die silently
                res.success, res.message = False, "%s: %s" % (type(e).__name__, e)
            if res.success:
                self.get_logger().info(res.message)
            else:
                self.get_logger().error(res.message)
            return res

        def robot_points(self):
            """Positions of the arm's own TF frames in base_link (the top camera's obstacle filter, as GraspMolmo's node does it)."""
            import yaml
            pts = []
            try:
                frames = yaml.safe_load(self.tf_buf.all_frames_as_yaml()) or {}
            except Exception:
                frames = {}
            for f in frames:
                if any(x in f for x in ("topcam", "camera", "base_link", "world", "map", "odom")):
                    continue
                try:
                    q = self.tf_buf.lookup_transform("base_link", f, rclpy.time.Time()).transform.translation
                    pts.append((q.x, q.y, q.z))
                except Exception:
                    continue
            return pts

        def run_place(self):
            t0 = time.time()
            L = self.look
            if not L or self.last_served is None or L.get("open") is None:
                return False, "REFUSED: no grasps from this look (/p5/grasps first, and it must have succeeded)"
            r = None
            while time.time() - t0 < 5.0:
                r = self.results.get(self.last_served, {}).get("target")
                if r is not None:
                    break
                time.sleep(0.1)
            if r is None:
                return False, "REFUSED: no GraspMolmo place disc for this look (/graspmolmo/place first)"
            tmask = W.image_to_array(r, np.uint8)
            vs, us = np.nonzero(tmask > 0)
            if len(us) == 0:
                return False, "REFUSED: GraspMolmo's place disc is empty"
            uv0 = (float(us.mean()), float(vs.mean()))
            new_mask, note, cmask = tmask, "GraspMolmo's place point unchanged (P5_PLACE=off)", None
            rep = {"version": P5_NODE_VERSION, "unix_time": time.time(), "uv0": uv0, "mode": self.place_mode}
            if self.place_mode != "off":
                cnote = "SAM2 off"
                if self.sam is not None and self.sam_state.startswith("ready") and L.get("bgr") is not None:
                    d = tempfile.mkdtemp(prefix="p5_sam2_place_")
                    img = os.path.join(d, "rgb.png")
                    cv2.imwrite(img, L["bgr"])
                    try:
                        sr = self.sam.request({"cmd": "segment", "image": img, "point": [uv0[0], uv0[1]], "out_dir": d}, timeout=60.0)
                    except WorkerError as e:
                        sr = {"ok": False, "error": str(e)}
                    if sr.get("ok"):
                        masks = [cv2.imread(pth, cv2.IMREAD_GRAYSCALE) for pth in sr["masks"]]
                        i, cnote = choose_container_mask(masks, sr["scores"], uv0)
                        cmask = masks[i] if i is not None else None
                    else:
                        cnote = "SAM2 failed: %s" % sr.get("error", "?")[:150]
                pr = PC.choose_place(L["depth"], L["K"], L["R"], L["t"], uv0, cmask, self.geom, L["open"], L["jaw"])
                rep.update({k: v for k, v in pr.items() if k not in ("xyz0",)}, container_note=cnote)
                if pr.get("uv") is None:
                    note = "place point UNCHANGED: %s (%s)" % (pr["why"], cnote)
                else:
                    new_mask = PC.place_disc(tmask.shape, pr["uv"])
                    note = "%s%s; release jaw %.0f deg fits; %s; %s" % (
                        "" if pr["ok"] else "WARNING: ", pr["why"], pr["jaw_deg"], pr.get("container", ""), cnote)
                    # the top camera's boxes again, around the NEW point (GraspMolmo's landing zone around its own point hid the wall next to it)
                    try:
                        import p3_obstacles
                        import p3_scene
                        boxes = p3_obstacles.obstacle_boxes(L["depth"], L["K"], L["R"], L["t"], p3_tablez.table_z()[0], target_mask=L["mask"] > 0,
                                                            place_xyz=pr["xyz"], robot_points=self.robot_points())
                        if self.scene is None:
                            self.scene = p3_scene.AsyncScene(timeout=8.0)
                        ok, snote = self.scene.set_obstacles(boxes)
                        note += "; top-camera boxes rebuilt around it: %s (%s)" % (p3_obstacles.describe(boxes), snote if ok else "NOT CONFIRMED: " + snote)
                        rep["top_boxes"] = len(boxes)
                    except Exception as e:
                        note += "; top-camera boxes NOT rebuilt (%s: %s) - GraspMolmo's stay" % (type(e).__name__, e)
            m = Image()
            m.header = r.header
            m.height, m.width, m.encoding, m.step = new_mask.shape[0], new_mask.shape[1], "mono8", new_mask.shape[1]
            m.data = ((new_mask > 0).astype(np.uint8) * 255).tobytes()
            self.pub_tgt.publish(m)                                       # -> target_pose (plate point), the fusion node (landing zone)
            try:
                if L.get("bgr") is not None:
                    pl = {"uv0": uv0, "uv": rep.get("uv") or uv0, "container": cmask}
                    write_view(render_view(L["bgr"], L["K"], L["R"], L["t"], self.geom, L["mask"], L.get("res"), L.get("gpoint"),
                                           L.get("view_lines", []) + [("PLACE: " if not note.startswith("WARNING") else "") + note[:140]], pl))
                    # 8d: picture 2/3 - SAM2's object and the place, now; picture 3/3 - the grasps (+ place) 4 s later, in the same window
                    info = (L.get("res") or {}).get("info", {})
                    l2 = ["2/3  SAM2: the object (green outline)%s; %s" % (
                              (", %.0f mm tall" % (1000 * info["height"])) if info.get("height") else "", L.get("mask_note", "")[:60]),
                          "place: GraspMolmo's point (cyan x) -> used (magenta +), the container (purple outline)",
                          ("PLACE: " if not note.startswith("WARNING") else "") + note[:120]]
                    write_view(render_view(L["bgr"], L["K"], L["R"], L["t"], self.geom, L["mask"], None, L.get("gpoint"), l2, pl), "stage2.png")
                    popup(os.path.join(VIEW_DIR, "stage2.png"), self.popup_state)
                    threading.Timer(4.0, lambda: popup(os.path.join(VIEW_DIR, "view.png"), self.popup_state)).start()
                    self.get_logger().info("pictures: 2/3 now, 3/3 in 4 s")
            except Exception as e:
                self.get_logger().warn("live view not drawn: %s" % e)
            rep["note"] = note
            try:
                json.dump(rep, open(os.path.join(RUNS, "p5_place_latest.json"), "w"), indent=1, default=float)
            except Exception as e:
                self.get_logger().warn("place report not written: %s" % e)
            return True, "%s (%.1f s)" % (note, time.time() - t0)

        def tf(self, frame):
            tr = self.tf_buf.lookup_transform("base_link", frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
            q, t = tr.transform.rotation, tr.transform.translation
            return quat_to_R(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])

        def run(self):
            t_start = time.time()
            k, r = None, None
            while time.time() - t_start < 5.0:                           # the topics may arrive just after /graspmolmo/segment returned
                k, r = pick_result(self.results, self.last_served)
                if k is not None:
                    break
                time.sleep(0.2)
            if k is None:
                return False, "REFUSED: " + r
            d, info = self.last.get("depth"), self.last.get("info")
            if d is None or info is None:
                return False, "REFUSED: no top-camera depth / camera_info"
            gm_mask = W.image_to_array(r["mask"], np.uint8)
            depth = W.depth_to_m(d)
            if gm_mask.shape != depth.shape:
                return False, "REFUSED: mask %s != depth %s (registered depth needed)" % (gm_mask.shape, depth.shape)
            K = np.array(info.k, dtype=float).reshape(3, 3)
            frame = r["mask"].header.frame_id or info.header.frame_id
            R_bc, t_bc = self.tf(frame)
            pc = r["point"].point
            p_cam = np.array([pc.x, pc.y, pc.z])
            gpoint = R_bc @ p_cam + t_bc
            uv = project(p_cam, K)
            try:                                                          # 8d: picture 1/3 - GraspMolmo's pick point, at once
                bgr1 = image_bgr(self.last["color"]) if "color" in self.last else None
                if bgr1 is not None:
                    p1 = os.path.join(VIEW_DIR, "stage1.png")
                    write_view(render_view(bgr1, K, R_bc, t_bc, self.geom, gm_mask, None, gpoint,
                                           ["1/3  GraspMolmo: its pick point (cyan) and its rough region (green)",
                                            "next: SAM2 cuts out the object, the grasps, then the place point"]), "stage1.png")
                    popup(p1, self.popup_state)
            except Exception as e:
                self.get_logger().warn("picture 1 not shown: %s" % e)
            mask, mask_note = self.object_mask(gm_mask, uv)
            if mask is None:
                self.last_served = k
                return False, "REFUSED: " + mask_note
            mmsg = Image()
            mmsg.header = r["mask"].header
            mmsg.height, mmsg.width, mmsg.encoding, mmsg.step = mask.shape[0], mask.shape[1], "mono8", mask.shape[1]
            mmsg.data = ((mask > 0).astype(np.uint8) * 255).tobytes()
            self.pub_mask.publish(mmsg)                                   # -> the fusion node leaves exactly this object out of the wrist boxes
            obj, scene = gather_points(mask, depth, K, R_bc, t_bc, uv)
            self.look = {"depth": depth, "K": K, "R": R_bc, "t": t_bc, "mask": mask,
                         "bgr": image_bgr(self.last["color"]) if "color" in self.last else None, "open": None, "jaw": None}
            fused, fused_note = None, "off (P5_CLOUD_CHECK=off)"
            if self.cloud_check:
                t0 = time.time()
                while time.time() - self.last.get("fused_t", 0) > 3.0 and time.time() - t0 < 3.0:
                    time.sleep(0.2)
                if time.time() - self.last.get("fused_t", 0) <= 3.0:
                    fused = FU.cloud_xyz(self.last["fused"])
                    fused_note = "%d voxels" % len(fused)
                else:
                    fused_note = "NOT CHECKED: no fresh /p5/fused_points (is p5_fuse_node running?)"
            table_z = p3_tablez.table_z()[0]
            res = GC.generate(obj, scene, fused, gpoint, table_z, self.fixed_clearance, cfg=self.cfg, geom=self.geom)
            self.last_served = k
            now, stamp_str = time.time(), time.strftime("%Y-%m-%d %H:%M:%S")
            clr_note = ("fixed P5_TIP_CLEARANCE %.1f mm" % (1000 * self.fixed_clearance) if self.fixed_clearance else
                        "25 %% of the %.0f mm local height, 6-10 mm" % (1000 * res["info"].get("height", 0)) if "height" in res["info"] else "?")
            snap = os.path.join(SNAPS, "p5_%s.npz" % time.strftime("%Y%m%d_%H%M%S"))
            try:
                save_snapshot(snap, obj, scene, fused, gpoint, table_z, self.fixed_clearance, self.cfg, self.geom,
                              {"fused_note": fused_note, "frame": frame, "mask_note": mask_note})
            except Exception as e:
                self.get_logger().warn("snapshot not saved: %s" % e)
            g_rep, r_rep = reports(res, clr_note, stamp_str, now)
            g_rep["fused_check"], g_rep["snapshot"], g_rep["gripper"], g_rep["mask"] = fused_note, snap, self.geom_note, mask_note
            if res["ok"]:
                g_rep["keepout"] = p5_keepout.corridor(res["selected"], self.geom, res["info"]["open_width"])
            try:
                os.makedirs(RUNS, exist_ok=True)
                if res["ok"]:
                    json.dump(g_rep, open(REPORT, "w"), indent=1)
                elif os.path.exists(REPORT):
                    os.remove(REPORT)                                     # never leave an opening from an earlier look
                json.dump(r_rep, open(RELAY_REPORT, "w"), indent=1, default=float)
            except Exception as e:
                self.get_logger().warn("report not written: %s" % e)
            self.publish_markers(build_markers(res, gpoint, self.geom))
            try:
                if self.look.get("bgr") is not None:
                    if res["ok"]:
                        g1 = res["selected"][0]
                        head = ["3/3  GRASP: G1 jaw %.0f deg, tilt %+.0f, object %.0f mm -> opening %.0f mm, fingertips %.0f mm above the floor" % (
                            g1["jaw_deg"], g1["tilt_deg"], 1000 * g1["width"], 1000 * g1["open"], g1["tip_mm"])]
                    else:
                        head = ["NO GRASP: " + res["why"][:150]]   # (3/3, shown at once)
                    head.append("mask: " + mask_note[:100])
                    self.look["view_lines"] = head
                    self.look["res"], self.look["gpoint"] = res, gpoint
                    write_view(render_view(self.look["bgr"], K, R_bc, t_bc, self.geom, mask, res, gpoint, head))
                    self.look["mask_note"] = mask_note
                    if not res["ok"]:                                    # 8a: no grasp -> no place step: show WHY now (picture 3)
                        popup(os.path.join(VIEW_DIR, "view.png"), self.popup_state)
            except Exception as e:
                self.get_logger().warn("live view not drawn: %s" % e)
            self.get_logger().info("object mask: " + mask_note)
            for line in GC.summary(res, 8):
                self.get_logger().info(line)
            if not res["ok"]:
                return False, "NO GRASP: %s (fused cloud: %s; snapshot %s)" % (res["why"], fused_note, snap)
            out = PoseArray()
            out.header.frame_id = "base_link"
            out.header.stamp = self.get_clock().now().to_msg()
            for s in res["selected"]:
                p = Pose()
                p.position.x, p.position.y, p.position.z = (float(v) for v in s["tcp"])
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = R_to_quat(s["R"])
                out.poses.append(p)
            self.pub.publish(out)
            g1 = res["selected"][0]
            self.look.update(open=res["info"]["open_width"], jaw=g1["jaw_deg"])
            so_note = self.scene_objects(obj, res["info"]["floor_z"])
            return True, ("%d grasp(s) to MoveIt; G1 jaw %.0f deg, tilt %+.0f deg, object %.0f mm -> opening %.0f mm, fingertips %.0f mm above the floor; "
                          "object %.0f mm tall (%s); mask: %s; fused cloud: %s; %s; %.2f s" % (
                              len(res["selected"]), g1["jaw_deg"], g1["tilt_deg"], 1000 * g1["width"], 1000 * g1["open"], g1["tip_mm"],
                              1000 * res["info"]["height"], res["info"]["floor_note"], mask_note.split(";")[0], fused_note, so_note, time.time() - t_start))

        def object_mask(self, gm_mask, uv):
            """-> (mask, note) or (None, why). SAM2 first; GraspMolmo's depth region as fallback, refused when merged with its surroundings."""
            fallback = ""
            if self.sam is not None and self.sam_state.startswith("ready"):
                bgr = image_bgr(self.last["color"]) if "color" in self.last else None
                if bgr is None or bgr.shape[:2] != gm_mask.shape:
                    fallback = "SAM2 not used: no colour image of the depth's size"
                else:
                    d = tempfile.mkdtemp(prefix="p5_sam2_")
                    img = os.path.join(d, "rgb.png")
                    cv2.imwrite(img, bgr)
                    try:
                        rep = self.sam.request({"cmd": "segment", "image": img, "point": [float(uv[0]), float(uv[1])], "out_dir": d}, timeout=60.0)
                    except WorkerError as e:
                        rep = {"ok": False, "error": str(e)}
                    if rep.get("ok"):
                        masks = [cv2.imread(pth, cv2.IMREAD_GRAYSCALE) for pth in rep["masks"]]
                        i, note = choose_sam_mask(masks, rep["scores"], uv)
                        if i is not None:
                            return masks[i], note + ", %.1f s" % rep["seconds"]
                        fallback = note
                    else:
                        fallback = "SAM2 failed: %s" % rep.get("error", "?")[:200]
            elif self.sam is not None:
                fallback = "SAM2 %s" % self.sam_state
            else:
                fallback = "SAM2 off (P5_MASK=%s)" % self.mask_mode
            merged, f = region_merged(gm_mask, uv)
            if merged:
                return None, ("%s; GraspMolmo's own region is cut off by its %d px search circle on %.0f %% of its outline: merged with its surroundings "
                              "(sink walls, a neighbour) or bigger than 16 cm - not grasped" % (fallback, GM_RADIUS_PX, 100 * f))
            return gm_mask, "%s -> GraspMolmo's depth region (%d px, %.0f %% of its outline on the search circle)" % (fallback, int((gm_mask > 0).sum()), 100 * f)

        def scene_objects(self, obj, floor_z):
            """The object in hand (carrot, as slice boxes) and the wrist camera box -> MoveIt; once, the camera box vs the calibration."""
            cam = None
            try:
                import p3_scene
                if self.scene is None:
                    self.scene = p3_scene.AsyncScene(timeout=8.0)
                slices = (SO.object_slices(obj, floor_z, table_z=p3_tablez.table_z()[0])        # 7k: never into MoveIt's table box
                          if os.environ.get("P5_OBJECT_BOX", "on") != "off" else [])
                cam = SO.camera_box(self.geom) if os.environ.get("P5_CAMERA_BOX", "on") != "off" else None
                note = SO.apply(self.scene, slices, cam)
            except Exception as e:
                note = "scene objects NOT added (%s: %s)" % (type(e).__name__, e)
            if cam is not None and not getattr(self, "cam_checked", False):
                self.cam_checked = True
                try:
                    tr = self.tf_buf.lookup_transform("tcp_link", os.environ.get("P5_WRISTCAM_FRAME", "armcam_link"), rclpy.time.Time(),
                                                      timeout=rclpy.duration.Duration(seconds=1.0))
                    p = np.array([tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z])
                    inside = SO.inside_box(p, cam, grow=0.02)
                    msg = "the calibrated wrist camera is at (%+.0f, %+.0f, %+.0f) mm in tcp_link: %s" % (
                        1000 * p[0], 1000 * p[1], 1000 * p[2], "inside the 8 cm camera box" if inside else
                        "OUTSIDE the 8 cm camera box - the box may be on the wrong side (send this line)")
                    if inside:
                        self.get_logger().info(msg)
                    else:
                        self.get_logger().warn(msg)
                    note += "; " + msg
                except Exception as e:
                    self.get_logger().warn("camera box not checked against the calibration (%s)" % e)
            self.get_logger().info(note)
            return note

        def publish_markers(self, mks):
            arr = MarkerArray()
            clear = Marker()
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)
            for m in mks:
                k = Marker()
                k.header.frame_id = "base_link"
                k.header.stamp = self.get_clock().now().to_msg()
                k.ns, k.id = m["ns"], int(m["id"])
                k.type = {"cube": Marker.CUBE, "sphere": Marker.SPHERE, "text": Marker.TEXT_VIEW_FACING}[m["type"]]
                k.action = Marker.ADD
                k.pose.position.x, k.pose.position.y, k.pose.position.z = (float(v) for v in m["position"])
                k.pose.orientation.x, k.pose.orientation.y, k.pose.orientation.z, k.pose.orientation.w = (float(v) for v in m["quat"])
                k.scale.x, k.scale.y, k.scale.z = (max(1e-3, float(v)) for v in m["scale"])
                k.color.r, k.color.g, k.color.b, k.color.a = (float(v) for v in m["rgba"])
                k.text = m.get("text", "")
                arr.markers.append(k)
            self.pub_mk.publish(arr)

    rclpy.init()
    node = P5Grasp()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
