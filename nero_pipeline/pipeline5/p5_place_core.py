#!/usr/bin/env python3
"""p5_place_core.py - PIPELINE 5: move GraspMolmo's place point to where the gripper fits (no ROS in here, tested offline).

Why: GraspMolmo's "center of the basket" point sometimes lands at the basket's wall. The obstacle steps keep a 6 cm landing zone free around the
place point, so that wall was invisible to MoveIt and the gripper pushed the basket (and MTC searched long for a release pose).

How (top camera, base_link, metres):
  container  SAM2 clicked at GraspMolmo's place point (the whole basket); without it: the floor-level surface connected to the point
  floor      20th percentile height of the container = its inside floor
  candidates floor-level points of the container within shift_max of GraspMolmo's point
  fits?      at release the fingertips are release_tip above that floor and the fingers open to the grasp opening; MTC releases with the
             yaw within +-60 deg of the grasp's (P2_PLACE_YAW=grasp), so a spot fits if SOME yaw in that range is clear:
               jaw span: nothing higher than the fingertips within opening/2 + finger thickness along the jaw, finger width across it
                         (tapered: a point s above the fingertips meets only the finger parts below s)
               body:     nothing higher than the body's underside under the 165 x 55 mm body
  choice     the nearest candidate that fits (GraspMolmo's point itself if it fits); none fits -> the least blocked one, flagged
"""
import math

import numpy as np

P5_PLACE_CORE_VERSION = "p5-place-core-v3"   # v3: the camera box where it really is (beside the gripper) | v2: camera counts
# v1

CFG = {
    "shift_max": 0.08,      # m: the place point may move this far from GraspMolmo's point
    "floor_pct": 20.0,
    "floor_band": 0.015,    # candidates: within this of the container's floor
    "release_tip": 0.010,   # fingertips this far above the floor at release
    "margin": 0.010,
    "max_hits": 3,          # depth noise
    "cand_step_px": 3,
    "max_candidates": 400,
    "yaw_step_deg": 15.0,
    "yaw_range_deg": 60.0,  # = MTC's P2_PLACE_YAW_RANGE_DEG (release yaw around the grasp's)
}


def points_px(depth_m, K, R_bc, t_bc, us, vs):
    z = depth_m[vs, us].astype(np.float64)
    ok = np.isfinite(z) & (z > 0.05)
    us, vs, z = us[ok], vs[ok], z[ok]
    cam = np.stack([(us - K[0, 2]) * z / K[0, 0], (vs - K[1, 2]) * z / K[1, 1], z], 1)
    return cam @ np.asarray(R_bc).T + np.asarray(t_bc), us, vs


def lift_px(depth_m, K, R_bc, t_bc, uv, r=4):
    h, w = depth_m.shape
    u, v = int(round(uv[0])), int(round(uv[1]))
    vv, uu = np.mgrid[max(0, v - r):min(h, v + r + 1), max(0, u - r):min(w, u + r + 1)]
    P, _, _ = points_px(depth_m, K, R_bc, t_bc, uu.ravel(), vv.ravel())
    return None if len(P) == 0 else np.median(P, axis=0)


def fallback_container(depth_m, K, R_bc, t_bc, uv0, radius_px, band):
    """No SAM2 mask: the floor-level pixels CONNECTED to the place point (4-neighbours, height within `band` of the point's local floor)."""
    h, w = depth_m.shape
    u0, v0 = int(round(uv0[0])), int(round(uv0[1]))
    v_lo, v_hi, u_lo, u_hi = max(0, v0 - radius_px), min(h, v0 + radius_px + 1), max(0, u0 - radius_px), min(w, u0 + radius_px + 1)
    vv, uu = np.mgrid[v_lo:v_hi, u_lo:u_hi]
    Z = np.full(vv.shape, np.nan)
    P, us, vs = points_px(depth_m, K, R_bc, t_bc, uu.ravel(), vv.ravel())
    Z[vs - v_lo, us - u_lo] = P[:, 2]
    near = np.hypot(uu - u0, vv - v0) <= 10
    zf = np.nanpercentile(Z[near], 30) if np.isfinite(Z[near]).any() else np.nan
    m = np.zeros(depth_m.shape, bool)
    if not np.isfinite(zf):
        return m
    ok = np.abs(Z - zf) <= band
    seen = np.zeros(ok.shape, bool)
    s = np.argwhere(ok & near)
    if not len(s):
        return m
    stack = [tuple(s[np.argmin(np.hypot(s[:, 0] - (v0 - v_lo), s[:, 1] - (u0 - u_lo)))])]
    while stack:
        a, b = stack.pop()
        if seen[a, b] or not ok[a, b]:
            continue
        seen[a, b] = True
        for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            c, d = a + da, b + db
            if 0 <= c < ok.shape[0] and 0 <= d < ok.shape[1] and not seen[c, d] and ok[c, d]:
                stack.append((c, d))
    m[v_lo:v_hi, u_lo:u_hi] = seen
    return m


def footprint_hits(F, B, c, theta, open_w, geom, margin, z_tip=None, Cm=None):
    """Points the gripper would touch at release (or on its straight way down to it), jaw line at compass angle theta through c:
      fingers: a point s above the fingertips only ever meets the finger parts BELOW s (the parts above never come down that far), so the finger's
               width and thickness AT s count (tapered, measured) - along the jaw opening/2 + thickness, across the width; the object between the
               fingers counts too. F = points above the fingertips (z_tip given: tapered; None: the body-end sizes everywhere, conservative)
      body:    the 165 x 55 mm body vs B (points above the body's underside)."""
    j = np.array([math.cos(theta), math.sin(theta)])
    n = np.array([-j[1], j[0]])
    hits = 0
    if len(F):
        d = F[:, :2] - c[:2]
        if z_tip is None:
            wa, tb = np.full(len(F), geom["FINGER_W_ROOT"]), np.full(len(F), geom["FINGER_T_ROOT"])
        else:
            f = np.clip((F[:, 2] - z_tip) / (geom["TIP_Z"] - geom["FINGER_ROOT_Z"]), 0.0, 1.0)
            wa = geom["FINGER_W_TIP"] + f * (geom["FINGER_W_ROOT"] - geom["FINGER_W_TIP"])
            tb = geom["FINGER_T_TIP"] + f * (geom["FINGER_T_ROOT"] - geom["FINGER_T_TIP"])
        hits += int(((np.abs(d @ j) <= 0.5 * open_w + tb + margin) & (np.abs(d @ n) <= 0.5 * wa + margin)).sum())
    if len(B):
        d = B[:, :2] - c[:2]
        hits += int(((np.abs(d @ j) <= 0.5 * geom["BODY_Y"] + margin) & (np.abs(d @ n) <= 0.5 * geom["BODY_X"] + margin)).sum())
    if Cm is not None and len(Cm) and geom.get("CAM_SZ", 0) > 0:         # the wrist camera box: tcp_link x = n, y = j for a straight-down hand;
        d = Cm[:, :2] - (c[:2] + geom["CAM_CX"] * n + geom["CAM_CY"] * j)  # Cm = points above the box's underside
        hits += int(((np.abs(d @ j) <= 0.5 * geom["CAM_SY"] + margin) & (np.abs(d @ n) <= 0.5 * geom["CAM_SX"] + margin)).sum())
    return hits


def choose_place(depth_m, K, R_bc, t_bc, uv0, container_mask, geom, open_w, grasp_jaw_deg=None, cfg=None, window_px=130):
    """-> dict: ok (fits), uv, xyz (base_link), shift_m, jaw_deg (a release jaw angle that fits), hits, floor_z, why/notes.
    grasp_jaw_deg: MTC releases with the yaw within +-yaw_range of the grasp's (P2_PLACE_YAW=grasp); None = any yaw."""
    C = dict(CFG, **(cfg or {}))
    out = {"version": P5_PLACE_CORE_VERSION, "uv0": [float(uv0[0]), float(uv0[1])]}
    p0 = lift_px(depth_m, K, R_bc, t_bc, uv0)
    if p0 is None:
        return dict(out, ok=False, uv=None, why="no depth at GraspMolmo's place point")
    out["xyz0"] = p0.tolist()
    h, w = depth_m.shape
    u0, v0 = int(round(uv0[0])), int(round(uv0[1]))
    vv, uu = np.mgrid[max(0, v0 - window_px):min(h, v0 + window_px + 1):2, max(0, u0 - window_px):min(w, u0 + window_px + 1):2]
    S, _, _ = points_px(depth_m, K, R_bc, t_bc, uu.ravel(), vv.ravel())
    cm = np.asarray(container_mask) > 0 if container_mask is not None else None
    if cm is None or cm.sum() < 50:
        cm = fallback_container(depth_m, K, R_bc, t_bc, uv0, window_px, C["floor_band"])
        out["container"] = "floor-level surface connected to the point (%d px; no SAM2 mask)" % cm.sum()
    else:
        out["container"] = "SAM2 mask (%d px)" % cm.sum()
    cv_, cu_ = np.nonzero(cm)
    if len(cu_) < 20:
        return dict(out, ok=False, uv=None, why="no container surface around the place point")
    sel = (cu_ % C["cand_step_px"] == 0) & (cv_ % C["cand_step_px"] == 0)
    Pc, cu, cv = points_px(depth_m, K, R_bc, t_bc, cu_[sel], cv_[sel])
    if len(Pc) < 10:
        return dict(out, ok=False, uv=None, why="no depth on the container")
    floor = float(np.percentile(Pc[:, 2], C["floor_pct"]))
    out["floor_z"] = floor
    dist = np.hypot(Pc[:, 0] - p0[0], Pc[:, 1] - p0[1])
    near = (np.abs(Pc[:, 2] - floor) <= C["floor_band"]) & (dist <= C["shift_max"])
    Pc, cu, cv, dist = Pc[near], cu[near], cv[near], dist[near]
    if len(Pc) == 0:
        return dict(out, ok=False, uv=None, why="no floor-level container surface within %.0f cm of GraspMolmo's point" % (100 * C["shift_max"]))
    order = np.argsort(dist)
    if len(order) > C["max_candidates"]:                                 # keep the nearest ones densely, thin out the rest
        order = order[np.unique(np.round(np.linspace(0, len(order) - 1, C["max_candidates"])).astype(int))]
    z_tip = floor + C["release_tip"]
    z_body = z_tip + (geom["TIP_Z"] - geom["FINGER_ROOT_Z"])
    reach = C["shift_max"] + 0.5 * geom["BODY_Y"] + 0.03
    S = S[np.hypot(S[:, 0] - p0[0], S[:, 1] - p0[1]) <= reach]
    F, B = S[S[:, 2] > z_tip], S[S[:, 2] > z_body]
    Cm = S[S[:, 2] > z_tip + geom["TIP_Z"] - (geom["CAM_CZ"] + 0.5 * geom["CAM_SZ"])] if geom.get("CAM_SZ", 0) > 0 else None
    if grasp_jaw_deg is None:
        yaws = list(np.arange(0.0, 180.0, C["yaw_step_deg"]))
    else:
        k = int(C["yaw_range_deg"] // C["yaw_step_deg"])
        yaws = [grasp_jaw_deg] + [grasp_jaw_deg + sgn * i * C["yaw_step_deg"] for i in range(1, k + 1) for sgn in (1, -1)]
    out.update(z_tip=z_tip, z_body=z_body, yaws_tried=len(yaws), candidates=int(len(order)))
    best = None
    for i in order:
        for yd in yaws:
            n = footprint_hits(F, B, Pc[i], math.radians(yd), open_w, geom, C["margin"], z_tip, Cm)
            if n <= C["max_hits"]:
                shift = float(dist[i])
                return dict(out, ok=True, uv=[float(cu[i]), float(cv[i])], xyz=Pc[i].tolist(), shift_m=shift, jaw_deg=float(yd % 180), hits=n,
                            why="GraspMolmo's point fits" if shift < 0.008 else "moved %.0f mm from GraspMolmo's point to where the gripper fits" % (1000 * shift))
            if best is None or n < best[0]:
                best = (n, i, yd)
    n, i, yd = best
    return dict(out, ok=False, uv=[float(cu[i]), float(cv[i])], xyz=Pc[i].tolist(), shift_m=float(dist[i]), jaw_deg=float(yd % 180), hits=n,
                why="no spot within %.0f cm clears the gripper at any release yaw tried; using the least blocked (%d points in the way)" % (
                    100 * C["shift_max"], n))


def place_disc(shape, uv, radius_px=6):
    h, w = shape
    yy, xx = np.mgrid[0:h, 0:w]
    return ((np.hypot(xx - uv[0], yy - uv[1]) <= radius_px) * 255).astype(np.uint8)
