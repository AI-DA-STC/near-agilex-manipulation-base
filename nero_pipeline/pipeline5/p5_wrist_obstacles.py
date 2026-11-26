"""
p4_wrist_obstacles.py - PIPELINE 4: obstacle boxes from the WRIST camera, in 3D (no ROS in here, tested offline).

Unlike pipeline 3's top-camera boxes (columns from the table up, nothing above 30 cm), these boxes sit at their REAL height: something hanging
in mid-air becomes a box in mid-air, and the space under it stays free. Same rules as pipeline 3 for what is NOT an obstacle:
  * the table and flat things: lower than MIN_ABOVE_TABLE above the real table (p3_tablez),
  * the object being picked: everything inside its 3D box (from GraspMolmo's object region seen by the top camera, widened by TARGET_MARGIN);
    a group of voxels touching that box is dropped WHOLE only if it stays within REACH_BEYOND_M of the box (a handle, a stalk); a group reaching
    farther (a sink wall the object leans on) keeps everything outside the box (v3),
  * the place: its landing zone (LANDING_RADIUS around the place point, all heights) and the plate (PLATE_RADIUS, lower than RIM_HEIGHT),
  * noise: voxels with fewer than MIN_VOXEL_POINTS points and groups smaller than MIN_GROUP_VOXELS.
(The robot itself and the camera's range are removed before this module, by p4_fuse_node.)
Voxels are merged into as few boxes as possible (runs along x, then y, then z), each widened by MARGIN_M. A box whose lowest voxel is in the
lowest kept layer (it STANDS on the table) reaches down to the table, so the 2.5 cm dropped as table noise leaves no gap under a wall; boxes higher
up stay in mid-air.
"""
from collections import deque

import numpy as np

P4_WRIST_OBSTACLES_VERSION = "p4-wrist-obstacles-v3"   # v3: a group touching the object loses only its part near the object if it reaches far beyond (a sink wall stays)
# v2:   # v2: boxes starting near the table reach down to it (standing objects have no gap under them)

VOXEL_M = 0.02
MIN_ABOVE_TABLE = 0.025      # same as pipeline 3
MAX_ABOVE_TABLE = 0.80
MIN_VOXEL_POINTS = 3
MIN_GROUP_VOXELS = 3
TARGET_MARGIN = 0.03         # m around the object being picked
LANDING_RADIUS = 0.06        # same as pipeline 3
PLATE_RADIUS = 0.13
RIM_HEIGHT = 0.035
MARGIN_M = 0.01
REACH_BEYOND_M = 0.04        # v3: a group touching the object that reaches farther than this beyond its box is a separate structure (sink wall)
BOTTOM_GAP = 0.002           # never overlap MoveIt's table box
MAX_BOXES = 80


# ---------------------------------------------------------------- images (top camera) -> 3D
def image_to_array(msg, dtype):
    """sensor_msgs/Image -> 2D numpy array (handles row padding via msg.step)."""
    item = np.dtype(dtype).itemsize
    a = np.frombuffer(bytes(msg.data), dtype=dtype)
    per_row = msg.step // item
    return a.reshape(msg.height, per_row)[:, :msg.width]


def depth_to_m(msg, scale=0.001):
    if msg.encoding == "32FC1":
        d = image_to_array(msg, "<f4").astype(np.float64)
        return np.nan_to_num(d)
    return image_to_array(msg, "<u2").astype(np.float64) * scale


def mask_points_base(mask, depth_m, K, R, t):
    """Pixels of a mask with valid depth -> base_link points (N,3)."""
    vs, us = np.nonzero(np.asarray(mask) > 0)
    z = depth_m[vs, us]
    ok = np.isfinite(z) & (z > 0)
    vs, us, z = vs[ok], us[ok], z[ok]
    cam = np.stack([(us - K[0][2]) * z / K[0][0], (vs - K[1][2]) * z / K[1][1], z], axis=1)
    return cam @ np.asarray(R, dtype=np.float64).T + np.asarray(t, dtype=np.float64)


def target_box(points_base, table_z, margin=TARGET_MARGIN):
    """The object being picked -> (lo xyz, hi xyz): its extent widened by margin, from the table up. None if no points."""
    if len(points_base) < 10:
        return None
    lo = points_base.min(axis=0) - margin
    hi = points_base.max(axis=0) + margin
    lo[2] = table_z
    return lo, hi


def place_point(points_base):
    """The place disc -> its 3D centre (median), or None."""
    if len(points_base) < 3:
        return None
    return np.median(points_base, axis=0)


# ---------------------------------------------------------------- points -> voxels -> boxes
def keep_points(P, table_z, place_xyz=None):
    """Drop the table / flat things, the place's landing zone and plate (pipeline 3's rules)."""
    h = P[:, 2] - table_z
    keep = (h > MIN_ABOVE_TABLE) & (h < MAX_ABOVE_TABLE)
    if place_xyz is not None:
        r = np.hypot(P[:, 0] - place_xyz[0], P[:, 1] - place_xyz[1])
        keep &= r > LANDING_RADIUS
        keep &= ~((r < PLATE_RADIUS) & (h < RIM_HEIGHT))
    return P[keep]


def voxelize(P, voxel=VOXEL_M, min_points=MIN_VOXEL_POINTS):
    """-> dict {(i, j, k): n_points} of voxels with at least min_points points."""
    if len(P) == 0:
        return {}
    keys, counts = np.unique(np.floor(P / voxel).astype(np.int64), axis=0, return_counts=True)
    return {tuple(int(v) for v in k): int(c) for k, c in zip(keys, counts) if c >= min_points}


def groups(voxels):
    """26-connected groups of voxels -> list of lists of keys."""
    seen, out = set(), []
    for start in voxels:
        if start in seen:
            continue
        comp, q = [], deque([start])
        seen.add(start)
        while q:
            c = q.popleft()
            comp.append(c)
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    for dk in (-1, 0, 1):
                        n = (c[0] + di, c[1] + dj, c[2] + dk)
                        if n in voxels and n not in seen:
                            seen.add(n)
                            q.append(n)
        out.append(comp)
    return out


def voxel_touches_box(key, box, voxel=VOXEL_M):
    lo, hi = box
    vlo = np.array(key, dtype=np.float64) * voxel
    vhi = vlo + voxel
    return bool(np.all(vhi >= lo) and np.all(vlo <= hi))


def beyond_box(key, box, voxel=VOXEL_M):
    """How far (m) a voxel's centre lies outside a box (0 inside)."""
    lo, hi = box
    c = (np.array(key, dtype=np.float64) + 0.5) * voxel
    return float(np.linalg.norm(np.maximum(0.0, np.maximum(lo - c, c - hi))))


def clean_voxels(voxels, tbox=None, voxel=VOXEL_M):
    """Drop noise groups; for groups touching the object being picked: drop the whole group if it stays near the object (a handle),
    else drop only its voxels touching the object's box (v3: a sink wall the object leans on stays an obstacle). -> (kept voxel set, note dict)."""
    kept, n_noise, n_target, n_split = set(), 0, 0, 0
    for comp in groups(voxels):
        if len(comp) < MIN_GROUP_VOXELS:
            n_noise += 1
            continue
        if tbox is not None:
            touching = [k for k in comp if voxel_touches_box(k, tbox, voxel)]
            if touching:
                if max(beyond_box(k, tbox, voxel) for k in comp) <= REACH_BEYOND_M:
                    n_target += 1
                    continue
                n_split += 1
                rest = [k for k in comp if not voxel_touches_box(k, tbox, voxel)]
                if len(rest) >= MIN_GROUP_VOXELS:
                    kept.update(rest)
                continue
        kept.update(comp)
    return kept, {"noise_groups": n_noise, "target_groups": n_target, "split_groups": n_split}


def merge_boxes(vox, voxel=VOXEL_M):
    """Voxel keys -> list of (i0, i1, j0, j1, k0, k1): runs along x, merged along y, then along z. Covers exactly the voxels."""
    rows = {}
    for i, j, k in vox:
        rows.setdefault((j, k), []).append(i)
    runs = []
    for (j, k), xs in rows.items():
        xs.sort()
        s = p = xs[0]
        for i in xs[1:] + [None]:
            if i is not None and i == p + 1:
                p = i
                continue
            runs.append((s, p, j, k))
            if i is not None:
                s = p = i
    by_xk = {}
    for i0, i1, j, k in runs:
        by_xk.setdefault((i0, i1, k), []).append(j)
    rects = []
    for (i0, i1, k), js in by_xk.items():
        js.sort()
        s = p = js[0]
        for j in js[1:] + [None]:
            if j is not None and j == p + 1:
                p = j
                continue
            rects.append((i0, i1, s, p, k))
            if j is not None:
                s = p = j
    by_xy = {}
    for i0, i1, j0, j1, k in rects:
        by_xy.setdefault((i0, i1, j0, j1), []).append(k)
    out = []
    for (i0, i1, j0, j1), ks in by_xy.items():
        ks.sort()
        s = p = ks[0]
        for k in ks[1:] + [None]:
            if k is not None and k == p + 1:
                p = k
                continue
            out.append((i0, i1, j0, j1, s, p))
            if k is not None:
                s = p = k
    return out


def to_boxes(blocks, table_z, voxel=VOXEL_M, margin=MARGIN_M, max_boxes=MAX_BOXES):
    """Merged blocks -> pipeline 3 style box dicts (centre, size, top_z, points), biggest first, at most max_boxes. -> (boxes, n_dropped)"""
    boxes = []
    floor = table_z + BOTTOM_GAP
    for i0, i1, j0, j1, k0, k1 in blocks:
        x0, x1 = i0 * voxel - margin, (i1 + 1) * voxel + margin
        y0, y1 = j0 * voxel - margin, (j1 + 1) * voxel + margin
        grounded = k0 * voxel <= table_z + MIN_ABOVE_TABLE + voxel          # starts in the lowest kept layer: it stands on the table
        z0, z1 = (floor if grounded else max(k0 * voxel - margin, floor)), (k1 + 1) * voxel + margin
        if z1 <= z0:
            continue
        boxes.append({"centre": [(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2], "size": [x1 - x0, y1 - y0, z1 - z0],
                      "top_z": z1, "bottom_z": z0, "points": int((i1 - i0 + 1) * (j1 - j0 + 1) * (k1 - k0 + 1))})
    boxes.sort(key=lambda b: -b["size"][0] * b["size"][1] * b["size"][2])
    return boxes[:max_boxes], max(0, len(boxes) - max_boxes)


def wrist_boxes(P, table_z, tbox=None, place_xyz=None):
    """The whole step: wrist points in base_link (robot and out-of-range already removed) -> (boxes, note dict)."""
    note = {"points_in": int(len(P))}
    Q = keep_points(P, table_z, place_xyz)
    note["points_above_table"] = int(len(Q))
    vox = voxelize(Q)
    note["voxels"] = len(vox)
    kept, n = clean_voxels(vox, tbox)
    note.update(n)
    note["voxels_kept"] = len(kept)
    boxes, dropped = to_boxes(merge_boxes(kept), table_z)
    note["boxes"] = len(boxes)
    note["boxes_dropped_over_limit"] = dropped
    return boxes, note


def describe(boxes, note):
    if not boxes:
        return "no wrist obstacles (%d points above the table, %d voxels, %d noise group(s), %d target group(s) left out)" % (
            note.get("points_above_table", 0), note.get("voxels", 0), note.get("noise_groups", 0), note.get("target_groups", 0))
    lo = min(b["bottom_z"] for b in boxes)
    hi = max(b["top_z"] for b in boxes)
    return ("%d wrist box(es) from %d voxels, between z %.2f and %.2f m; left out: %d noise group(s), %d group(s) of the object being picked, "
            "%d structure(s) touching it kept except next to it%s"
            % (len(boxes), note["voxels_kept"], lo, hi, note.get("noise_groups", 0), note.get("target_groups", 0), note.get("split_groups", 0),
               "" if not note.get("boxes_dropped_over_limit") else "; %d small box(es) over the limit dropped" % note["boxes_dropped_over_limit"]))
