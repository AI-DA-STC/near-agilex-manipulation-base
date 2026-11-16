"""
p3_obstacles.py - PIPELINE 3: obstacle boxes for MoveIt's planning scene, from ONE depth frame (the one GraspMolmo looked at). No ROS in here.

What becomes an obstacle: depth points in the work area that stick up from the REAL table (p3_tablez) by more than MIN_ABOVE_TABLE, EXCEPT
  * flat things: everything lower than MIN_ABOVE_TABLE (2.5 cm) - they cannot block the arm, and it keeps depth noise out,
  * the object being picked (its region from GraspMolmo's grasp point, widened by EXCLUDE_TARGET_PX),
  * the robot: everything within BASE_RADIUS of its base axis, within LINK_RADIUS of any of its links (positions from TF), or higher than MAX_Z,
  * the plate: its rim (anything within PLATE_RADIUS of the place point and lower than RIM_HEIGHT above the table) and its landing zone
    (everything within LANDING_RADIUS of the place point). Taller things standing ON the plate stay obstacles.
The points are grouped on a GRID_M grid (touching cells = one object) and each group becomes a box from just above the table to its top,
widened by MARGIN_M on every side. Boxes are reset on every run: the node removes the previous run's boxes before adding new ones.
"""
from collections import deque

import numpy as np

P3_OBSTACLES_VERSION = "p3-obstacles-v6"   # v5: plain Python numbers (numpy ints broke saving the run)
# v6: the WHOLE object being picked is excluded - everything CONNECTED (on the GRID_M grid) to its region, not just the region within ~8 cm of
# GraspMolmo's point. A long object's far end had become an obstacle box right where the fingers open ("gripper_link2 collides with p3_obst_29").
# v4: obstacles from a HEIGHT MAP (one GRID_M patch at a time, as tall as what is there), merged into rectangles of similar height - instead of
# one solid box per object. A container (a sink, a bowl) keeps its free inside: its walls become thin boxes, its floor is below the threshold.
HEIGHT_TOL = 0.01            # m: neighbouring patches within this height of each other are merged into one box
MAX_HEIGHTMAP_BOXES = 150

STEP_PX = 2                  # use every 2nd pixel in both directions (a quarter of the points; plenty for 2 cm cells)
MIN_ABOVE_TABLE = 0.025      # m: lower than this = table. Clear of depth noise on the 12.7 mm plate the robot stands on (1.5 cm was not:
                             # noisy points on it chained into a false box), the dinner plate's floor and rim, and flat things (tape)
MAX_Z = 0.30                 # m above base_link: higher = the robot arm, not an object on the table
WORK_X = (-1.00, -0.08)      # m, base_link: the table area in front of the robot
WORK_Y = (-0.65, 0.65)
BASE_RADIUS = 0.22           # m around the robot's base axis: the robot and the plate it stands on
LINK_RADIUS = 0.10           # m around every robot link position (TF): the arm itself
EXCLUDE_TARGET_PX = 15       # px: the object being picked, widened by this (about 2.7 cm at the table)
PLATE_RADIUS = 0.13          # m: the plate around the place point ...
RIM_HEIGHT = 0.035           # m above the table: ... where anything lower is the plate itself (its floor and rim)
LANDING_RADIUS = 0.06        # m around the place point: always kept free (the object is put down there)
GRID_M = 0.02                # m: grouping grid
MIN_POINTS = 25              # points: smaller groups are noise
MIN_CELL_POINTS = 3          # points: a grid cell with fewer is noise and does not link groups together
MARGIN_M = 0.01              # m added on every side of a box
BOTTOM_GAP = 0.002           # m: boxes start this far above the table (never overlap the table box)
MAX_BOXES = 30


def points_in_base(depth_m, K, R_bc, t_bc, step=STEP_PX):
    """Valid depth pixels -> (base_link points Nx3, their pixel rows, their pixel columns)."""
    d = np.asarray(depth_m, dtype=np.float64)[::step, ::step]
    vs, us = np.mgrid[0:d.shape[0], 0:d.shape[1]]
    vs, us = vs * step, us * step
    ok = np.isfinite(d) & (d > 0)
    z = d[ok]
    cam = np.stack([(us[ok] - K[0][2]) * z / K[0][0], (vs[ok] - K[1][2]) * z / K[1][1], z], axis=1)
    return cam @ np.asarray(R_bc, dtype=np.float64).T + np.asarray(t_bc, dtype=np.float64), vs[ok], us[ok]


def dilate(mask, px):
    """Binary dilation by a square of half-size px (pure numpy)."""
    m = np.asarray(mask, dtype=bool)
    if px <= 0 or not m.any():
        return m
    out = m.copy()
    for dv in range(-px, px + 1):
        for du in range(-px, px + 1):
            src = m[max(0, -dv):m.shape[0] - max(0, dv), max(0, -du):m.shape[1] - max(0, du)]
            out[max(0, dv):max(0, dv) + src.shape[0], max(0, du):max(0, du) + src.shape[1]] |= src
    return out


def obstacle_points(P, rows, cols, table_z, target_mask=None, place_xyz=None, robot_points=()):
    """-> boolean array: which of the base_link points P are obstacle points (see the module docstring)."""
    h = P[:, 2] - table_z
    keep = (h > MIN_ABOVE_TABLE) & (P[:, 2] < MAX_Z)
    keep &= (P[:, 0] > WORK_X[0]) & (P[:, 0] < WORK_X[1]) & (P[:, 1] > WORK_Y[0]) & (P[:, 1] < WORK_Y[1])
    keep &= np.hypot(P[:, 0], P[:, 1]) > BASE_RADIUS
    for rp in robot_points:
        keep &= np.linalg.norm(P - np.asarray(rp, dtype=np.float64), axis=1) > LINK_RADIUS
    if target_mask is not None:
        keep &= ~dilate(target_mask, EXCLUDE_TARGET_PX)[rows, cols]
    if place_xyz is not None:
        r = np.hypot(P[:, 0] - place_xyz[0], P[:, 1] - place_xyz[1])
        keep &= ~((r < PLATE_RADIUS) & (h < RIM_HEIGHT))          # the plate itself
        keep &= r > LANDING_RADIUS                                  # the landing zone
    return keep


def group_boxes(P, table_z):
    """Obstacle points -> boxes [{"centre": [x, y, z], "size": [dx, dy, dz], "top_z": z, "points": n}], biggest first, at most MAX_BOXES."""
    if len(P) == 0:
        return []
    cells = {}
    ij = np.floor(P[:, :2] / GRID_M).astype(int)
    for k, (i, j) in enumerate(map(tuple, ij)):
        cells.setdefault((i, j), []).append(k)
    cells = {c: ks for c, ks in cells.items() if len(ks) >= MIN_CELL_POINTS}
    seen, boxes = set(), []
    for start in cells:
        if start in seen:
            continue
        comp, q = [], deque([start])
        seen.add(start)
        while q:
            c = q.popleft()
            comp.extend(cells[c])
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    n = (c[0] + di, c[1] + dj)
                    if n in cells and n not in seen:
                        seen.add(n)
                        q.append(n)
        if len(comp) < MIN_POINTS:
            continue
        Q = P[comp]
        lo = Q[:, :2].min(axis=0) - MARGIN_M
        hi = Q[:, :2].max(axis=0) + MARGIN_M
        top = float(np.percentile(Q[:, 2], 98)) + MARGIN_M         # 98th percentile: a few noisy high points do not inflate the box
        bottom = table_z + BOTTOM_GAP
        boxes.append({"centre": [float((lo[0] + hi[0]) / 2), float((lo[1] + hi[1]) / 2), (top + bottom) / 2],
                      "size": [float(hi[0] - lo[0]), float(hi[1] - lo[1]), top - bottom], "top_z": top, "points": len(comp)})
    boxes.sort(key=lambda b: -b["points"])
    return boxes[:MAX_BOXES]


def heightmap_boxes(P, table_z):
    """v4: obstacle points -> boxes from a height map. Noise groups (fewer than MIN_POINTS points) are dropped first; every remaining GRID_M
    patch is as tall as its points (90th percentile); runs of neighbouring patches of similar height are merged along x, then identical runs
    along y. Each box goes from just above the table to its top + MARGIN_M, widened by MARGIN_M on every side."""
    if len(P) == 0:
        return []
    keep_groups = group_boxes(P, table_z)                       # which points belong to real objects (not noise)
    if not keep_groups:
        return []
    inside = np.zeros(len(P), dtype=bool)
    for b in keep_groups:
        cx, cy, _ = b["centre"]; sx, sy, _ = b["size"]
        inside |= (np.abs(P[:, 0] - cx) <= sx / 2) & (np.abs(P[:, 1] - cy) <= sy / 2)
    P = P[inside]
    ij = np.floor(P[:, :2] / GRID_M).astype(int)
    cells = {}
    for k, c in enumerate(map(tuple, ij)):
        cells.setdefault(c, []).append(k)
    height = {c: float(np.percentile(P[ks, 2], 90)) for c, ks in cells.items() if len(ks) >= MIN_CELL_POINTS}
    runs = []                                                   # merge along x: (j, i0, i1, top)
    for j in sorted({c[1] for c in height}):
        row = sorted(i for (i, jj) in height if jj == j)
        start = prev = row[0]; top = height[(start, j)]
        for i in row[1:] + [None]:
            if i is not None and i == prev + 1 and abs(height[(i, j)] - top) <= HEIGHT_TOL:
                top = max(top, height[(i, j)]); prev = i
                continue
            runs.append([j, start, prev, top])
            if i is not None:
                start = prev = i; top = height[(i, j)]
    rects = []                                                  # merge identical runs along y: (i0, i1, j0, j1, top)
    for j, i0, i1, top in sorted(runs, key=lambda r: (r[1], r[2], r[0])):
        if rects and rects[-1][0] == i0 and rects[-1][1] == i1 and rects[-1][3] == j - 1 and abs(rects[-1][4] - top) <= HEIGHT_TOL:
            rects[-1][3] = j; rects[-1][4] = max(rects[-1][4], top)
        else:
            rects.append([i0, i1, j, j, top])
    bottom = table_z + BOTTOM_GAP
    boxes = []
    for i0, i1, j0, j1, top in rects:
        i0, i1, j0, j1, top = int(i0), int(i1), int(j0), int(j1), float(top)
        x0, x1 = i0 * GRID_M - MARGIN_M, (i1 + 1) * GRID_M + MARGIN_M
        y0, y1 = j0 * GRID_M - MARGIN_M, (j1 + 1) * GRID_M + MARGIN_M
        t = top + MARGIN_M
        boxes.append({"centre": [float((x0 + x1) / 2), float((y0 + y1) / 2), float((t + bottom) / 2)],
                      "size": [float(x1 - x0), float(y1 - y0), float(t - bottom)], "top_z": float(t), "points": int((i1 - i0 + 1) * (j1 - j0 + 1))})
    boxes.sort(key=lambda b: -b["size"][0] * b["size"][1])
    return boxes[:MAX_HEIGHTMAP_BOXES]


def components(P):
    """Connected groups of points on the GRID_M grid (cells with at least MIN_CELL_POINTS points; 8-neighbours) -> list of index arrays."""
    if len(P) == 0:
        return []
    cells = {}
    for k, c in enumerate(map(tuple, np.floor(P[:, :2] / GRID_M).astype(int))):
        cells.setdefault(c, []).append(k)
    cells = {c: ks for c, ks in cells.items() if len(ks) >= MIN_CELL_POINTS}
    seen, out = set(), []
    for start in cells:
        if start in seen:
            continue
        comp, q = [], deque([start])
        seen.add(start)
        while q:
            c = q.popleft()
            comp.extend(cells[c])
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    n = (c[0] + di, c[1] + dj)
                    if n in cells and n not in seen:
                        seen.add(n)
                        q.append(n)
        out.append(np.array(comp))
    return out


def obstacle_boxes(depth_m, K, R_bc, t_bc, table_z, target_mask=None, place_xyz=None, robot_points=()):
    """The whole step: depth frame -> obstacle boxes in base_link (v4: from a height map, so containers keep their free inside;
    v6: the whole object being picked - everything connected to its region - is left out)."""
    P, rows, cols = points_in_base(depth_m, K, R_bc, t_bc)
    keep = obstacle_points(P, rows, cols, table_z, None, place_xyz, robot_points)       # everything but the target first
    Pk, rk, ck = P[keep], rows[keep], cols[keep]
    if target_mask is not None and len(Pk):
        hit = dilate(target_mask, EXCLUDE_TARGET_PX)[rk, ck]
        drop = hit.copy()
        for comp in components(Pk):                             # every group that touches the target's region IS the target
            if hit[comp].any():
                drop[comp] = True
        Pk = Pk[~drop]
    return heightmap_boxes(Pk, table_z)


def object_height(depth_m, K, R_bc, t_bc, mask, table_z):
    """v3: the height of the object in `mask` (its region from GraspMolmo's grasp point) above the REAL table, in m: the MEDIAN of its top
    surface (all points within 6 mm of its 95th percentile). A plain high percentile would read ~4 mm too tall with 2 mm depth noise - too
    much for a 1 cm pen. None if the region has no valid depth."""
    vs, us = np.nonzero(np.asarray(mask, dtype=bool))
    d = np.asarray(depth_m, dtype=np.float64)[vs, us]
    ok = np.isfinite(d) & (d > 0)
    if ok.sum() < 10:
        return None
    vs, us, z = vs[ok], us[ok], d[ok]
    cam = np.stack([(us - K[0][2]) * z / K[0][0], (vs - K[1][2]) * z / K[1][1], z], axis=1)
    base_z = cam @ np.asarray(R_bc, dtype=np.float64)[2] + float(np.asarray(t_bc, dtype=np.float64)[2])
    p95 = float(np.percentile(base_z, 95))
    return float(np.median(base_z[base_z >= p95 - 0.006]) - table_z)


def describe(boxes):
    if not boxes:
        return "no obstacles on the table (apart from the object being picked and the plate)"
    return "%d obstacle box(es), highest %.1f cm: " % (len(boxes), 100 * max(b["size"][2] for b in boxes)) + "; ".join(
        "(%.2f, %+.2f) %.0fx%.0f cm, %.1f cm tall" % (b["centre"][0], b["centre"][1], 100 * b["size"][0], 100 * b["size"][1], 100 * (b["size"][2]))
        for b in boxes[:6]) + ("; ..." if len(boxes) > 6 else "")
