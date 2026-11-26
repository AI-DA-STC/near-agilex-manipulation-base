"""Pipeline 3 geometry: turn GraspMolmo's pixels into the masks pipeline 1's nodes already understand. No ROS imports.

object_mask(): the object under the grasp point = connected pixels near the point that stick up above the table.
               Contact-GraspNet and recentre.py use it exactly like the Grounded-SAM-2 object mask.
place_mask():  a small disc around the plate point. target_pose_ros's centre of a disc is the point itself, so the
               place position is GraspMolmo's point, lifted to 3D with target_pose_ros's own "floor" depth.
"""
import cv2
import numpy as np


def object_mask(depth_m, uv, radius_px=45, ring_px=15, above_table_m=0.008, min_px=30):
    """Returns (mask uint8 0/255 or None, message).

    radius_px     search radius around the point (45 px is about 8 cm at the table, fx 488.8, 0.87 m)
    ring_px       width of the ring just outside it, used to measure the table depth
    above_table_m pixels at least this much closer to the camera than the table count as object
    """
    h, w = depth_m.shape
    u, v = float(uv[0]), float(uv[1])
    yy, xx = np.mgrid[0:h, 0:w]
    d = np.hypot(xx - u, yy - v)
    valid = depth_m > 0
    ring = (d > radius_px) & (d <= radius_px + ring_px) & valid
    if ring.sum() < 20:
        return None, "no table depth around the grasp point"
    # 75th percentile = the far (table) side, even if the object or plate spills into the ring
    table = float(np.percentile(depth_m[ring], 75))
    obj = ((d <= radius_px) & valid & (depth_m < table - above_table_m)).astype(np.uint8)
    n, lab = cv2.connectedComponents(obj)
    if n <= 1:
        return None, f"nothing above the table within {radius_px} px of the grasp point (table at {table:.3f} m)"
    ui, vi = int(round(u)), int(round(v))
    k = lab[vi, ui] if (0 <= vi < h and 0 <= ui < w) else 0
    if k == 0:                                   # point on an edge or a depth hole: take the nearest region
        ys, xs = np.nonzero(lab)
        i = int(np.argmin((xs - u) ** 2 + (ys - v) ** 2))
        k = lab[ys[i], xs[i]]
    m = lab == k
    if m.sum() < min_px:
        return None, f"object region too small ({int(m.sum())} px)"
    return (m * 255).astype(np.uint8), f"object {int(m.sum())} px, table at {table:.3f} m"


def place_mask(depth_m, uv, radius_px=6):
    h, w = depth_m.shape
    yy, xx = np.mgrid[0:h, 0:w]
    m = (np.hypot(xx - uv[0], yy - uv[1]) <= radius_px) & (depth_m > 0)
    if m.sum() < 5:
        return None, f"no depth at the place point ({uv[0]:.0f}, {uv[1]:.0f})"
    return (m * 255).astype(np.uint8), f"place disc {int(m.sum())} px"


def lift(uv, depth_m, K, r=5):
    """Pixel -> camera-frame xyz using the median valid depth in a (2r+1)^2 window; None if no depth."""
    h, w = depth_m.shape
    u, v = int(round(uv[0])), int(round(uv[1]))
    if not (0 <= u < w and 0 <= v < h):
        return None
    win = depth_m[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1]
    vals = win[win > 0]
    if vals.size == 0:
        return None
    z = float(np.median(vals))
    return [(uv[0] - K[0, 2]) * z / K[0, 0], (uv[1] - K[1, 2]) * z / K[1, 1], z]


def annotate(bgr, obj_mask=None, grasp_uv=None, place_uv=None):
    out = bgr.copy()
    if obj_mask is not None:
        over = out.copy(); over[obj_mask > 0] = (255, 120, 0)
        out = cv2.addWeighted(over, 0.4, out, 0.6, 0)
    if grasp_uv is not None:
        cv2.circle(out, (int(grasp_uv[0]), int(grasp_uv[1])), 7, (255, 60, 0), 2)
        cv2.putText(out, "grasp", (int(grasp_uv[0]) + 9, int(grasp_uv[1]) - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 60, 0), 1)
    if place_uv is not None:
        cv2.circle(out, (int(place_uv[0]), int(place_uv[1])), 7, (0, 0, 255), 2)
        cv2.putText(out, "place", (int(place_uv[0]) + 9, int(place_uv[1]) - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    return out
