"""
Pure geometry for the target (plate) pose - no ROS imports, so it can be tested and reused by tools.

Given the target's 2D mask, the depth image (metres) and the camera intrinsics it returns the target's position in the camera's optical frame.

center_mode - how the plate's centre is found (all in pixel space, then back-projected):
    "mean"   the average of every mask pixel back-projected to 3D. This was the ORIGINAL behaviour. A hole in the mask (an object lying on the plate,
             a reflection) pulls the centre away from the hole's side.
    "hull"   the area centroid of the mask's convex hull. Holes inside the plate do not matter; for a solid mask it equals "mean". DEFAULT.
    "circle" the centre of the smallest circle that encloses the mask. Good for round or square plates; sensitive to stray mask pixels at the edge.
    "bbox"   the centre of the mask's bounding box.
z_mode - which height is reported:
    "floor"       the MEDIAN depth of the pixels near the centre: the plate's inner surface, where an object will rest. DEFAULT.
    "top_surface" the single closest pixel (the rim, or one noisy pixel). The ORIGINAL behaviour.
    "centroid"    the mean depth of every mask pixel.
None of these can fix a mask that does not cover the whole plate (arm in the way, plate partly out of view): check the picture from
launch/tools/plate_center_check.py.
"""
import math

import numpy as np

try:
    import cv2
except ImportError:          # the centre modes "hull" and "circle" then fall back to "mean"
    cv2 = None

CENTER_MODES = ("mean", "hull", "circle", "bbox")
Z_MODES = ("floor", "top_surface", "centroid")


def extract_target(mask, depth_m, K, z_mode="floor", center_mode="hull", center_fraction=0.5):
    """mask: HxW (non-zero = target); depth_m: HxW metres (0 = invalid); K: 3x3 intrinsics. Returns a dict (see keys below)."""
    if z_mode not in Z_MODES:
        raise ValueError(f"unknown z_mode {z_mode!r}, use one of {Z_MODES}")
    if center_mode not in CENTER_MODES:
        raise ValueError(f"unknown center_mode {center_mode!r}, use one of {CENTER_MODES}")
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        raise ValueError("Target mask is empty (no detected pixels)")
    d = depth_m[ys, xs].astype(np.float64)
    valid = d > 0
    if not np.any(valid):
        raise ValueError("All target pixels have invalid (zero) depth")
    xs, ys, d = xs[valid], ys[valid], d[valid]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    n = len(xs)

    mean_px = (float(xs.mean()), float(ys.mean()))
    bbox_px = (float(xs.min() + xs.max()) / 2.0, float(ys.min() + ys.max()) / 2.0)
    hull_px, circle_px = mean_px, mean_px
    if cv2 is not None and n >= 3:
        pts = np.column_stack([xs, ys]).astype(np.float32)
        hull = cv2.convexHull(pts)
        mom = cv2.moments(hull)
        if mom["m00"] > 1e-6:
            hull_px = (float(mom["m10"] / mom["m00"]), float(mom["m01"] / mom["m00"]))
        (cu, cv_), _r = cv2.minEnclosingCircle(hull)
        circle_px = (float(cu), float(cv_))
    centers = {"mean": mean_px, "hull": hull_px, "circle": circle_px, "bbox": bbox_px}
    u0, v0 = centers[center_mode]

    r_eq = math.sqrt(n / math.pi)                                   # radius of a disc with the same pixel count
    near = (xs - u0) ** 2 + (ys - v0) ** 2 <= (center_fraction * r_eq) ** 2
    floor_z = float(np.median(d[near])) if int(near.sum()) >= 30 else float(np.median(d))
    top_z = float(np.min(d))
    rim_z = float(np.percentile(d, 1.0))                            # a robust "highest point" (ignores a few stray pixels)
    mean_z = float(np.mean(d))
    z = {"floor": floor_z, "top_surface": top_z, "centroid": mean_z}[z_mode]

    if center_mode == "mean":                                       # original behaviour: average of the back-projected 3D points
        x = float(np.mean((xs - cx) * d / fx))
        y = float(np.mean((ys - cy) * d / fy))
    else:
        x = float((u0 - cx) * floor_z / fx)
        y = float((v0 - cy) * floor_z / fy)
    return dict(x=x, y=y, z=float(z), n=int(n), center_px=(u0, v0), mean_px=mean_px, hull_px=hull_px, circle_px=circle_px, bbox_px=bbox_px,
                floor_z=floor_z, top_z=top_z, rim_z=rim_z, mean_z=mean_z, diameter_m=float(2 * r_eq * floor_z / fx),
                z_mode=z_mode, center_mode=center_mode)


def describe(r):
    """One readable line for the service response / log."""
    fx_per_px = r["diameter_m"] / (2 * math.sqrt(r["n"] / math.pi))          # metres per pixel at the plate
    def cm(a, b): return 100 * fx_per_px * math.hypot(a[0] - b[0], a[1] - b[1])
    return (f"position=({r['x']:.3f}, {r['y']:.3f}, {r['z']:.3f}) from {r['n']} pixels, z_mode='{r['z_mode']}', center_mode='{r['center_mode']}' | "
            f"plate about {100 * r['diameter_m']:.0f} cm across (area-equivalent) | centre pixels: mean ({r['mean_px'][0]:.0f},{r['mean_px'][1]:.0f}) "
            f"hull ({r['hull_px'][0]:.0f},{r['hull_px'][1]:.0f}) circle ({r['circle_px'][0]:.0f},{r['circle_px'][1]:.0f}) bbox ({r['bbox_px'][0]:.0f},{r['bbox_px'][1]:.0f}); "
            f"mean vs hull differ {cm(r['mean_px'], r['hull_px']):.1f} cm, hull vs circle {cm(r['hull_px'], r['circle_px']):.1f} cm | "
            f"depth: floor {r['floor_z']:.3f} m, highest 1% {r['rim_z']:.3f} m (rim {100 * (r['floor_z'] - r['rim_z']):.1f} cm above floor), "
            f"single closest pixel {r['top_z']:.3f} m")
