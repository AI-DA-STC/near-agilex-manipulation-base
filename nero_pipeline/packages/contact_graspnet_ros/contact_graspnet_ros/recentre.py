"""
Optional re-centring of Contact-GraspNet grasps on the object (pure numpy, no ROS, so it can be tested on its own).

Why: for a small box Contact-GraspNet often proposes narrow "pinch the rim" grasps (opening 1-2 cm on a 5 cm cube) whose closing centre sits at the
cube's edge. The gripper then opens about 7 cm around that edge and half of it is off the cube. Re-centring keeps every grasp's ORIENTATION and moves
its closing centre onto the middle of the object's top face (a little below the top), so the fingers close around the body.

Frames: everything here is in the camera frame the grasps are published in (x right, y down, z forward = roughly "down" for a camera looking at the table).
A grasp's pose is the hand base; its closing centre is CGN_GRIPPER_DEPTH further along the approach axis (the pose's z axis).
"""
import math

import numpy as np

CGN_GRIPPER_DEPTH = 0.1034      # hand base -> grasp centre along the approach axis (same constant as the MTC script)


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def object_top_middle(depth_m, mask, K, z_min=0.2, z_max=1.6, top_band=0.012):
    """-> (middle xyz in the camera frame, number of top-face points, top-face size xyz) or None.
    The top face = the masked points within `top_band` metres of the nearest 5 % (the points closest to the camera)."""
    v, u = np.nonzero(mask)
    z = depth_m[v, u]
    ok = np.isfinite(z) & (z > z_min) & (z < z_max)
    if ok.sum() < 20:
        return None
    u, v, z = u[ok], v[ok], z[ok]
    P = np.c_[(u - K[0][2]) * z / K[0][0], (v - K[1][2]) * z / K[1][1], z]
    top = P[P[:, 2] <= np.percentile(P[:, 2], 5) + top_band]
    if len(top) < 10:
        return None
    return top.mean(0), len(top), top.max(0) - top.min(0)


def recentre_grasps(grasps, middle, depth_below_top, max_offset, gripper_depth=CGN_GRIPPER_DEPTH):
    """Move every grasp's closing centre to middle + (0, 0, depth_below_top), keeping its orientation.
    Grasps whose closing centre is more than max_offset (sideways, x-y) from `middle` are dropped.  -> (new grasp dicts, number dropped)"""
    middle = np.asarray(middle, float)
    target = middle + np.array([0.0, 0.0, depth_below_top])
    out, dropped = [], 0
    for g in grasps:
        a = quat_to_R(*g["quaternion"])[:, 2]
        p = np.asarray(g["position"], float)
        close = p + gripper_depth * a
        if math.hypot(close[0] - middle[0], close[1] - middle[1]) > max_offset:
            dropped += 1
            continue
        g2 = dict(g)
        g2["position"] = [float(v) for v in (target - gripper_depth * a)]
        g2["recentred_from"] = [float(v) for v in p]
        out.append(g2)
    return out, dropped
