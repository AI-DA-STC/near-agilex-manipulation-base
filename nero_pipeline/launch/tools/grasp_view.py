#!/usr/bin/env python3
"""
grasp_view.py - Open3D viewer for grasp_watch.py (run it in the contact_graspnet conda env; grasp_watch starts it for you).
    python grasp_view.py /tmp/grasp_debug.json
Everything is drawn in base_link:
   grey cloud   = scene          green cloud = the segmented object
   grey sticks  = every raw Contact-GraspNet candidate, from its hand base to its grasp centre (orange cross = where it closes)
   blue / cyan / yellow grippers = the (up to 3) tcp targets handed to MTC, #0 / #1 / #2
   GREEN gripper = the candidate the arm really used      RED gripper = where the gripper really closed
The window refreshes by itself when the json changes. Close the window to quit.
"""
import json, os, sys, time
import numpy as np

DEPTH, SEG = "/tmp/last_depth.npy", "/tmp/last_segmap.npy"
COLORS = [(0.1, 0.3, 1.0), (0.0, 0.9, 0.9), (1.0, 0.85, 0.0)]


def gripper_segments(T, width=0.06):
    """Gripper outline in the tcp_link frame (fingers span z 0.062..0.138 along the approach axis, jaws along y)."""
    h = width / 2
    pts = np.array([[0, 0, 0], [0, 0, 0.062], [0, -h, 0.062], [0, h, 0.062], [0, -h, 0.138], [0, h, 0.138]], float)
    lines = [(0, 1), (2, 3), (2, 4), (3, 5)]
    return (T[:3, :3] @ pts.T).T + T[:3, 3], lines


def stick_segments(T, length=0.1034):
    """Raw Contact-GraspNet candidate: origin = hand base; the grasp centre (fingertips) is 0.1034 m along the approach axis z."""
    o = T[:3, 3]; z = T[:3, 2]; x = T[:3, 0]; y = T[:3, 1]
    tip = o + length * z
    pts = np.array([o, tip, tip - 0.01 * x, tip + 0.01 * x, tip - 0.01 * y, tip + 0.01 * y])
    return pts, [(0, 1)], [(2, 3), (4, 5)]


def backproject(depth, K, T_base_cam, seg=None, stride=3):
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    H, W = depth.shape
    v, u = np.mgrid[0:H:stride, 0:W:stride]
    z = depth[::stride, ::stride]
    ok = np.isfinite(z) & (z > 0.2) & (z < 1.6)
    p = np.c_[(u[ok] - cx) * z[ok] / fx, (v[ok] - cy) * z[ok] / fy, z[ok]]
    obj = np.zeros(len(p), bool)
    if seg is not None:
        vv, uu = np.mgrid[0:H, 0:W]
        s = seg > 0
        zz = depth[s]; okk = np.isfinite(zz) & (zz > 0.2) & (zz < 1.6)
        po = np.c_[(uu[s][okk] - cx) * zz[okk] / fx, (vv[s][okk] - cy) * zz[okk] / fy, zz[okk]]
        p = np.vstack([p, po]); obj = np.r_[obj, np.ones(len(po), bool)]
    R, t = np.array(T_base_cam)[:3, :3], np.array(T_base_cam)[:3, 3]
    return (R @ p.T).T + t, obj


def scene_geometry(js):
    """-> list of (kind, points, lines_or_None, colors_or_color)  [pure numpy so it can be tested without a window]"""
    g = []
    if js.get("T_base_cam") and js.get("K") and os.path.exists(DEPTH):
        seg = np.load(SEG) if os.path.exists(SEG) else None
        P, obj = backproject(np.load(DEPTH), js["K"], js["T_base_cam"], seg)
        col = np.where(obj[:, None], np.array([[0.1, 0.8, 0.1]]), np.array([[0.65, 0.65, 0.65]]))
        g.append(("cloud", P, None, col))
    for T in js.get("raw", []):
        p, l, cross = stick_segments(np.array(T))
        g.append(("lines", p, l, (0.55, 0.55, 0.55)))        # stick: hand base -> grasp centre
        g.append(("lines", p, cross, (1.0, 0.5, 0.0)))        # orange cross = where that grasp wants to close
    used = js.get("used")
    for i, T in enumerate(js.get("cands", [])):
        p, l = gripper_segments(np.array(T))
        g.append(("lines", p, l, (0.0, 0.8, 0.0) if i == used else COLORS[i % 3]))
    if js.get("actual") is not None:
        p, l = gripper_segments(np.array(js["actual"]), width=0.075); g.append(("lines", p, l, (1.0, 0.0, 0.0)))
    return g


def main():
    import open3d as o3d
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/grasp_debug.json"
    vis = o3d.visualization.Visualizer(); vis.create_window("grasp debug (base_link)", 1280, 800)
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1))
    last, first = 0, True
    while vis.poll_events():
        try:
            m = os.path.getmtime(path)
        except OSError:
            m = 0
        if m != last and m:
            last = m
            try:
                js = json.load(open(path))
            except Exception:
                time.sleep(0.2); continue
            vis.clear_geometries(); vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1), reset_bounding_box=False)
            for kind, p, l, c in scene_geometry(js):
                if kind == "cloud":
                    pc = o3d.geometry.PointCloud(); pc.points = o3d.utility.Vector3dVector(p); pc.colors = o3d.utility.Vector3dVector(c)
                    vis.add_geometry(pc, reset_bounding_box=first)
                else:
                    ls = o3d.geometry.LineSet(); ls.points = o3d.utility.Vector3dVector(p); ls.lines = o3d.utility.Vector2iVector(l)
                    ls.colors = o3d.utility.Vector3dVector([c] * len(l)); vis.add_geometry(ls, reset_bounding_box=False)
            first = False
        vis.update_renderer(); time.sleep(0.03)
    vis.destroy_window()


if __name__ == "__main__":
    main()
