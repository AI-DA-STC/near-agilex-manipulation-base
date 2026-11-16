#!/usr/bin/env python
"""
p3_grasp_view.py - Open3D viewer for p3_grasp_watch.py (started by it, in the contact_graspnet conda env). Everything in base_link:

   the colour point cloud of the moment Contact-GraspNet ran
   every Contact-GraspNet grasp as a small gripper: GREEN = nearly straight down ... YELLOW = tilted up to 55 deg (MoveIt may use these),
                                                   GREY = tilted more than 55 deg (dropped)
   BLUE gripper(s) = what the relay passed on to MoveIt (after the 90-deg jaw turn: its fingers are where the REAL fingers close)
   the axes at the origin = base_link (red x, green y, blue z)

Each gripper: a stick from the hand base to the palm, the palm, and the two fingers (closing along the line between them).
The window reloads by itself after every run. Close it to quit (p3_grasp_watch.py keeps running; restart it to reopen the window).
"""
import math
import os
import sys
import time

import numpy as np

P3_GRASP_VIEW_VERSION = "p3-graspview-v1"
PALM_Z, TIP_Z, OPEN_W = 0.066, 0.112, 0.08      # Contact-GraspNet's gripper (Panda-like): palm and fingertips along the approach, opening width
MAX_TILT = 55.0


def tilt_deg(G):
    """Angle between the grasp's approach (its z axis) and straight down."""
    return math.degrees(math.acos(max(-1.0, min(1.0, -float(G[2, 2])))))


def tilt_colour(t):
    if t > MAX_TILT:
        return (0.55, 0.55, 0.55)
    f = t / MAX_TILT                                    # 0 = straight down (green) ... 1 = 55 deg (yellow)
    return (f, 0.85, 0.1)


def gripper_lines(G, jaw_axis=0, width=OPEN_W):
    """Points and line index pairs of a gripper drawn at pose G (4x4, base_link). jaw_axis 0 = fingers apart along the pose's x
    (Contact-GraspNet's own grasps), 1 = along y (the relay's turned grasps, = the real Nero fingers)."""
    h = width / 2.0
    e = np.zeros(3); e[jaw_axis] = h
    local = np.array([[0, 0, 0], [0, 0, PALM_Z], -e + [0, 0, PALM_Z], e + [0, 0, PALM_Z], -e + [0, 0, TIP_Z], e + [0, 0, TIP_Z]], dtype=float)
    pts = (G[:3, :3] @ local.T).T + G[:3, 3]
    return pts, [(0, 1), (2, 3), (2, 4), (3, 5)]


def build_lines(grasps, chosen):
    """All grippers as one set of points, line pairs and colours (pure numpy)."""
    P, L, C = [], [], []
    for G, jaw, col in [(G, 0, tilt_colour(tilt_deg(G))) for G in grasps] + [(G, 1, (0.1, 0.35, 1.0)) for G in chosen]:
        pts, lines = gripper_lines(G, jaw)
        base = len(P)
        P.extend(pts.tolist())
        L.extend([(base + a, base + b) for a, b in lines])
        C.extend([col] * len(lines))
    return np.array(P).reshape(-1, 3), np.array(L, dtype=int).reshape(-1, 2), np.array(C).reshape(-1, 3)


def summary(grasps, chosen):
    t = [tilt_deg(G) for G in grasps]
    kept = sum(1 for x in t if x <= MAX_TILT)
    s = "%d Contact-GraspNet grasp(s): %d within 55 deg of straight down (green..yellow), %d dropped (grey)" % (len(grasps), kept, len(grasps) - kept)
    if len(chosen):
        s += "; passed on to MoveIt (blue): %s deg" % ", ".join("%.0f" % tilt_deg(G) for G in chosen)
    return s


def main():
    import open3d as o3d
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/p3_view/scene.npz"
    print("p3_grasp_view %s - waiting for %s (run p3_look.sh) ..." % (P3_GRASP_VIEW_VERSION, path), flush=True)
    vis = o3d.visualization.Visualizer()
    vis.create_window("pipeline 3 - every Contact-GraspNet grasp", width=1280, height=900)
    vis.get_render_option().point_size = 2.0
    shown, first = None, True
    while True:
        if os.path.exists(path) and os.path.getmtime(path) != shown:
            try:
                d = np.load(path)
                shown = os.path.getmtime(path)
                vis.clear_geometries()
                pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(d["points"].astype(np.float64)))
                pc.colors = o3d.utility.Vector3dVector(d["colours"].astype(np.float64))
                vis.add_geometry(pc, reset_bounding_box=first)
                P, L, C = build_lines(d["grasps"], d["chosen"])
                if len(L):
                    ls = o3d.geometry.LineSet(o3d.utility.Vector3dVector(P), o3d.utility.Vector2iVector(L))
                    ls.colors = o3d.utility.Vector3dVector(C)
                    vis.add_geometry(ls, reset_bounding_box=first)
                vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1), reset_bounding_box=first)
                first = False
                print("[%s] %s" % (time.strftime("%H:%M:%S"), summary(d["grasps"], d["chosen"])), flush=True)
            except Exception as e:                       # a half-written file: try again on the next poll
                print("could not load the scene yet (%s)" % e, flush=True)
        if not vis.poll_events():
            break
        vis.update_renderer()
        time.sleep(0.05)
    vis.destroy_window()
    return 0


if __name__ == "__main__":
    sys.exit(main())
