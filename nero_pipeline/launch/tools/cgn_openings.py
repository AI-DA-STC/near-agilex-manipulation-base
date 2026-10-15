#!/usr/bin/env python3
"""
cgn_openings.py - why do Contact-GraspNet's grasps close BESIDE the cube even though their contact points are on it?
Runs inside the `contact_graspnet` conda env on the last detection (/tmp/last_depth.npy, /tmp/last_segmap.npy; K from /tmp/grasp_debug.json or
/tmp/last_K.json), with the pipeline's own settings (local regions, filter on, forward_passes 5, a few tries). Moves nothing.

v2: offsets are now measured from the middle of the cube's TOP FACE (the points closest to the camera), because the middle of all visible points is
pulled toward whichever side faces the camera sees. It also lists the grasps closest to that middle, counts how many are central, and shows tilt.

As far as I know, Contact-GraspNet puts a grasp's pose half a predicted opening away from the contact point it found:
      closing centre = contact point + (opening / 2) along the baseline.
If the predicted opening is wider than the cube, the closing centre is pushed sideways off the cube by about (opening/2 - cube half-width).
This script prints, per grasp: score, predicted opening, distance contact->closing centre (should be about opening/2), and the offsets of the
contact point and of the closing centre from the middle of the cube's visible points, in the CAMERA frame (x right, y down, z forward), in cm.
    conda run -n contact_graspnet --no-capture-output python cgn_openings.py --ckpt_dir ~/contact_graspnet_pytorch/checkpoints/contact_graspnet
"""
import argparse, contextlib, io, json, os, sys
import numpy as np
from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator
from contact_graspnet_pytorch import config_utils
from contact_graspnet_pytorch.checkpoints import CheckpointIO

CLOSE = 0.1034      # hand base -> grasp centre along the approach axis

ap = argparse.ArgumentParser()
ap.add_argument("--ckpt_dir", required=True)
ap.add_argument("--depth", default="/tmp/last_depth.npy"); ap.add_argument("--segmap", default="/tmp/last_segmap.npy")
ap.add_argument("--passes", type=int, default=5); ap.add_argument("--tries", type=int, default=4)
a = ap.parse_args()

K = None
for path in ("/tmp/grasp_debug.json", "/tmp/last_K.json"):
    try:
        js = json.load(open(path)); K = js["K"] if isinstance(js, dict) else js; print("K from %s: fx=%.1f cx=%.1f cy=%.1f" % (path, K[0], K[2], K[5])); break
    except Exception:
        pass
if K is None:
    sys.exit("no camera matrix found: run a detection with grasp_watch.py running, or write /tmp/last_K.json")
K = np.array(K, float).reshape(3, 3)
depth = np.load(a.depth).astype(np.float64); mask = np.load(a.segmap)

est = GraspEstimator(config_utils.load_config(a.ckpt_dir, batch_size=a.passes))
CheckpointIO(checkpoint_dir=os.path.join(a.ckpt_dir, "checkpoints"), model=est.model).load("model.pt")
with contextlib.redirect_stdout(io.StringIO()):
    pc_full, pc_segments, _ = est.extract_point_clouds(depth, K, segmap=mask.astype(np.int32), rgb=None, skip_border_objects=False, z_range=[0.2, 1.8])
seg = pc_segments.get(1)
if seg is None or len(seg) == 0:
    sys.exit("no points in the object mask")
ext = seg.max(0) - seg.min(0)
print("object: %d visible points, visible size x %.1f  y %.1f  z %.1f cm (includes side faces and edge pixels)" % (len(seg), *(100 * ext)))
top = seg[seg[:, 2] <= np.percentile(seg[:, 2], 5) + 0.012]          # the points closest to the camera = the top face
centre = top.mean(0); top_ext = top.max(0) - top.min(0)
print("top face: %d points, middle (%.3f, %.3f, %.3f) m in the camera frame, size x %.1f  y %.1f cm   <- offsets below are measured from this middle" % (len(top), *centre, 100 * top_ext[0], 100 * top_ext[1]))
Rbc = None
try:
    Rbc = np.array(json.load(open("/tmp/grasp_debug.json"))["T_base_cam"])[:3, :3]
except Exception:
    pass

rows = []
for attempt in range(1, a.tries + 1):
    with contextlib.redirect_stdout(io.StringIO()):
        pred, scores, contact, openings = est.predict_scene_grasps(pc_full, pc_segments=pc_segments, local_regions=True, filter_grasps=True, forward_passes=a.passes)
    for k, poses in pred.items():
        for i, T in enumerate(poses):
            p, ax = T[:3, 3], T[:3, 2]; close = p + CLOSE * ax; cp = np.asarray(contact[k][i]).ravel()[:3]
            w = float(openings[k][i]) if openings and k in openings else float("nan")
            tilt = float(np.degrees(np.arccos(np.clip(-(Rbc @ ax)[2], -1, 1)))) if Rbc is not None else float("nan")
            dm = 100 * (close - centre)
            rows.append((float(scores[k][i]), 100 * w, 100 * np.linalg.norm(close - cp), 100 * (cp - centre), dm, tilt, float(np.hypot(dm[0], dm[1]))))
    if rows:
        print("grasps found on try %d of %d (forward_passes=%d)" % (attempt, a.tries, a.passes)); break
if not rows:
    sys.exit("no grasps in %d tries - nothing to analyse (that alone says the cube is hard for this network)" % a.tries)
N = len(rows)
off = np.array([r[6] for r in rows]); sc = np.array([r[0] for r in rows])
print("\n%d grasps in total. Closing centre within  1 cm: %d   1.5 cm: %d   2 cm: %d   3 cm: %d   of the top-face middle (sideways in the image plane)" %
      (N, (off <= 1).sum(), (off <= 1.5).sum(), (off <= 2).sum(), (off <= 3).sum()))
b = int(np.argmax(sc)); print("the best-scoring grasp (score %.3f) closes %.1f cm from that middle; the median grasp closes %.1f cm from it." % (sc[b], off[b], float(np.median(off))))
cen = [r for r in rows if r[6] <= 2.0]
if cen:
    print("best score among grasps within 2 cm of the middle: %.3f  (tilt %.0f deg)" % (max(r[0] for r in cen), [r for r in cen if r[0] == max(x[0] for x in cen)][0][5]))
rows.sort(key=lambda r: r[6])
print("\nthe 15 grasps CLOSEST to the top-face middle:")
print("  score  open  tilt(base) | sideways dist | closing centre vs top-face middle (cm)   [camera frame x right, y down, z forward]")
for s_, w, cc, dc, dm, tilt, o in rows[:15]:
    print("  %.3f  %4.1f  %6.0f     |   %5.1f cm   | %+6.1f %+6.1f %+6.1f" % (s_, w, tilt, o, *dm))
w_med = float(np.nanmedian([r[1] for r in rows])); cube_w = 100 * max(top_ext[0], top_ext[1])
print("\nmedian predicted opening %.1f cm; the top face is %.1f cm across." % (w_med, cube_w))
if w_med < 0.5 * cube_w:
    print("-> the openings are much NARROWER than the object: the network is pinching edges/corners, not holding the body. That fits grasps that close beside the middle.")
elif w_med > cube_w + 1.5:
    print("-> the network opens about %.1f cm wider than the object (centre pushed sideways by about %.1f cm)." % (w_med - cube_w, w_med / 2 - cube_w / 2))
else:
    print("-> the opening is about the object's size.")
