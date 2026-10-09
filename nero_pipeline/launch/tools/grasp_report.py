"""
grasp_report.py - numbers instead of a picture: where does each predicted grasp close, relative to the object?

Run in the contact_graspnet conda env, from ~/contact_graspnet_pytorch (it imports contact_graspnet_pytorch):
    conda activate contact_graspnet
    cd ~/contact_graspnet_pytorch
    python ~/nero_pipeline_launch/tools/grasp_report.py

Input : /tmp/last_depth.npy and /tmp/last_segmap.npy (written by contact_graspnet_ros on every detection).
Output: for the 12 best grasps: score, predicted opening (cm), tilt from the camera axis, and the distance of the
        contact point from the object centre along / across its long axis (cm). For a cube both are just
        sideways distances. Values above about 2 cm mean the grasp is not on the object.
The camera intrinsics K are hard-coded below for our aligned 640x480 depth; edit them if the camera changes.
"""
import numpy as np
from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator
from contact_graspnet_pytorch import config_utils
from contact_graspnet_pytorch.checkpoints import CheckpointIO
import os

CKPT = os.path.expanduser("~/contact_graspnet_pytorch/checkpoints/contact_graspnet")
K = np.array([488.77835083, 0, 317.80917358, 0, 488.77835083, 216.84880066, 0, 0, 1]).reshape(3, 3)
PASSES = 5

def analyse(centroid, axis_long, axis_short, grasps):
    """grasps: list of (score, opening, approach_vec, contact_pt). prints one line per grasp."""
    print("rank score  open   tilt  along-carrot  across-carrot   (contact point vs carrot centre, cm)")
    for i, (sc, op, a, cp) in enumerate(sorted(grasps, key=lambda g: -g[0])[:12]):
        d = cp - centroid
        tilt = np.degrees(np.arccos(np.clip(a[2], -1, 1)))
        print("%4d %.3f %5.1f %5.0f %10.1f %13.1f" % (i, sc, 100 * (op if op is not None else np.nan), tilt, 100 * d @ axis_long, 100 * d @ axis_short))

def carrot_axes(pts):
    c = np.median(pts, axis=0)
    u, s, vt = np.linalg.svd(pts - c, full_matrices=False)
    return c, vt[0], vt[1], s

if __name__ == "__main__":
    depth = np.load("/tmp/last_depth.npy"); segmap = np.load("/tmp/last_segmap.npy")
    cfg = config_utils.load_config(CKPT, batch_size=PASSES)
    est = GraspEstimator(cfg)
    CheckpointIO(checkpoint_dir=CKPT + "/checkpoints", model=est.model).load("model.pt")
    pc_full, pc_seg, _ = est.extract_point_clouds(depth, K, segmap=segmap, rgb=None, skip_border_objects=False, z_range=[0.2, 1.8])
    grasps, scores, contacts, openings = est.predict_scene_grasps(pc_full, pc_segments=pc_seg, local_regions=True, filter_grasps=True, forward_passes=PASSES)
    for seg, pts in pc_seg.items():
        c, ax_l, ax_s, sv = carrot_axes(pts)
        print("segment", seg, ": %d points, centre (camera frame) %s, extent along long/short axis: %.1f / %.1f cm (approx)" % (len(pts), np.round(c, 3), 2 * 100 * sv[0] / np.sqrt(len(pts)) * 1.7, 2 * 100 * sv[1] / np.sqrt(len(pts)) * 1.7))
        gl = [(scores[seg][i], (openings[seg][i] if openings else None), grasps[seg][i][:3, 2], contacts[seg][i]) for i in range(len(grasps[seg]))]
        analyse(c, ax_l, ax_s, gl)
