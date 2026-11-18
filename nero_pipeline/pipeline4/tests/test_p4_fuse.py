"""Offline tests for p4_fuse_node's pure numpy parts (no ROS needed). Run: python3 test_p4_fuse.py"""
import os, sys, math, types
import numpy as np
_here = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(_here, ".."), os.path.join(_here, "..", "pipeline4")]   # installed (pipeline4/tests) or bundle layout
import p4_fuse_node as F

rng = np.random.default_rng(1)
results = []
def check(name, ok, info=""):
    results.append(ok); print("%-4s %s %s" % ("PASS" if ok else "FAIL", name, info))

def fake_msg(pts, padded=True):
    step = 16 if padded else 12
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3, "offsets": [0, 4, 8], "itemsize": step})
    rec = np.zeros(len(pts), dt); rec["x"], rec["y"], rec["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
    return types.SimpleNamespace(fields=[types.SimpleNamespace(name=n, offset=o) for n, o in (("x", 0), ("y", 4), ("z", 8))],
                                 point_step=step, width=len(pts), height=1, data=rec.tobytes())

# 1 parsing, NaN/zero removal, stride
P = rng.uniform(-1, 1, (1000, 3)); P[3] = np.nan; P[9] = 0
check("parse drops NaN and zero points", len(F.cloud_xyz(fake_msg(P))) == 998)
check("parse with stride 2", len(F.cloud_xyz(fake_msg(P), stride=2)) == 500)
check("parse unpadded 12-byte points", len(F.cloud_xyz(fake_msg(P, padded=False))) == 998)

# 2 range filter on optical depth
p = np.array([[0, 0, 0.05], [0, 0, 0.2], [0, 0, 0.6]])
check("wrist range 0.07-0.50 keeps only the middle point", len(F.range_filter(p, 0.07, 0.50)) == 1)

# 3 camera above the table looking down sees a table point at the right place, with offset
R = F.qmat(1.0, 0.0, 0.0, 0.0)                      # optical Z -> base -Z
q = F.to_base(np.array([[0.03, 0.0, 0.40]]), R, (-0.36, -0.02, 0.40))
check("camera-down transform puts the table point at z=0", np.allclose(q, [[-0.33, -0.02, 0.0]]), str(np.round(q, 4)))
q2 = F.to_base(np.array([[0.03, 0.0, 0.40]]), R, (-0.36, -0.02, 0.40), offset=(0, 0, -0.01))
check("top offset -1 cm lowers the point by 1 cm", np.allclose(q2[0, 2], -0.01))

# 4 crop
box = (-0.9, 0.1, -0.6, 0.6, -0.03, 0.6)
p = np.array([[-0.4, 0, 0.0], [-0.4, 0, -0.10], [0.5, 0, 0.1], [-0.4, 0.9, 0.1]])
check("crop keeps only the in-workspace point", len(F.crop(p, box)) == 1)

# 5 self filter: a vertical arm segment removes points near it, keeps table points away from it
chain = [(0, 0, 0), (0, 0, 0.4), (-0.2, 0, 0.4)]
p = np.array([[0.03, 0, 0.2], [-0.1, 0.02, 0.41], [-0.4, 0, 0.0], [-0.1, 0.2, 0.4]])
out = F.self_filter(p, chain, 0.07)
check("self filter removes the 2 points on the arm, keeps the 2 others", len(out) == 2 and np.allclose(out[0], [-0.4, 0, 0]))
check("self filter with no TF chain keeps everything", len(F.self_filter(p, [], 0.07)) == 4)

# 6 fusion: overlap -> BOTH, top-only, wrist-only; voxel centres
table = np.c_[rng.uniform(-0.5, -0.2, (5000, 2)), np.zeros(5000)]
top = table[table[:, 0] < -0.30]                        # top sees x < -0.30
wrist = table[table[:, 0] > -0.40]                      # wrist sees x > -0.40  -> overlap in (-0.40, -0.30)
cen, fl = F.fuse_voxels(top, wrist, 0.01)
both_x = cen[fl == F.BOTH][:, 0]
check("fusion labels: all three sources present", set(np.unique(fl)) == {1, 2, 3})
check("BOTH voxels lie in the overlap band", both_x.min() > -0.41 and both_x.max() < -0.29, "%.3f..%.3f" % (both_x.min(), both_x.max()))
check("one voxel per occupied 1 cm cell (no duplicates)", len(np.unique(np.floor(cen / 0.01), axis=0)) == len(cen))
c0, f0 = F.fuse_voxels(np.zeros((0, 3)), np.zeros((0, 3)), 0.01)
check("fusion of two empty clouds is empty", len(c0) == 0 and len(f0) == 0)
c1, f1 = F.fuse_voxels(top, np.zeros((0, 3)), 0.01)
check("wrist missing -> everything top-only", set(np.unique(f1)) == {1})

# 7 packing for RViz
b = F.pack_xyzrgb(cen, fl)
check("packed size is 16 bytes per voxel", len(b) == 16 * len(cen))
rec = np.frombuffer(b, dtype=np.dtype({"names": ["x", "rgb"], "formats": ["<f4", "<u4"], "offsets": [0, 12], "itemsize": 16}))
check("colours: magenta for BOTH, blue for top-only",
      rec["rgb"][fl == F.BOTH][0] == (230 << 16 | 0 << 8 | 230) and rec["rgb"][fl == F.TOP_ONLY][0] == (0 << 16 | 120 << 8 | 255))
check("x survives packing", np.allclose(rec["x"], cen[:, 0].astype(np.float32)))

# 8 speed on a full-size cloud (Jetson will be slower; this is a laptop-class reference)
import time
big_t = np.c_[rng.uniform(-0.8, 0.0, (150000, 2)), rng.uniform(0, 0.1, 150000)]
big_w = np.c_[rng.uniform(-0.5, -0.3, (100000, 2)), rng.uniform(0, 0.1, 100000)]
t0 = time.time(); F.fuse_voxels(F.self_filter(big_t, chain, 0.07), F.self_filter(big_w, chain, 0.07), 0.01); dt = time.time() - t0
check("250k points filtered + fused in under 1.5 s here", dt < 1.5, "%.2f s" % dt)

print("\n%d/%d passed" % (sum(results), len(results)))
sys.exit(0 if all(results) else 1)
