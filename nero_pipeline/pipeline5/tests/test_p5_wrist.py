"""Offline tests for p4_wrist_obstacles (no ROS). Run: python3 test_p4_wrist.py"""
import os, sys, types, time
import numpy as np
_here = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(_here, ".."), os.path.join(_here, "..", "pipeline5")]
import p5_wrist_obstacles as W

rng = np.random.default_rng(2)
results = []
def check(name, ok, info=""):
    results.append(bool(ok)); print("%-4s %s %s" % ("PASS" if ok else "FAIL", name, info))

def surface(x0, x1, y0, y1, z0, z1, n):
    return np.c_[rng.uniform(x0, x1, n), rng.uniform(y0, y1, n), rng.uniform(z0, z1, n)]

TZ = -0.0109                                   # your real table height
table = surface(-0.60, -0.15, -0.30, 0.30, TZ - 0.002, TZ + 0.002, 40000)
bar = surface(-0.45, -0.25, -0.02, 0.02, TZ + 0.35, TZ + 0.37, 3000)           # MID-AIR bar, 35 cm up (pipeline 3 ignores > 30 cm)
cube = surface(-0.42, -0.37, 0.12, 0.17, TZ, TZ + 0.05, 2000)                   # the object being picked
sink_wall = surface(-0.36, -0.20, -0.21, -0.20, TZ, TZ + 0.12, 2000)            # a sink wall near the place point
sink_floor = surface(-0.34, -0.22, -0.20, -0.08, TZ + 0.01, TZ + 0.012, 2000)
noise = np.c_[rng.uniform(-0.6, -0.2, 6), rng.uniform(-0.3, 0.3, 6), rng.uniform(0.1, 0.5, 6)]   # isolated specks
place = np.array([-0.28, -0.14, TZ + 0.011])
tbox = W.target_box(cube, TZ)
P = np.vstack([table, bar, cube, sink_wall, sink_floor, noise])

t0 = time.time(); boxes, note = W.wrist_boxes(P, TZ, tbox, place); dt = time.time() - t0
print("   ", W.describe(boxes, note))

def covered(pts, boxes):
    c = np.zeros(len(pts), bool)
    for b in boxes:
        lo = np.array(b["centre"]) - np.array(b["size"]) / 2; hi = np.array(b["centre"]) + np.array(b["size"]) / 2
        c |= np.all((pts >= lo) & (pts <= hi), axis=1)
    return c

check("mid-air bar fully covered by boxes", covered(bar, boxes).all())
bar_boxes = [b for b in boxes if b["bottom_z"] > TZ + 0.25]
check("bar boxes stay in mid-air (bottom above 30 cm, not down to the table)", bar_boxes and min(b["bottom_z"] for b in bar_boxes) > TZ + 0.30,
      "bottom %.3f" % min(b["bottom_z"] for b in bar_boxes))
under = surface(-0.42, -0.28, -0.01, 0.01, TZ + 0.05, TZ + 0.30, 500)
check("space UNDER the bar stays free", not covered(under, boxes).any())
check("table is not an obstacle", not covered(table[np.abs(table[:, 0] + 0.55) < 0.03], boxes).any())
check("object being picked is left out", not covered(cube, boxes).any())
wall_up = sink_wall[sink_wall[:, 2] > TZ + W.BOTTOM_GAP]          # boxes stop 2 mm above the table on purpose (never overlap the table box)
check("sink wall IS an obstacle, down to 2 mm above the table", covered(wall_up, boxes).all(), "%.1f%% covered" % (100 * covered(wall_up, boxes).mean()))
land = surface(place[0] - 0.04, place[0] + 0.04, place[1] - 0.04, place[1] + 0.04, TZ + 0.03, TZ + 0.25, 300)
check("landing zone above the place point stays free", not covered(land, boxes).any())
check("isolated noise specks removed", not covered(noise, boxes).any())
check("boxes never reach into the table box", min(b["bottom_z"] for b in boxes) >= TZ + W.BOTTOM_GAP - 1e-9)
check("few boxes (merged)", len(boxes) <= 20, "%d boxes" % len(boxes))
check("fast enough", dt < 2.0, "%.2f s for %d points" % (dt, len(P)))

# merge exactness: union of merged blocks == the voxel set
vox = W.voxelize(np.vstack([bar, sink_wall]))
blocks = W.merge_boxes(set(vox))
rebuilt = set((i, j, k) for i0, i1, j0, j1, k0, k1 in blocks for i in range(i0, i1 + 1) for j in range(j0, j1 + 1) for k in range(k0, k1 + 1))
check("merged boxes cover exactly the voxels (no more, no less)", rebuilt == set(vox), "%d voxels -> %d boxes" % (len(vox), len(blocks)))

# an object touching the picked object's box is left out with it (pipeline 3 v6 rule)
handle = surface(-0.37, -0.30, 0.14, 0.15, TZ + 0.03, TZ + 0.04, 400)           # sticks out of the cube
b2, n2 = W.wrist_boxes(np.vstack([cube, handle]), TZ, tbox, None)
check("a part connected to the picked object is left out too", not b2 and n2["target_groups"] >= 1)

# v3: an object lying against a sink wall: the wall stays an obstacle (except right next to the object), a short handle still goes
wall = surface(-0.43, -0.28, 0.175, 0.185, TZ, TZ + 0.12, 3000)                  # touches the cube's box (cube y 0.12-0.17, margin 3 cm)
b5, n5 = W.wrist_boxes(np.vstack([cube, wall]), TZ, tbox, None)
far_wall = wall[(wall[:, 0] < tbox[0][0] - 0.03) | (wall[:, 0] > tbox[1][0] + 0.03)]
check("sink wall touching the object stays an obstacle away from it", n5["split_groups"] == 1 and covered(far_wall[far_wall[:, 2] > TZ + W.BOTTOM_GAP], b5).all(),
      "%d boxes, split %d" % (len(b5), n5["split_groups"]))
check("...and the object itself is still not an obstacle", not covered(cube, b5).any())

# no place point and no target: everything above the table becomes boxes
b3, _ = W.wrist_boxes(np.vstack([table, bar]), TZ, None, None)
check("without target/place: the bar still becomes boxes", covered(bar, b3).all())
b4, n4 = W.wrist_boxes(np.zeros((0, 3)), TZ, tbox, place)
check("no points -> no boxes, no crash", b4 == [] and n4["voxels"] == 0)

# image helpers: 16UC1 depth with row padding, mono8 mask, a camera looking straight down
h, w = 48, 64
depth_mm = np.full((h, w), 870, np.uint16)
padded = np.zeros((h, w + 4), np.uint16); padded[:, :w] = depth_mm
dmsg = types.SimpleNamespace(data=padded.tobytes(), height=h, width=w, step=(w + 4) * 2, encoding="16UC1")
d = W.depth_to_m(dmsg)
check("16UC1 depth with row padding decodes to metres", d.shape == (h, w) and np.allclose(d, 0.87))
fmsg = types.SimpleNamespace(data=np.full((h, w), 0.5, np.float32).tobytes(), height=h, width=w, step=w * 4, encoding="32FC1")
check("32FC1 depth decodes", np.allclose(W.depth_to_m(fmsg), 0.5))
mask = np.zeros((h, w), np.uint8); mask[20:28, 30:38] = 255
mmsg = types.SimpleNamespace(data=mask.tobytes(), height=h, width=w, step=w)
check("mono8 mask decodes", (W.image_to_array(mmsg, np.uint8) == mask).all())
K = [[60, 0, 32], [0, 60, 24], [0, 0, 1]]
R = W.np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)                       # optical Z down
pts = W.mask_points_base(mask, d, K, R, (-0.365, -0.018, 0.856))
check("mask pixels land on the table (z = 0.856 - 0.87)", np.allclose(pts[:, 2], -0.014))
tb = W.target_box(pts, TZ)
check("target box spans the mask, from the table up", tb[0][2] == TZ and tb[1][2] > -0.014 and tb[0][0] < pts[:, 0].min())
check("place point = middle of the disc", np.allclose(W.place_point(pts)[:2], np.median(pts, axis=0)[:2]))

print("\n%d/%d passed" % (sum(results), len(results)))
sys.exit(0 if all(results) else 1)
