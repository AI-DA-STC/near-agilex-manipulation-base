#!/usr/bin/env python3
"""
mask_depth_check.py - is the mask (made on the COLOUR image) lined up with the DEPTH image it is applied to?   numpy only, moves nothing.
Reads /tmp/last_depth.npy (metres) and /tmp/last_segmap.npy (the mask the grasp node used on its last detection).
A cube on a table is CLOSER to the camera than the table around it. The script marks the pixels that are clearly closer than the ring of pixels around
the mask (at least --min_height metres), then slides the mask over the depth image by up to +-12 px and scores the overlap (IoU) for every shift.
If the best shift is not about (0, 0), mask and depth are not aligned by that many pixels. 1 px is about 1.7 mm at 0.84 m, so 10 px is about 17 mm.
    /usr/bin/python3 mask_depth_check.py
"""
import argparse, sys
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--depth", default="/tmp/last_depth.npy"); ap.add_argument("--segmap", default="/tmp/last_segmap.npy")
ap.add_argument("--fx", type=float, default=488.8, help="only used to convert pixels to millimetres")
ap.add_argument("--range", type=int, default=12); ap.add_argument("--min_height", type=float, default=0.015, help="how much closer than the table counts as object (m)")
a = ap.parse_args()
d = np.load(a.depth).astype(float); m = np.load(a.segmap) > 0
valid = np.isfinite(d) & (d > 0.05)


def dilate(mask, r):
    out = mask.copy(); H, W = out.shape
    for _ in range(r):
        p = np.pad(out, 1); out = np.any([p[dy:dy + H, dx:dx + W] for dy in range(3) for dx in range(3)], axis=0)
    return out


if m.sum() == 0:
    sys.exit("the mask is empty")
d3, d7 = dilate(m, 3), dilate(m, 7)
ys, xs = np.nonzero(m)
print("depth %s, %.0f%% valid pixels; mask %d px, bbox x %d-%d y %d-%d" % (d.shape, 100 * valid.mean(), m.sum(), xs.min(), xs.max(), ys.min(), ys.max()))


ring = (dilate(m, 12) & ~dilate(m, 6)) & valid
if ring.sum() < 100:
    sys.exit("too few valid depth pixels around the mask to judge")
ref = float(np.median(d[ring])); window = dilate(m, 12 + a.range)
obj = valid & window & (d < ref - a.min_height)
ins = d[m & valid]
print("table (ring around the mask) %.3f m, masked pixels %.3f m (median) -> object stands %.1f mm above the ring" % (ref, np.median(ins) if len(ins) else float("nan"), 1000 * (ref - np.median(ins)) if len(ins) else float("nan")))
if obj.sum() < 20:
    sys.exit("VERDICT: no pixels stand clearly (%.0f mm) above the ring in this window - the mask is not on a raised object, or depth is too sparse here." % (1000 * a.min_height))


def iou(dx, dy):
    ms = np.roll(np.roll(m, dy, 0), dx, 1) & valid
    union = (ms | obj).sum()
    return float((ms & obj).sum() / union) if union else 0.0


R = a.range
grid = {(dx, dy): iou(dx, dy) for dx in range(-R, R + 1) for dy in range(-R, R + 1)}
i0 = grid[(0, 0)]; best = max(grid, key=lambda k: grid[k])
print("\noverlap (IoU) of the mask with the 'closer than the table' pixels:  as is %.2f   best %.2f at a shift of dx %+d px, dy %+d px  (about %.0f mm sideways at 0.84 m)" %
      (i0, grid[best], best[0], best[1], 1000 * np.hypot(*best) * 0.84 / a.fx))
print("\n  IoU for shifts along x (dy=0) and along y (dx=0):")
print("  shift  " + " ".join("%5d" % s_ for s_ in range(-R, R + 1, 3)))
print("  x      " + " ".join("%5.2f" % grid[(s_, 0)] for s_ in range(-R, R + 1, 3)))
print("  y      " + " ".join("%5.2f" % grid[(0, s_)] for s_ in range(-R, R + 1, 3)))
if max(abs(best[0]), abs(best[1])) <= 2 or grid[best] - i0 < 0.10:
    print("\nVERDICT: mask and depth are ALIGNED (the best shift is within 2 px, or gains less than 0.10 overlap). The 'beside the cube' problem is not a colour/depth misalignment.")
else:
    print("\nVERDICT: the mask looks SHIFTED by about (%+d, %+d) px against the depth (overlap %.2f at the best shift vs %.2f as is). Depth and colour are not lined up, or the colour-to-depth mapping is wrong." %
          (best[0], best[1], grid[best], i0))
