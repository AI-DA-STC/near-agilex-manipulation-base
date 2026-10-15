import os
HERE = os.path.dirname(os.path.abspath(__file__)); BASE = os.path.dirname(HERE)
import sys, math
sys.path.insert(0, os.path.join(BASE, "target_pose_ros"))
import numpy as np
import plate_geometry as G

K = np.array([[488.778, 0, 317.809], [0, 488.778, 216.849], [0, 0, 1.0]])
H, W = 480, 640
rng = np.random.default_rng(3)
FLOOR, RIM = 0.850, 0.825                                   # plate floor depth, rim 2.5 cm closer to the camera
CU, CV, R = 360.0, 250.0, 90.0                              # true plate centre (pixels) and radius
yy, xx = np.mgrid[0:H, 0:W]
dist = np.hypot(xx - CU, yy - CV)
plate = dist <= R
depth = np.full((H, W), 0.90)                               # table far below
depth[plate] = FLOOR + rng.normal(0, 0.002, plate.sum())
rim = plate & (dist >= R - 7); depth[rim] = RIM + rng.normal(0, 0.002, rim.sum())
out_y, out_x = np.where(plate)
for k in rng.choice(len(out_x), 5, replace=False): depth[out_y[k], out_x[k]] = 0.70      # 5 stray pixels far closer to the camera
true_x = (CU - K[0, 2]) * FLOOR / K[0, 0]; true_y = (CV - K[1, 2]) * FLOOR / K[1, 1]

def err_cm(r): return 100 * math.hypot(r["x"] - true_x, r["y"] - true_y)
fails = []
def check(c, m):
    print(("  PASS  " if c else "  FAIL  ") + m)
    if not c: fails.append(m)

print("true plate centre in the camera frame: x=%.4f y=%.4f, floor depth %.3f, rim %.3f" % (true_x, true_y, FLOOR, RIM))
print("\n=== 1. complete mask")
mask = plate.astype(np.uint8)
res = {c: G.extract_target(mask, depth, K, "floor", c) for c in G.CENTER_MODES}
for c, r in res.items(): print("   %-7s centre error %.2f cm" % (c, err_cm(r)))
check(all(err_cm(r) < 0.6 for r in res.values()), "every centre mode finds the centre of a complete plate (< 6 mm)")
r_floor = G.extract_target(mask, depth, K, "floor", "hull"); r_top = G.extract_target(mask, depth, K, "top_surface", "hull"); r_cen = G.extract_target(mask, depth, K, "centroid", "hull")
print("   z: floor %.3f   top_surface %.3f   centroid %.3f   (true floor %.3f)" % (r_floor["z"], r_top["z"], r_cen["z"], FLOOR))
check(abs(r_floor["z"] - FLOOR) < 0.004, "'floor' reports the plate floor (within 4 mm)")
check(abs(r_top["z"] - 0.70) < 1e-9, "'top_surface' (old behaviour) is fooled by one stray pixel: reports 0.700")
check(r_floor["rim_z"] < FLOOR - 0.015, "the 1% 'rim' estimate ignores the stray pixels but sees the 2.5 cm rim")

print("\n=== 2. an object lying on the plate removes a 50x50 px block from the mask (upper right of the plate)")
m2 = mask.copy(); m2[int(CV - 70):int(CV - 20), int(CU + 15):int(CU + 65)] = 0
r2 = {c: G.extract_target(m2, depth, K, "floor", c) for c in G.CENTER_MODES}
for c, r in r2.items(): print("   %-7s centre error %.2f cm" % (c, err_cm(r)))
check(err_cm(r2["mean"]) > 0.8, "'mean' (old) is pulled away by the hole (> 8 mm off)")
check(err_cm(r2["hull"]) < 0.5 and err_cm(r2["circle"]) < 0.5, "'hull' and 'circle' ignore the hole (< 5 mm)")

print("\n=== 3. a WEDGE of the plate edge is missing from the mask (e.g. the arm covers it)")
m3 = mask.copy(); m3[(xx > CU + 20) & (yy < CV - 20) & plate] = 0
r3 = {c: G.extract_target(m3, depth, K, "floor", c) for c in G.CENTER_MODES}
for c, r in r3.items(): print("   %-7s centre error %.2f cm" % (c, err_cm(r)))
check(err_cm(r3["mean"]) > 1.0, "'mean' (old) is pulled off by more than 1 cm")
check(err_cm(r3["circle"]) < 0.5 and err_cm(r3["bbox"]) < 0.5, "'circle' and 'bbox' survive a missing wedge (the remaining arc still defines the extremes)")
print("\n=== 3b. only a left-hand SLICE (less than half) of the plate is in the mask: nothing can recover the centre")
m3b = mask.copy(); m3b[(xx > CU - 30) & plate] = 0
r3b = {c: G.extract_target(m3b, depth, K, "floor", c) for c in G.CENTER_MODES}
for c, r in r3b.items(): print("   %-7s centre error %.2f cm" % (c, err_cm(r)))
check(all(err_cm(r) > 3.0 for r in r3b.values()), "all modes are off by more than 3 cm, so the picture check (plate_center_check.py) is the real test")
print("   response line for 3b:\n   " + G.describe(G.extract_target(m3b, depth, K, "floor", "hull")))

print("\n=== 4. legacy behaviour preserved: center_mode='mean' + z_mode='top_surface' equals the original formulas")
ys, xs = np.where(mask > 0); d = depth[ys, xs]
old_x = float(np.mean((xs - K[0, 2]) * d / K[0, 0])); old_y = float(np.mean((ys - K[1, 2]) * d / K[1, 1])); old_z = float(np.min(d))
leg = G.extract_target(mask, depth, K, "top_surface", "mean")
check(abs(leg["x"] - old_x) < 1e-12 and abs(leg["y"] - old_y) < 1e-12 and abs(leg["z"] - old_z) < 1e-12, "identical to the original node's numbers")
print("\n=== 5. errors")
for bad, args in (("empty mask", (np.zeros((H, W), np.uint8), depth)), ("all-zero depth", (mask, np.zeros((H, W))))):
    try: G.extract_target(args[0], args[1], K); check(False, bad + " should raise")
    except ValueError as e: check(True, "%s raises: %s" % (bad, e))
try: G.extract_target(mask, depth, K, "nope"); check(False, "bad z_mode should raise")
except ValueError: check(True, "unknown z_mode raises")
print("\nRESULT:", "ALL PASSED" if not fails else "%d FAILED: %s" % (len(fails), fails)); sys.exit(1 if fails else 0)
