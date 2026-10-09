"""
upsample.py - make a denser copy of a saved detection (test of the "point density" hypothesis, docs/10).

    python upsample.py <factor> <depth.npy> <segmap.npy> "<K as 9 numbers>"
    e.g. python upsample.py 2 /tmp/last_depth.npy /tmp/last_segmap.npy "[488.77835083, 0, 317.80917358, 0, 488.77835083, 216.84880066, 0, 0, 1]"

Writes /tmp/up_depth.npy and /tmp/up_segmap.npy and prints the matching K_up to pass to the viewer. Bilinear for depth
(never across holes), nearest for the mask; K is scaled with the half-pixel convention so 3D positions are preserved
(checked: median difference 0.13 mm). Runs anywhere with numpy and OpenCV (the contact_graspnet env has both).
"""
import numpy as np, cv2, sys
S = int(sys.argv[1]) if len(sys.argv) > 1 else 2
src_d, src_m = sys.argv[2], sys.argv[3]
K = np.array(eval(sys.argv[4])).reshape(3, 3)
d = np.load(src_d).astype(np.float32); m = np.load(src_m)
H, W = d.shape
du = cv2.resize(d, (W * S, H * S), interpolation=cv2.INTER_LINEAR)
du[cv2.resize(d, (W * S, H * S), interpolation=cv2.INTER_NEAREST) <= 0.05] = 0      # no interpolated points over holes
mu = cv2.resize(m.astype(np.uint8), (W * S, H * S), interpolation=cv2.INTER_NEAREST).astype(np.int32)
Ku = K.copy(); Ku[0, 0] *= S; Ku[1, 1] *= S; Ku[0, 2] = S * (K[0, 2] + 0.5) - 0.5; Ku[1, 2] = S * (K[1, 2] + 0.5) - 0.5
np.save("/tmp/up_depth.npy", du.astype(np.float64)); np.save("/tmp/up_segmap.npy", mu)
print("upsampled %dx: depth %s, mask pixels %d -> %d" % (S, du.shape, int((m > 0).sum()), int((mu > 0).sum())))
print("K_up =", "[" + ", ".join("%.8f" % x for x in Ku.reshape(-1)) + "]")
