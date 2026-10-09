"""
nero_fk.py - forward kinematics of the AgileX Nero (7 joints + tcp_link), built from the URDF joint origins.

Validated against RViz: at the home pose [-3.49e-05, 1.6879, 1.75e-05, -0.1366, -0.0759, 0.0025, 0.0404] link7 is at
(-0.57793, -0.02341, 0.10525) and tcp_link (with tcp_offset [0.037, 0, -0.0235, -1.5708, 0, -1.5708]) at (-0.6148, 0.0002, 0.1063),
both identical to the TF values RViz showed.
Joint limits are the URDF limits rounded slightly inward. Collisions are NOT modelled.
"""
import numpy as np
def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]); Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]); Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx
def Tm(xyz, r=(0, 0, 0)):
    T = np.eye(4); T[:3, :3] = rpy(*r); T[:3, 3] = xyz; return T
H = np.pi / 2
JOINTS = [  # (origin xyz, origin rpy)  - exactly the URDF you pasted (all axes are local z)
    ((0, 0, 0.138), (0, 0, 0)), ((0, 0, 0), (H, np.pi, 0)), ((0, -0.31, 0), (-H, 0, np.pi)), ((0, 0, 0), (-H, 0, np.pi)),
    ((0, -0.27001, 0), (H, -H, 0)), ((0, 0, 0), (H, -H, 0)), ((0, -0.0235, 0), (H, 0, 0))]
LIMITS = [(-2.70526, 2.70526), (-1.74, 1.74), (-2.75, 2.75), (-1.01, 2.14), (-2.75, 2.75), (-0.73, 0.95), (-H, H)]
TCP = Tm((0.037, 0, -0.0235), (-1.5708, 0, -1.5708))
def fk(q, tcp=True):
    T = np.eye(4)
    for (xyz, r), qi in zip(JOINTS, q):
        c, s = np.cos(qi), np.sin(qi); Rq = np.eye(4); Rq[:2, :2] = [[c, -s], [s, c]]
        T = T @ Tm(xyz, r) @ Rq
    return T @ TCP if tcp else T
