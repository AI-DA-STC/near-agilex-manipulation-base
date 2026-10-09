#!/usr/bin/env python3
"""
reach_check.py - can the Nero point its gripper downward at a target? (kinematic check, no collisions)

    python3 reach_check.py X Y Z [--cone 45] [--sweep]

X Y Z   target position of tcp_link in base_link, metres (the place target printed by the pipeline includes the clearance,
        e.g. plate surface 0.025 + PLACE_Z_CLEARANCE_M 0.18 = 0.205)
--cone  how far the gripper axis may tilt from straight down, degrees (the place cone corners reach about 45)
--sweep also print the largest reachable radius along the same bearing, for several cone sizes (slow, about a minute)

Uses random-restart least-squares IK with the joint limits (needs numpy and scipy; system Python is fine).
Result "REACHABLE" means a joint configuration exists; MTC's own IK may still fail near the limit, so keep a margin.
Findings with this tool on 2026-10-08: a downward gripper reaches about 0.58 to 0.60 m at height 0.205 m;
the plate target (-0.583, -0.201, 0.205), radius 0.617 m, was not reachable.
"""
import argparse, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nero_fk as N
from scipy.optimize import least_squares

lo = np.array([a for a, b in N.LIMITS]); hi = np.array([b for a, b in N.LIMITS])
DOWN = np.array([0, 0, -1.0])


def solve(target, cone_deg, starts=300, seed=0):
    rng = np.random.default_rng(seed); best = (1e9, float("nan"))

    def res(q):
        T = N.fk(q); ang = np.arccos(np.clip(T[:3, 2] @ DOWN, -1, 1))
        return np.r_[(T[:3, 3] - target) * 10, max(0.0, ang - np.radians(cone_deg)) * 3]

    for _ in range(starts):
        r = least_squares(res, rng.uniform(lo, hi), bounds=(lo, hi), max_nfev=60)
        T = N.fk(r.x); err = float(np.linalg.norm(T[:3, 3] - target)); ang = float(np.degrees(np.arccos(np.clip(T[:3, 2] @ DOWN, -1, 1))))
        if ang <= cone_deg + 0.5 and err < best[0]:
            best = (err, ang)
        if best[0] < 0.002:
            break
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("x", type=float); ap.add_argument("y", type=float); ap.add_argument("z", type=float)
    ap.add_argument("--cone", type=float, default=45.0); ap.add_argument("--sweep", action="store_true")
    a = ap.parse_args(); t = np.array([a.x, a.y, a.z])
    err, ang = solve(t, a.cone)
    print("target (%.3f, %.3f, %.3f), radius %.3f m, gripper within %.0f deg of straight down: %s  (closest %.0f mm%s)" %
          (a.x, a.y, a.z, np.hypot(a.x, a.y), a.cone, "REACHABLE" if err < 0.003 else "NOT REACHABLE", 1000 * err,
           ", tilt %.0f deg" % ang if ang == ang else ""))
    if a.sweep:
        th = np.arctan2(a.y, a.x)
        print("largest reachable radius along this bearing at z = %.3f m:" % a.z)
        for cone in (10, 20, 30, 45):
            ok = [r for r in np.arange(0.46, 0.66, 0.01) if solve(np.array([r * np.cos(th), r * np.sin(th), a.z]), cone, starts=150)[0] < 0.003]
            print("  tilt <= %2d deg: about %.2f m" % (cone, max(ok) if ok else 0.0))


if __name__ == "__main__":
    main()
