#!/usr/bin/env python3
"""p5_gripper.py - PIPELINE 5: the Nero gripper as boxes, in tcp_link (docs/01): +Z = approach (fingers point along +Z), Y = the line the jaws
close along, X = Y x Z. Origin = the gripper base. No ROS in here.

From docs/01 (AgileX Piper finger meshes): fingers from z = 0.062 (body) to the tips at z = 0.138.
MEASURED (ruler, 2026-11-18): finger width A 20 mm at the tip -> 55 mm at the body, thickness B 7 mm -> 25 mm, grip face C 70 mm from the tip,
body D 165 mm along the jaw line x E 55 mm. Wrist camera + mount + cable: 8 x 8 x 8 cm on the gripper's side (tcp_link +x), where the calibration puts the camera.
ASSUMED: the inside (gripping) faces are flat and parallel (C = 70 mm, nearly the whole finger); the thickness grows OUTWARD, the width
symmetrically. A tapered finger is modelled as FINGER_SEGMENTS stacked boxes, each as big as its body-side end (never smaller than the finger).
Any value can be overridden in ~/.p5_gripper.yaml (lower-case keys, metres); the values in use are printed in every report.
"""
import os

P5_GRIPPER_VERSION = "p5-gripper-v5"   # v5: the camera box where the calibration puts the camera (beside the gripper, tcp_link +x)
# v4: + the wrist camera box (8 x 8 x 8 cm on the back of tcp_link - WRONG side: it hit link5)
# v3: 6 boxes per finger
# v2: measured sizes, tapered fingers as stacked boxes
# v1: provisional sizes, one box per finger

DEFAULTS = {
    "TIP_Z": 0.138,          # docs/01: tcp_link -> fingertips along the approach
    "FINGER_ROOT_Z": 0.062,  # docs/01: tcp_link -> where the fingers leave the body
    "PAD_H": 0.070,          # C: grip face height, from the tip up
    "FINGER_W_TIP": 0.020,   # A at the tip   (across the jaw line, X)
    "FINGER_W_ROOT": 0.055,  # A at the body
    "FINGER_T_TIP": 0.007,   # B at the tip   (along the jaw line, Y, outward from the grip face)
    "FINGER_T_ROOT": 0.025,  # B at the body
    "BODY_Y": 0.165,         # D: body along the jaw line
    "BODY_X": 0.055,         # E: body across the jaw line
    "CAM_CX": 0.0675,        # the wrist camera + mount + cable: an 8 x 8 cm base on the gripper's side face (x = BODY_X/2 = 27.5 mm), 8 cm out
    "CAM_CY": 0.0,           #   along tcp_link +x; centre and size in tcp_link. The calibration puts the D405 at (+89, -14, +22) mm: inside.
    "CAM_CZ": 0.022,
    "CAM_SX": 0.08,
    "CAM_SY": 0.08,
    "CAM_SZ": 0.08,
    "FINGER_SEGMENTS": 6,     # v3: 6 (was 3): closer to the real taper, still never smaller than the finger
    "OPEN_MIN": 0.03,        # smallest opening pipeline 5 asks for
    "OPEN_MAX": 0.10,        # the Nero: joint "gripper" 0.1 = 10 cm
    "APPROACH_M": 0.08,      # MTC: pregrasp 8 cm back along the approach, then a straight line in
}
YAML_PATH = os.path.expanduser("~/.p5_gripper.yaml")


def load(path=YAML_PATH):
    """-> (geometry dict, note). Values from ~/.p5_gripper.yaml override the defaults (lower-case keys)."""
    g, note = dict(DEFAULTS), "measured defaults (%s)" % P5_GRIPPER_VERSION
    try:
        import yaml
        if os.path.exists(path):
            d = yaml.safe_load(open(path)) or {}
            changed = [k for k in DEFAULTS if k.lower() in d]
            for k in changed:
                g[k] = float(d[k.lower()])
            note += "; from %s: %s" % (path, ", ".join(changed) if changed else "nothing")
    except Exception as e:  # a broken file must not stop the pipeline silently: say so
        note += " (could not read %s: %s)" % (path, e)
    g["FINGER_SEGMENTS"] = max(1, int(g["FINGER_SEGMENTS"]))
    return g, note


def pad_mid(g):
    """tcp_link -> middle of the grip face, along the approach."""
    return g["TIP_Z"] - 0.5 * g["PAD_H"]


def finger_size_at(g, s):
    """(width A, thickness B) at distance s (m) from the tip, linear taper."""
    L = g["TIP_Z"] - g["FINGER_ROOT_Z"]
    f = min(1.0, max(0.0, s / L))
    return (g["FINGER_W_TIP"] + f * (g["FINGER_W_ROOT"] - g["FINGER_W_TIP"]),
            g["FINGER_T_TIP"] + f * (g["FINGER_T_ROOT"] - g["FINGER_T_TIP"]))


def boxes(g, open_w, sweep=0.0):
    """The gripper as axis-aligned boxes in tcp_link, opened to open_w: {name: (lo xyz, hi xyz)}; finger boxes are named finger+<k> / finger-<k>
    (k = 0 at the tip). sweep > 0: each box is stretched back along -Z by sweep = the volume it passes through during the straight approach."""
    w2, n = 0.5 * open_w, g["FINGER_SEGMENTS"]
    L = g["TIP_Z"] - g["FINGER_ROOT_Z"]
    out = {}
    for k in range(n):
        s0, s1 = k * L / n, (k + 1) * L / n                  # distance from the tip: this segment spans s0..s1
        fw, ft = finger_size_at(g, s1)                       # as big as its body-side end
        z1, z0 = g["TIP_Z"] - s0, g["TIP_Z"] - s1 - sweep
        out["finger+%d" % k] = ((-0.5 * fw, w2, z0), (0.5 * fw, w2 + ft, z1))
        out["finger-%d" % k] = ((-0.5 * fw, -w2 - ft, z0), (0.5 * fw, -w2, z1))
    out["body"] = ((-0.5 * g["BODY_X"], -0.5 * g["BODY_Y"], -sweep), (0.5 * g["BODY_X"], 0.5 * g["BODY_Y"], g["FINGER_ROOT_Z"]))
    if g.get("CAM_SZ", 0) > 0:
        lo, hi = camera_lo_hi(g)
        out["camera"] = ((lo[0], lo[1], lo[2] - sweep), tuple(hi))
    return out


def camera_lo_hi(g):
    """The camera box's corners in tcp_link."""
    c, h = (g["CAM_CX"], g["CAM_CY"], g["CAM_CZ"]), (0.5 * g["CAM_SX"], 0.5 * g["CAM_SY"], 0.5 * g["CAM_SZ"])
    return [c[i] - h[i] for i in range(3)], [c[i] + h[i] for i in range(3)]


def finger_names(bx):
    return [k for k in bx if k.startswith("finger")]
