"""
p3_fingers.py - where the two fingertips of a grasp really are. Shared by the relay (p3_jaw_fix.py) and the picture (p3_picture.py), so both check
the same thing. No ROS in here.

A grasp pose as MoveIt receives it (after the relay's 90-deg jaw turn) is the tcp_link target: its Z axis is the approach, its Y axis the line the
REAL Nero fingers close along (docs/01, verified in RViz). The fingertips are FINGERTIP_REACH along the approach and, because MoveIt opens the
gripper fully before the approach, OPEN_WIDTH_M apart along Y. When a grasp is ROLLED (that line not level with the table), one fingertip is lower
than the centre line - the case "one finger hits the table while the other flies". Checking only the centre missed it.
"""
import os

import numpy as np

P3_FINGERS_VERSION = "p3-fingers-v2"   # v2: the Nero opens to 100 mm (MoveIt log: gripper 0.100, fingers +-0.050)
FINGERTIP_REACH = 0.138                                             # m, grasp origin -> fingertips along the approach (gripper model)
OPEN_WIDTH_M = float(os.environ.get("P3_GRIPPER_OPEN_M", "0.10"))  # m, fingertip to fingertip when fully open (gripper_open = 0.100)
JAW_AXIS = 1                                                        # the pose's Y axis = the real Nero jaw line


def fingertips(origin, R, reach=FINGERTIP_REACH, open_width=OPEN_WIDTH_M, jaw_axis=JAW_AXIS):
    """origin (3,), R (3x3, columns = the pose's axes, both in base_link) -> the two fingertip points (2 x 3)."""
    o = np.asarray(origin, dtype=float)
    R = np.asarray(R, dtype=float)
    centre = o + reach * R[:, 2]
    half = 0.5 * open_width * R[:, jaw_axis]
    return np.array([centre - half, centre + half])


def lowest_tip_z(origin, R, **kw):
    return float(fingertips(origin, R, **kw)[:, 2].min())


def roll_deg(R, jaw_axis=JAW_AXIS):
    """How far the jaw line is from level with the table (0 = both fingertips at the same height)."""
    j = np.asarray(R, dtype=float)[:, jaw_axis]
    return float(np.degrees(np.arcsin(min(1.0, abs(j[2]) / max(1e-12, np.linalg.norm(j))))))
