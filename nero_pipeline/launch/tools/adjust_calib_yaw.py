#!/usr/bin/env python3
"""
Applies a small yaw (left/right, viewed from the robot's base looking
outward along the arm) correction directly to the calibration file's
rotation, without needing to redo the whole calibration process.

Usage:
    python3 adjust_calib_yaw.py <degrees>

Positive degrees = one direction, negative = the other - since "which way
is right" depends on viewing convention, just try a small value (e.g. 5),
check the point cloud overlay in RViz, and flip the sign if it moved the
wrong way. Re-run with a different value as many times as needed - it
always applies the correction on top of the CURRENT file, not
cumulatively stacking on old attempts (it reads fresh each time).
"""

import sys
import shutil
from pathlib import Path

import numpy as np
import yaml

CALIB_PATH = Path.home() / ".ros2/easy_handeye2/calibrations/nero_eye_to_hand_calib.calib"


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def matrix_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return [x, y, z, w]


def yaw_matrix(degrees):
    theta = np.radians(degrees)
    c, s = np.cos(theta), np.sin(theta)
    # Rotation about the base_link's own Z (vertical) axis
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1],
    ])


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 adjust_calib_yaw.py <degrees>")
        sys.exit(1)

    degrees = float(sys.argv[1])

    with open(CALIB_PATH) as f:
        content = yaml.safe_load(f)

    t = content["transform"]["translation"]
    r = content["transform"]["rotation"]
    old_quat = [r["x"], r["y"], r["z"], r["w"]]

    R_old = quat_to_matrix(old_quat)
    R_yaw = yaw_matrix(degrees)
    # Apply the yaw correction in base_link's own frame (pre-multiply)
    R_new = R_yaw @ R_old
    new_quat = matrix_to_quat(R_new)

    content["transform"]["rotation"]["x"] = float(new_quat[0])
    content["transform"]["rotation"]["y"] = float(new_quat[1])
    content["transform"]["rotation"]["z"] = float(new_quat[2])
    content["transform"]["rotation"]["w"] = float(new_quat[3])

    backup_path = CALIB_PATH.with_suffix(".calib.bak")
    shutil.copy(CALIB_PATH, backup_path)
    print(f"Backed up original to {backup_path}")

    with open(CALIB_PATH, "w") as f:
        yaml.safe_dump(content, f, default_flow_style=False)

    print(f"Applied {degrees} degree yaw correction.")
    print(f"Old rotation: {old_quat}")
    print(f"New rotation: {new_quat}")
    print()
    print("Now restart the publisher to apply:")
    print("  ros2 launch easy_handeye2 publish.launch.py name:=nero_eye_to_hand_calib")
    print()
    print("If it rotated the WRONG way, restore the backup and try the opposite sign:")
    print(f"  cp {backup_path} {CALIB_PATH}")


if __name__ == "__main__":
    main()
