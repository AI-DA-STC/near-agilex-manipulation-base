"""
p3_tablez.py - ONE place for the real table height (base_link z of the table top, metres) used by every pipeline 3 tool:
MoveIt's table box (p3_ready.sh), the fingertip check (p3_picture.py), the tip lift (p3_jaw_fix.py) and the calibration check (p3_calib_check.py).

The robot stands on a plate, so the table the objects stand on is BELOW base_link's zero by the plate's thickness (e.g. -0.0127 for half an inch).
Set it once:   /usr/bin/python3 ~/nero_pipeline_launch/pipeline3/p3_calib_check.py --set-table-z -0.0127
It is stored in ~/.p3_table_z. P3_TABLE_Z in the environment overrides the file (for a one-off test).
"""
import os

P3_TABLEZ_VERSION = "p3-tablez-v4"
TABLE_Z_FILE = os.path.expanduser("~/.p3_table_z")
TIP_MIN_FILE = os.path.expanduser("~/.p3_tip_min")      # the MAXIMUM fingertip clearance above the table (m) - used for tall objects
DEFAULT_TIP_MIN = 0.015                                   # 1.5 cm by default (5 mm let the fingers touch the table on the real arm)
TIP_FLOOR_FILE = os.path.expanduser("~/.p3_tip_floor")   # v4: the MINIMUM fingertip clearance (m) - used for thin objects (a pen)
DEFAULT_TIP_FLOOR = 0.004                                 # 4 mm by default
TIP_FRACTION = 0.4                                        # v4: clearance = 40% of the object's height, between the minimum and the maximum


def table_z():
    """-> (table top z in base_link [m], where the value came from)"""
    env = os.environ.get("P3_TABLE_Z", "").strip()
    if env:
        return float(env), "P3_TABLE_Z"
    try:
        with open(TABLE_Z_FILE) as f:
            return float(f.read().strip()), TABLE_Z_FILE
    except (OSError, ValueError):
        return 0.0, "default 0 (no %s yet)" % TABLE_Z_FILE


def tip_min():
    """-> (minimum fingertip clearance above the table [m], where it came from). Used by the tip lift AND the fingertip check."""
    env = os.environ.get("P3_TIP_MIN_M", "").strip()
    if env:
        return float(env), "P3_TIP_MIN_M"
    try:
        with open(TIP_MIN_FILE) as f:
            return float(f.read().strip()), TIP_MIN_FILE
    except (OSError, ValueError):
        return DEFAULT_TIP_MIN, "default %.0f mm (no %s)" % (1000 * DEFAULT_TIP_MIN, TIP_MIN_FILE)


def tip_floor():
    """-> (minimum fingertip clearance [m], where it came from)"""
    env = os.environ.get("P3_TIP_FLOOR_M", "").strip()
    if env:
        return float(env), "P3_TIP_FLOOR_M"
    try:
        with open(TIP_FLOOR_FILE) as f:
            return float(f.read().strip()), TIP_FLOOR_FILE
    except (OSError, ValueError):
        return DEFAULT_TIP_FLOOR, "default %.0f mm" % (1000 * DEFAULT_TIP_FLOOR)


def clearance_for(object_height_m):
    """v4: how far above the table the fingertips must stay for an object of this height (m):
    TIP_FRACTION of the height, at least tip_floor(), at most tip_min(). An unknown height (None or <= 0) gets the maximum (the safe side).
    -> (clearance [m], one-line explanation)"""
    hi, lo = tip_min()[0], tip_floor()[0]
    lo = min(lo, hi)
    if object_height_m is None or object_height_m <= 0:
        return hi, "%.0f mm (object height unknown: the maximum)" % (1000 * hi)
    c = min(hi, max(lo, TIP_FRACTION * object_height_m))
    why = "minimum" if c == lo else "maximum" if c == hi else "%.0f%% of its height" % (100 * TIP_FRACTION)
    return c, "%.0f mm for a %.1f cm tall object (%s; rule: %.0f%% of the height, %.0f to %.0f mm)" % (
        1000 * c, 100 * object_height_m, why, 100 * TIP_FRACTION, 1000 * lo, 1000 * hi)


def set_tip_floor(value_m, path=None):
    path = path or TIP_FLOOR_FILE
    v = float(value_m)
    if not 0.002 <= v <= 0.03:
        raise SystemExit("STOP: a minimum fingertip clearance of %.1f mm is not plausible (expected 2 to 30 mm; give metres, e.g. 0.004)" % (1000 * v))
    with open(path, "w") as f:
        f.write("%.4f\n" % v)
    return v


def set_tip_min(value_m, path=None):
    path = path or TIP_MIN_FILE
    v = float(value_m)
    if not 0.002 <= v <= 0.05:
        raise SystemExit("STOP: a fingertip clearance of %.1f mm is not plausible (expected 2 to 50 mm; give metres, e.g. 0.015)" % (1000 * v))
    with open(path, "w") as f:
        f.write("%.4f\n" % v)
    return v


def set_table_z(value_m, path=None):
    path = path or TABLE_Z_FILE
    v = float(value_m)
    if not -0.30 < v < 0.10:
        raise SystemExit("STOP: a table height of %+.4f m is not plausible (expected between -0.30 and +0.10 m)" % v)
    with open(path, "w") as f:
        f.write("%.4f\n" % v)
    return v
