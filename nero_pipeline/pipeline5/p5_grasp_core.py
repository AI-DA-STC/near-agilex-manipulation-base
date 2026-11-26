#!/usr/bin/env python3
"""p5_grasp_core.py - PIPELINE 5, TIER 1: grasps for the Nero gripper made from GraspMolmo's point and the top camera's points.
No training, no Contact-GraspNet, no ROS in here (tested offline). Everything in base_link, metres.

INPUTS
  obj_pts    top-camera points of GraspMolmo's object region (the mask under its point)
  scene_pts  top-camera points around the object (fine, incl. the floor and the neighbours)
  fused_pts  /p4/fused_points-style voxel centres (both cameras, arm removed); may be empty
  gpoint     GraspMolmo's grasp point in 3D
  table_z    MoveIt's table height;  clearance  fingertip clearance above the floor (m), or None = tip_clearance() of the height measured here

STEPS
  1. centre   GraspMolmo's point (snapped onto the object if it is just off it)
  2. floor    what the object stands on, from the points around it (table or a sink floor); height = its top surface - that floor
  3. jaw      jaw angles every 15 deg; the narrowest few are kept, plus the object's narrow and long axes near the point
  4. pose     for each jaw angle x offset along the object x tilt (across the jaws, the jaw line stays LEVEL): grip faces centred on the
              object's mid-height, or as low as the fingertip clearance allows when the object is shorter than the 70 mm grip face; re-centred
              across the jaw on the object, fingertips >= clearance above the floor
  5. checks   opening fits, fingertips above the floor, fingers and gripper body (also along the 8 cm approach) free of the object's top, the
              neighbours (fine top-camera points) and anything in the fused cloud (incl. things above the object)
  6. score    narrow, near GraspMolmo's point, little tilt, room around the fingers, good pad overlap, comfortable reach
  7. pick     the best few distinct ones, with ONE common opening (MTC opens the gripper once) - re-checked at that opening
"""
import math

import numpy as np

import p5_gripper

P5_GRASP_CORE_VERSION = "p5-grasp-core-v8l"   # 7l: a floor measured below the table -> the table is the floor; pads must cover half the height (was 60 %) |   # v8: top-camera points block everything BELOW them (columns); the camera (one side) checked on both sides
# v7: v7: jaws off-centre by up to 8 mm (room beside a wall); fused-cloud margin 3 mm (voxel MEANS now)
# v6: v6: higher grasps (+5/+10 mm) when the low one is blocked; corner limit 1 mm; centring up to 40 mm; blocking points kept
# v5: v5: the wrist camera box is checked too (neighbours, fused cloud)
# v4: v4: jaw angles from the WHOLE object; floor from a 1-3 cm ring around the object (was: 10 cm around the point)
# v3: v3: clearance None = pipeline 5's rule from the LOCAL height; minimum 6 mm
# v2: v2: tapered fingers (measured), width measured where the fingers touch
# v1: one box per finger
DOWN = np.array([0.0, 0.0, -1.0])

CFG = {
    "max_tilt_deg": 20.0,      # tier 1
    "tilt_step_deg": 10.0,
    "open_margin": 0.02,       # opening = object width + this
    "jaw_step_deg": 15.0,
    "n_angles": 3,             # narrowest jaw angles kept (+ the narrow axis)
    "offset_m": 0.01,          # slide along the object: 0 and +-this
    "snap_max": 0.015,         # GraspMolmo's point may be this far off the object
    "recentre_max": 0.04,      # re-centring across the jaw (object asymmetric about the point; v6: 40 mm, closing along a long object needs it)
    "patch_r": 0.05,           # object points this close to the centre (xy) describe it locally
    "slab_extra": 0.005,       # width measured in a slab of finger width + this
    "min_height": 0.008,       # flatter than this: no grasp
    "min_overlap_frac": 0.5,   # pads must cover min(min_overlap, frac * height) of the object (7l: 0.5 - with the 6 mm fingertip clearance, 0.6
                               # made anything under ~15 mm tall ungraspable: a low lying eggplant, 2026-11-24)
    "min_overlap": 0.010,
    "floor_ignore": 0.006,     # fine points this close to the floor are floor (the fingertip clearance handles the floor)
    "fused_floor_ignore": 0.012,
    "fused_obj_dilate": 0.015, # fused voxels this close (xy) to the object and not above its top are the object itself
    "fine_margin": 0.003,      # boxes grown by this for fine points
    "fused_margin": 0.003,     # ... and for fused voxel points (v7: 3 mm - the fusion now gives each voxel's MEAN point, on the surface; was 5 for centres)
    "column_up": 0.50,         # v8: for TOP-camera points the boxes reach this far up: it sees only tops, so what it sees blocks everything below it
    "palm_gap": 0.010,         # the object's top must stay this far below the gripper body
    "palm_noise": 0.004,       # placement allowance for depth noise above the measured top
    "max_hits_fine": 4,        # depth noise tolerance (points)
    "max_hits_fused": 2,
    "min_tip_corner": 0.001,   # no part of a finger lower than this above the floor (v6: 1 mm, was 3; a tilted finger's corner dips a little)
    "raise_steps": (0.0, 0.005, 0.010),
    "shift_steps": (0.0, 0.004, -0.004, 0.008, -0.008),   # v7: the jaws off-centre along the jaw line (the near finger touches first and slides
    "shift_keep": 0.002,       #   the object toward the middle) - room for one finger beside a wall; the object keeps >= shift_keep from each finger
    "w_shift": 0.3,            # cost per cm off-centre   # v6: the same grasp 0 / 5 / 10 mm higher (a finger blocked low - a sink's curved corner - may fit higher)
    "w_raise": 0.3,            # cost per cm higher
    "reach_soft": 0.45, "reach_max": 0.55,
    "n_out": 3,
    "open_spread": 0.015,      # the grasps passed on need at most this much more opening than the best one (MTC opens once, to the widest)
    "w_width": 1.0, "w_point": 0.5, "w_tilt": 0.4, "w_clear": 0.5, "w_overlap": 0.3, "w_reach": 0.5,
}


# ---------------------------------------------------------------- fingertip clearance (pipeline 5's own rule)
def tip_clearance(height, frac=0.25, lo=0.006, hi=0.010):
    """Fingertips above the floor: 25 % of the object's height, 6-10 mm (p3_tablez notes that 5 mm let the fingers touch the table on the real arm). Pipeline 3's rule (40 %, up to 15 mm) was made for Contact-GraspNet's
    lifted grasps; with ~30 mm pads it keeps the pads above mid-height on anything under ~6 cm. None height -> the maximum."""
    if height is None or height <= 0:
        return hi
    return float(min(hi, max(lo, frac * height)))


# ---------------------------------------------------------------- geometry helpers
def jaw_frame(theta, tilt):
    """tcp_link orientation (columns X, Y, Z in base_link): jaw line Y LEVEL at compass angle theta, approach Z tilted by `tilt` (rad) ACROSS the
    jaws from straight down (both fingertips stay at the same height)."""
    y = np.array([math.cos(theta), math.sin(theta), 0.0])
    xl = np.cross(y, DOWN)
    z = math.cos(tilt) * DOWN + math.sin(tilt) * xl
    return np.column_stack([np.cross(y, z), y, z])


def to_frame(P, tcp, R):
    return (np.asarray(P, float).reshape(-1, 3) - tcp) @ R


def in_box(q, box, margin=0.0):
    lo, hi = np.asarray(box[0]) - margin, np.asarray(box[1]) + margin
    return np.all((q >= lo) & (q <= hi), axis=1)


def box_dist(q, box):
    lo, hi = np.asarray(box[0]), np.asarray(box[1])
    d = np.maximum(np.maximum(lo - q, q - hi), 0.0)
    return np.linalg.norm(d, axis=1)


def box_corners(box):
    lo, hi = box
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def angle_mod180(a):
    return a % math.pi


def angle_diff180(a, b):
    d = abs(angle_mod180(a) - angle_mod180(b))
    return min(d, math.pi - d)


class CellSet:
    """xy cells (5 mm) of a point set, grown by `grow` m: fast 'is this point near the object' tests."""
    def __init__(self, pts, grow, cell=0.005):
        self.cell = cell
        c = np.floor(np.asarray(pts, float)[:, :2] / cell).astype(np.int64)
        r = int(math.ceil(grow / cell))
        offs = np.array([(i, j) for i in range(-r, r + 1) for j in range(-r, r + 1) if i * i + j * j <= r * r + r], dtype=np.int64)
        allc = (c[:, None, :] + offs[None, :, :]).reshape(-1, 2) if len(c) else np.zeros((0, 2), np.int64)
        self.codes = np.unique(self._code(allc))

    @staticmethod
    def _code(c):
        return (c[:, 0] + 2 ** 20) * 2 ** 21 + (c[:, 1] + 2 ** 20)

    def contains(self, pts):
        c = np.floor(np.asarray(pts, float)[:, :2] / self.cell).astype(np.int64)
        return np.isin(self._code(c), self.codes)


# ---------------------------------------------------------------- measurements
def local_floor(scene_pts, obj_pts, table_z, rings=((0.01, 0.03), (0.01, 0.05))):
    """What the object stands on: a low percentile (20th) of the points in a narrow ring AROUND ITS OUTLINE (1-3 cm, else 1-5 cm), not the object.
    v4: was everything within 10 cm of the point - in a small sink that reached the table outside it and called the sink floor an obstacle.
    A ring that is mostly wall reads higher -> the fingertips stay higher (the safe side). -> (floor z, note)."""
    if scene_pts is None or len(scene_pts) == 0:
        return table_z, "table (no points around the object)"
    S = np.asarray(scene_pts, float)
    inner = CellSet(obj_pts, rings[0][0]).contains(S)
    for r0, r1 in rings:
        ring = S[~inner & CellSet(obj_pts, r1).contains(S)]
        if len(ring) >= 30:
            f = float(np.percentile(ring[:, 2], 20))
            return f, "measured %.0f-%.0f cm around the object (%+.1f mm vs the table)" % (100 * r0, 100 * r1, 1000 * (f - table_z))
    return table_z, "table (too few points around the object)"


def top_height(pts, floor):
    """The object's top above the floor: the median of its top surface (points within 6 mm of the 95th percentile), as pipeline 3 measures it."""
    z = np.asarray(pts, float)[:, 2]
    p95 = float(np.percentile(z, 95))
    return float(np.median(z[z >= p95 - 0.006]) - floor)


def width_along(xy, c, jaw, slab_half):
    """Object extent along the jaw line through c, from the points in a slab of +-slab_half across it. -> (width, centre shift along jaw) or None."""
    n = np.array([-jaw[1], jaw[0]])
    d = xy - c
    sel = np.abs(d @ n) <= slab_half
    if sel.sum() < 15:
        return None
    s = d[sel] @ jaw
    lo, hi = np.percentile(s, [2, 98])
    return float(hi - lo), float(0.5 * (hi + lo))


def narrow_axis(xy):
    """Compass angle of the object's narrow direction (PCA minor axis) and the elongation ratio."""
    d = xy - xy.mean(0)
    w, v = np.linalg.eigh(d.T @ d / max(1, len(d)))
    minor = v[:, 0]
    return math.atan2(minor[1], minor[0]), float(math.sqrt(max(w[1], 1e-12) / max(w[0], 1e-12)))


# ---------------------------------------------------------------- the generator
def generate(obj_pts, scene_pts, fused_pts, gpoint, table_z, clearance, cfg=None, geom=None):
    """-> dict: ok, why (if not ok), selected (best few, common opening), candidates (all, with reasons), info."""
    C = dict(CFG, **(cfg or {}))
    G = dict(p5_gripper.DEFAULTS, **(geom or {}))
    info = {"version": P5_GRASP_CORE_VERSION, "gripper_version": p5_gripper.P5_GRIPPER_VERSION, "cfg": C, "geom": G}
    fail = lambda why: {"ok": False, "why": why, "selected": [], "candidates": [], "info": info}

    O = np.asarray(obj_pts, float).reshape(-1, 3)
    S = np.zeros((0, 3)) if scene_pts is None else np.asarray(scene_pts, float).reshape(-1, 3)
    F = np.zeros((0, 3)) if fused_pts is None else np.asarray(fused_pts, float).reshape(-1, 3)
    g = np.asarray(gpoint, float)
    if len(O) < 30:
        return fail("too few object points (%d)" % len(O))

    # 1-2. floor, object above it, centre
    floor, floor_note = local_floor(S, O, table_z)
    if floor < table_z:                                          # 7l: an object on the table cannot reach below it - the TABLE is its floor
        floor_note += "; below MoveIt's table by %.0f mm -> the table is used" % (1000 * (table_z - floor))
        floor = table_z
    O = O[O[:, 2] > floor + 0.005]
    if len(O) < 30:
        return fail("the object region is (almost) all at floor height")
    d0 = np.hypot(O[:, 0] - g[0], O[:, 1] - g[1])
    i0 = int(np.argmin(d0))
    if d0[i0] > C["snap_max"]:
        return fail("GraspMolmo's point is %.0f mm from the object (max %.0f mm)" % (1000 * d0[i0], 1000 * C["snap_max"]))
    c0 = g[:2].copy() if d0[i0] < 0.003 else O[i0, :2].copy()
    patch = O[np.hypot(O[:, 0] - c0[0], O[:, 1] - c0[1]) < C["patch_r"]]
    if len(patch) < 30:
        return fail("too few object points near the grasp point (%d)" % len(patch))
    h = top_height(patch, floor)
    floor_tips = max(floor, table_z)            # MoveIt's table box is at table_z: never lower the fingertips below it
    info.update(floor_z=floor, floor_note=floor_note, height=h, centre_xy=c0.tolist(), snapped_mm=1000 * float(d0[i0]),
                object_points=int(len(O)), patch_points=int(len(patch)))
    if clearance is None:
        clearance = tip_clearance(h)
    info["clearance"] = clearance
    if h < C["min_height"]:
        return fail("the object is only %.0f mm tall near the point (min %.0f mm)" % (1000 * h, 1000 * C["min_height"]))
    oxy = O[:, :2]                                # v4: the whole object (the slab keeps it local across the jaw)

    # 3. jaw angles: every jaw_step + the narrow axis; keep the narrowest few
    # the width is measured in a slab as wide as the fingers where they touch the object (they taper: wider higher up)
    slab = 0.5 * p5_gripper.finger_size_at(G, min(h, G["PAD_H"]))[0] + C["slab_extra"]
    minor, elong = narrow_axis(oxy)
    info.update(narrow_axis_deg=math.degrees(angle_mod180(minor)), elongation=elong)
    widths = []
    for a in list(np.arange(0.0, math.pi, math.radians(C["jaw_step_deg"]))) + [angle_mod180(minor)]:
        jaw = np.array([math.cos(a), math.sin(a)])
        r = width_along(oxy, c0, jaw, slab)
        if r is not None:
            widths.append((r[0], a))
    widths.sort()
    angles = []
    for w, a in widths:
        if all(angle_diff180(a, b) >= math.radians(C["jaw_step_deg"]) - 1e-6 for b in angles):
            angles.append(a)
        if len(angles) >= C["n_angles"]:
            break
    for extra in (minor, minor + 0.5 * math.pi):   # the narrow axis, and the long axis (used when the narrow side is blocked and it still fits)
        if all(angle_diff180(extra, b) > math.radians(7.5) for b in angles):
            angles.append(angle_mod180(extra))
    info["jaw_widths_mm"] = {"%.0f" % math.degrees(a): round(1000 * w, 1) for w, a in widths}

    # fine points that are NOT the object (neighbours) and fused voxels that are not the object
    S_up = S[S[:, 2] > floor + C["floor_ignore"]] if len(S) else S
    near_obj = CellSet(O, 0.005)
    S_other = S_up[~near_obj.contains(S_up)] if len(S_up) else S_up
    O_up = O[O[:, 2] > floor + C["floor_ignore"]]
    top_z = floor + h
    if len(F):
        F = F[F[:, 2] > floor + C["fused_floor_ignore"]]
        is_obj = CellSet(O, C["fused_obj_dilate"]).contains(F) & (F[:, 2] <= top_z + C["fused_obj_dilate"])
        F = F[~is_obj]

    # 4. poses
    tilts = [0.0]
    t = C["tilt_step_deg"]
    while t <= C["max_tilt_deg"] + 1e-6:
        tilts += [math.radians(t), -math.radians(t)]
        t += C["tilt_step_deg"]
    offsets = [0.0] + ([C["offset_m"], -C["offset_m"]] if C["offset_m"] > 0 else [])
    cands = []
    for a in angles:
        jaw = np.array([math.cos(a), math.sin(a)])
        along = np.array([-jaw[1], jaw[0]])
        for off in offsets:
            c = c0 + off * along
            r = width_along(O[:, :2], c, jaw, slab)
            base = {"jaw_deg": math.degrees(a), "offset_mm": 1000 * off}
            if r is None:
                cands.append(dict(base, ok=False, why="no object under the jaws at this offset"))
                continue
            w, mid = r
            if abs(mid) > C["recentre_max"]:
                cands.append(dict(base, ok=False, width=w, why="object %.0f mm off-centre between the jaws" % (1000 * abs(mid))))
                continue
            c2 = c + mid * jaw
            own_open = max(G["OPEN_MIN"], w + C["open_margin"])
            for sh in C["shift_steps"]:                           # v7: jaws off-centre along the jaw line (the object stays >= shift_keep from a finger)
                if abs(sh) > 0.5 * (own_open - w) - C["shift_keep"] + 1e-9:
                    continue
                c3 = c2 + sh * jaw
                for tl in tilts:
                    R = jaw_frame(a, tl)
                    ca = math.cos(tl)
                    pc_z0 = max(floor + 0.5 * h,                                              # pads at mid-height, unless
                                floor_tips + clearance + 0.5 * G["PAD_H"] * ca,               # the fingertips would be too low, or
                                top_z + C["palm_gap"] + C["fine_margin"] + C["palm_noise"] - (p5_gripper.pad_mid(G) - G["FINGER_ROOT_Z"]) * ca)   # the body would touch a tall object's top
                    for up in C["raise_steps"]:
                        pc_z = pc_z0 + up
                        pad_lo, pad_hi = pc_z - 0.5 * G["PAD_H"] * ca, pc_z + 0.5 * G["PAD_H"] * ca
                        overlap = min(pad_hi, top_z) - max(pad_lo, floor)
                        pad = np.array([c3[0], c3[1], pc_z])
                        tcp = pad - p5_gripper.pad_mid(G) * R[:, 2]
                        cands.append(dict(base, tilt_deg=math.degrees(tl), raise_mm=1000 * up, shift_mm=1000 * sh, width=w, recentre_mm=1000 * mid,
                                          own_open=own_open, overlap=overlap, pad=pad, tcp=tcp, R=R, ok=True, why=""))

    # 5. checks
    def check(cd, open_w):
        if open_w > G["OPEN_MAX"] + 1e-9:
            return False, "object %.0f mm wide + %.0f mm margin > the gripper's %.0f mm" % (
                1000 * cd["width"], 1000 * C["open_margin"], 1000 * G["OPEN_MAX"]), {}
        need = min(C["min_overlap"], C["min_overlap_frac"] * h)
        if cd["overlap"] < need - 1e-9:
            return False, "pads cover only %.0f mm of the object (need %.0f)" % (1000 * cd["overlap"], 1000 * need), {}
        tcp, R = cd["tcp"], cd["R"]
        bx0, bxs = p5_gripper.boxes(G, open_w), p5_gripper.boxes(G, open_w, sweep=G["APPROACH_M"])
        bcol = p5_gripper.boxes(G, open_w, sweep=C["column_up"])     # for top-camera points (columns): a 30 cm post's top is above the pregrasp
        if "camera" in bxs:
            # v8: the fingers are symmetric, the camera is on ONE side: the same grasp rolled 180 deg about the approach (MTC's 'flip') puts it on
            # the other side. Keep the orientation whose camera box is clearer (MTC still tries both; MoveIt rejects a blocked one).
            def cam_hits(Rx):
                n = 0
                if len(S_other) or len(O_up):
                    P = np.vstack([S_other, O_up]) if len(O_up) else S_other
                    n += int(in_box(to_frame(P, tcp, Rx), bcol["camera"], C["fine_margin"]).sum())
                if len(F):
                    n += int(in_box(to_frame(F, tcp, Rx), bxs["camera"], C["fused_margin"]).sum())
                return n
            Rf = R @ np.diag([-1.0, -1.0, 1.0])
            if cam_hits(R) > 0 and cam_hits(Rf) < cam_hits(R):
                R = Rf
                cd["R"], cd["camera_side_flipped"] = Rf, True
        fn = p5_gripper.finger_names(bx0)
        corners = np.vstack([box_corners(bx0[k]) for k in fn]) @ R.T + tcp
        tip_z = (tcp + G["TIP_Z"] * R[:, 2])[2]
        if tip_z - floor_tips < clearance - 1e-4:
            return False, "fingertips %.0f mm above the floor (need %.0f)" % (1000 * (tip_z - floor_tips), 1000 * clearance), {}
        if corners[:, 2].min() - floor_tips < C["min_tip_corner"]:
            return False, "a finger corner %.0f mm above the floor" % (1000 * (corners[:, 2].min() - floor_tips)), {}
        reach = float(np.hypot(cd["pad"][0], cd["pad"][1]))
        if reach > C["reach_max"]:
            return False, "%.2f m from the arm's base (max %.2f)" % (reach, C["reach_max"]), {}
        # the gripper body must stay palm_gap above the object: grow the body box down (+Z) by palm_gap for the object's points
        body_palm = (bxs["body"][0], (bxs["body"][1][0], bxs["body"][1][1], bxs["body"][1][2] + C["palm_gap"]))
        qO, qS, qF = to_frame(O_up, tcp, R), to_frame(S_other, tcp, R), to_frame(F, tcp, R)
        for side in ("finger+", "finger-"):
            segs = [k for k in fn if k.startswith(side)]
            m = np.any([in_box(qO, bcol[k], C["fine_margin"]) for k in segs], axis=0)
            if int(m.sum()) > C["max_hits_fine"]:
                cd["hit_pts"] = O_up[m][:80].tolist()
                return False, "%s would land on the object (%d points)" % (side, int(m.sum())), {}
            if len(qS):
                m = np.any([in_box(qS, bcol[k], C["fine_margin"]) for k in segs], axis=0)
                if int(m.sum()) > C["max_hits_fine"]:
                    cd["hit_pts"] = S_other[m][:80].tolist()
                    return False, "%s would hit a neighbour (%d top-camera points, up to %.0f mm above the floor)" % (
                        side, int(m.sum()), 1000 * (S_other[m][:, 2].max() - floor)), {}
        n = int(in_box(qO, body_palm, C["fine_margin"]).sum())
        if n > C["max_hits_fine"]:
            return False, "gripper body within %.0f mm of the object's top (%d points)" % (1000 * C["palm_gap"], n), {}
        m = in_box(qS, bcol["body"], C["fine_margin"])
        if int(m.sum()) > C["max_hits_fine"]:
            cd["hit_pts"] = S_other[m][:80].tolist()
            return False, "gripper body would hit a neighbour (%d top-camera points)" % int(m.sum()), {}
        if "camera" in bxs:
            SO_ = np.vstack([S_other, O_up]) if len(O_up) else S_other
            m = in_box(to_frame(SO_, tcp, R), bcol["camera"], C["fine_margin"])
            if int(m.sum()) > C["max_hits_fine"]:
                cd["hit_pts"] = SO_[m][:80].tolist()
                return False, "the wrist camera would hit something (%d top-camera points)" % int(m.sum()), {}
        for name, b in bxs.items():
            m = in_box(qF, b, C["fused_margin"])
            n = int(m.sum())
            if n > C["max_hits_fused"]:
                cd["hit_pts"] = F[m][:80].tolist()
                return False, "%s would hit an obstacle seen in the fused cloud (%d voxels)" % (name.rstrip("0123456789"), n), {}
        q = np.vstack([qS, qF]) if len(qF) else qS
        dmin = min([float(box_dist(q, bx0[k]).min()) for k in fn]) if len(q) else 1.0
        return True, "", {"clear_m": dmin, "reach_m": reach, "tip_mm": 1000 * (tip_z - floor_tips)}

    def cost(cd, m):
        terms = {
            "width": C["w_width"] * cd["width"] / G["OPEN_MAX"],
            "point": C["w_point"] * abs(cd["offset_mm"]) / 20.0,
            "tilt": C["w_tilt"] * abs(cd["tilt_deg"]) / 20.0,
            "clear": C["w_clear"] * max(0.0, 1.0 - m["clear_m"] / 0.02),
            "overlap": C["w_overlap"] * (1.0 - min(1.0, cd["overlap"] / G["PAD_H"])),
            "raise": C["w_raise"] * cd.get("raise_mm", 0.0) / 10.0,
            "shift": C["w_shift"] * abs(cd.get("shift_mm", 0.0)) / 10.0,
            "reach": C["w_reach"] * max(0.0, (m["reach_m"] - C["reach_soft"]) / 0.10),
        }
        return sum(terms.values()), terms

    for cd in cands:
        if not cd["ok"]:
            continue
        ok, why, m = check(cd, cd["own_open"])
        cd["ok"], cd["why"] = ok, why
        if ok:
            cd.update(m)
            cd["cost"], cd["terms"] = cost(cd, m)
    good = sorted([cd for cd in cands if cd["ok"]], key=lambda d: d["cost"])

    # 7. pick: distinct, one common opening, re-checked at that opening
    def distinct(a, b):
        return angle_diff180(math.radians(a["jaw_deg"]), math.radians(b["jaw_deg"])) >= math.radians(10) or np.linalg.norm(a["tcp"] - b["tcp"]) >= 0.01

    sel = []
    for cd in good:
        if len(sel) >= C["n_out"]:
            break
        if not all(distinct(cd, s) for s in sel):
            continue
        if sel and cd["own_open"] > sel[0]["own_open"] + C["open_spread"] + 1e-9:
            cd["note"] = "skipped: needs %.0f mm more opening than the best grasp" % (1000 * (cd["own_open"] - sel[0]["own_open"]))
            continue
        trial = sel + [cd]
        W = max(s["own_open"] for s in trial)
        if all(check(s, W)[0] for s in trial):
            sel = trial
        else:
            cd["note"] = "skipped: fails at the common opening %.0f mm" % (1000 * W)
    open_w = max([s["own_open"] for s in sel], default=None)
    for r, s in enumerate(sel, 1):
        s["rank"], s["open"] = r, open_w
    info["open_width"] = open_w
    if not sel:
        reasons = {}
        for cd in cands:
            reasons[cd["why"].split(" (")[0]] = reasons.get(cd["why"].split(" (")[0], 0) + 1
        return {"ok": False, "why": "no grasp passed: " + "; ".join("%d x %s" % (n, k) for k, n in sorted(reasons.items(), key=lambda kv: -kv[1])),
                "selected": [], "candidates": cands, "info": info}
    return {"ok": True, "why": "", "selected": sel, "candidates": cands, "info": info}


def summary(res, n=12):
    """Readable lines for the log and the terminal."""
    i = res["info"]
    out = ["%s: %s" % (P5_GRASP_CORE_VERSION, "OK" if res["ok"] else "NO GRASP - " + res["why"])]
    if "height" in i:
        out.append("object: %.0f mm tall, floor %s, narrow axis %.0f deg (elongation %.1f), point snapped %.0f mm" % (
            1000 * i["height"], i["floor_note"], i.get("narrow_axis_deg", float("nan")), i.get("elongation", float("nan")), i["snapped_mm"]))
    for s in res["selected"]:
        out.append("  G%d: jaw %3.0f deg, tilt %+3.0f deg, offset %+3.0f mm, jaws off-centre %+2.0f mm, raised %2.0f mm, object %.0f mm wide -> opening %.0f mm, fingertips %.0f mm above the "
                   "floor, %s free around the fingers, cost %.2f" % (s["rank"], s["jaw_deg"], s["tilt_deg"], s["offset_mm"], s.get("shift_mm", 0), s.get("raise_mm", 0), 1000 * s["width"],
                                                                      1000 * s["open"], s["tip_mm"],
                                                                      "> 50 mm" if s["clear_m"] > 0.05 else "%.0f mm" % (1000 * s["clear_m"]), s["cost"]))
    rej = [c for c in res["candidates"] if not c["ok"]]
    if rej:
        groups = {}
        for c in rej:                                   # one line per jaw angle and reason (tilts/offsets counted)
            groups.setdefault((round(c["jaw_deg"]), c["why"].split(" (")[0]), []).append(c)
        out.append("  rejected %d of %d:" % (len(rej), len(res["candidates"])))
        for (jaw, why), cs in list(groups.items())[:n]:
            out.append("    jaw %3d deg (%d tilt/offset variant%s): %s" % (jaw, len(cs), "s" if len(cs) > 1 else "", cs[0]["why"]))
    return out
