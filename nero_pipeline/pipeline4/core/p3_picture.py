#!/usr/bin/env python3
"""
p3_picture.py - PIPELINE 3: a picture of the GRASP and the PLACE spot exactly as MoveIt receives them (after pose_transform), on the camera image of
the latest GraspMolmo call. Nothing moves. run_pick_and_place3.sh runs it after pose_transform and opens the picture; by hand:

    conda deactivate; source ~/agx_arm_ws/install/setup.bash
    /usr/bin/python3 ~/nero_pipeline_launch/pipeline3/p3_picture.py

Reads the latched /grasp_poses_transformed (all grasps, base_link) and /target_pose_transformed (the plate, base_link), the camera's place in base_link
from TF, and ~/p3_runs/latest/ (rgb.png, answers.json with K and Molmo's points). Writes ~/p3_runs/latest/grasp_place.png and grasp_place.txt.

In the picture:
  GOLD  G1  = the grasp MTC tries FIRST: pipeline 1's MTC drops grasps tilted more than 55 deg from straight down and tries the most top-down first
              (it plans up to 6 and executes the cheapest solution: usually, not always, G1)
  ORANGE    = the other grasps MTC tries;  GREY = grasps MTC drops (too tilted) or does not reach (beyond the first 6)
  each bar  = the line the REAL fingers close along: tcp_link's Y axis (docs/01; pipeline 1's MTC uses the pose as the tcp_link target as it is),
              at the closing centre (gripper origin + 0.1034 m along the approach), 5 cm long. v3: v1/v2 drew the X axis, which is where
              Contact-GraspNet intended the jaws - 90 deg from where the arm really closes (that is what p3_jaw_fix corrects)
  MAGENTA + = the PLACE spot (the plate point MoveIt receives); CYAN o = where Molmo pointed (grasp) and CYAN + (place)
"""
import json
import os
import subprocess
import sys
import time

import numpy as np

P3_PICTURE_VERSION = "p3-picture-v10"
# v10: grasps in the ORDER MoveIt receives them (pipeline 3's MTC tries them in that order: G1 = Contact-GraspNet's best SAFE grasp, from the
# relay), BOTH fingertips checked (p3_fingers.py), and the relay's report (which of Contact-GraspNet's grasps were passed / rejected and why).
FINGERTIP_REACH = 0.138       # m from the grasp pose's origin (= tcp_link, pipeline 1's MTC uses the pose as it is) to the LOWEST fingertip,
                              # along the approach: measured from the gripper model by pipeline 2's p2_gripper_extent ("hand base + 13.8 cm")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p3_tablez                                                      # noqa: E402  v5: the real table top, shared by all pipeline 3 tools
import p3_fingers                                                     # noqa: E402  v10: both fingertips
TABLE_Z, TABLE_Z_SOURCE = p3_tablez.table_z()
TIP_MIN_M, TIP_MIN_SOURCE = p3_tablez.tip_min()
RUN = os.path.expanduser("~/p3_runs/latest")
GRIPPER_DEPTH = 0.1034
MAX_TILT_DEG = 55.0           # = pipeline 1's MTC
MAX_CANDIDATES = 6            # = pipeline 1's MTC


def quat_to_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 1e-12 else 0.0
    return np.array([[1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
                     [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
                     [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def describe_grasps(poses):
    """poses: [(pos xyz, quat xyzw)] in base_link -> list of dicts with origin, closing centre, approach, closing axis, tilt; and MTC's order."""
    out = []
    for i, (p, q) in enumerate(poses):
        R = quat_to_matrix(*q)
        o = np.asarray(p, dtype=np.float64)
        a = R[:, 2]
        tilt = float(np.degrees(np.arccos(np.clip(a @ np.array([0.0, 0.0, -1.0]), -1.0, 1.0))))
        tip_z = p3_fingers.lowest_tip_z(o, R)                          # v10: the LOWER of the two fingertips (a rolled grasp has one lower)
        out.append({"index": i, "origin": o, "approach": a, "closing": R[:, 1], "centre": o + GRIPPER_DEPTH * a, "tilt_deg": tilt,
                    "roll_deg": p3_fingers.roll_deg(R), "tip_z": tip_z, "tip_clear_m": tip_z - TABLE_Z})   # tcp_link Y = the real jaw line
    kept = [g for g in out if g["tilt_deg"] <= MAX_TILT_DEG][:MAX_CANDIDATES]   # v10: pipeline 3's MTC keeps the order it receives
    for k, g in enumerate(kept):
        g["mtc_order"] = k + 1
    return out, kept


def project(points_base, T_base_cam, K):
    """base_link points -> pixels, with the camera pose T_base_cam (camera -> base_link)."""
    inv = np.linalg.inv(T_base_cam)
    pc = np.asarray(points_base, dtype=np.float64) @ inv[:3, :3].T + inv[:3, 3]
    uv = pc @ np.asarray(K).T
    return uv[:, :2] / uv[:, 2:3]


def draw_obstacles(dr, boxes, T_base_cam, K):
    """v7: each obstacle box MoveIt knows about, as its top rectangle (light blue) with its height."""
    for k, b in enumerate(boxes or []):
        cx, cy, _ = b["centre"]; sx, sy, _ = b["size"]; top = b["top_z"]
        corners = [(cx - sx / 2, cy - sy / 2, top), (cx + sx / 2, cy - sy / 2, top), (cx + sx / 2, cy + sy / 2, top), (cx - sx / 2, cy + sy / 2, top)]
        uv = [tuple(p) for p in project(corners, T_base_cam, K)]
        dr.line(uv + [uv[0]], fill=(90, 200, 255), width=2)
        dr.text((uv[0][0] + 3, uv[0][1] + 2), "OB%d %.0fcm" % (k + 1, 100 * (top - (b["centre"][2] - b["size"][2] / 2))), fill=(90, 200, 255))


def draw(rgb, grasps, kept, place_xyz, T_base_cam, K, molmo, footer, title=""):
    from PIL import Image, ImageDraw
    img = Image.fromarray(rgb)
    dr = ImageDraw.Draw(img)
    draw_obstacles(dr, molmo.get("obstacles"), T_base_cam, K)
    order = sorted(grasps, key=lambda g: (2 if g.get("mtc_order") == 1 else 1 if "mtc_order" in g else 0))   # grey, then orange, then G1 on top
    for g in order:
        ends = project([g["centre"] - 0.025 * g["closing"], g["centre"] + 0.025 * g["closing"]], T_base_cam, K)
        mo = g.get("mtc_order")
        col, w = ((255, 200, 0), 5) if mo == 1 else (((255, 120, 0), 3) if mo else ((150, 150, 150), 2))
        dr.line([tuple(ends[0]), tuple(ends[1])], fill=col, width=w)
        c = project([g["centre"]], T_base_cam, K)[0]
        if mo:
            dr.text((c[0] + 6, c[1] + 4), "G%d" % mo, fill=col)
    if place_xyz is not None:
        x, y = project([place_xyz], T_base_cam, K)[0]
        for d in ((-14, 0, 14, 0), (0, -14, 0, 14)):
            dr.line([(x + d[0], y + d[1]), (x + d[2], y + d[3])], fill=(255, 0, 255), width=3)
        dr.text((x + 10, y + 10), "PLACE", fill=(255, 0, 255))
    if molmo.get("grasp_px"):
        x, y = molmo["grasp_px"]
        dr.ellipse([x - 7, y - 7, x + 7, y + 7], outline=(0, 230, 255), width=2)
    if molmo.get("place_px"):
        x, y = molmo["place_px"]
        dr.line([(x - 7, y - 7), (x + 7, y + 7)], fill=(0, 230, 255), width=2)
        dr.line([(x - 7, y + 7), (x + 7, y - 7)], fill=(0, 230, 255), width=2)
    # v7: the title and the numbers go in their own strips ABOVE and BELOW the camera image, so they never hide part of it
    W, H = img.size
    TH, FH = 32, 14 * len(footer) + 6
    canvas = Image.new("RGB", (W, TH + H + FH), (0, 0, 0))
    canvas.paste(img, (0, TH))
    dc = ImageDraw.Draw(canvas)
    dc.text((4, 2), title[:110], fill=(255, 255, 0))                      # WHICH run - so a stale picture is obvious
    dc.text((4, 17), "bar = where the REAL fingers close | GOLD G1 = MTC tries first | orange = others | magenta = PLACE | blue = OBSTACLES", fill=(255, 255, 255))
    for k, line in enumerate(footer):
        dc.text((4, TH + H + 3 + 14 * k), line[:110], fill=(255, 255, 255))
    return canvas


def run_clearance(answers):
    """v8: the fingertip clearance demanded for THIS run's object (m) and why - the same rule as the tip lift (p3_tablez.clearance_for),
    from the object height the GraspMolmo node saved. An older run without a height gets the maximum (the safe side)."""
    return p3_tablez.clearance_for((answers or {}).get("object_height_m"))


def relay_report(path=os.path.expanduser("~/p3_runs/relay_latest.json"), max_age_s=600):
    """v10: the relay's account of Contact-GraspNet's grasps (passed / rejected and why), if it is from this run (younger than max_age_s)."""
    import json
    try:
        if time.time() - os.path.getmtime(path) > max_age_s:
            return []
        r = json.load(open(path))
    except (OSError, ValueError):
        return []
    L = ["  CGN      Contact-GraspNet gave %d grasp(s), its best first (fingertip clearance: %s):" % (r.get("cgn_grasps", 0), r.get("clearance", "?"))]
    for g in r.get("grasps", [])[:10]:
        L.append("           #%-2d tilt %3.0f deg, roll %3.0f deg, lower fingertip %+6.1f mm  -> %s" % (
            g["rank"], g["tilt_deg"], g["roll_deg"], g["lowest_finger_mm"], g["result"]))
    if len(r.get("grasps", [])) > 10:
        L.append("           ... (%d more in ~/p3_runs/relay_latest.json)" % (len(r["grasps"]) - 10))
    return L


def scene_check(answers, moveit_ids):
    """v9: does MoveIt hold exactly this run's obstacle boxes? (p3_obst_0 .. p3_obst_<n-1>)"""
    if moveit_ids is None:
        return "  SCENE    could not ask MoveIt which obstacle boxes it holds (its /get_planning_scene service did not answer)"
    want = len((answers or {}).get("obstacles") or [])
    if moveit_ids == ["p3_obst_%d" % k for k in range(want)]:
        return "  SCENE    MoveIt holds exactly this run's %d obstacle box(es): MATCH" % want
    return ("  SCENE    MISMATCH: MoveIt holds %d obstacle box(es) (%s) but this run found %d - plans use the wrong obstacles. "
            "Run: p3_scene.py clear, then p3_look.sh again" % (len(moveit_ids), ", ".join(moveit_ids) or "none", want))


def summary(grasps, kept, place_xyz, answers):
    clearance, why = run_clearance(answers)
    L = ["=" * 100, "PIPELINE 3 - PICK AND PLACE POINTS (what MoveIt received; base_link metres: x forward/back, y sideways, z up; the TABLE is at z = %+.1f mm)" % (1000 * TABLE_Z), "=" * 100]
    if answers:
        L.append("  task     pick \"%s\", place on \"%s\"   (GraspMolmo %s, answers %.0f s + %.0f s)" % (
            answers.get("source_prompt"), answers.get("target_prompt"), answers.get("dtype"), answers.get("grasp_seconds", 0), answers.get("place_seconds", 0)))
    if kept:
        g = kept[0]
        L.append("  PICK     G1 (MTC tries it first): fingers close at x=%+.3f y=%+.3f z=%+.3f, %.0f deg from straight down; gripper origin x=%+.3f y=%+.3f z=%+.3f" % (
            *g["centre"], g["tilt_deg"], *g["origin"]))
        L.append("           %d grasp(s) from Contact-GraspNet: %d kept by MTC's 55-deg rule (it tries up to %d), %d dropped" % (
            len(grasps), len([x for x in grasps if x["tilt_deg"] <= MAX_TILT_DEG]), MAX_CANDIDATES, len([x for x in grasps if x["tilt_deg"] > MAX_TILT_DEG])))
    else:
        L.append("  PICK     NONE of the %d grasps is within 55 deg of straight down: MTC has nothing to plan" % len(grasps))
    for g in kept:                                                  # v4: the fingertips of EVERY grasp MTC may execute
        if g["tip_clear_m"] < clearance:
            L.append("  FINGERS  G%d: lowest fingertip at z=%+.1f cm = %+.1f cm above the table -> TOO LOW (rule: at least %.1f cm): it would hit the table"
                     % (g["mtc_order"], 100 * g["tip_z"], 100 * g["tip_clear_m"], 100 * clearance))
    if kept:
        low = min(kept, key=lambda g: g["tip_clear_m"])
        if low["tip_clear_m"] >= clearance:
            L.append("  FINGERS  every fingertip clears the table: the lowest (G%d) is %.1f cm above it (rule: at least %.1f cm)" % (
                low["mtc_order"], 100 * low["tip_clear_m"], 100 * clearance))
    if kept:
        L.append("           clearance demanded for this object: %s" % why)          # shown whether the check passes or not
    if place_xyz is not None:
        L.append("  PLACE    plate point x=%+.3f y=%+.3f z=%+.3f (MTC releases the object above it)" % tuple(place_xyz))
    if answers is not None:
        L.append("  OBSTACLES in MoveIt's planning scene: %s" % answers.get("obstacles_note", "(this run was saved by an older node: none)"))
    if kept and place_xyz is not None:
        L.append("  MOVE     %.1f cm sideways from the pick to the place" % (100 * np.hypot(*(np.asarray(place_xyz[:2]) - kept[0]["centre"][:2]))))
    L.append("=" * 100)
    return L


def read_ros(timeout=10.0, camera_frame=None):
    """(grasp PoseArray, target PoseStamped, 4x4 camera->base_link or None)."""
    import rclpy
    import tf2_ros
    from geometry_msgs.msg import PoseArray, PoseStamped
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)
    rclpy.init()
    node = Node("p3_picture")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    got = {}
    node.create_subscription(PoseArray, "/grasp_poses_transformed", lambda m: got.__setitem__("a", m), latched)
    node.create_subscription(PoseStamped, "/target_pose_transformed", lambda m: got.__setitem__("t", m), latched)
    T = None
    t_end = time.time() + timeout
    while time.time() < t_end and (len(got) < 2 or T is None):
        rclpy.spin_once(node, timeout_sec=0.2)
        if T is None and camera_frame:
            try:
                tr = buf.lookup_transform("base_link", camera_frame, rclpy.time.Time())
                q, p = tr.transform.rotation, tr.transform.translation
                T = np.eye(4)
                T[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
                T[:3, 3] = [p.x, p.y, p.z]
            except Exception:
                pass
    moveit_ids = None                                       # v9: which obstacle boxes MoveIt REALLY holds (its own service)
    try:
        import p3_scene
        moveit_ids = p3_scene.Scene(node, timeout=5.0).obstacle_ids()
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return got.get("a"), got.get("t"), T, moveit_ids


def open_picture(path):
    if os.environ.get("P3_SHOW_PICTURE", "1") == "0":
        return
    for viewer in ("eog", "xdg-open"):
        try:
            args = [viewer, "--new-instance", path] if viewer == "eog" else [viewer, path]      # a NEW eog window, never an old one brought forward
            subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
            return
        except OSError:
            continue


def main(argv=None, reader=read_ros):
    print("p3_picture %s" % P3_PICTURE_VERSION, flush=True)
    try:
        answers = json.load(open(os.path.join(RUN, "answers.json")))
    except (OSError, ValueError):
        answers = None
    res = reader(camera_frame=(answers or {}).get("frame"))
    arr, tgt, T = res[:3]
    moveit_ids = res[3] if len(res) > 3 else None
    if arr is None or len(arr.poses) == 0:
        print("STOP: nothing on /grasp_poses_transformed - run  ros2 service call /pose_transform/transform_all std_srvs/srv/Trigger  first")
        return 1
    poses = [((p.position.x, p.position.y, p.position.z), (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)) for p in arr.poses]
    grasps, kept = describe_grasps(poses)
    place = None if tgt is None else np.array([tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z])
    lines = summary(grasps, kept, place, answers)
    lines[-1:-1] = [scene_check(answers, moveit_ids)] + relay_report()
    print("\n".join(lines), flush=True)
    too_low = [g for g in kept if g["tip_clear_m"] < run_clearance(answers)[0]]
    if answers is None or "K" not in answers or T is None or not os.path.exists(os.path.join(RUN, "rgb.png")):
        if too_low:
            return 3
        print("(no picture: %s)" % ("no GraspMolmo run saved in ~/p3_runs/latest" if answers is None else
                                    "this run was saved by the older GraspMolmo node (p3-node-v1, no camera intrinsics): restart start_pipeline3.sh" if "K" not in answers else
                                    "the camera's place in base_link is not in TF (is the hand-eye publisher running?)" if T is None else "rgb.png missing"))
        return 0
    from PIL import Image
    rgb = np.array(Image.open(os.path.join(RUN, "rgb.png")).convert("RGB"))
    footer = [l.strip() for l in lines[3:-1]]
    title = "PICK \"%s\" -> PLACE \"%s\"   (GraspMolmo run %s)" % (answers.get("source_prompt"), answers.get("target_prompt"), answers.get("time"))
    img = draw(rgb, grasps, kept, place, T, np.array(answers["K"]), answers, footer, title=title)
    run_dir = os.path.realpath(RUN)                 # v2: this run's OWN folder - an image viewer still showing an older run cannot be mistaken for it
    out = os.path.join(run_dir, "grasp_place.png")
    img.save(out)
    with open(os.path.join(RUN, "grasp_place.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("PICTURE  %s   (title: %s; gold G1 = what MTC tries first, magenta = place; opened in a NEW window)" % (out, title), flush=True)
    open_picture(out)
    return 3 if too_low else 0


if __name__ == "__main__":
    sys.exit(main())
