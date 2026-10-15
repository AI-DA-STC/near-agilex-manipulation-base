#!/usr/bin/env python3
"""
ik_probe.py - which of the candidate grasps can MoveIt's IK actually reach?  Takes seconds (no planning), moves nothing. System python, MoveIt must be running.

For each tcp target the last detection handed to MTC (/tmp/grasp_debug.json, written by grasp_watch.py after transform_all) it asks MoveIt's /compute_ik for the
PREGRASP pose (the grasp pose moved PREGRASP_OFFSET_M back along its approach axis), exactly as the MTC task does, from the arm's start pose, for:
   orig  = the candidate as it is                (the MTC script has this variant COMMENTED OUT)
   flip  = rolled 180 degrees about its approach axis (the ONLY variant the MTC script tries)
each with collisions OFF ("free") and ON ("coll").  Then a sweep of the roll angle about the approach axis for the first candidate.
    /usr/bin/python3 ik_probe.py [--start initial|pose_90deg|current] [--n 6]
"""
import argparse, json, math, os, re, sys
import numpy as np

PRE = 0.08


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0); return (R[2, 1] - R[1, 2]) * s, (R[0, 2] - R[2, 0]) * s, (R[1, 0] - R[0, 1]) * s, 0.25 / s
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]); return 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s
    if i == 1:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]); return (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s
    s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]); return (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s


def variant_pose(T, roll_deg):
    """pregrasp (position, rotation) for candidate T rolled by roll_deg about its own approach (z) axis, moved PRE back along that axis"""
    R = np.array(T)[:3, :3] @ rot_z(math.radians(roll_deg)); p = np.array(T)[:3, 3] - PRE * R[:, 2]
    return p, R


def tilt_deg(T):
    return math.degrees(math.acos(max(-1.0, min(1.0, -np.array(T)[2, 2]))))


def start_joints(which, script):
    try:
        txt = open(os.path.expanduser(script)).read()
        m = re.search(r'"%s":\s*\[([^\]]*)\]' % which, txt) or (re.search(r'^POSE_90DEG\s*=\s*\[([^\]]*)\]', txt, re.M) if which == "pose_90deg" else None)
        v = [float(x) for x in m.group(1).split(",")]
        return v if len(v) == 7 else None
    except Exception:
        return None


def fmt(free, coll):
    return "ok" if coll else ("COLL" if free else "no")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default="/tmp/grasp_debug.json"); ap.add_argument("--n", type=int, default=6)
    ap.add_argument("--start", default="initial", help="initial | pose_90deg | current (seed = the arm's present joints)")
    ap.add_argument("--script", default="~/nero_pipeline_launch/scripts/move_to_pose_90deg.py", help="where the start poses are read from")
    ap.add_argument("--sweep_step", type=int, default=30)
    a = ap.parse_args(argv)
    if not os.path.exists(a.json):
        sys.exit("%s does not exist: run a detection through transform_all with grasp_watch.py running first" % a.json)
    cands = json.load(open(a.json)).get("cands", [])[:a.n]
    if not cands:
        sys.exit("no candidates in the json")

    import rclpy
    from rclpy.node import Node
    from builtin_interfaces.msg import Duration
    from moveit_msgs.srv import GetPositionIK
    rclpy.init(); node = Node("ik_probe")
    cli = node.create_client(GetPositionIK, "/compute_ik")
    if not cli.wait_for_service(timeout_sec=10.0):
        print("/compute_ik is not available - is move_group running?"); rclpy.shutdown(); return 2
    seed = None if a.start == "current" else start_joints(a.start, a.script)
    print("IK seed: %s" % ("the arm's current joints" if seed is None else "%s start pose %s" % (a.start, [round(x, 3) for x in seed])))
    if a.start != "current" and seed is None:
        print("  (could not read the %s start pose from %s - using the current joints)" % (a.start, a.script))

    def ik(p, R, avoid):
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = "arm"; r.ik_link_name = "tcp_link"; r.avoid_collisions = bool(avoid); r.timeout = Duration(sec=0, nanosec=300_000_000)
        r.pose_stamped.header.frame_id = "base_link"
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = [float(v) for v in p]
        q = to_quat(R); r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y, r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w = [float(v) for v in q]
        if seed is not None:
            r.robot_state.joint_state.name = ["joint%d" % (k + 1) for k in range(7)]; r.robot_state.joint_state.position = list(seed)
        f = cli.call_async(req); rclpy.spin_until_future_complete(node, f, timeout_sec=3.0)
        return f.result() is not None and f.result().error_code.val == 1

    print("\n #  tilt |  orig: free coll | flip (what MTC tries): free coll |")
    got_flip = got_orig = got_any = 0
    for i, T in enumerate(cands):
        res = {}
        for name, roll in (("orig", 0), ("flip", 180)):
            p, R = variant_pose(T, roll); res[name] = (ik(p, R, False), ik(p, R, True))
        got_flip += res["flip"][1]; got_orig += res["orig"][1]; got_any += (res["flip"][1] or res["orig"][1])
        print("%2d %4.0f |  orig: %-4s %-4s | flip: %-4s %-4s" % (i, tilt_deg(T), "ok" if res["orig"][0] else "no", fmt(res["orig"][0], res["orig"][1]), "ok" if res["flip"][0] else "no", fmt(res["flip"][0], res["flip"][1])))
    n = len(cands)
    print("\nwith collisions on: flip works for %d of %d candidates (what the MTC task uses today), orig for %d of %d, either for %d of %d." % (got_flip, n, got_orig, n, got_any, n))
    print("\nroll sweep about the approach axis, candidate 0 (tilt %.0f deg): which roll angles have an IK solution (collisions on)" % tilt_deg(cands[0]))
    row = []
    for roll in range(0, 360, a.sweep_step):
        p, R = variant_pose(cands[0], roll); row.append((roll, ik(p, R, True)))
    print("  roll  " + " ".join("%4d" % r for r, _ in row)); print("  IK    " + " ".join("%4s" % ("ok" if ok else "-") for _, ok in row))
    okc = sum(1 for _, ok in row if ok)
    print("  -> %d of %d roll angles are reachable%s" % (okc, len(row), "; for a cube any of them is a valid grasp, so trying several would make planning far more reliable." if okc else "."))
    rclpy.shutdown(); return 0


if __name__ == "__main__":
    sys.exit(main())
