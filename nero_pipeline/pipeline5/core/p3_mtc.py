#!/usr/bin/env python3
"""
PIPELINE 3 copy of pipeline 2's p2_mtc.py (P3_MTC_VERSION below). The ONLY change: ARM_MAX_VEL 0.3 (was 0.1).
Why pipeline 3 uses it instead of pipeline 1's MTC: pipeline 1 releases at ONE fixed yaw and executes MTC's cheapest solution, so the wrist
turns up to ~180 deg (at the grasp and again before the place). This copy lets the release yaw follow the grasp (P2_PLACE_YAW=grasp), offers both
equivalent grasp orientations and executes the SMOOTHEST solution (P2_SOLUTION_CHOICE=smooth) - pipeline 2's tested fix for exactly that.
Its settings keep pipeline 2's names (P2_...). Run by run_pick_and_place3.sh through p3_mtc.launch.py; P3_MTC=pipeline1 uses pipeline 1's MTC.

Builds and executes a MoveIt Task Constructor pick-and-place task for the
Nero arm, using REAL detected grasp candidates (from contact_graspnet_ros,
via pose_transform_ros) and the REAL detected target pose (from
target_pose_ros, via pose_transform_ros).

ARCHITECTURE CHANGE from the first draft: earlier versions tried to
pre-select a single "reachable" grasp candidate ourselves, by calling
/compute_ik directly before building the task. This was abandoned after
repeated, evidence-confirmed failures of that approach:
  - it checked the wrong pose (raw grasp instead of the offset pregrasp
    pose the real task actually uses)
  - it didn't check collisions, while MTC's real ComputeIK stage does by
    default
  - even after fixing both of those, IK for a 7-DOF arm is SEED-DEPENDENT:
    our standalone check and the real task's ComputeIK stage can seed
    from genuinely different starting joint configurations and reach
    different conclusions about the SAME target pose, since the real
    task's seed evolves stage-by-stage in a way an external, isolated
    check cannot replicate.

This version instead feeds ALL grasp candidates directly to MTC as
parallel stages inside an `Alternatives` container, letting the REAL
task's own solver - with its own real, evolving seed state - decide
which one is actually reachable. This is structurally correct rather
than an external approximation.

Same architecture for placement: since the user has no requirement on
final placement orientation, both the top candidate's orientation and
its 180-degree "roll flip" (a physically equivalent grasp for a
parallel-jaw gripper) are offered as Alternatives for the preplace/place
stage, rather than rigidly forcing one specific orientation.

Stages (matching the architecture's approach -> grasp -> attach -> lift ->
transit -> place -> retreat description):

    CurrentState
    -> Connect                                    (get to pregrasp)
    -> Alternatives(pregrasp IK per candidate)    (try each candidate x 2 orientation variants)
    -> MoveTo "gripper_open"
    -> MoveRelative "approach"                    (linear descent to the real grasp pose)
    -> ModifyPlanningScene allow_collisions       (permit touching the object)
    -> MoveTo "gripper_close"
    -> ModifyPlanningScene attach_object
    -> MoveRelative "lift"
    -> Connect                                    (transit)
    -> Alternatives(preplace IK per orientation)  (try both orientation options at the target)
    -> MoveRelative "place" (descend)
    -> ModifyPlanningScene detach_object
    -> MoveTo "gripper_open"                      (release)
    -> MoveRelative "retreat"

Built from individually-verified low-level MTC stages (ComputeIK, MoveTo,
MoveRelative, ModifyPlanningScene, GeneratePose, Connect, CurrentState,
Alternatives) - NOT the higher-level Pick/Place/SimpleGrasp/SimpleUnGrasp
composite classes, since those expect their wrapped generator stage to
expose pregrasp/grasp properties that GeneratePose (a fixed, non-sampling
pose generator) does not have.

Prerequisite: run the full perception chain first (in separate terminals,
as already established):
    ros2 service call /gsam2/segment std_srvs/srv/Trigger
    ros2 service call /contact_graspnet/detect_grasps std_srvs/srv/Trigger
    ros2 service call /target_pose/extract std_srvs/srv/Trigger
    ros2 service call /pose_transform/transform_all std_srvs/srv/Trigger
Then run this script (via its launch file, NOT bare `ros2 run` - it needs
the full MoveIt config, see the launch file's own comments) - it reads
/grasp_poses_transformed and /target_pose_transformed once, then builds
and executes the task.
"""

P3_MTC_VERSION = "p3-mtc-v3"

P4_MTC_VERSION = "p4-mtc-v1"
P5_MTC_VERSION = "p5-mtc-v1i"   # 7i: the lowering may stop early (>= 1 cm) and release there | 7g: release opening = object + 1 cm   # PIPELINE 5 copy: the opening comes from ~/p3_runs/p5_grasp_latest.json   # PIPELINE 4 core copy: the gripper opens only as wide as the grasp step says (p4_make_core.py)


def _p4_open_width():
    """-> (opening in m, why) from pipeline 4's grasp step, or (None, why) = fully open."""
    import json as _j, os as _o, time as _t
    if _o.environ.get("P5_GRIPPER_WIDTH", "on") == "off":
        return None, "P5_GRIPPER_WIDTH=off"
    path = _o.path.expanduser("~/p3_runs/p5_grasp_latest.json")
    try:
        d = _j.load(open(path))
    except Exception:
        return None, "no %s (is p5_grasp_node running?)" % path
    age = _t.time() - float(d.get("unix_time", 0))
    if age > 1800:
        return None, "the grasp step's width is %.0f min old" % (age / 60)
    w = float(d.get("open_width_m", 0))
    if not 0.03 <= w <= 0.10:
        return None, "width %.3f m is outside 0.03-0.10" % w
    return w, "pipeline 5 grasp step of %s" % d.get("time")


def _p5_release_width(open_w):
    """7g: at RELEASE open only to the object's width + P5_RELEASE_EXTRA_M (default 1 cm), never more than the grasp opening: the object drops
    as soon as the fingers move a few mm, and opening fully pushed a finger into the container's edge (2026-11-24: 'release gripper' failed,
    gripper_link1 vs a wrist box). -> (width, why) or (None, why) to keep the grasp opening."""
    import json as _j, os as _o
    try:
        d = _j.load(open(_o.path.expanduser("~/p3_runs/p5_grasp_latest.json")))
        obj = max(float(g["width_m"]) for g in d["grasps"])
    except Exception as e:
        return None, "object width unknown (%s)" % e
    extra = float(_o.environ.get("P5_RELEASE_EXTRA_M", "0.010"))
    w = min(open_w, obj + extra)
    return w, "object %.0f mm + %.0f mm (P5_RELEASE_EXTRA_M)" % (1000 * obj, 1000 * extra)


def _p4_gripper_goal(stage, what):
    w, why = _p4_open_width()
    if w is not None and what == "release":
        rw, rwhy = _p5_release_width(w)
        if rw is not None:
            w, why = rw, rwhy
    if w is None:
        stage.setGoal("gripper_open")
        print(f"PIPELINE 5: gripper {what}: FULLY open (0.100 m) - {why}")
        return
    try:
        stage.setGoal({"gripper": w})
        print(f"PIPELINE 5: gripper {what}: {w:.3f} m wide - {why}")
    except Exception as e:
        stage.setGoal("gripper_open")
        print(f"PIPELINE 5: gripper {what}: could not set {w:.3f} m ({e}); FULLY open instead")
# v3: P3_PLACE_HEIGHT (default "surface"): the release height follows the MEASURED surface at the place point (the target pose's z: the top of a
# cube, the floor of a plate). The object's bottom is released P3_RELEASE_GAP_M (default 1.5 cm) above that surface. The old "auto" height assumed a
# plate lying on the table (floor a fixed PLATE_THICKNESS_M up) and ignored the target's height: stacking on a 5 cm cube released the object
# 2 cm INSIDE it. P3_PLACE_HEIGHT=auto restores the old height.
# v2: P3_PLACE_AT_PADS (default 1): the place IK puts the FINGER PADS (tcp_link + OUR_FINGER_REACH along the gripper axis), i.e. the object,
# on the target instead of tcp_link. The release orientation may tilt up to +-PLACE_CONE_DEG; with tcp_link on the target, a 20 deg tilt swung
# the object up to 0.14 m x sin(20 deg) = 4.8 cm off the target (seen on the real arm: "the place lands further from the base").
# For a straight-down release the arm ends exactly where it did before. P3_PLACE_AT_PADS=0 = the old behaviour.
import copy
import sys
import time
import math

import rclpy
from rclpy.action import ActionClient
from moveit_task_constructor_msgs.action import ExecuteTaskSolution
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, TwistStamped

# Must match pose_transform_ros's publisher QoS exactly for the "replay
# last message to a late-joining subscriber" behavior to actually work.
LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

GRASP_CANDIDATES_TOPIC = "/grasp_poses_transformed"
TARGET_POSE_TOPIC = "/target_pose_transformed"
MAX_CANDIDATES_IN_TASK = 6
                            # candidate has repeatedly failed purely on
                            # reachability (Contact-GraspNet's score
                            # measures grip quality, not whether THIS arm
                            # can reach that angle), so some fallback is
                            # needed, but this stays minimal rather than
                            # searching broadly

ARM_GROUP = "arm"
GRIPPER_GROUP = "gripper"
IK_FRAME_LINK = "tcp_link"  # matches the SRDF's end_effector parent_link
BASE_FRAME = "base_link"

PREGRASP_OFFSET_M = 0.08   # how far above the grasp pose to approach from
PREPLACE_OFFSET_M = 0.50   # how far above the target to descend from
PLACE_Z_CLEARANCE_M = 0.18  # extra height above the plate's own surface for
                            # the release point. NOTE: 2cm was tried and
                            # failed across all 14 real post-pick seeds in
                            # testing (confirmed via the real task itself,
                            # not standalone compute_ik checks, which are
                            # unreliable here due to IK seed-dependency).
                            # 5cm matches a height already proven reachable
                            # across multiple real seeds earlier the same
                            # day at this exact (x, y) location.
APPROACH_MIN, APPROACH_MAX = 0.078, 0.08   # PIPELINE 5: the FULL approach or no plan (0.02 let it close 2 cm above the object, silently)
LIFT_MIN, LIFT_MAX = 0.05, 0.10   # was 0.02, 0.05; the lift is now a straight line (LIFT_CARTESIAN)
PLACE_MIN, PLACE_MAX = 0.02, 0.05
RETREAT_MIN, RETREAT_MAX = 0.02, 0.15
MAX_TILT_DEG = 55.0             # PIPELINE 2: NOT used any more (the grasp is trusted as GraspMolmo chose it); kept only so the file diffs cleanly

# ---- PIPELINE 2 (GraspMolmo + M2T2) -------------------------------------------------------------------------------------------------------
# This file is the pipeline-2 copy of mtc_pick_and_place.py. Pipeline 1 keeps its own, untouched file. The ONLY behavioural change is how the
# grasp candidates are chosen: the grasp that GraspMolmo picked (poses[0]) is TRUSTED exactly as it is - no tilt filter, no "most top-down first"
# re-sorting. Everything else (pregrasp/approach/lift/transit/place logic, the flip variant, retiming, execution switch) is unchanged.
#   P2_GRASP_FALLBACKS=0 (default): plan ONLY GraspMolmo's pick (poses[0]) - strict.
#   P2_GRASP_FALLBACKS=N          : if that one cannot be planned, also try the next N grasps in GraspMolmo's own ranking (poses[1..N]), in that order.
import os as _os
P2_GRASP_FALLBACKS = int(_os.environ.get("P2_GRASP_FALLBACKS", "0"))

# P2_ORIENTATION_VARIANTS (default "orig,flip"): which orientations of GraspMolmo's grasp MTC may use for the pregrasp IK.
#   orig = the grasp exactly as given;  flip = the SAME grasp rolled 180 degrees about its approach axis (the same contact points for a symmetric parallel gripper).
#   Which of the two is the SHORT way for the wrist depends on how the cube sits: from the initial pose (joint 7 at +0.43 rad) one orientation needs the wrist to turn about 30-60 degrees, the other
#   about 115-145 degrees (measured with nero_fk.py on two real grasps: the 'turns 180 when it grasps' you see). Offering both lets MTC and P2_SOLUTION_CHOICE take the short way.
#   The release yaw follows EACH orientation (see P2_PLACE_YAW), so the hand does not turn between pick and place whichever one is used. "flip" alone is pipeline 1's behaviour.
P2_ORIENTATION_VARIANTS = _os.environ.get("P2_ORIENTATION_VARIANTS", "orig,flip")

# P2_SOLUTION_CHOICE (default "smooth"): MTC finds up to 50 solutions and its own lowest-cost one does not always look right (a big flip of one joint can still be 'cheap').
#   smooth = publish/execute the solution whose BIGGEST single-joint travel (summed over every arm segment) is smallest, then the smallest total travel; the numbers are printed.
#   mtc    = MTC's own lowest-cost solution (pipeline 1's behaviour).
P2_SOLUTION_CHOICE = _os.environ.get("P2_SOLUTION_CHOICE", "smooth")

# P2_PLACE_YAW (default "grasp"): the hand's YAW at the release.
#   fixed = pipeline 1: the release orientation is a top-down hand with the SAME yaw (180 deg) whatever the cube's orientation; only tilt (roll/pitch) is sampled.
#   grasp = the release yaw starts from the yaw the hand had at the grasp and is sampled within +- P2_PLACE_YAW_RANGE_DEG (default 60) around it. The cube is released in the orientation it was
#           picked up in (or close to it), so the wrist does not have to turn between pick and place. With the fixed yaw, joint 7 swung from about -1.5 rad at the pick to +1.5 rad at the place
#           (3 rad, the 'weird flip' of the arm: 4.6 rad of wrist travel in all, measured with nero_fk.py on a real grasp of yours); with the yaw near the grasp's it stays near -1.5 (1.8 rad in all).
#           Exactly the grasp's yaw can be unreachable at the plate (a narrow blocked gap), which is why a range is sampled and MTC / P2_SOLUTION_CHOICE pick the least-swing solution.
P2_PLACE_YAW = _os.environ.get("P2_PLACE_YAW", "grasp")

# P2_CONNECT_PLANNER (default "interp"): how the first move (current pose -> pregrasp) is planned.
#   interp  = a direct joint-space interpolation first (every joint moves straight from start to goal: smooth and deterministic, collision-checked), the planner only as a fallback: exactly
#             what the 'transit' stage already does.
#   planner = pipeline 1's behaviour. The code asks for the Pilz planner, but MoveIt falls back to OMPL ("Failed to find 'pilz_industrial_motion_planner.planning_plugin' ... Using OMPL"), a RANDOM
#             planner that can wander and twist the arm in odd ways even when a direct path exists.
P2_CONNECT_PLANNER = _os.environ.get("P2_CONNECT_PLANNER", "interp")
P2_PLACE_YAW_RANGE_DEG = float(_os.environ.get("P2_PLACE_YAW_RANGE_DEG", "60"))


def pose_matrix(pose):
    """3x3 rotation matrix of a geometry_msgs Pose."""
    import math as _m
    x, y, z, w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    n = _m.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def hand_yaw_deg(pose):
    """Azimuth (0..360 deg, counter-clockwise seen from above) of the hand's x axis: the 'yaw' of a top-down hand."""
    import math as _m
    R = pose_matrix(pose)
    return _m.degrees(_m.atan2(R[1][0], R[0][0])) % 360.0


def place_seed_quat(yaw_deg):
    """(x, y, z, w) of a top-down hand (180 deg about x) turned by yaw_deg about the world z axis. yaw 180 is pipeline 1's fixed seed (0, 1, 0, 0)."""
    import math as _m
    h = _m.radians(yaw_deg) / 2.0
    return (_m.cos(h), _m.sin(h), 0.0, 0.0)


def orientation_variants(spec=None):
    """['orig', 'flip'] (or one of them, or flip first) from P2_ORIENTATION_VARIANTS; anything else stops with a clear message."""
    spec = P2_ORIENTATION_VARIANTS if spec is None else spec
    names = []
    for n in [v.strip() for v in spec.split(",") if v.strip()]:
        if n not in ("orig", "flip"):
            names = []
            break
        if n not in names:
            names.append(n)
    if not names:
        raise SystemExit("STOP: P2_ORIENTATION_VARIANTS must be 'orig', 'flip' or 'orig,flip' (got '%s')." % spec)
    return names


def solution_joint_travel(solution_msg):
    """Cumulative absolute joint travel over every ARM segment of a solution message: ({joint name: radians}, total radians). Gripper segments and one-point segments are ignored."""
    per = {}
    for st in solution_msg.sub_trajectory:
        jt = st.trajectory.joint_trajectory
        names = list(jt.joint_names)
        if len(jt.points) < 2 or any(n.startswith("gripper") for n in names):
            continue
        for p0, p1 in zip(jt.points, jt.points[1:]):
            for n, a, b in zip(names, p0.positions, p1.positions):
                per[n] = per.get(n, 0.0) + abs(b - a)
    return per, sum(per.values())


def rank_solutions(solutions, to_msg):
    """One row per readable solution: index, the joint that travels most, how far, and the total travel. Index loops only (the MTC list type is a sequence, not necessarily iterable)."""
    rows = []
    for k in range(len(solutions)):
        try:
            per, total = solution_joint_travel(to_msg(solutions[k]))
        except Exception:
            continue
        if per:
            jmax = max(per, key=per.get)
            rows.append({"index": k, "max_joint": jmax, "max_travel": per[jmax], "total": total, "wrist": per.get("joint5", 0.0) + per.get("joint6", 0.0) + per.get("joint7", 0.0)})
    return rows


def segment_report(solution_msg, phase_names=None):
    """Lines: for every ARM segment of a solution the cumulative |change| of each joint (rad), a TOTAL row and the biggest single entry, so you can see WHICH stage turns WHICH joint.
    Segment names come from phase_names when their number equals the number of arm segments (as retime_solution assumes), else 'seg k'. [] if there is no arm motion."""
    segs = []
    for k, st in enumerate(solution_msg.sub_trajectory):
        jt = st.trajectory.joint_trajectory
        names = list(jt.joint_names)
        if len(jt.points) >= 2 and not any(n.startswith("gripper") for n in names):
            segs.append((k, names, jt.points))
    if not segs:
        return []
    labels = list(phase_names) if phase_names and len(phase_names) == len(segs) else ["seg %d" % k for k, _, _ in segs]
    joints = segs[0][1]
    short = [("j" + n[5:]) if n.startswith("joint") else n[:5] for n in joints]
    rows, total, biggest = [], [0.0] * len(joints), (0.0, "", "")
    for label, (k, names, pts) in zip(labels, segs):
        trav = [0.0] * len(joints)
        for p0, p1 in zip(pts, pts[1:]):
            for i, (a, b) in enumerate(zip(p0.positions, p1.positions)):
                if i < len(trav):
                    trav[i] += abs(b - a)
        for i, v in enumerate(trav):
            total[i] += v
            if v > biggest[0]:
                biggest = (v, joints[i] if i < len(joints) else str(i), label)
        rows.append("  %-9s" % label + "".join("%7.2f" % v for v in trav))
    head = ["Joint travel of the chosen solution (rad, cumulative |change| per stage):", "  %-9s" % "stage" + "".join("%7s" % s for s in short)]
    foot = ["  %-9s" % "TOTAL" + "".join("%7.2f" % v for v in total), "  the biggest single entry: %s in '%s': %.2f rad (%.0f deg)" % (biggest[1], biggest[2], biggest[0], 57.29578 * biggest[0])]
    return head + rows + foot


P2_MAX_JOINT_STAGE_RAD = float(_os.environ.get("P2_MAX_JOINT_STAGE_RAD", "3.0"))
# P2_EXECUTE_VIA (default "auto"): how the retimed plan is executed.
#   auto        the action execute_task_solution (MTC's own, needs MoveIt's 'capabilities'); if it does not exist in the running MoveIt (AgileX's combined launch does not forward 'capabilities'),
#               the same trajectories one segment at a time through move_group's /execute_trajectory action: the route RViz's Plan & Execute uses
#   task        only execute_task_solution (pipeline 1's route); if it is missing: a clear message, nothing moves
#   trajectory  only /execute_trajectory
P2_EXECUTE_VIA = _os.environ.get("P2_EXECUTE_VIA", "auto").strip().lower()
if P2_EXECUTE_VIA not in ("auto", "task", "trajectory"):
    raise SystemExit("STOP: P2_EXECUTE_VIA must be 'auto', 'task' or 'trajectory' (got '%s')." % P2_EXECUTE_VIA)
# P2_MAX_JOINT_STAGE_RAD (default 3.0; 0 = off): the plan is NOT executed if any joint turns more than this many radians in ONE stage (3 rad is about 170 degrees: a limit-to-limit swing, the
# 'weird flip' of the arm). MoveIt's planning is random, so p2_run.sh simply plans again. Plan-only runs are never refused: you can always look at them.


def biggest_stage_joint(solution_msg, phase_names=None):
    """(rad, joint name, stage label) of the biggest cumulative |change| of any joint in any single ARM stage of a solution message; None if there is no arm motion."""
    segs = []
    for k, st in enumerate(solution_msg.sub_trajectory):
        jt = st.trajectory.joint_trajectory
        names = list(jt.joint_names)
        if len(jt.points) >= 2 and not any(n.startswith("gripper") for n in names):
            segs.append((names, jt.points))
    if not segs:
        return None
    labels = list(phase_names) if phase_names and len(phase_names) == len(segs) else ["seg %d" % k for k in range(len(segs))]
    best = (0.0, "", "")
    for label, (names, pts) in zip(labels, segs):
        trav = [0.0] * len(names)
        for p0, p1 in zip(pts, pts[1:]):
            for i, (a, b) in enumerate(zip(p0.positions, p1.positions)):
                if i < len(trav):
                    trav[i] += abs(b - a)
        for i, v in enumerate(trav):
            if v > best[0]:
                best = (v, names[i], label)
    return best


def refusal_message(biggest, limit):
    """None if the plan may be executed, else the text that explains the refusal."""
    import math as _m
    if biggest is None or limit <= 0 or biggest[0] <= limit:
        return None
    return ("REFUSING to execute: the chosen plan turns %s by %.2f rad (%.0f deg) in the '%s' stage, more than the limit of %.2f rad (P2_MAX_JOINT_STAGE_RAD): that is a limit-to-limit swing of the arm. "
            "MoveIt's planning is random, so plan again (p2_run.sh does that for you); set P2_MAX_JOINT_STAGE_RAD=0 to allow it." % (biggest[1], biggest[0], _m.degrees(biggest[0]), biggest[2], limit))


def choose_execute_route(via, task_server_up):
    """-> (route, message). route: 'task' | 'trajectory' | None (nothing can be executed). message: what to tell the user, or None."""
    via = (via or "auto").strip().lower()
    if via == "trajectory":
        return "trajectory", "P2_EXECUTE_VIA=trajectory: executing the retimed plan one segment at a time through /execute_trajectory (the route RViz's Plan & Execute uses)."
    if task_server_up:
        return "task", None
    if via == "task":
        return None, "Execution did NOT run: P2_EXECUTE_VIA=task and the action execute_task_solution is not available in the running MoveIt (waited 10 s). Nothing moved."
    return "trajectory", ("NOTE: the action execute_task_solution is not available in the running MoveIt (AgileX's combined launch does not forward 'capabilities'). "
                          "Executing the same retimed plan one segment at a time through /execute_trajectory instead: the route RViz's Plan & Execute uses.")


def execute_via_trajectories(node, rclpy_mod, client_cls, action_type, solution_msg, server_timeout=10.0, log=print):
    """Runs the (already retimed) solution one sub-trajectory at a time through move_group's /execute_trajectory action. Returns MoveIt's error code of the first failing segment, 1 when every
    segment succeeded, or None when nothing could be run (no server, no answer). Stops at the first problem: the segments after it are NOT sent."""
    ac = client_cls(node, action_type, "execute_trajectory")
    if not ac.wait_for_server(timeout_sec=server_timeout):
        log("Execution did NOT run: the action /execute_trajectory is not available either (waited %.0f s). MoveIt cannot execute anything. Nothing moved." % server_timeout)
        return None
    segs = [(k, st) for k, st in enumerate(solution_msg.sub_trajectory) if len(st.trajectory.joint_trajectory.points) > 0]
    if not segs:
        log("Execution did NOT run: the plan has no trajectory points. Nothing moved.")
        return None
    log("Executing %d trajectory segment(s), one after the other, through /execute_trajectory (a failing segment stops the rest; STOP freezes the arm)." % len(segs))
    for n, (k, st) in enumerate(segs, 1):
        jt = st.trajectory.joint_trajectory
        last = jt.points[-1].time_from_start
        dur = last.sec + last.nanosec * 1e-9
        tag = "segment %d/%d (stage seg %d, %s, %d pts, %.1f s)" % (n, len(segs), k, "+".join(list(jt.joint_names)[:2]) + ("..." if len(jt.joint_names) > 2 else ""), len(jt.points), dur)
        goal = action_type.Goal()
        goal.trajectory = st.trajectory
        fut = ac.send_goal_async(goal)
        rclpy_mod.spin_until_future_complete(node, fut, timeout_sec=20.0)
        gh = fut.result() if fut.done() else None
        if gh is None or not gh.accepted:
            log("Execution STOPPED at %s: the goal was REJECTED by /execute_trajectory (or got no answer within 20 s). The segments before it were executed; the rest were NOT sent." % tag)
            return None
        rf = gh.get_result_async()
        rclpy_mod.spin_until_future_complete(node, rf, timeout_sec=3.0 * dur + 30.0)
        if not rf.done():
            log("Execution STOPPED at %s: no result within %.0f s. The arm may still be moving: use STOP. The rest were NOT sent." % (tag, 3.0 * dur + 30.0))
            return None
        code = rf.result().result.error_code.val
        log("  %s: error_code.val = %s" % (tag, code))
        if code != 1:
            log("Execution STOPPED at %s with MoveIt error code %s. The segments after it were NOT sent; the arm is where that segment left it (STOP freezes it; restart with the arm at home)." % (tag, code))
            return code
    return 1


def choose_solution(solutions, choice=None, to_msg=None, log=print):
    """Index of the solution to publish and execute. 'smooth' (default): the smallest biggest-single-joint travel; ties -> smaller total travel -> MTC's order. 'mtc': MTC's own #0."""
    import math
    choice = P2_SOLUTION_CHOICE if choice is None else choice
    if choice not in ("smooth", "mtc"):
        raise SystemExit("STOP: P2_SOLUTION_CHOICE must be 'smooth' or 'mtc' (got '%s')." % choice)
    if choice == "mtc" or len(solutions) <= 1:
        log("Solution choice: MTC's own lowest-cost solution (#0)%s." % ("" if choice == "mtc" else " (it is the only one)"))
        return 0
    try:
        rows = rank_solutions(solutions, to_msg or (lambda s: s.toMsg()))
    except Exception as e:
        log("Solution choice: could not compare the solutions (%s): using MTC's lowest-cost solution (#0)." % e)
        return 0
    if not rows:
        log("Solution choice: could not read the joint motion of the solutions: using MTC's lowest-cost solution (#0).")
        return 0
    best = min(rows, key=lambda r: (int(r["max_travel"] / 0.25), round(r["wrist"], 3), round(r["total"], 3), r["index"]))   # biggest joint in 0.25 rad bands, then how much the WRIST turns, then the total
    first = next((r for r in rows if r["index"] == 0), None)
    fmt = lambda r: "biggest single joint %s %.2f rad (%.0f deg), wrist (j5+j6+j7) %.2f rad, total %.2f rad" % (r["max_joint"], r["max_travel"], math.degrees(r["max_travel"]), r["wrist"], r["total"])
    log("Solution choice: the smoothest of %d solution(s) (smallest biggest-single-joint travel in 0.25 rad bands, then the least wrist turning; set P2_SOLUTION_CHOICE=mtc for MTC's own lowest cost)." % len(solutions))
    if first is not None:
        log("  MTC's lowest-cost solution #0: %s" % fmt(first))
    log("  chosen #%d: %s%s" % (best["index"], fmt(best), "  (the same as MTC's)" if best["index"] == 0 else ""))
    return best["index"]


def select_candidates_trust(poses, fallbacks):
    """GraspMolmo's pick first, then (optionally) its next-ranked grasps. No filter, no re-sorting."""
    return list(poses)[:1 + max(0, int(fallbacks))]
CGN_GRIPPER_DEPTH = 0.1034   # check against the grep above
OUR_FINGER_REACH = 0.14      # tcp_link origin -> middle of our finger pads; tune below
GRASP_OFFSET_X = 0.0   # m, world-frame nudge of every grasp (negative = farther from the base)
GRASP_OFFSET_Y = 0.0

# ===================== LIFT / TRANSIT / PLACE BEHAVIOUR (added 2026-10-12) =====================
# Each switch can be turned off to get the previous behaviour back.
APPLY_TCP_CONVERSION = False    # True: pass every candidate through to_tcp_pose() (jaw-axis and finger-depth conversion).
                                # False = what the arm has been doing so far: the raw Contact-GraspNet pose IS the tcp_link target,
                                # so OUR_FINGER_REACH only matters for the release-height calculation below, and CGN_GRIPPER_DEPTH /
                                # GRASP_OFFSET_X/Y have no effect.
LIFT_CARTESIAN = True           # lift and retreat in a straight vertical line (False: joint-space planner, which can wander)
TRANSIT_JOINT_INTERP_FIRST = True   # try a plain joint-space interpolation first (smooth, deterministic); the sampling planner only as fallback
PLACE_HEIGHT_MODE = "auto"      # "auto": release height computed from the grasp (see auto_release_tcp_z); "fixed": PLACE_Z_CLEARANCE_M above the plate
PLACE_RELEASE_GAP_M = 0.01      # auto: gap between the object's bottom and the plate floor at the moment the gripper opens
PLATE_THICKNESS_M = 0.02        # auto: how far the plate floor is above the table the object stood on  <-- MEASURE THIS with a ruler. 0.02 is a
                                # deliberately cautious guess: too small pushes the object INTO the plate, too large only means a bigger drop
PLACE_DESCEND = True            # hover above the release point, then straight down (False: go directly to the release point)
PLACE_HOVER_M = 0.06            # hover height above the release point
PLACE_CONE_DEG = 20             # +- tilt per axis allowed for the release orientation (was 32). Smaller = smaller drop error, fewer IK options
PLACE_OFFSET_X = 0.0            # m, world-frame nudge of the release point (for centring on the plate); negative X = farther from the base
PLACE_OFFSET_Y = 0.0
AUTO_Z_MIN, AUTO_Z_MAX = 0.05, 0.40   # sanity range (base_link z) for the automatic release height; outside it the fixed mode is used
# Optional speed-up of single phases (1.0 = unchanged). Applied only when the solution has exactly the expected number of arm segments.
PHASE_SPEED = {"connect": 1.0, "approach": 1.0, "lift": 1.0, "transit": 1.0, "descend": 1.0, "retreat": 1.0}
# ===============================================================================================

def to_tcp_pose(p):
    # tcp_link axes: approach = +Z, jaw = Y.  Contact-GraspNet: approach = +Z, jaw = X.
    # Rotate -90 deg about local Z so tcp Y lands on the grasp's X.
    qx, qy, qz, qw = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
    rx, ry, rz, rw = 0.0, 0.0, -0.70710678, 0.70710678
    nq = (qw*rx + qx*rw + qy*rz - qz*ry,
          qw*ry - qx*rz + qy*rw + qz*rx,
          qw*rz + qx*ry - qy*rx + qz*rw,
          qw*rw - qx*rx - qy*ry - qz*rz)
    # approach axis in world, then slide so our finger pads (not the Panda's) hit the contact point
    ax = 2 * (qx * qz + qw * qy)
    ay = 2 * (qy * qz - qw * qx)
    az = 1 - 2 * (qx * qx + qy * qy)
    shift = CGN_GRIPPER_DEPTH - OUR_FINGER_REACH
    out = Pose()
    out.position.x = p.position.x + ax * shift + GRASP_OFFSET_X
    out.position.y = p.position.y + ay * shift + GRASP_OFFSET_Y
    out.position.z = p.position.z + az * shift
    out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = nq
    return out

def quat_mul(a, b):
    # (x, y, z, w) tuples; returns a*b
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def axis_angle_quat(axis, deg):
    half = math.radians(deg) / 2.0
    s = math.sin(half)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half))

def tilt_from_down_deg(q):
    # approach axis = local +z of the pose, expressed in base_link
    az = 1 - 2 * (q.x * q.x + q.y * q.y)
    return math.degrees(math.acos(max(-1.0, min(1.0, -az))))

def approach_z_component(q):
    """z component (in base_link) of a pose's local +Z (approach) axis: about -1 when the gripper points straight down."""
    return 1 - 2 * (q.x * q.x + q.y * q.y)


def surface_release_tcp_z(grasp_poses, surface_z, table_z, gap):
    """v3: tcp_link height at the release (straight down), from the MEASURED surface the object is put on.
    At the grasp the object's bottom stands on the table (table_z), and the pads hold it (pad_z - table_z) above its bottom. At the release the
    object's bottom must be `gap` above the surface, so the pads must be at surface_z + (pad_z - table_z) + gap, and tcp_link OUR_FINGER_REACH
    above them. The HIGHEST value over the candidates is used (never lower than intended). -> (tcp_z or None, per-candidate list)"""
    zs = []
    for p in grasp_poses:
        pad_z = p.position.z + approach_z_component(p.orientation) * OUR_FINGER_REACH
        zs.append(surface_z + (pad_z - table_z) + gap + OUR_FINGER_REACH)
    if not zs:
        return None, []
    return max(zs), zs


def auto_release_tcp_z(grasp_poses):
    """
    Height for tcp_link at the moment of release, worked out from the GRASP itself so it adapts to any object.

    At the pick the object's bottom rests on the table, and the finger pads (OUR_FINGER_REACH along the approach axis from tcp_link)
    hold it at some height above the table. The plate floor is PLATE_THICKNESS_M above the same table. To put the object's bottom
    PLACE_RELEASE_GAP_M above the plate floor the pads must be at the same height as at the grasp plus (plate thickness + gap).
    The table and plate heights from perception cancel out, so only the grasp pose and two measured numbers are needed.

    Assumes a (near) vertical release; a tilted release makes the drop a little larger, never smaller.
    Uses the HIGHEST value over the candidates so the object is never lower than intended whichever candidate MTC picks.
    Returns (tcp_z or None, per-candidate tcp_z list).
    """
    zs = []
    for p in grasp_poses:
        pad_z = p.position.z + approach_z_component(p.orientation) * OUR_FINGER_REACH
        zs.append(pad_z + PLATE_THICKNESS_M + PLACE_RELEASE_GAP_M + OUR_FINGER_REACH)
    if not zs:
        return None, []
    return max(zs), zs


# ======================= ARM SPEED: ONE PLACE =======================
# Every segment of the solution is retimed from these limits just
# before it is sent (OMPL and gripper segments arrive with no timing at
# all; Cartesian ones are overridden too, so one policy covers everything).
ARM_MAX_VEL = float(_os.environ.get("P5_ARM_MAX_VEL", "0.5"))   # rad/s per arm joint <- main speed knob (8e: 0.5, P5_ARM_MAX_VEL; was 0.3)
ARM_MAX_ACC = 1.0        # rad/s^2 per arm joint   <- how gently it starts/stops
GRIPPER_MAX_VEL = 0.03   # m/s
GRIPPER_MAX_ACC = 0.3    # m/s^2
MIN_STEP_S = 0.001


def retime_solution(solution_msg, phase_names=None):
    """Recompute time_from_start for every waypoint (trapezoidal profile).

    phase_names: optional list naming the ARM segments in order (connect, approach, lift, transit, descend, retreat).
    The PHASE_SPEED multipliers are applied only when the solution has exactly that many arm segments."""
    import math
    total = 0.0
    arm_idx = [k for k, t in enumerate(solution_msg.sub_trajectory)
               if len(t.trajectory.joint_trajectory.points) >= 2
               and not any(n.startswith("gripper") for n in t.trajectory.joint_trajectory.joint_names)]
    phase_of = {}
    if phase_names:
        if len(arm_idx) == len(phase_names):
            phase_of = dict(zip(arm_idx, phase_names))
        else:
            print(f"PHASE_SPEED not applied: the solution has {len(arm_idx)} arm segments, expected {len(phase_names)} {phase_names}")
    for seg_k, traj in enumerate(solution_msg.sub_trajectory):
        jt = traj.trajectory.joint_trajectory
        pts = jt.points
        if len(pts) < 2:
            continue
        f = PHASE_SPEED.get(phase_of.get(seg_k), 1.0)
        grip = [n.startswith("gripper") for n in jt.joint_names]
        vmax = [GRIPPER_MAX_VEL if g else ARM_MAX_VEL * f for g in grip]
        amax = [GRIPPER_MAX_ACC if g else ARM_MAX_ACC * f for g in grip]
        # time each step would take at full speed (slowest joint decides)
        need = [max(abs(b - a) / v for a, b, v in zip(p0.positions, p1.positions, vmax))
                for p0, p1 in zip(pts, pts[1:])]
        d = [0.0]
        for n in need:
            d.append(d[-1] + n)
        D = d[-1]
        A = min(a / v for a, v in zip(amax, vmax))      # accel in "speed units"/s
        if D < 1e-9:
            times = [i * 0.01 for i in range(len(pts))]
        else:
            if D >= 1.0 / A:                             # trapezoid
                T = D + 1.0 / A
                ramp_d = 0.5 / A

                def t_of(x):
                    if x <= ramp_d:
                        return math.sqrt(2 * x / A)
                    if x >= D - ramp_d:
                        return T - math.sqrt(2 * max(D - x, 0.0) / A)
                    return 1.0 / A + (x - ramp_d)
            else:                                        # triangle
                T = 2 * math.sqrt(D / A)

                def t_of(x):
                    if x <= D / 2:
                        return math.sqrt(2 * x / A)
                    return T - math.sqrt(2 * max(D - x, 0.0) / A)
            times = [t_of(x) for x in d]
        for i in range(1, len(times)):                   # strictly increasing
            if times[i] < times[i - 1] + MIN_STEP_S:
                times[i] = times[i - 1] + MIN_STEP_S
        for i, p in enumerate(pts):
            sec = int(times[i])
            nsec = int(round((times[i] - sec) * 1e9))
            if nsec >= 1000000000:
                sec, nsec = sec + 1, nsec - 1000000000
            p.time_from_start.sec = sec
            p.time_from_start.nanosec = nsec
            if i == 0 or i == len(pts) - 1:
                p.velocities = [0.0] * len(p.positions)
            else:
                dt = times[i + 1] - times[i - 1]
                p.velocities = [(q1 - q0) / dt for q0, q1 in
                                zip(pts[i - 1].positions, pts[i + 1].positions)]
            p.accelerations = []
        total += times[-1]
    print(f"Retimed solution: total motion time {total:.1f} s "
          f"(arm <= {ARM_MAX_VEL} rad/s, gripper <= {GRIPPER_MAX_VEL} m/s)")
# ====================================================================


def flip_180_about_local_z(pose_stamped):
    """
    Rotates a pose 180 degrees about its OWN local z-axis (the approach
    direction). For a parallel-jaw gripper, this produces a PHYSICALLY
    EQUIVALENT grasp (same object contact points, gripper just rolled
    180 degrees) - often reachable by a completely different joint
    configuration even when the original orientation isn't.
    """
    result = copy.deepcopy(pose_stamped)
    q = result.pose.orientation
    x1, y1, z1, w1 = q.x, q.y, q.z, q.w
    x2, y2, z2, w2 = 0.0, 0.0, 1.0, 0.0  # 180 deg about z
    result.pose.orientation.x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    result.pose.orientation.y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    result.pose.orientation.z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    result.pose.orientation.w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return result


def flip_180_about_tcp_local_x(pose_stamped):
    result = copy.deepcopy(pose_stamped)
    q = result.pose.orientation
    x1, y1, z1, w1 = q.x, q.y, q.z, q.w
    x2, y2, z2, w2 = 0.0, 0.0, 1.0, 0.0  # 180° about Z, not X
    result.pose.orientation.x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    result.pose.orientation.y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    result.pose.orientation.z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    result.pose.orientation.w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return result


def offset_pose_along_local_z(pose_stamped, distance):
    # ORIGINAL version, restored - for Contact-GraspNet's OWN grasp
    # poses, which use their own, separate approach-axis convention
    # (unrelated to tcp_link's real geometry) - used for PREGRASP.
    q = pose_stamped.pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    zx = 2 * (x * z + w * y)
    zy = 2 * (y * z - w * x)
    zz = 1 - 2 * (x * x + y * y)
    result = copy.deepcopy(pose_stamped)
    result.pose.position.x -= zx * distance
    result.pose.position.y -= zy * distance
    result.pose.position.z -= zz * distance
    return result


class PoseGrabber(Node):
    """Briefly subscribes to grab the latest grasp candidates + target pose, then done.

    No IK pre-checking happens here anymore (see module docstring for why
    that approach was abandoned) - this just collects the raw data MTC
    needs, and lets the real task decide reachability itself.
    """

    def __init__(self):
        super().__init__("mtc_pose_grabber")
        self.grasp_candidates = None
        self.target_pose = None
        self.create_subscription(
            PoseArray, GRASP_CANDIDATES_TOPIC, self._grasp_candidates_cb, LATCHED_QOS
        )
        self.create_subscription(PoseStamped, TARGET_POSE_TOPIC, self._target_cb, LATCHED_QOS)

    def _grasp_candidates_cb(self, msg):
        self.grasp_candidates = msg

    def _target_cb(self, msg):
        self.target_pose = msg

    def wait_for_both(self, timeout_sec=10.0):
        start = time.time()
        while self.grasp_candidates is None or self.target_pose is None:
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - start > timeout_sec:
                return False
        return True


def grab_poses():
    rclpy.init()    
    node = PoseGrabber()
    node.get_logger().info(
        f"Waiting for {GRASP_CANDIDATES_TOPIC} and {TARGET_POSE_TOPIC} "
        f"(run the perception chain services first if this hangs)..."
    )
    ok = node.wait_for_both(timeout_sec=10.0)
    grasp_candidates = node.grasp_candidates
    target_pose = node.target_pose
    node.destroy_node()
    rclpy.shutdown()

    if not ok or not grasp_candidates or len(grasp_candidates.poses) == 0:
        print("Timed out or got no grasp candidates. Run the pipeline-2 perception first:\n"
              "  ~/m2t2_tools/p2_run.sh perceive\n"
              "(it ends with: ros2 service call /pose_transform/transform_all std_srvs/srv/Trigger)")
        sys.exit(1)

    # Optional manual overrides - lets us test MTC's task construction with
    # known-good, trusted coordinates, decoupled from any current
    # perception/calibration accuracy issues (e.g. calibration drift).
    import os
    override_grasp = os.environ.get("MTC_OVERRIDE_GRASP")  # "x,y,z"
    override_target = os.environ.get("MTC_OVERRIDE_TARGET")  # "x,y,z"
    if override_grasp:
        x, y, z = map(float, override_grasp.split(","))
        for p in grasp_candidates.poses:
            p.position.x, p.position.y, p.position.z = x, y, z
        print(f"OVERRIDE: grasp candidates forced to ({x}, {y}, {z})")
    if override_target:
        x, y, z = map(float, override_target.split(","))
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        print(f"OVERRIDE: target forced to ({x}, {y}, {z})")

    return grasp_candidates, target_pose


def build_and_run_task(grasp_candidates, target_pose):
    import rclcpp
    from moveit.task_constructor import core, stages

    rclcpp.init()
    node = rclcpp.Node("nero_mtc_pick_and_place")

    # Pilz is a DETERMINISTIC industrial point-to-point planner (PTP) -
    # not a randomized sampling search like OMPL/RRTConnect. Same input,
    # same output, every time - directly addressing repeated run-to-run
    # inconsistency that OMPL's randomized nature caused. Confirmed
    # available on this system (seen in earlier move_group plugin
    # listings: "pilz_industrial_motion_planner/CommandPlanner").
    pipeline = core.PipelinePlanner(node, "pilz_industrial_motion_planner")
    pipeline.planner = "PTP"
    arm_planner = [(ARM_GROUP, pipeline)]
    cart = core.CartesianPath()
    cart.step_size = 0.005
    # pipeline.max_velocity_scaling_factor = 0.01
    # pipeline.max_acceleration_scaling_factor = 0.01
    # cart.max_velocity_scaling_factor = 0.01
    # cart.max_acceleration_scaling_factor = 0.01

    ik_frame = PoseStamped()
    ik_frame.header.frame_id = IK_FRAME_LINK
    ik_frame.pose.orientation.w = 1.0
    place_at_pads = _os.environ.get("P3_PLACE_AT_PADS", "1") != "0"
    place_ik_frame = copy.deepcopy(ik_frame)                  # v2: the finger pads (where the object is held), for the place IK only
    if place_at_pads:
        place_ik_frame.pose.position.z = OUR_FINGER_REACH

    frame_id = grasp_candidates.header.frame_id
    print(f"Grasp candidates arrive in frame: {grasp_candidates.header.frame_id}")
    candidate_poses = select_candidates_trust(grasp_candidates.poses, P2_GRASP_FALLBACKS)
    for i, p in enumerate(grasp_candidates.poses):
        tilt = tilt_from_down_deg(p.orientation)
        used = i < len(candidate_poses)
        print(f"  grasp {i}: pos=({p.position.x:.3f}, {p.position.y:.3f}, "
            f"{p.position.z:.3f}) tilt={tilt:.0f} deg -> {'USED (' + ('GraspMolmo pick' if i == 0 else 'fallback ' + str(i)) + ')' if used else 'not used'}")
    print(f"PIPELINE 2: trusting GraspMolmo's grasp as chosen (no tilt filter, no re-sorting); fallbacks={P2_GRASP_FALLBACKS}")
    if APPLY_TCP_CONVERSION:
        candidate_poses = [to_tcp_pose(p) for p in candidate_poses]
        print("APPLY_TCP_CONVERSION is on: candidates converted with to_tcp_pose()")
    if not candidate_poses:
        print("No grasp candidates - nothing to plan.")
        return

    task = core.Task()
    task.name = "nero pick and place"
    task.introspection()
    task.loadRobotModel(node)

    task.add(stages.CurrentState("current"))

    # --- Approach: try every candidate (x2 orientation variants) as
    #     parallel alternatives, using MTC's own real solving context ---
    if P2_CONNECT_PLANNER not in ("interp", "planner"):
        raise SystemExit("STOP: P2_CONNECT_PLANNER must be 'interp' or 'planner' (got '%s')." % P2_CONNECT_PLANNER)
    if P2_CONNECT_PLANNER == "interp":
        connect_options = core.Fallbacks("connect options")
        connect_options.add(stages.Connect("connect to pregrasp (joint interpolation)", [(ARM_GROUP, core.JointInterpolationPlanner())]))
        connect_options.add(stages.Connect("connect to pregrasp (planner)", arm_planner))
        task.add(connect_options)
    else:
        task.add(stages.Connect("connect to pregrasp", arm_planner))

    pregrasp_alternatives = core.Alternatives("pregrasp IK options")
    for i, candidate in enumerate(candidate_poses):
        for variant_name, pose in [
            (name, candidate if name == "orig" else flip_180_about_local_z(
                PoseStamped(header=grasp_candidates.header, pose=candidate)
            ).pose)
            for name in orientation_variants()
        ]:
            pregrasp_ps = PoseStamped(header=grasp_candidates.header, pose=pose)
            pregrasp_ps = offset_pose_along_local_z(pregrasp_ps, PREGRASP_OFFSET_M)

            gen = stages.GeneratePose(f"pregrasp pose {i}-{variant_name}")
            gen.pose = pregrasp_ps
            gen.setMonitoredStage(task["current"])

            ik = stages.ComputeIK(f"pregrasp IK {i}-{variant_name}", gen)
            ik.group = ARM_GROUP
            ik.ik_frame = ik_frame
            ik.max_ik_solutions = 20
            ik.properties.configureInitFrom(
                core.Stage.PropertyInitializerSource.INTERFACE, ["target_pose"]
            )
            pregrasp_alternatives.add(ik)

    task.add(pregrasp_alternatives)

    # PIPELINE 5: the gripper may touch the object (carrot) from HERE on, i.e. while opening and during the approach too. The carrot is a box
    # model of the object (wider than its neck, where the grasp is); the grasp step already checked the real fingers against its real points.
    allow_touch = stages.ModifyPlanningScene("allow gripper-object contact")
    # (the LINK names: MTC reads a plain string as one link name - "gripper" is no link, so pipeline 3's allowance never applied)
    allow_touch.allowCollisions(["carrot"], ["gripper_base", "gripper_link1", "gripper_link2", "gripper_flange", "tcp_link"], True)
    task.add(allow_touch)

    open_gripper = stages.MoveTo("open gripper", pipeline)
    open_gripper.group = GRIPPER_GROUP
    _p4_gripper_goal(open_gripper, "open")
    task.add(open_gripper)

    approach = stages.MoveRelative("approach", cart)
    approach.group = ARM_GROUP
    approach.ik_frame = ik_frame
    approach_twist = TwistStamped()
    approach_twist.header.frame_id = IK_FRAME_LINK
    approach_twist.twist.linear.z = 1.0  # local +z = toward the grasp pose
    approach.setDirection(approach_twist)
    approach.min_distance = APPROACH_MIN
    approach.max_distance = APPROACH_MAX
    task.add(approach)

    # --- Grasp: close, attach (PIPELINE 5: contact with the object was allowed before opening, above) ---

    close_gripper = stages.MoveTo("close gripper", pipeline)
    close_gripper.group = GRIPPER_GROUP
    close_gripper.setGoal("gripper_close")
    task.add(close_gripper)

    attach = stages.ModifyPlanningScene("attach carrot")
    attach.attachObject("carrot", IK_FRAME_LINK)
    task.add(attach)

    # --- Lift ---
    lift = stages.MoveRelative("lift", cart if LIFT_CARTESIAN else pipeline)
    lift.group = ARM_GROUP
    lift.ik_frame = ik_frame
    lift_twist = TwistStamped()
    lift_twist.header.frame_id = BASE_FRAME
    lift_twist.twist.linear.z = 1.0
    lift.setDirection(lift_twist)
    lift.min_distance = LIFT_MIN
    lift.max_distance = LIFT_MAX
    task.add(lift)

    # --- Transit ---
    if TRANSIT_JOINT_INTERP_FIRST:
        # smooth, deterministic joint-space interpolation first; the sampling planner is only the fallback
        transit = core.Fallbacks("transit options")
        transit.add(stages.Connect("transit (joint interpolation)", [(ARM_GROUP, core.JointInterpolationPlanner())]))
        transit.add(stages.Connect("transit (planner)", arm_planner))
        task.add(transit)
    else:
        task.add(stages.Connect("transit", arm_planner))

    # --- Place: try both orientation options (top candidate's own
    #     orientation, and its 180 flip) as parallel alternatives -
    #     the user has no requirement on final placement orientation. ---
    base_place_position = target_pose.pose.position
    place_x = base_place_position.x + PLACE_OFFSET_X
    place_y = base_place_position.y + PLACE_OFFSET_Y
    place_z = base_place_position.z + PLACE_Z_CLEARANCE_M          # "fixed" mode
    height_note = f"fixed: plate z {base_place_position.z:.3f} + clearance {PLACE_Z_CLEARANCE_M:.3f}"
    p3_height = _os.environ.get("P3_PLACE_HEIGHT", "surface")
    if p3_height == "surface":                                     # v3: follow the measured surface (cube top, plate floor)
        sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
        import p3_tablez
        table_z = p3_tablez.table_z()[0]
        gap = float(_os.environ.get("P3_RELEASE_GAP_M", "0.015"))
        surf_z, surf_zs = surface_release_tcp_z(candidate_poses, base_place_position.z, table_z, gap)
        if surf_z is not None and AUTO_Z_MIN <= surf_z <= AUTO_Z_MAX:
            place_z = surf_z
            height_note = (f"surface: the place surface is at z {base_place_position.z:+.3f} (measured), the object's bottom is released "
                           f"{1000 * gap:.0f} mm above it; release tcp z {surf_z:.3f} (per candidate {', '.join(f'{z:.3f}' for z in surf_zs)}; "
                           f"table {table_z:+.4f}, finger reach {OUR_FINGER_REACH:.3f})")
        else:
            print(f"WARNING: surface-based release height {surf_z} is not usable (allowed {AUTO_Z_MIN}..{AUTO_Z_MAX}); using the fixed clearance instead.")
    elif PLACE_HEIGHT_MODE == "auto":
        auto_z, auto_zs = auto_release_tcp_z(candidate_poses)
        if auto_z is not None and AUTO_Z_MIN <= auto_z <= AUTO_Z_MAX:
            fixed_z = place_z                                      # what the old fixed method would have used
            place_z = auto_z
            height_note = (f"auto: grasp-based, release tcp z {auto_z:.3f} (per candidate "
                           f"{', '.join(f'{z:.3f}' for z in auto_zs)}; plate thickness {PLATE_THICKNESS_M:.3f}, gap {PLACE_RELEASE_GAP_M:.3f}, "
                           f"finger reach {OUR_FINGER_REACH:.3f}); the old fixed method would have used {fixed_z:.3f}, "
                           f"i.e. the object is released {1000 * (fixed_z - auto_z):.0f} mm lower")
            if max(auto_zs) - min(auto_zs) > 0.04:
                print(f"WARNING: the candidates' release heights differ by {max(auto_zs) - min(auto_zs):.3f} m; "
                      f"the highest is used, so the drop may be larger for some candidates.")
        else:
            print(f"WARNING: automatic release height {auto_z} is not usable (allowed {AUTO_Z_MIN}..{AUTO_Z_MAX}); using the fixed clearance instead.")
    hover = PLACE_HOVER_M if PLACE_DESCEND else 0.0
    ik_place_z = place_z + hover                                   # the pose the place IK solves for (hover point, or the release point itself)
    if place_at_pads:
        ik_place_z -= OUR_FINGER_REACH                             # v2: the target is now the PADS' height (tcp_link stays OUR_FINGER_REACH above them when straight down)
        print(f"Place: the finger pads (the object) are put on the target, {OUR_FINGER_REACH:.3f} m along the gripper from tcp_link, so a tilted release "
              f"(up to +-{PLACE_CONE_DEG} deg) does not shift the object; P3_PLACE_AT_PADS=0 = the old behaviour (tcp_link on the target)")

    def make_quat(x, y, z, w):
        from geometry_msgs.msg import Quaternion
        return Quaternion(x=x, y=y, z=z, w=w)

    # ONE well-defined "release" pose, formatted the same way as a grasp
    # pose (a real 6-DoF position + orientation, not just a bare position)
    # - the release point is the plate's own center, PLACE_Z_CLEARANCE_M
    # above its surface, with the gripper oriented straight down.
    #
    # This top-down orientation (flip_y: 0,1,0,0) isn't an arbitrary guess
    # - it's backed by real evidence from reach_sweep.py testing, where it
    # succeeded far more often than any other tried orientation across
    # many different positions, strongly indicating it's close to this
    # gripper's natural "pointing straight down" convention. Its 180-flip
    # (rolling which way the fingers point, without changing the overall
    # top-down approach direction) is offered as a second option, same as
    # the grasp side's own orig/flip pairing.
    top_down_orientation = make_quat(1.0, 0.0, 0.0, 0.0)
    TOP_DOWN = (1.0, 0.0, 0.0, 0.0)   # the flip_x you confirmed visually
    TILT_DEGS = [20, 40]
    
    place_ps = PoseStamped()
    place_ps.header.frame_id = BASE_FRAME
    place_ps.pose.position.x = place_x
    place_ps.pose.position.y = place_y
    place_ps.pose.position.z = ik_place_z
    if P2_PLACE_YAW not in ("grasp", "fixed"):
        raise SystemExit("STOP: P2_PLACE_YAW must be 'grasp' or 'fixed' (got '%s')." % P2_PLACE_YAW)
    place_yaws = {"fixed": 180.0}                            # pipeline 1's fixed seed
    place_variants = ["fixed"]
    if P2_PLACE_YAW == "grasp" and len(candidate_poses) > 0:
        place_variants = list(orientation_variants())
        for _v in place_variants:                            # one release seed per orientation variant: the yaw of THAT variant's hand
            _ref = candidate_poses[0] if _v == "orig" else flip_180_about_local_z(PoseStamped(header=grasp_candidates.header, pose=candidate_poses[0])).pose
            place_yaws[_v] = hand_yaw_deg(_ref)
    place_yaw = place_yaws[place_variants[0]]
    place_ps.pose.orientation = make_quat(*place_seed_quat(place_yaw))   # exact top-down seed (yaw = the grasp's, or the fixed 180 deg)

    place_iks = []
    for _v in place_variants:
        _ps = PoseStamped()
        _ps.header.frame_id = BASE_FRAME
        _ps.pose.position.x, _ps.pose.position.y, _ps.pose.position.z = place_x, place_y, ik_place_z
        _ps.pose.orientation = make_quat(*place_seed_quat(place_yaws[_v]))
        _suffix = "" if len(place_variants) == 1 else f" ({_v})"
        gen = stages.GenerateRandomPose("place pose cone" + _suffix)
        gen.pose = _ps
        gen.setMonitoredStage(task["lift"])
        gen.set_max_solutions(40 if len(place_variants) == 1 else 25)
        W = math.radians(2 * PLACE_CONE_DEG)   # full range -> +-PLACE_CONE_DEG per axis
        gen.sample_dimension(stages.PoseDimension.ROLL, W)
        gen.sample_dimension(stages.PoseDimension.PITCH, W)
        if P2_PLACE_YAW == "grasp":
            gen.sample_dimension(stages.PoseDimension.YAW, math.radians(2 * P2_PLACE_YAW_RANGE_DEG))   # +- P2_PLACE_YAW_RANGE_DEG around that variant's grasp yaw

        ik = stages.ComputeIK("place IK cone" + _suffix, gen)
        ik.group = ARM_GROUP
        ik.ik_frame = place_ik_frame                               # v2: the pads on the target (ik_frame itself if P3_PLACE_AT_PADS=0)
        ik.max_ik_solutions = 20
        ik.properties.configureInitFrom(
            core.Stage.PropertyInitializerSource.INTERFACE, ["target_pose"]
        )
        place_iks.append(ik)
    

    # Plan DIRECTLY to the final place pose via free-form joint-space
    # planning (same technique already proven completely reliable for
    # "transit") - NOT a preplace-offset + short Cartesian descent. A
    # Cartesian motion (MoveRelative) can fail even when both its
    # endpoints are individually reachable, if the straight line between
    # them crosses a joint limit or singularity partway through -
    # confirmed: "place" failed consistently across all 10 very different
    # preplace solutions, pointing to exactly this kind of path-level
    # issue rather than an endpoint reachability problem. Since precise
    # straight-line placement doesn't matter here (only "does it land on
    # the plate"), free-form planning directly to the rest pose is a
    # reasonable simplification that avoids the whole class of problem.
    place_alternatives = core.Alternatives("place IK options")
    for _ik in place_iks:
        place_alternatives.add(_ik)
    task.add(place_alternatives)

    if hover > 0:
        # straight down from the hover point to the release point (a short Cartesian move: far more reliable than the old 0.5 m one)
        descend = stages.MoveRelative("place descend", cart)
        descend.group = ARM_GROUP
        descend.ik_frame = ik_frame
        descend_twist = TwistStamped()
        descend_twist.header.frame_id = BASE_FRAME
        descend_twist.twist.linear.z = -1.0
        descend.setDirection(descend_twist)
        # 7i: if something is in the way, stop there and let go (the object drops the rest) - the user's choice; was hover - 5 mm (all the way)
        descend.min_distance = min(hover, max(0.0, float(_os.environ.get("P5_DESCEND_MIN_M", "0.01"))))
        descend.max_distance = hover
        task.add(descend)
    phase_names = ["connect", "approach", "lift", "transit"] + (["descend"] if hover > 0 else []) + ["retreat"]

    detach = stages.ModifyPlanningScene("detach carrot")
    detach.detachObject("carrot", IK_FRAME_LINK)
    task.add(detach)

    release_gripper = stages.MoveTo("release gripper", pipeline)
    release_gripper.group = GRIPPER_GROUP
    _p4_gripper_goal(release_gripper, "release")
    task.add(release_gripper)

    # --- Retreat ---
    retreat = stages.MoveRelative("retreat", cart if LIFT_CARTESIAN else pipeline)
    retreat.group = ARM_GROUP
    retreat.ik_frame = ik_frame
    retreat_twist = TwistStamped()
    retreat_twist.header.frame_id = BASE_FRAME
    retreat_twist.twist.linear.z = 1.0
    retreat.setDirection(retreat_twist)
    retreat.min_distance = RETREAT_MIN
    retreat.max_distance = RETREAT_MAX
    task.add(retreat)

    print(f"Trying {len(candidate_poses)} grasp candidate(s) x{len(orientation_variants())} orientation variant(s) each ({', '.join(orientation_variants())}; 'flip' = the same grasp rolled 180 deg about the approach axis).")
    print(f"First move (current pose -> pregrasp): {'a direct joint interpolation first, the planner only as fallback' if P2_CONNECT_PLANNER == 'interp' else 'the planner (pipeline 1)'}.")
    if P2_PLACE_YAW == "grasp":
        if len(place_variants) == 1:
            print(f"Place yaw: follows the grasp (seed {place_yaw:.0f} deg, sampled +-{P2_PLACE_YAW_RANGE_DEG:.0f} deg) so the wrist does not turn between pick and place; P2_PLACE_YAW=fixed restores the fixed 180 deg of pipeline 1.")
        else:
            print("Place yaw: follows the grasp, one release seed per orientation (" + ", ".join(f"{v} {place_yaws[v]:.0f} deg" for v in place_variants) + f"; sampled +-{P2_PLACE_YAW_RANGE_DEG:.0f} deg each) so the wrist does not turn between pick and place whichever is used; P2_PLACE_YAW=fixed restores the fixed 180 deg of pipeline 1.")
    else:
        print("Place yaw: FIXED 180 deg (P2_PLACE_YAW=fixed, pipeline 1's behaviour).")
    print(f"Target position: ({place_x:.3f}, {place_y:.3f}, {place_z:.3f})  [release point]   plate pose seen: "
          f"({base_place_position.x:.3f}, {base_place_position.y:.3f}, {base_place_position.z:.3f})")
    print(f"Place height -> {height_note}")
    print(f"Place: hover {hover:.3f} m above the release point, cone +-{PLACE_CONE_DEG} deg, descend={'on' if hover > 0 else 'off'}; "
          f"lift {LIFT_MIN:.2f}-{LIFT_MAX:.2f} m ({'straight line' if LIFT_CARTESIAN else 'planner'}); "
          f"transit {'joint interpolation first' if TRANSIT_JOINT_INTERP_FIRST else 'planner only'}")
    print("Planning...")

    if task.plan(50 if len(orientation_variants()) == 1 else 100):
        print(f"Found {len(task.solutions)} solution(s).")
        chosen_index = choose_solution(task.solutions)
        chosen_solution = task.solutions[chosen_index]
        task.publish(chosen_solution)
        try:
            for _line in segment_report(chosen_solution.toMsg(), phase_names):
                print(_line)
        except Exception as _e:
            print(f"(could not tabulate the joint travel: {_e})")
        print("\nPlan found and published to RViz's Motion Planning Tasks panel for review.")
        import os
        answer = os.environ.get("MTC_EXECUTE", "").strip().lower()
        print(f"MTC_EXECUTE={answer!r} (set MTC_EXECUTE=y to execute, anything else skips)")
        refused = False
        if answer == "y":
            try:
                _msg = refusal_message(biggest_stage_joint(chosen_solution.toMsg(), phase_names), P2_MAX_JOINT_STAGE_RAD)
            except Exception as _e:
                _msg = None
                print(f"(could not check the joint travel before executing: {_e})")
            if _msg:
                print(_msg)
                refused = True
        if answer == "y" and not refused:
            # Bypass task.execute() entirely - it silently discards the
            # real, detailed error_code from the action result whenever
            # execution fails, always returning a generic FAILURE (99999)
            # placeholder instead. Call the action directly to see the
            # REAL reason.
            import rclpy as rclpy_module
            try:
                rclpy_module.init()
            except Exception:
                pass  # already initialized, fine
            exec_node = rclpy_module.create_node("mtc_execute_helper")
            action_client = None
            task_up = False
            if P2_EXECUTE_VIA in ("auto", "task"):
                action_client = ActionClient(exec_node, ExecuteTaskSolution, "execute_task_solution")
                task_up = bool(action_client.wait_for_server(timeout_sec=10.0))   # (the answer used to be ignored: a missing action made the next wait last forever)
            route, route_msg = choose_execute_route(P2_EXECUTE_VIA, task_up)
            if route_msg:
                print(route_msg)
            solution_msg = chosen_solution.toMsg()

            if route is not None:
                for _k, _st in enumerate(solution_msg.sub_trajectory):
                    _pts = _st.trajectory.joint_trajectory.points
                    if _pts:
                        _l = _pts[-1].time_from_start
                        print(f"DIAG seg {_k}: {len(_pts)} pts, last t={_l.sec + _l.nanosec * 1e-9:.2f}s, "
                              f"velocities={'yes' if _pts[-1].velocities else 'no'}, "
                              f"joints={list(_st.trajectory.joint_trajectory.joint_names)[:2]}")
                retime_solution(solution_msg, phase_names)

            if route == "task":
                goal = ExecuteTaskSolution.Goal()
                goal.solution = solution_msg
                send_goal_future = action_client.send_goal_async(goal)
                rclpy_module.spin_until_future_complete(exec_node, send_goal_future, timeout_sec=30.0)
                goal_handle = send_goal_future.result() if send_goal_future.done() else None
                if not goal_handle or not goal_handle.accepted:
                    print("Goal was REJECTED by the action server (not even accepted, or no answer within 30 s). Nothing was executed.")
                else:
                    result_future = goal_handle.get_result_async()
                    rclpy_module.spin_until_future_complete(exec_node, result_future)
                    result = result_future.result()
                    print(f"REAL error_code.val = {result.result.error_code.val}")
                    print(f"Result status (0=UNKNOWN,1=SUCCEEDED,2=CANCELED,3=ABORTED) = {result.status}")
            elif route == "trajectory":
                from moveit_msgs.action import ExecuteTrajectory
                code = execute_via_trajectories(exec_node, rclpy_module, ActionClient, ExecuteTrajectory, solution_msg)
                if code is not None:
                    print(f"REAL error_code.val = {code}")
                    print("Executed through /execute_trajectory" + (": every segment succeeded." if code == 1 else ": it stopped at the first failing segment (above)."))
            exec_node.destroy_node()
        elif not refused:
            print("Skipped execution - plan was found but not run.")
            _p5_save_plan(chosen_solution.toMsg(), phase_names)   # PIPELINE 5

        if not refused and _os.environ.get("P5_PLAN_EXIT", "0") != "1":   # PIPELINE 5: p5_go.sh does not wait here
            print("\nKeeping node alive indefinitely so RViz's 'Motion Planning "
                  "Tasks' panel can select and execute solutions directly. "
                  "Ctrl+C to exit when done.")
            try:
                while True:
                    time.sleep(1.0)
            except KeyboardInterrupt:
                pass
    else:
        print("Planning FAILED - no valid solution found across all tried "
              "candidates/orientations. Check RViz's MotionPlanning Tasks "
              "panel for which stage(s) failed and why.")

    del pipeline
    del arm_planner


def _p5_save_plan(solution_msg, phase_names):
    """PIPELINE 5: the chosen plan, retimed exactly as for execution, saved so p5_execute.sh can run it WITHOUT planning again
    (~/p3_runs/p5_plan_latest.bin = moveit_task_constructor_msgs/Solution, + .json: which look it belongs to, the arm's start joints)."""
    import json as _j
    import time as _t
    try:
        from rclpy.serialization import serialize_message
        retime_solution(solution_msg, phase_names)
        try:
            refusal = refusal_message(biggest_stage_joint(solution_msg, phase_names), P2_MAX_JOINT_STAGE_RAD)
        except Exception:
            refusal = None
        d = _os.path.expanduser("~/p3_runs")
        look = None
        try:
            look = _j.load(open(_os.path.join(d, "p5_grasp_latest.json")))["unix_time"]
        except Exception:
            pass
        js = solution_msg.start_scene.robot_state.joint_state
        with open(_os.path.join(d, "p5_plan_latest.bin.tmp"), "wb") as f:
            f.write(serialize_message(solution_msg))
        _os.replace(_os.path.join(d, "p5_plan_latest.bin.tmp"), _os.path.join(d, "p5_plan_latest.bin"))
        meta = {"unix_time": _t.time(), "time": _t.strftime("%Y-%m-%d %H:%M:%S"), "look_unix_time": look, "refusal": refusal,
                "segments": len(solution_msg.sub_trajectory), "start_joints": dict(zip(list(js.name), [float(p) for p in js.position]))}
        with open(_os.path.join(d, "p5_plan_latest.json"), "w") as f:
            _j.dump(meta, f, indent=1)
        print("PIPELINE 5: plan saved for p5_execute.sh (%d segments%s)" % (meta["segments"], "; REFUSED for execution: " + refusal if refusal else ""))
    except Exception as e:
        print("PIPELINE 5: the plan could NOT be saved (%s: %s) - p5_execute.sh will refuse; plan again" % (type(e).__name__, e))


def main():
    grasp_candidates, target_pose = grab_poses()
    build_and_run_task(grasp_candidates, target_pose)


if __name__ == "__main__":
    main()
