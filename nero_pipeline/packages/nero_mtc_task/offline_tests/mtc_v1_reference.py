#!/usr/bin/env python3
"""
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
APPROACH_MIN, APPROACH_MAX = 0.02, 0.08
LIFT_MIN, LIFT_MAX = 0.02, 0.05
PLACE_MIN, PLACE_MAX = 0.02, 0.05
RETREAT_MIN, RETREAT_MAX = 0.02, 0.15
MAX_TILT_DEG = 55.0
CGN_GRIPPER_DEPTH = 0.1034   # check against the grep above
OUR_FINGER_REACH = 0.14      # tcp_link origin -> middle of our finger pads; tune below
GRASP_OFFSET_X = 0.0   # m, world-frame nudge of every grasp (negative = farther from the base)
GRASP_OFFSET_Y = 0.0

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

# ======================= ARM SPEED: ONE PLACE =======================
# Every segment of the solution is retimed from these limits just
# before it is sent (OMPL and gripper segments arrive with no timing at
# all; Cartesian ones are overridden too, so one policy covers everything).
ARM_MAX_VEL = 0.1        # rad/s per arm joint     <- main speed knob
ARM_MAX_ACC = 1.0        # rad/s^2 per arm joint   <- how gently it starts/stops
GRIPPER_MAX_VEL = 0.03   # m/s
GRIPPER_MAX_ACC = 0.3    # m/s^2
MIN_STEP_S = 0.001


def retime_solution(solution_msg):
    """Recompute time_from_start for every waypoint (trapezoidal profile)."""
    import math
    total = 0.0
    for traj in solution_msg.sub_trajectory:
        jt = traj.trajectory.joint_trajectory
        pts = jt.points
        if len(pts) < 2:
            continue
        grip = [n.startswith("gripper") for n in jt.joint_names]
        vmax = [GRIPPER_MAX_VEL if g else ARM_MAX_VEL for g in grip]
        amax = [GRIPPER_MAX_ACC if g else ARM_MAX_ACC for g in grip]
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
        print("Timed out or got no grasp candidates. Run the perception "
              "chain services first:\n"
              "  ros2 service call /gsam2/segment std_srvs/srv/Trigger\n"
              "  ros2 service call /contact_graspnet/detect_grasps std_srvs/srv/Trigger\n"
              "  ros2 service call /target_pose/extract std_srvs/srv/Trigger\n"
              "  ros2 service call /pose_transform/transform_all std_srvs/srv/Trigger")
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

    frame_id = grasp_candidates.header.frame_id
    print(f"Grasp candidates arrive in frame: {grasp_candidates.header.frame_id}")
    usable = []
    for i, p in enumerate(grasp_candidates.poses):
        tilt = tilt_from_down_deg(p.orientation)
        keep = tilt <= MAX_TILT_DEG
        print(f"  candidate {i}: pos=({p.position.x:.3f}, {p.position.y:.3f}, "
            f"{p.position.z:.3f}) tilt={tilt:.0f} deg -> {'KEEP' if keep else 'drop'}")
        if keep:
            usable.append(p)
    usable.sort(key=lambda q: tilt_from_down_deg(q.orientation))   # most top-down first
    candidate_poses = usable[:MAX_CANDIDATES_IN_TASK]
    if not candidate_poses:
        print("No grasp candidates within the top-down cone - nothing to plan.")
        return

    task = core.Task()
    task.name = "nero pick and place"
    task.introspection()
    task.loadRobotModel(node)

    task.add(stages.CurrentState("current"))

    # --- Approach: try every candidate (x2 orientation variants) as
    #     parallel alternatives, using MTC's own real solving context ---
    task.add(stages.Connect("connect to pregrasp", arm_planner))

    pregrasp_alternatives = core.Alternatives("pregrasp IK options")
    for i, candidate in enumerate(candidate_poses):
        for variant_name, pose in [
            # ("orig", candidate),
            ("flip", flip_180_about_local_z(
                PoseStamped(header=grasp_candidates.header, pose=candidate)
            ).pose),
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

    open_gripper = stages.MoveTo("open gripper", pipeline)
    open_gripper.group = GRIPPER_GROUP
    open_gripper.setGoal("gripper_open")
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

    # --- Grasp: allow touching the object, close, attach ---
    allow_touch = stages.ModifyPlanningScene("allow gripper-object contact")
    allow_touch.allowCollisions("carrot", GRIPPER_GROUP, True)
    task.add(allow_touch)

    close_gripper = stages.MoveTo("close gripper", pipeline)
    close_gripper.group = GRIPPER_GROUP
    close_gripper.setGoal("gripper_close")
    task.add(close_gripper)

    attach = stages.ModifyPlanningScene("attach carrot")
    attach.attachObject("carrot", IK_FRAME_LINK)
    task.add(attach)

    # --- Lift ---
    lift = stages.MoveRelative("lift", pipeline)
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
    task.add(stages.Connect("transit", arm_planner))

    # --- Place: try both orientation options (top candidate's own
    #     orientation, and its 180 flip) as parallel alternatives -
    #     the user has no requirement on final placement orientation. ---
    base_place_position = target_pose.pose.position
    place_z = base_place_position.z + PLACE_Z_CLEARANCE_M

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
    place_ps.pose.position.x = base_place_position.x
    place_ps.pose.position.y = base_place_position.y
    place_ps.pose.position.z = place_z
    place_ps.pose.orientation = make_quat(0.0, 1.0, 0.0, 0.0)   # exact top-down seed

    gen = stages.GenerateRandomPose("place pose cone")
    gen.pose = place_ps
    gen.setMonitoredStage(task["lift"])
    gen.set_max_solutions(40)
    W = math.radians(64)   # full range -> +-32 deg per axis, corners reach ~45 deg total
    gen.sample_dimension(stages.PoseDimension.ROLL, W)
    gen.sample_dimension(stages.PoseDimension.PITCH, W)
    # gen.sample_dimension(stages.PoseDimension.YAW, W)

    ik = stages.ComputeIK("place IK cone", gen)
    ik.group = ARM_GROUP
    ik.ik_frame = ik_frame
    ik.max_ik_solutions = 20
    ik.properties.configureInitFrom(
        core.Stage.PropertyInitializerSource.INTERFACE, ["target_pose"]
    )
    

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
    place_alternatives.add(ik)
    task.add(place_alternatives)

    detach = stages.ModifyPlanningScene("detach carrot")
    detach.detachObject("carrot", IK_FRAME_LINK)
    task.add(detach)

    release_gripper = stages.MoveTo("release gripper", pipeline)
    release_gripper.group = GRIPPER_GROUP
    release_gripper.setGoal("gripper_open")
    task.add(release_gripper)

    # --- Retreat ---
    retreat = stages.MoveRelative("retreat", pipeline)
    retreat.group = ARM_GROUP
    retreat.ik_frame = ik_frame
    retreat_twist = TwistStamped()
    retreat_twist.header.frame_id = BASE_FRAME
    retreat_twist.twist.linear.z = 1.0
    retreat.setDirection(retreat_twist)
    retreat.min_distance = RETREAT_MIN
    retreat.max_distance = RETREAT_MAX
    task.add(retreat)

    print(f"Trying {len(candidate_poses)} grasp candidate(s) x2 orientation variants each.")
    print(f"Target position: ({base_place_position.x:.3f}, {base_place_position.y:.3f}, {place_z:.3f})")
    print("Planning...")

    if task.plan(50):
        print(f"Found {len(task.solutions)} solution(s).")
        task.publish(task.solutions[0])
        print("\nPlan found and published to RViz's Motion Planning Tasks panel for review.")
        import os
        answer = os.environ.get("MTC_EXECUTE", "").strip().lower()
        print(f"MTC_EXECUTE={answer!r} (set MTC_EXECUTE=y to execute, anything else skips)")
        if answer == "y":
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
            action_client = ActionClient(exec_node, ExecuteTaskSolution, "execute_task_solution")
            action_client.wait_for_server(timeout_sec=10.0)
            goal = ExecuteTaskSolution.Goal()
            solution_msg = task.solutions[0].toMsg()

            for _k, _st in enumerate(solution_msg.sub_trajectory):
                _pts = _st.trajectory.joint_trajectory.points
                if _pts:
                    _l = _pts[-1].time_from_start
                    print(f"DIAG seg {_k}: {len(_pts)} pts, last t={_l.sec + _l.nanosec * 1e-9:.2f}s, "
                          f"velocities={'yes' if _pts[-1].velocities else 'no'}, "
                          f"joints={list(_st.trajectory.joint_trajectory.joint_names)[:2]}")
            retime_solution(solution_msg)
            goal.solution = solution_msg
            send_goal_future = action_client.send_goal_async(goal)
            rclpy_module.spin_until_future_complete(exec_node, send_goal_future)
            goal_handle = send_goal_future.result()
            if not goal_handle or not goal_handle.accepted:
                print("Goal was REJECTED by the action server (not even accepted).")
            else:
                result_future = goal_handle.get_result_async()
                rclpy_module.spin_until_future_complete(exec_node, result_future)
                result = result_future.result()
                print(f"REAL error_code.val = {result.result.error_code.val}")
                print(f"Result status (0=UNKNOWN,1=SUCCEEDED,2=CANCELED,3=ABORTED) = {result.status}")
            exec_node.destroy_node()
        else:
            print("Skipped execution - plan was found but not run.")

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


def main():
    grasp_candidates, target_pose = grab_poses()
    build_and_run_task(grasp_candidates, target_pose)


if __name__ == "__main__":
    main()
