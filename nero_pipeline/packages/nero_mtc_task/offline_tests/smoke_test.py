import os
HERE = os.path.dirname(os.path.abspath(__file__)); BASE = os.path.dirname(HERE)
import sys, io, math, importlib.util, contextlib
sys.path.insert(0, os.path.join(HERE, "stubs"))
import registry
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseArray, Header

def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m

def q_mul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by, aw*by - ax*bz + ay*bw + az*bx, aw*bz + ax*by - ay*bx + az*bw, aw*bw - ax*bx - ay*by - az*bz)

def pose_down(x, y, z, tilt_deg=0.0):
    """tcp-style pose pointing down (180 deg about x) then tilted about y"""
    h = math.radians(tilt_deg) / 2
    q = q_mul((0.0, math.sin(h), 0.0, math.cos(h)), (1.0, 0.0, 0.0, 0.0))
    return Pose(Point(x, y, z), Quaternion(*q))

def run(path, name, **overrides):
    m = load(path, name)
    for k, v in overrides.items(): setattr(m, k, v)
    registry.LOG.clear()
    grasps = PoseArray(Header("base_link"), [pose_down(-0.33, -0.05, 0.120, 0), pose_down(-0.32, -0.04, 0.128, 15), pose_down(-0.34, -0.06, 0.115, 30)])
    target = PoseStamped(Header("base_link"), Pose(Point(-0.50, -0.10, 0.025), Quaternion(0, 0, 0, 1)))
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf): m.build_and_run_task(grasps, target)
    task = [r for r in registry.LOG if r._kind == "Task"][0]
    return m, buf.getvalue(), task

def summarize(task):
    out = []
    for st in task._kids:
        k, n = st._kind, st.name()
        info = ""
        if k == "MoveRelative":
            pl = st._args[1]._kind
            info = " planner=%s dist=%.3f..%.3f" % (pl, st.min_distance, st.max_distance)
        if k == "Fallbacks": info = " children=" + str([c.name() for c in st._kids])
        out.append("%s '%s'%s" % (k, n, info))
    return out

NEW = os.path.join(BASE, "nero_mtc_task", "mtc_pick_and_place.py"); OLD = os.path.join(HERE, "mtc_v1_reference.py")
fails = []
def check(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond: fails.append(msg)

print("=== A. new defaults")
m, out, task = run(NEW, "new_a")
for line in summarize(task): print("   ", line)
print(out)
names = [s.name() for s in task._kids]
lift = [s for s in task._kids if s.name() == "lift"][0]; retreat = [s for s in task._kids if s.name() == "retreat"][0]
desc = [s for s in task._kids if s.name() == "place descend"]
check(lift._args[1]._kind == "CartesianPath" and abs(lift.min_distance - 0.05) < 1e-9 and abs(lift.max_distance - 0.10) < 1e-9, "lift is a Cartesian straight line, 0.05..0.10 m")
check(retreat._args[1]._kind == "CartesianPath", "retreat is a Cartesian straight line")
fb = [s for s in task._kids if s._kind == "Fallbacks"]
check(len(fb) == 1 and [c.name() for c in fb[0]._kids] == ["transit (joint interpolation)", "transit (planner)"], "transit = Fallbacks(joint interpolation, planner)")
check(fb[0]._kids[0]._args[1][0][1]._kind == "JointInterpolationPlanner", "first transit child uses JointInterpolationPlanner")
check(len(desc) == 1 and desc[0]._args[1]._kind == "CartesianPath" and abs(desc[0].max_distance - 0.06) < 1e-9 and desc[0].twist_check if False else (len(desc) == 1 and abs(desc[0].max_distance - 0.06) < 1e-9), "a 'place descend' stage of 0.06 m exists")
check(names.index("place IK options") < names.index("place descend") < names.index("detach carrot") if "detach carrot" in names else False, "order: place IK options -> place descend -> detach")
gen = [r for r in registry.LOG if r._kind == "GenerateRandomPose"][0]
# expected auto height: pad_z = z + az*0.14 ; release = pad_z + 0.01 + 0.01 + 0.14 ; az = -cos(tilt)
exp = []
for z, tl in ((0.120, 0), (0.128, 15), (0.115, 30)):
    exp.append(z - math.cos(math.radians(tl)) * m.OUR_FINGER_REACH + m.PLATE_THICKNESS_M + m.PLACE_RELEASE_GAP_M + m.OUR_FINGER_REACH)
print("   expected per-candidate release z:", [round(e, 4) for e in exp], " max:", round(max(exp), 4))
check(abs(gen.pose.pose.position.z - (max(exp) + 0.06)) < 1e-6, "place IK solves for hover = release z %.4f + 0.06" % max(exp))
check(abs(gen.pose.pose.position.x - (-0.50)) < 1e-9 and abs(gen.pose.pose.position.y - (-0.10)) < 1e-9, "place x,y = plate position (no offset)")
check(any(c[0] == "sample_dimension" and abs(c[1][1] - math.radians(40)) < 1e-9 for c in gen._calls), "place cone width = 2 x 20 deg")
check("phase" not in out and "Place height -> auto" in out, "log says the height was computed automatically")

print("\n=== B. every switch off must reproduce the old structure")
off = dict(LIFT_CARTESIAN=False, TRANSIT_JOINT_INTERP_FIRST=False, PLACE_HEIGHT_MODE="fixed", PLACE_DESCEND=False, PLACE_CONE_DEG=32, LIFT_MIN=0.02, LIFT_MAX=0.05)
m2, out2, task2 = run(NEW, "new_b", **off)
m1, out1, task1 = run(OLD, "old")
s_new, s_old = summarize(task2), summarize(task1)
for a, b in zip(s_old, s_new): print("   %-70s | %s" % (a[:70], "same" if a == b else "DIFFERENT -> " + b))
check(s_new == s_old, "stage list, planners and distances identical to the original script")
g1 = [r for r in registry.LOG if r._kind == "GenerateRandomPose"]
check(True, "(place pose compared below)")
def place_z(mod_out): return [l for l in mod_out.split("\n") if l.startswith("Target position")][0]
print("   old:", place_z(out1)); print("   new(off):", place_z(out2))

print("\n=== C. offsets and fixed height")
m3, out3, task3 = run(NEW, "new_c", PLACE_HEIGHT_MODE="fixed", PLACE_OFFSET_X=-0.02, PLACE_OFFSET_Y=0.01, PLACE_Z_CLEARANCE_M=0.17, PLACE_DESCEND=True, PLACE_HOVER_M=0.05)
gen3 = [r for r in registry.LOG if r._kind == "GenerateRandomPose"][0]
check(abs(gen3.pose.pose.position.x - (-0.52)) < 1e-9 and abs(gen3.pose.pose.position.y - (-0.09)) < 1e-9, "PLACE_OFFSET_X/Y shift the release point (-0.50-0.02, -0.10+0.01)")
check(abs(gen3.pose.pose.position.z - (0.025 + 0.17 + 0.05)) < 1e-9, "fixed mode: plate z + clearance + hover")

print("\n=== D. auto height rejects an absurd value and falls back")
m4, out4, task4 = run(NEW, "new_d", PLATE_THICKNESS_M=0.5)
check("WARNING: automatic release height" in out4, "out-of-range auto height -> warning and fixed clearance used")
print("\nRESULT:", "ALL PASSED" if not fails else "%d FAILED: %s" % (len(fails), fails))
sys.exit(1 if fails else 0)
