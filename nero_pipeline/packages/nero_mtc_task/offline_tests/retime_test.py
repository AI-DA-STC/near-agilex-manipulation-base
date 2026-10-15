import os
HERE = os.path.dirname(os.path.abspath(__file__)); BASE = os.path.dirname(HERE)
import sys, io, contextlib, importlib.util
sys.path.insert(0, os.path.join(HERE, "stubs"))
spec = importlib.util.spec_from_file_location("m", os.path.join(BASE, "nero_mtc_task", "mtc_pick_and_place.py")); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
from types import SimpleNamespace as NS
ARM = ["joint%d" % i for i in range(1, 8)]
def seg(names, n, step):                       # n points, every joint moves `step` rad per point
    pts = [NS(positions=[i * step] * len(names), time_from_start=NS(sec=0, nanosec=0), velocities=[], accelerations=[]) for i in range(n)]
    return NS(trajectory=NS(joint_trajectory=NS(joint_names=names, points=pts)))
def solution():   # connect, open, approach, close, lift, transit, descend, release, retreat   (+ an empty IK stage entry)
    return NS(sub_trajectory=[seg(ARM, 20, 0.02), seg(["gripper"], 2, 0.05), seg(ARM, 10, 0.01), seg(["gripper"], 2, 0.05),
                              seg(ARM, 8, 0.01), seg(ARM, 30, 0.02), NS(trajectory=NS(joint_trajectory=NS(joint_names=[], points=[]))),
                              seg(ARM, 6, 0.01), seg(["gripper"], 2, 0.05), seg(ARM, 8, 0.01)])
def times(sol):
    return [round(s.trajectory.joint_trajectory.points[-1].time_from_start.sec + s.trajectory.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9, 2)
            if s.trajectory.joint_trajectory.points else None for s in sol.sub_trajectory]
phases = ["connect", "approach", "lift", "transit", "descend", "retreat"]
buf = io.StringIO()
a = solution(); b = solution(); c = solution(); d = solution()
with contextlib.redirect_stdout(buf):
    m.retime_solution(a)                                                  # old call signature must still work
    m.retime_solution(b, phases)                                          # all factors 1.0 -> identical to the old result
    m.PHASE_SPEED["transit"] = 3.0; m.PHASE_SPEED["connect"] = 2.0
    m.retime_solution(c, phases)
    m.retime_solution(d, phases[:-1])                                     # wrong number of phase names -> must not scale
ta, tb, tc, td = times(a), times(b), times(c), times(d)
print("segment end times (s)"); print(" old call      ", ta); print(" phases, x1.0  ", tb); print(" transit x3, connect x2", tc); print(" wrong name count      ", td)
print("message when the count does not match:", [l for l in buf.getvalue().split("\n") if "PHASE_SPEED not applied" in l])
ok = True
def check(cond, msg):
    global ok; print(("  PASS  " if cond else "  FAIL  ") + msg); ok = ok and cond
check(ta == tb, "phase names with all factors 1.0 give exactly the old timing")
check(tc[5] < ta[5] * 0.45 and tc[0] < ta[0] * 0.65, "transit (x3) and connect (x2) became much shorter")
check(tc[2] == ta[2] and tc[4] == ta[4] and tc[9] == ta[9], "approach, lift and retreat unchanged")
check(tc[1] == ta[1] and tc[3] == ta[3] and tc[8] == ta[8], "gripper segments unchanged")
check(td == ta and any("PHASE_SPEED not applied" in l for l in buf.getvalue().split("\n")), "wrong number of arm segments -> speeds untouched and a message is printed")
# strictly increasing timestamps everywhere
bad = 0
for s in c.sub_trajectory:
    t = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in s.trajectory.joint_trajectory.points]
    bad += sum(1 for x, y in zip(t, t[1:]) if y <= x)
check(bad == 0, "all timestamps strictly increasing (the controller requires it)")
sys.exit(0 if ok else 1)
