#!/usr/bin/env python3
"""p5_exec_plan.py - PIPELINE 5: execute the plan p5_plan.sh / p5_go.sh saved (~/p3_runs/p5_plan_latest.bin), WITHOUT planning again.

Refuses (nothing moves) when:
  * there is no saved plan, it is older than P5_PLAN_MAX_AGE_S (default 900 s), or it is not from the latest look (p5_grasp_latest.json),
  * MTC's joint-travel check refused it at planning time (a big single-joint swing),
  * the arm is not where the plan starts (any arm joint off by more than P5_START_TOL_RAD, default 0.03 rad): it moved since planning.
Runs it like pipeline 3's MTC does: the action execute_task_solution if MoveIt offers it, else one segment at a time through /execute_trajectory
(P2_EXECUTE_VIA=auto|task|trajectory; the functions are taken from core/p3_mtc.py itself, so there is one tested version of them).
The saved plan is used ONCE: it is renamed after an execution attempt, so running this again cannot replay it from a new arm position.
"""
import ast
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
P5_EXEC_VERSION = "p5-exec-plan-v2"   # v2: reads BOTH joint-state topics (best-effort QoS), prefers the real arm's feedback
# v1: picked the topic before discovery -> /joint_states, and missed best-effort publishers
RUNS = os.path.expanduser("~/p3_runs")
PLAN, META, LOOK = (os.path.join(RUNS, f) for f in ("p5_plan_latest.bin", "p5_plan_latest.json", "p5_grasp_latest.json"))


def mtc_functions(names=("execute_via_trajectories", "choose_execute_route")):
    """The execution helpers from core/p3_mtc.py (parsed, not imported: importing it would load MoveIt Task Constructor)."""
    src = open(os.path.join(HERE, "core", "p3_mtc.py")).read()
    fns = [n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef) and n.name in names]
    ns = {}
    exec(compile(ast.Module(body=fns, type_ignores=[]), "p3_mtc.py", "exec"), ns)
    return ns


def check_plan(meta, look, now, max_age_s):
    """-> None if the saved plan may run, else why not (pure, tested offline)."""
    if meta is None:
        return "no saved plan - run p5_plan.sh (or p5_go.sh) first"
    if now - meta.get("unix_time", 0) > max_age_s:
        return "the saved plan is %.0f min old - plan again" % ((now - meta["unix_time"]) / 60)
    if look is None or meta.get("look_unix_time") is None or abs(look - meta["look_unix_time"]) > 1e-3:
        return "the saved plan is not from the latest look - plan again"
    if meta.get("refusal"):
        return "MTC refused this plan for execution: %s" % meta["refusal"]
    return None


def check_start(start_joints, current, tol):
    """-> None if every arm joint of the plan's start is within tol of the arm now, else why not (pure, tested offline)."""
    arm = {k: v for k, v in start_joints.items() if k.startswith("joint")}
    if not arm:
        return "the saved plan has no arm start state"
    missing = [k for k in arm if k not in current]
    if missing:
        return "no current value for %s (is the arm running?)" % ", ".join(missing)
    worst = max(arm, key=lambda k: abs(arm[k] - current[k]))
    d = abs(arm[worst] - current[worst])
    if d > tol:
        return "the arm is not where the plan starts (%s off by %.3f rad > %.3f): it moved since planning - plan again" % (worst, d, tol)
    return None


def current_joints(feedback, plain):
    """The arm's joints now: the real arm's /feedback/joint_states when it has the arm joints, else /joint_states (pure, tested offline).
    -> (values, source)."""
    if any(k.startswith("joint") for k in feedback):
        return dict(feedback), "/feedback/joint_states"
    if any(k.startswith("joint") for k in plain):
        return dict(plain), "/joint_states"
    return {}, "no joint states received"


def main():
    pass
    meta = look = None
    try:
        meta = json.load(open(META))
    except Exception:
        pass
    try:
        look = json.load(open(LOOK))["unix_time"]
    except Exception:
        pass
    why = check_plan(meta, look, time.time(), float(os.environ.get("P5_PLAN_MAX_AGE_S", "900")))
    if why is None and not os.path.exists(PLAN):
        why = "the saved plan file is missing - plan again"
    if why:
        print("NOT EXECUTED: " + why, flush=True)
        return 1

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_task_constructor_msgs.action import ExecuteTaskSolution
    from moveit_task_constructor_msgs.msg import Solution

    sol = deserialize_message(open(PLAN, "rb").read(), Solution)
    rclpy.init()
    node = rclpy.create_node("p5_exec_plan")
    fb, js = {}, {}
    # both topics, with the sensor QoS (receives from best-effort AND reliable publishers)
    node.create_subscription(JointState, "/feedback/joint_states", lambda m: fb.update(zip(m.name, m.position)), qos_profile_sensor_data)
    node.create_subscription(JointState, "/joint_states", lambda m: js.update(zip(m.name, m.position)), qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if any(k.startswith("joint") for k in fb) or (time.time() - t0 > 3.0 and any(k.startswith("joint") for k in js)):
            break
    current, topic = current_joints(fb, js)
    why = check_start(meta["start_joints"], current, float(os.environ.get("P5_START_TOL_RAD", "0.03")))
    if why:
        print("NOT EXECUTED: " + why, flush=True)
        node.destroy_node()
        return 1
    os.replace(PLAN, PLAN + ".used")                          # used once: never replayed from another arm position
    print("EXECUTING (hand on STOP)", flush=True)
    F = mtc_functions()
    via = os.environ.get("P2_EXECUTE_VIA", "auto").strip().lower()
    task_up = False
    ac = None
    if via in ("auto", "task"):
        ac = ActionClient(node, ExecuteTaskSolution, "execute_task_solution")
        task_up = bool(ac.wait_for_server(timeout_sec=3.0))
    route, route_msg = F["choose_execute_route"](via, task_up)
    if route_msg:
        print(route_msg, flush=True)
    code = None
    if route == "task":
        goal = ExecuteTaskSolution.Goal()
        goal.solution = sol
        fut = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        gh = fut.result() if fut.done() else None
        if not gh or not gh.accepted:
            print("Goal REJECTED by execute_task_solution (or no answer within 30 s). Nothing was executed.", flush=True)
        else:
            rf = gh.get_result_async()
            rclpy.spin_until_future_complete(node, rf)
            code = rf.result().result.error_code.val
            print("execute_task_solution: error_code.val = %s" % code, flush=True)
    elif route == "trajectory":
        code = F["execute_via_trajectories"](node, rclpy, ActionClient, ExecuteTrajectory, sol, log=lambda m: print(m, flush=True))
    node.destroy_node()
    ok = code == 1
    print("EXECUTED: every segment succeeded." if ok else "EXECUTION DID NOT COMPLETE (code %s) - see above; STOP freezes the arm." % code, flush=True)
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
