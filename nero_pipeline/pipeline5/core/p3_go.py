#!/usr/bin/env python3
"""
p3_go.py - PIPELINE 3 (GraspMolmo + Contact-GraspNet) in ONE command, with timing for every step. A copy of pipeline 2's p2_go.py that wraps
~/nero_pipeline_launch/run_pick_and_place3.sh (pipeline 1's run sequence with GraspMolmo as step 1). Needs start_pipeline3.sh running.

    conda deactivate; source ~/agx_arm_ws/install/setup.bash
    /usr/bin/python3 ~/nero_pipeline_launch/pipeline3/p3_go.py          # start pose -> GraspMolmo -> CGN -> plate -> plan -> EXECUTE (asks EXECUTE once, first)
    /usr/bin/python3 ~/nero_pipeline_launch/pipeline3/p3_go.py --plan   # the same but plan only (Ctrl+C when you have looked at it in RViz)
    (what to pick / where to place: P3_SOURCE / P3_TARGET in the terminal that runs start_pipeline3.sh)

Every line is printed with the time since the start. Each step is announced when it starts and when it is done, and a timing table is printed
at the end (also when you press Ctrl+C or a step stops). The full log and the table are saved in ~/p3_runs/go_logs/.
"""
import argparse
import os
import re
import signal
import subprocess
import sys
import time

GO_VERSION = "p3-go-v6"   # + PIPELINE 5 8b: concise terminal (Brief), P5_VERBOSE=1 = everything

# (regex on a line of the run script's output, step name). The first match of each starts that step; the previous step ends there.
# v3: the perception steps are named from p2_perceive's own banner lines ("# 2/4  <text>"), so the table fits both the worker path
# (M2T2 and the GraspMolmo questions in parallel) and the old one-shot path. A name of None means: use the banner text.
STEPS = [
    (r"=== 0: moving the arm to the start pose", "Move arm to start pose"),
    (r"^# 1/4  ", None),
    (r"^# 2/4  ", None),
    (r"^# 3/4  ", None),
    (r"asking Molmo \(", "  GraspMolmo: grasp question (one-shot path)"),
    (r"asking Molmo for the place spot", "  GraspMolmo: place question (one-shot path)"),
    (r"files in .*/graspmolmo", "Decision report + reach check"),
    (r"^# 4/4  ", "Publish poses"),
    (r"=== 2/3", "pose_transform to base_link"),
    (r"=== reusing the latest perception", "Reuse latest perception"),
    (r"=== 3/3", "Grasp + place picture"),
    (r"=== MoveIt Task Constructor", "MTC planning"),
    (r"Solution choice:", "Choose smoothest solution + retime"),
]
EXEC_STEPS = [(r"Retimed solution", "Execute on the arm")]                          # execute mode only
PLAN_STEPS = [(r"Plan found and published", "Looking at the plan (until Ctrl+C)")]  # plan mode only
DONE_RE = re.compile(r"Keeping node alive indefinitely")                           # MTC finished; it then idles
EXEC_OK_RE = re.compile(r"REAL error_code\.val = (-?\d+)")
FOUND_RE = re.compile(r"Found (\d+) solution")
DETAIL_RES = [(re.compile(r"\[M2T2\] finished in ([\d.]+) s"), "M2T2 %s s"),
              (re.compile(r"\[GraspMolmo\] grasp answered in ([\d.]+) s"), "GraspMolmo grasp answer %s s"),
              (re.compile(r"\[GraspMolmo\] place answered in ([\d.]+) s"), "place answer %s s")]
FAIL_RE = re.compile(r"Failing stage")
PLAN_FAILED_RE = re.compile(r"Planning FAILED")             # pipeline 1's MTC: no solution at all (its process still exits cleanly)
DIED_RE = re.compile(r"process has died")                    # ros2 launch: the MTC process crashed (the log filter in the pipe hides its exit code)
MIN_HOME_MOVE_S_REAL = float(os.environ.get("P3_MIN_HOME_S", "8"))   # v6: the start-pose move on the REAL arm takes at least 8 s (was 15 s; the
                                                                       # 4 s default was too fast). P3_MIN_HOME_S=15 restores the old value.


def fmt(t):
    return "%6.1f s" % t



class Brief:
    """PIPELINE 5 (8b): what the TERMINAL shows - one line per step, one short result per service (ok / FAILED), failures with their reason.
    Pure (tested offline). The log file still gets every line; P5_VERBOSE=1 prints everything as before."""
    KEEP_MTC = ("Failing stage", "min_fraction", "min_distance not reached", "GOAL_STATE_INVALID", "Collision between", "colliding with",
                "Planning FAILED", "solution(s).", "PIPELINE 5: plan saved", "the plan could NOT be saved", "Retimed solution: total motion time")
    TRIGGER = re.compile(r"Trigger_Response\(success=(True|False), message=(['\"])(.*)\2\)\s*$")

    def __init__(self):
        self.in_picture = False

    @staticmethod
    def short(msg, n=150, clauses=2):
        msg = re.sub(r"\s*\(ran alongside[^)]*\)", "", msg)
        msg = re.sub(r"^(Target pose: position=\([^)]*\)).*$", r"\1", msg)
        parts = [p.strip() for p in msg.split("; ") if p.strip()]
        out = "; ".join(parts[:clauses]) if parts else msg
        return out if len(out) <= n else out[:n - 3] + "..."

    def step(self, ann):
        """A Timer line -> the terminal line (or None)."""
        a = ann.strip()
        m = re.match(r">>> STEP (\d+): (.*?)\s+\(started at ([\d.]+ s)\)", a)
        if m:                                                    # 8c: the time since the run started, after the step number
            return "[%s] %-9s %s" % (m.group(1), m.group(3), m.group(2))
        m = re.match(r"<<< done: .* in ([\d.]+ s)", a)
        if m:
            return "    done in %s" % m.group(1)                     # 8c: how long the step took
        return a if a else None

    def line(self, text):
        """A line of the run (without its time stamp) -> the terminal line (or None)."""
        t = text.strip()
        if not t:
            return None
        if t.startswith("p3_picture"):                               # the picture's long text report: the pop-up shows it
            self.in_picture = True
            return None
        if self.in_picture:
            if t.startswith("PICTURE "):
                self.in_picture = False
            elif "STOP" in t or "FAILED" in t:
                return "    ! " + t[:160]
            return None
        m = self.TRIGGER.search(t)
        if m:
            return ("    ok  " + self.short(m.group(3))) if m.group(1) == "True" else ("    FAILED  " + self.short(m.group(3), 300, 4))
        if t.startswith(("waiting for service", "requester:", "response:", "[INFO] [launch]", "WARNING:root:", "(p3_logfilter")):
            return None
        if "[python3-1]" in t:
            body = t.split("[python3-1]", 1)[1].strip()
            if body.startswith("[") and "FAILED" not in body:          # MoveIt's own [INFO]/[WARN] lines: the log has them
                return None
            if any(k in body for k in self.KEEP_MTC) or re.search(r"\(\d+/\d+\): ", body):
                return "    " + body.split(" - no valid solution")[0][:200]
            return None
        if t.startswith(("!!!", "STOP", "NOT EXECUTED", "EXECUT", "PLAN ")) or "FAILED" in t or "Traceback" in t or "Error" in t:
            return "  " + t[:200]
        return None

class Timer:
    """Turns output lines into step start/end events. Pure (no I/O) so it can be tested."""

    def __init__(self, t0, mode="execute"):
        self.t0 = t0
        self.rules = STEPS + (EXEC_STEPS if mode == "execute" else PLAN_STEPS)
        self.exec_code = None
        self.plan_failed = False
        self.died = False
        self.steps = []            # [name, start, end]
        self.seen = set()
        self.found = []            # (time, n solutions) per successful planning attempt
        self.failed = []           # time of each failed planning attempt
        self.details = {}          # step index -> list of "M2T2 21.3 s" style notes (what ran inside a parallel step)

    def feed(self, line, now):
        """Returns a list of announcement strings for this line."""
        out = []
        m = EXEC_OK_RE.search(line)
        if m:
            self.exec_code = int(m.group(1))
        text = line.strip()
        for i, (rx, name) in enumerate(self.rules):
            if i not in self.seen and re.search(rx, text):
                self.seen.add(i)
                out += self._close(now)
                if name is None:                               # a perception banner: "# 2/4  M2T2 grasps + GraspMolmo questions IN PARALLEL ..."
                    name = re.sub(r"\s*\(.*$", "", re.sub(r"^# \d/4\s+", "", text))[:52]
                self.steps.append([name, now, None])
                out.append(">>> STEP %d: %s  (started at %s)" % (len(self.steps), name, fmt(now - self.t0).strip()))
        for rx, fmt_ in DETAIL_RES:
            d = rx.search(line)
            if d and self.steps:
                self.details.setdefault(len(self.steps) - 1, []).append(fmt_ % d.group(1))
        m = FOUND_RE.search(line)
        if m:
            self.found.append((now, int(m.group(1))))
            out.append("    planning attempt %d: FOUND %s solutions after %s of planning" % (
                len(self.found) + len(self.failed), m.group(1), fmt(now - self._mtc_start(now)).strip()))
        elif FAIL_RE.search(line):
            self.failed.append(now)
            out.append("    planning attempt %d: failed (MoveIt retries)" % (len(self.found) + len(self.failed)))
        return out

    def _mtc_start(self, now):
        for name, s, _ in self.steps:
            if name == "MTC planning":
                return s
        return now

    def _close(self, now):
        if self.steps and self.steps[-1][2] is None:
            self.steps[-1][2] = now
            return ["<<< done: %s in %s" % (self.steps[-1][0], fmt(now - self.steps[-1][1]).strip())]
        return []

    def finish(self, now, executed):
        """Close the last step. If MTC found a plan and execution was requested, split MTC into planning and execution."""
        out = []
        if self.steps and self.steps[-1][0] == "MTC planning" and self.steps[-1][2] is None and self.found:
            t_found = self.found[-1][0]
            self.steps[-1][2] = t_found
            out.append("<<< done: MTC planning in %s" % fmt(t_found - self.steps[-1][1]).strip())
            self.steps.append(["Execute on the arm (incl. MTC shutdown)" if executed else "Looking at the plan (until Ctrl+C)", t_found, now])
        out += self._close(now)
        return out

    def table(self, now):
        L = ["=" * 78, "TIMING  (%s)" % GO_VERSION, "=" * 78]
        for k, (name, s, e) in enumerate(self.steps, 1):
            extra = ""
            if name == "MTC planning":
                n = len(self.found) + len(self.failed)
                if n:
                    extra = "  (%d attempt(s): %d failed%s)" % (n, len(self.failed), ", last found %d solutions" % self.found[-1][1] if self.found else "")
            L.append(" %2d  %-52s %s%s" % (k, name, fmt((e if e is not None else now) - s), extra))
            if self.details.get(k - 1):
                L.append("       (inside: %s)" % ", ".join(self.details[k - 1]))
        L.append("     %-52s %s" % ("TOTAL", fmt(now - self.t0)))
        L.append("=" * 78)
        return L


NODE = "/p3_graspmolmo_node"
RUNS_LATEST = os.path.expanduser("~/p3_runs/latest")


def ros2(*args, timeout=25):
    try:
        r = subprocess.run(["ros2"] + list(args), capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout + r.stderr
    except (OSError, subprocess.TimeoutExpired) as e:
        return 1, str(e)


_REAL = {}


def real_arm():
    """v5: the REAL arm is detected from ROS itself (its driver publishes /feedback/joint_states; the simulation does not), not only from what is
    exported in this terminal - an unset variable must never make the real arm look simulated."""
    if "v" not in _REAL:
        env_says = os.environ.get("REAL_ARM", "0") == "1" or os.environ.get("JOINT_STATES_TOPIC", "") == "/feedback/joint_states"
        rc, out = ros2("topic", "list")
        _REAL["v"] = env_says or (rc == 0 and "/feedback/joint_states" in out.split())
    return _REAL["v"]


def set_and_show_prompts(task, target):
    """Set the GraspMolmo node's prompts (if given) and read them back. Returns (source, target) or raises SystemExit with a clear message."""
    import json as _json
    for name, val in (("source_prompt", task), ("target_prompt", target)):
        if val:
            rc, out = ros2("param", "set", NODE, name, _json.dumps(val))          # quoted: always a string, even "yes" or "1"
            if rc != 0 or "Set parameter successful" not in out:
                raise SystemExit("STOP: could not set %s on %s: %s\n(is start_pipeline3.sh running? the node is %s)" % (name, NODE, out.strip()[:200], NODE))
    got = []
    for name in ("source_prompt", "target_prompt"):
        rc, out = ros2("param", "get", NODE, name)
        m = re.search(r"String value is:\s*(.*)", out)
        if rc != 0 or not m:
            raise SystemExit("STOP: the GraspMolmo node %s does not answer (%s). Is start_pipeline3.sh running?" % (NODE, out.strip()[:160]))
        got.append(m.group(1).strip())
    return got[0], got[1]


def latest_perception():
    import json as _json
    try:
        a = _json.load(open(os.path.join(RUNS_LATEST, "answers.json")))
        return "pick \"%s\", place on \"%s\" (captured %s)" % (a.get("source_prompt"), a.get("target_prompt"), a.get("time"))
    except (OSError, ValueError):
        return "(no saved perception found in ~/p3_runs/latest)"


def main(argv=None):
    ap = argparse.ArgumentParser(description="Pipeline 2 in one command, with timing.")
    ap.add_argument("--plan", action="store_true", help="plan only, do not execute (Ctrl+C after looking at it in RViz)")
    ap.add_argument("--last", action="store_true", help="re-plan with the grasps and place spot already in pose_transform (a cheap retry after 'no IK found')")
    ap.add_argument("--task", help='what to pick, e.g. --task "carrot" (sets the GraspMolmo node\'s prompt for this and later runs)')
    ap.add_argument("--target", help='where to place it, e.g. --target "plate"')
    ap.add_argument("--run_script", default=os.path.expanduser("~/nero_pipeline_launch/pipeline5/p5_run.sh"))   # PIPELINE 5
    a = ap.parse_args(argv)
    mode = "plan" if a.plan else "execute"
    if not os.path.exists(a.run_script):
        print("STOP: %s not found (install pipeline 2 first)" % a.run_script)
        return 1

    if a.last and (a.task or a.target):
        print("STOP: --task/--target need a NEW perception (a new picture of the table). Leave out --last.")
        return 2
    if a.last:
        what = "reusing the latest perception: " + latest_perception()
    else:
        try:
            src, tgt = set_and_show_prompts(a.task, a.target)
        except SystemExit as e:
            print(e)
            return 1
        what = "NEW perception: pick \"%s\", place on \"%s\"" % (src, tgt)
    print("%s | %s | %s arm" % (mode.upper(), what, "REAL" if real_arm() else "simulated"), flush=True)
    env_extra = {}
    if real_arm() and not os.environ.get("JOINT_STATES_TOPIC"):
        env_extra["JOINT_STATES_TOPIC"] = "/feedback/joint_states"          # the start-pose move must listen to the REAL arm
    if real_arm():
        try:
            hm = float(os.environ.get("HOME_MOVE_SECONDS", "0"))
        except ValueError:
            hm = 0.0
        if hm < MIN_HOME_MOVE_S_REAL and os.environ.get("P3_ALLOW_FAST_HOME", "0") != "1":
            env_extra["HOME_MOVE_SECONDS"] = "%g" % MIN_HOME_MOVE_S_REAL
            if os.environ.get("P5_VERBOSE", "0") == "1": print("REAL arm: the start-pose move will take %g s (HOME_MOVE_SECONDS was %s; never less than %g s on the real arm)." % (
                MIN_HOME_MOVE_S_REAL, os.environ.get("HOME_MOVE_SECONDS", "not set"), MIN_HOME_MOVE_S_REAL))
        else:
            print("REAL arm: the start-pose move takes %g s (HOME_MOVE_SECONDS)." % hm)
    if mode == "execute" and real_arm():
        print("!!! This will move the REAL arm: start pose, then (after perception, the checks and planning) the pick and place.")
        print("!!! Have STOP typed in your STOP terminal, hand ready, workspace clear.")
        if input("Type EXECUTE to continue: ").strip() != "EXECUTE":
            print("not confirmed: nothing was run")
            return 1

    env = dict(os.environ, P2_YES="1", PYTHONUNBUFFERED="1", **env_extra)
    cmd = ["stdbuf", "-oL", "-eL", "bash", a.run_script, mode] + (["--last"] if a.last else [])
    logdir = os.path.expanduser("~/p3_runs/go_logs")
    os.makedirs(logdir, exist_ok=True)
    logpath = os.path.join(logdir, "run_%s_%s.txt" % (time.strftime("%Y%m%d_%H%M%S"), mode))
    log = open(logpath, "w")

    t0 = time.time()
    tm = Timer(t0, mode)

    verbose = os.environ.get("P5_VERBOSE", "0") == "1"
    brief = Brief()

    def _say_both(s):
        say(s, s)

    def say(s, shown=None):
        """PIPELINE 5 (8b): the log gets s; the terminal gets s (P5_VERBOSE=1) or its short form."""
        log.write(s + "\n")
        log.flush()
        if verbose:
            print(s, flush=True)
        elif shown:
            print(shown, flush=True)

    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True, bufsize=1, env=env,
                         start_new_session=True)
    interrupted = False
    try:
        for line in p.stdout:
            now = time.time()
            for ann in tm.feed(line, now):
                say(ann, brief.step(ann))
            say("[%s] %s" % (fmt(now - t0), line.rstrip("\n")), brief.line(line))
            if PLAN_FAILED_RE.search(line):
                tm.plan_failed = True
            if DIED_RE.search(line):
                tm.died = True
            if mode == "plan" and FOUND_RE.search(line):
                _say_both("PLAN FOUND (saved)")
            if mode == "execute" and DONE_RE.search(line):
                # MTC has finished executing and now idles for the RViz panel: close it ourselves so the run ends
                _say_both("[%s] p2_go: execution finished (MoveIt error code %s = %s); closing the MTC node" % (
                    fmt(time.time() - t0), tm.exec_code, "SUCCESS" if tm.exec_code == 1 else "NOT success"))
                try:
                    os.killpg(p.pid, signal.SIGINT)
                    p.wait(timeout=15)
                except Exception:
                    os.killpg(p.pid, signal.SIGKILL)
                    p.wait()
                break
        rc = p.wait()
        if mode == "execute" and tm.exec_code is not None:
            rc = 0 if tm.exec_code == 1 else 1
    except KeyboardInterrupt:
        interrupted = True
        _say_both("[%s] Ctrl+C: stopping the run (the planner / MTC node is closed)" % fmt(time.time() - t0))
        try:
            os.killpg(p.pid, signal.SIGINT)
            p.wait(timeout=15)
        except Exception:
            os.killpg(p.pid, signal.SIGKILL)
        rc = p.returncode

    now = time.time()
    for ann in tm.finish(now, executed=(mode == "execute" and not interrupted and rc == 0)):
        _say_both(ann)
    for line in tm.table(now):
        _say_both(line)
    if tm.died:
        rc = 1
        _say_both("MTC CRASHED (its process died): read the lines just above 'process has died' for the Python error. The arm stopped where it was.")
    elif tm.plan_failed:
        rc = 1
        _say_both("PLANNING FAILED - nothing moved.")
        _say_both("Retry with the same look:  P5_GO=1 bash ~/nero_pipeline_launch/pipeline5/p5_replan.sh")
    elif interrupted:
        _say_both("ended with Ctrl+C")
    elif rc != 0:
        _say_both("STOPPED (exit code %s) at the last step in the table - reason above" % rc)
    else:
        _say_both("finished OK")
    _say_both("log: %s" % logpath)
    log.close()
    return 0 if (rc == 0 or interrupted) else 1


if __name__ == "__main__":
    sys.exit(main())
