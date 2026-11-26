#!/usr/bin/env python3
"""Offline tests for step 6 (run flow + live view): saved-plan checks, MTC's executor functions, tidy, the live window's log parsing and drawing.
Run: /usr/bin/python3 ~/nero_pipeline_launch/pipeline5/tests/test_p5_run.py"""
import os, sys, tempfile, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); P5 = os.path.join(HERE, "..")
sys.path[:0] = [P5, os.path.join(P5, "core")]
import p5_exec_plan as X
import p5_tidy
import p5_view
results = []


def check(name, ok, info=""):
    results.append(bool(ok)); print("%-5s %s %s" % ("PASS" if ok else "FAIL", name, info))


now = time.time()
meta = {"unix_time": now - 60, "look_unix_time": 1000.0, "refusal": None, "start_joints": {"joint%d" % i: 0.1 * i for i in range(1, 8)}}
check("plan check: fresh plan of the latest look -> may run", X.check_plan(meta, 1000.0, now, 900) is None)
check("plan check: no plan -> refused", "no saved plan" in X.check_plan(None, 1000.0, now, 900))
check("plan check: old plan -> refused", "min old" in X.check_plan(dict(meta, unix_time=now - 2000), 1000.0, now, 900))
check("plan check: plan of an earlier look -> refused", "latest look" in X.check_plan(meta, 2000.0, now, 900))
check("plan check: MTC refused it -> refused", "refused" in X.check_plan(dict(meta, refusal="joint7 swings 3.2 rad"), 1000.0, now, 900))
cur = dict(meta["start_joints"])
check("start check: arm where the plan starts -> may run", X.check_start(meta["start_joints"], cur, 0.03) is None)
cur2 = dict(cur, joint4=cur["joint4"] + 0.2)
check("start check: arm moved since planning -> refused", "moved since planning" in X.check_start(meta["start_joints"], cur2, 0.03))
check("start check: no joint states -> refused", "is the arm running" in X.check_start(meta["start_joints"], {}, 0.03))
v, src = X.current_joints({"joint1": 0.5}, {"joint1": 0.1})
check("joints now: the real arm's feedback preferred", v["joint1"] == 0.5 and src == "/feedback/joint_states")
v, src = X.current_joints({}, {"joint1": 0.1})
check("joints now: /joint_states when there is no feedback (simulation)", v["joint1"] == 0.1 and src == "/joint_states")
check("joints now: nothing received -> empty (refused later)", X.current_joints({}, {"gripper": 0.0})[0] == {})
F = X.mtc_functions()
check("MTC's own executor functions are found in core/p3_mtc.py", callable(F.get("execute_via_trajectories")) and callable(F.get("choose_execute_route")))
route, _ = F["choose_execute_route"]("auto", False)
check("  ... no execute_task_solution -> one segment at a time (/execute_trajectory)", route == "trajectory")

# tidy: newest kept
home = tempfile.mkdtemp(); r = os.path.join(home, "p3_runs")
for i in range(15):
    d = os.path.join(r, "202611%02d_120000" % (i + 1)); os.makedirs(d); os.utime(d, (now - 1000 + i, now - 1000 + i))
os.makedirs(os.path.join(r, "go_logs"))
for i in range(35):
    f = os.path.join(r, "go_logs", "run_%02d.txt" % i); open(f, "w").write("x"); os.utime(f, (now - 1000 + i, now - 1000 + i))
os.makedirs(os.path.join(r, "keep_me"))
p5_tidy.main(home)
left = sorted(d for d in os.listdir(r) if d.startswith("2026"))
check("tidy: the 10 newest run folders kept, others removed", len(left) == 10 and left[-1] == "20261115_120000", left[:2])
check("tidy: 30 newest logs kept; other folders untouched", len(os.listdir(os.path.join(r, "go_logs"))) == 30 and os.path.isdir(os.path.join(r, "keep_me")))

# the live window: parse a real log (your format) and draw it
log = """p5_plan p5-plan-v2 (P5_WRIST=on)
>>> STEP 1: Move arm to start pose  (started at 0.0 s)
<<< done: Move arm to start pose in 3.0 s
>>> STEP 2: GraspMolmo: grasp point and object region  (started at 3.0 s)
<<< done: GraspMolmo: grasp point and object region in 28.3 s
>>> STEP 3: pipeline 5 grasp step: grasps from GraspMolmo's point  (started at 31.3 s)
[  33.0 s] response: success=True, message='3 grasp(s) to MoveIt; G1 jaw 120 deg ...'
<<< done: pipeline 5 grasp step: grasps from GraspMolmo's point in 4.0 s
>>> STEP 4: place point and obstacles  (started at 35.3 s)
[  50.0 s] response: success=True, message='moved 46 mm from GraspMolmo's point to where the gripper fits; release jaw 120 deg fits'
>>> STEP 7: MTC planning  (started at 70.0 s)
[  90.4 s] [python3-1] place IK cone (orig) (0/50):  eef in collision: gripper_base - p5_wrist_76
[  90.4 s] [python3-1] Planning FAILED - no valid solution found across all tried candidates/orientations.
""".splitlines()
steps, notes, fin = p5_view.parse(log)
check("live window: steps with their times", [s[0][:10] for s in steps] == ["Move arm t", "GraspMolmo", "pipeline 5", "place poin", "MTC planni"]
      and steps[0][1] == "3.0 s" and steps[1][2] == "done", steps)
check("live window: a failing step is red, with the reason in the notes", steps[-1][2] == "fail" and any("Planning FAILED" in t and bad for t, bad in notes))
check("live window: good news is kept too (grasps, moved place point)", any("grasp(s) to MoveIt" in t for t, _ in notes) and any("moved" in t for t, _ in notes))
try:
    import cv2  # noqa: F401
    img = p5_view.compose(np.zeros((500, 640, 3), np.uint8), "run_x_plan.txt", steps, notes, fin)
    check("live window: drawn (1180 x 620)", img.shape == (620, 1180, 3) and img.std() > 1)
    import p5_grasp_node as N
    import p5_gripper
    K = np.array([[489.0, 0, 320.0], [0, 489.0, 240.0], [0, 0, 1.0]]); R = np.array([[0.0, -1, 0], [-1, 0, 0], [0, 0, -1]]); t = np.array([-0.35, 0.05, 0.85])
    sel = {"rank": 1, "tcp": np.array([-0.35, 0.05, 0.15]), "R": np.array([[1.0, 0, 0], [0, -1, 0], [0, 0, -1]]), "open": 0.06, "own_open": 0.06}
    res = {"selected": [sel], "candidates": [sel]}
    pic = N.render_view(np.zeros((480, 640, 3), np.uint8), K, R, t, dict(p5_gripper.DEFAULTS), None, res, [-0.35, 0.05, 0.0], ["GRASP: test"],
                        {"uv0": (100.0, 100.0), "uv": (150.0, 120.0)})
    green = ((pic[:, :, 1] > 200) & (pic[:, :, 0] < 80) & (pic[:, :, 2] < 80)).sum()
    magenta = ((pic[:, :, 0] > 200) & (pic[:, :, 2] > 200) & (pic[:, :, 1] < 60)).sum()
    check("scene picture: G1's fingers (green) and the place point (magenta) drawn", green > 50 and magenta > 20, "green %d, magenta %d px" % (green, magenta))
    d = tempfile.mkdtemp(); N.write_view(pic, view_dir=d)
    check("scene picture written atomically", os.listdir(d) == ["view.png"])
except ImportError:
    check("live window drawing (needs cv2)", False, "cv2 missing")
# view1: OpenCV without window support -> the image viewer shows the live picture instead
import cv2 as _real_cv2


class _NoWindowCV:
    def __getattr__(self, k):
        return getattr(_real_cv2, k)

    def namedWindow(self, *a):
        raise _real_cv2.error("The function is not implemented. Rebuild the library with GTK+ support")


_started = []
_keep = (p5_view.start_viewer, p5_view.VIEW_DIR, sys.modules.get("cv2"), os.environ.get("DISPLAY"))
p5_view.start_viewer = lambda path: _started.append(path) or type("P", (), {"terminate": lambda self: None})()
p5_view.VIEW_DIR = tempfile.mkdtemp()
sys.modules["cv2"], os.environ["DISPLAY"] = _NoWindowCV(), ":0"
rc = p5_view.main(["--test"])
p5_view.start_viewer, p5_view.VIEW_DIR = _keep[0], _keep[1]
sys.modules["cv2"] = _keep[2]
check("live view: no OpenCV window support -> falls back to the image viewer", rc == 0 and len(_started) == 1 and _started[0].endswith("dashboard.png"))
# 7g: the release opens only to the object's width + 1 cm (never more than the grasp opening)
import ast as _ast, json as _json
_src = open(os.path.join(P5, "core", "p3_mtc.py")).read()
_fn = [x for x in _ast.parse(_src).body if isinstance(x, _ast.FunctionDef) and x.name == "_p5_release_width"][0]
_ns = {}
exec(compile(_ast.Module(body=[_fn], type_ignores=[]), "p3_mtc", "exec"), _ns)
_home = tempfile.mkdtemp(); os.makedirs(os.path.join(_home, "p3_runs"))
_old_home = os.environ.get("HOME"); os.environ["HOME"] = _home
_json.dump({"grasps": [{"width_m": 0.020}, {"width_m": 0.023}]}, open(os.path.join(_home, "p3_runs", "p5_grasp_latest.json"), "w"))
w, why = _ns["_p5_release_width"](0.043)
check("release: object 23 mm + 10 mm = 33 mm (was the full 43 mm grasp opening)", abs(w - 0.033) < 1e-9, why)
w, why = _ns["_p5_release_width"](0.030)
check("release: never wider than the grasp opening", abs(w - 0.030) < 1e-9)
os.remove(os.path.join(_home, "p3_runs", "p5_grasp_latest.json"))
w, why = _ns["_p5_release_width"](0.043)
check("release: object width unknown -> the grasp opening stays", w is None)
if _old_home:
    os.environ["HOME"] = _old_home

# 7g: no flicker - with nothing changing, the window is drawn once, not every 0.4 s
class _CountCV:
    WINDOW_NORMAL, WND_PROP_VISIBLE = 0, 0
    error = _real_cv2.error
    def __init__(self):
        self.shows, self.waits = 0, 0
    def __getattr__(self, k):
        return getattr(_real_cv2, k)
    def namedWindow(self, *a): pass
    def resizeWindow(self, *a): pass
    def imshow(self, *a): self.shows += 1
    def getWindowProperty(self, *a): return 1
    def waitKey(self, *a):
        self.waits += 1
        if self.waits > 12:
            raise KeyboardInterrupt
        return -1
_cv = _CountCV()
_keep2 = (sys.modules.get("cv2"), p5_view.VIEW_DIR, p5_view.LOGS, os.environ.get("DISPLAY"))
sys.modules["cv2"], p5_view.VIEW_DIR, p5_view.LOGS, os.environ["DISPLAY"] = _cv, tempfile.mkdtemp(), tempfile.mkdtemp(), ":0"
try:
    p5_view.main([])
except KeyboardInterrupt:
    pass
sys.modules["cv2"], p5_view.VIEW_DIR, p5_view.LOGS = _keep2[0], _keep2[1], _keep2[2]
check("live window: nothing changing for 12 refreshes -> drawn once (no flicker)", _cv.shows <= 2, "%d draws in %d refreshes" % (_cv.shows, _cv.waits))
# 7h: a log that keeps growing (MTC's noise lines) must NOT redraw the window
class _GrowCV(_CountCV):
    def waitKey(self, *a):
        self.waits += 1
        with open(os.path.join(p5_view.LOGS, "run_x_plan.txt"), "a") as f:
            f.write("[ 1.0 s] [python3-1] [WARN] noise line %d\n" % self.waits)
        if self.waits > 15:
            raise KeyboardInterrupt
        return -1
_cv2 = _GrowCV()
_keep3 = (sys.modules.get("cv2"), p5_view.VIEW_DIR, p5_view.LOGS)
sys.modules["cv2"], p5_view.VIEW_DIR, p5_view.LOGS = _cv2, tempfile.mkdtemp(), tempfile.mkdtemp()
open(os.path.join(p5_view.LOGS, "run_x_plan.txt"), "w").write(">>> STEP 1: Move arm to start pose  (started at 0.0 s)\n")
try:
    p5_view.main([])
except KeyboardInterrupt:
    pass
sys.modules["cv2"], p5_view.VIEW_DIR, p5_view.LOGS = _keep3
check("live window: 15 new noise lines in the log -> no redraws beyond the first picture", _cv2.shows <= 2, "%d draws" % _cv2.shows)
# 7i: the lowering into the container may stop early (>= P5_DESCEND_MIN_M, default 1 cm) and release there
_m = open(os.path.join(P5, "core", "p3_mtc.py")).read()
check("place: the lowering accepts stopping after 1 cm (then the object is let go there)",
      'descend.min_distance = min(hover, max(0.0, float(_os.environ.get("P5_DESCEND_MIN_M", "0.01"))))' in _m and "descend.max_distance = hover" in _m)
# 7j: GraspMolmo's answer as a picture for the live window, and no stop on a too-small own region
_gsrc = open(os.path.join(P5, "core", "p3_graspmolmo_node.py")).read()
_gfns = [x for x in _ast.parse(_gsrc).body if isinstance(x, _ast.FunctionDef) and x.name in ("p5_view_picture", "p5_disc")]
_gns = {}
exec(compile(_ast.Module(body=_gfns, type_ignores=[]), "p3_graspmolmo_node", "exec"), _gns)
_vd = tempfile.mkdtemp()
_img = np.full((480, 640, 3), 90, np.uint8)
_path = _gns["p5_view_picture"](_img, ["GraspMolmo: pick point for 'eggplant'", "WARNING: its own region: too small (20 px)"], None, (147, 218), _vd)
_pic = _real_cv2.imread(_path) if _path else None
_cyan = 0 if _pic is None else int(((_pic[:, :, 0] > 200) & (_pic[:, :, 1] > 200) & (_pic[:, :, 2] < 60)).sum())
check("picture A: GraspMolmo's pick point drawn for the live window, with its text", _pic is not None and _pic.shape[0] > 480 and _cyan > 20, "%s, %d cyan px" % (_path, _cyan))
_d = _gns["p5_disc"]((480, 640), (147, 218))
check("too-small own region -> a small patch around the point (the run goes on, SAM2 decides)", 150 < (_d > 0).sum() < 260 and _d[218, 147] == 255)
check("the node no longer returns a failure for a too-small own region", "omask = p5_disc(depth_m.shape, guv)" in _gsrc
      and 'return False, f"grasp point ({guv[0]:.0f}, {guv[1]:.0f}): {omsg}"' not in _gsrc)
# 8a/8c: one picture window; later looks replace the picture inside it (it stays where it was dragged); a closed window is reopened
import p5_grasp_node as _N
_opened = []


class _FakeProc:
    def __init__(self, path):
        _opened.append(path)
        self.closed = False

    def poll(self):
        return 0 if self.closed else None


_vd8 = tempfile.mkdtemp(); _src8 = os.path.join(_vd8, "view.png")
_real_cv2.imwrite(_src8, np.zeros((10, 10, 3), np.uint8))
_st8 = {}
_old_disp = os.environ.get("DISPLAY"); os.environ["DISPLAY"] = ":0"
_a = _N.popup(_src8, _st8, _vd8, opener=_FakeProc)
_real_cv2.imwrite(_src8, np.full((10, 10, 3), 255, np.uint8))
_b = _N.popup(_src8, _st8, _vd8, opener=_FakeProc)
check("pop-up: the next look replaces the picture in the SAME window (no new window: it stays where it was dragged)",
      _opened == [_a] and _a == _b and _real_cv2.imread(_b).mean() > 250)
_st8["proc"].closed = True
_c = _N.popup(_src8, _st8, _vd8, opener=_FakeProc)
check("pop-up: if the window was closed, the next look opens a new one", len(_opened) == 2)
os.environ["P5_POPUP"] = "off"
check("pop-up: P5_POPUP=off shows nothing", _N.popup(_src8, _st8, _vd8, opener=_FakeProc) is None)
del os.environ["P5_POPUP"]
# 8d: three pictures in the same window, in order: 1/3 GraspMolmo -> 2/3 SAM2 + place -> 3/3 grasps (4 s later)
_gn = open(os.path.join(P5, "p5_grasp_node.py")).read()
_i1, _i2, _i3 = _gn.find('"stage1.png")'), _gn.find('"stage2.png")'), _gn.find('threading.Timer(4.0, lambda: popup(os.path.join(VIEW_DIR, "view.png")')
check("pictures: 1/3 GraspMolmo (before SAM2), 2/3 SAM2 + place (at the place step), 3/3 grasps 4 s later - one window",
      0 < _i1 < _gn.find("mask, mask_note = self.object_mask(gm_mask, uv)") and _i2 > 0 and _i3 > _i2 and '"1/3  GraspMolmo' in _gn
      and '"2/3  SAM2' in _gn and '"3/3  GRASP' in _gn)
_p1 = _N.render_view(np.zeros((480, 640, 3), np.uint8), np.array([[489.0, 0, 320], [0, 489.0, 240], [0, 0, 1]]), np.diag([1.0, -1, -1]),
                     np.array([0, 0, 0.8]), dict(__import__("p5_gripper").DEFAULTS), None, None, [0.0, 0.0, 0.0], ["1/3  GraspMolmo"])
check("picture 1/3 has no grasp legend (only what it shows)", _p1.shape[0] < 480 + 2 * 18 + 9)
_st_sh = open(os.path.join(P5, "start_pipeline5.sh")).read()
check("start: no live window any more", 'start "p5_view"' not in _st_sh and "pictures: one pops up per look" in _st_sh)
# 8b: the concise terminal - a real run (2026-11-25 spoon, excerpt) through the filter
import re as _re
_gsrc2 = open(os.path.join(P5, "core", "p3_go.py")).read()
_bnode = [x for x in _ast.parse(_gsrc2).body if isinstance(x, _ast.ClassDef) and x.name == "Brief"][0]
_bns = {"re": _re}
exec(compile(_ast.Module(body=[_bnode], type_ignores=[]), "p3_go", "exec"), _bns)
_B = _bns["Brief"]()
_shown = []
for _ln in open(os.path.join(HERE, "data", "spoon_run_excerpt.txt")).read().splitlines():
    _o = _B.step(_ln) if _ln.startswith((">>>", "<<<")) else _B.line(_re.sub(r"^\[\s*[\d.]+ s\]\s?", "", _ln))
    if _o:
        _shown.append(_o)
_txt = "\n".join(_shown)
check("terminal: one line per step with its start time, a short ok line per service", "[2] 3.0 s     GraspMolmo: grasp point and object region" in _txt
      and "    done in 31.5 s" in _txt
      and "    ok  3 grasp(s) to MoveIt; G1 jaw 70 deg" in _txt and len(_shown) <= 30, "%d lines" % len(_shown))
check("terminal: the ros2 chatter, MoveIt's [INFO]/[WARN] lines and the picture's report are hidden",
      not any(k in _txt for k in ("waiting for service", "requester:", "response:", "[WARN]", "[INFO]", "Found a contact", "PICK     G1", "WARNING:root")))
check("terminal: failures keep their reason", "place descend (0/3): CartesianPath: min_fraction not met" in _txt
      and "FAILED  NO GRASP: no grasp passed" in _txt and "Planning FAILED" in _txt)
check("terminal: P5_VERBOSE=1 prints everything (switch in place)", 'os.environ.get("P5_VERBOSE", "0") == "1"' in _gsrc2)
# 8e: the pipeline's arm speed is 0.5 rad/s per joint (P5_ARM_MAX_VEL), below the joints' 1.0 rad/s limit
_m8e = open(os.path.join(P5, "core", "p3_mtc.py")).read()
check("speed: arm 0.5 rad/s per joint by default (P5_ARM_MAX_VEL), import order safe", 'ARM_MAX_VEL = float(_os.environ.get("P5_ARM_MAX_VEL", "0.5"))' in _m8e
      and _m8e.index("import os as _os") < _m8e.index("ARM_MAX_VEL = float("))
print("%s (%s, %s): %d/%d passed" % ("ALL OK" if all(results) else "FAILURES", X.P5_EXEC_VERSION, p5_view.P5_VIEW_VERSION, sum(results), len(results)))
sys.exit(0 if all(results) else 1)
