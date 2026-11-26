#!/usr/bin/env python3
"""p5_view.py - PIPELINE 5's live window ("Pipeline 5"), started by start_pipeline5.sh and closed with it. No ROS: it follows files.
  left:  the grasp node's picture of THIS look (P5_VIEW_DIR/view.png): object outline, grasps (G1 green ...), GraspMolmo's points, the place used
         - it appears as soon as the grasps exist and is redrawn when the place point is known, before transform and planning
  right: every step of the run in progress with its time (the newest log in ~/p3_runs/go_logs: p5_plan / p5_go / p5_replan / p5_execute),
         and the latest important lines - failures in red
Nothing is saved: the picture lives in P5_VIEW_DIR (/tmp/p5_view), which start_pipeline5.sh deletes on Ctrl+C.
"""
import glob
import os
import re
import sys
import time

import numpy as np

P5_VIEW_VERSION = "p5-view-v1d"   # v1d (7h): redraws only when what is SHOWN changes (log lines alone never) | v1c | v1b   # v1b (view1): falls back to the image viewer if OpenCV cannot open a window; --test
VIEW_DIR = os.environ.get("P5_VIEW_DIR", "/tmp/p5_view")
LOGS = os.path.expanduser("~/p3_runs/go_logs")
STEP_RE = re.compile(r">>> STEP (\d+): (.*?)\s+\(started at")
DONE_RE = re.compile(r"<<< done: (.*) in ([\d.]+ s)")
BAD = ("!!!", "STOP", "FAILED", "NOT EXECUTED", "REFUSED", "NO GRASP", "Failing stage", "WARNING", "DID NOT COMPLETE", "died")
GOOD = ("PLAN SAVED", "plan saved", "EXECUTED: every", "grasp(s) to MoveIt", "moved ", "point fits", "Found ", "keep-out:", "SAM2 proposal")
IGNORE = ("[python3-1]   ", "moveit_", "rcutils", "OMPL", "carrot")


def parse(lines):
    """Log lines -> (steps [(name, time str or None, state run/done/fail)], notes [(text, bad?)], finished?)."""
    steps, notes, finished = [], [], False
    for ln in lines:
        t = ln.strip()
        t = re.sub(r"^\[\s*[\d.]+ s\]\s*", "", t)
        m = STEP_RE.search(t)
        if m:
            if steps and steps[-1][2] == "run":
                steps[-1][2] = "done"
            steps.append([m.group(2), None, "run"])
            continue
        m = DONE_RE.search(t)
        if m and steps:
            steps[-1][1], steps[-1][2] = m.group(2), "done"
            continue
        if t.startswith("TIMING") or t.startswith("EXECUTED") or t.startswith("EXECUTION DID NOT") or t.startswith("NOT EXECUTED"):
            finished = True
        if any(x in t for x in IGNORE) and not any(b in t for b in BAD):
            continue
        bad = any(b in t for b in BAD)
        if bad or any(g in t for g in GOOD):
            t = re.sub(r"^\[python3-\d+\]\s*", "", t)
            t = re.sub(r"^.*response:.*?success=(True|False), message=['\"]", lambda m: "" if m.group(1) == "True" else "FAILED: ", t).rstrip("')\"")
            notes.append((t, bad or t.startswith("FAILED")))
    if finished and steps and steps[-1][2] == "run":
        steps[-1][2] = "fail" if notes and notes[-1][1] else "done"
    if notes and notes[-1][1] and steps and steps[-1][2] == "run":
        steps[-1][2] = "fail"
    return steps, notes[-7:], finished


def newest_log():
    fs = glob.glob(os.path.join(LOGS, "*.txt"))
    return max(fs, key=os.path.getmtime) if fs else None


def compose(view, log_name, steps, notes, finished, W=1180, H=620):
    import cv2
    canvas = np.full((H, W, 3), 24, np.uint8)
    if view is not None:
        s = min(700 / view.shape[1], (H - 10) / view.shape[0])
        v = cv2.resize(view, (int(view.shape[1] * s), int(view.shape[0] * s)))
        canvas[5:5 + v.shape[0], 5:5 + v.shape[1]] = v
    else:
        cv2.putText(canvas, "the picture appears as soon as the grasps exist", (20, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 160, 160), 1)
    x0 = 715
    put = lambda t, y, col=(230, 230, 230), sc=0.5, th=1: cv2.putText(canvas, t, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, sc, col, th)
    put("PIPELINE 5  %s" % ("(finished)" if finished else ("(running)" if steps else "")), 24, (255, 255, 255), 0.65, 2)
    put(os.path.basename(log_name or "no run yet"), 44, (150, 150, 150), 0.42)
    y = 72
    for name, t, st in steps[-14:]:
        col = {"run": (0, 210, 255), "done": (90, 220, 90), "fail": (80, 80, 255)}[st]
        mark = {"run": ">", "done": "ok", "fail": "X"}[st]
        put("%-3s %s" % (mark, name[:40]), y, col)
        cv2.putText(canvas, t or ("..." if st == "run" else ""), (W - 80, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        y += 22
    y = max(y + 10, 400)
    put("latest:", y, (150, 150, 150), 0.45)
    y += 20
    for t, bad in notes:
        for k in range(0, min(len(t), 150), 50):
            put(t[k:k + 50], y, (80, 80, 255) if bad else (200, 200, 200), 0.42)
            y += 17
            if y > H - 8:
                break
    return canvas


def open_window(cv2):
    """-> "cv2" if OpenCV can open a window here, else None (some OpenCV builds have no window support)."""
    try:
        cv2.namedWindow("Pipeline 5", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Pipeline 5", 1180, 620)
        cv2.imshow("Pipeline 5", np.zeros((40, 40, 3), np.uint8))
        cv2.waitKey(1)
        return "cv2"
    except Exception as e:                                         # cv2.error "The function is not implemented" etc.
        print("p5_view: OpenCV cannot open a window here (%s)" % str(e).splitlines()[0][:120], flush=True)
        return None


def start_viewer(path):
    """Fallback: the system image viewer on the live picture (it reloads the file when it changes). -> Popen or None."""
    import shutil
    import subprocess
    for viewer in ("eog", "xdg-open"):
        if shutil.which(viewer):
            return subprocess.Popen([viewer, path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return None


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    try:
        import cv2
    except ImportError:
        print("p5_view: no cv2 - no live window", flush=True)
        return 0
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print("p5_view: no display (DISPLAY not set) - no live window", flush=True)
        return 0
    os.makedirs(VIEW_DIR, exist_ok=True)
    mode = open_window(cv2)
    viewer = None
    dash = os.path.join(VIEW_DIR, "dashboard.png")
    if mode is None:
        cv2.imwrite(dash, compose(None, None, [], [], False))
        viewer = start_viewer(dash)
        mode = "viewer" if viewer else None
    if "--test" in argv:                                            # a 3 s self-test before a session
        img = compose(None, "self-test", [["self-test: this window works", "", "done"]], [("you can close this", False)], True)
        for _ in range(8):
            if mode == "cv2":
                cv2.imshow("Pipeline 5", img)
                cv2.waitKey(400)
            else:
                cv2.imwrite(dash, img)
                time.sleep(0.4)
        if viewer:
            viewer.terminate()
        print("p5_view self-test: %s" % {"cv2": "OK - the 'Pipeline 5' window opened",
                                          "viewer": "OK - OpenCV has no window support here; the image viewer (eog) shows it instead"}.get(mode, "FAILED - no window and no image viewer"), flush=True)
        return 0 if mode else 1
    print("p5_view %s: %s (picture from %s, steps from %s)" % (P5_VIEW_VERSION, {"cv2": "window 'Pipeline 5'", "viewer": "image viewer (eog) on " + dash}
          .get(mode, "NO window"), VIEW_DIR, LOGS), flush=True)
    if mode is None:
        return 1
    last = (None, None)
    view, log_name, steps, notes, finished = None, None, [], [], False
    img, first = None, True
    shown_last, drawn_at = None, 0.0
    try:
        while True:
            vp = os.path.join(VIEW_DIR, "view.png")
            lg = newest_log()
            key = (os.path.getmtime(vp) if os.path.exists(vp) else None, (lg, os.path.getmtime(lg)) if lg else None)
            changed = False
            if key != last:                                         # the files changed - but does what the window SHOWS change?
                last = key
                view_new = cv2.imread(vp) if key[0] else None
                log_name_new, steps_new, notes_new, fin_new = lg, steps, notes, finished
                if lg:
                    try:
                        steps_new, notes_new, fin_new = parse(open(lg, errors="replace").read().splitlines())
                    except OSError:
                        pass
                shown = (key[0], log_name_new, tuple(tuple(x) for x in steps_new), tuple(notes_new), fin_new)
                if shown != shown_last:
                    step_moved = shown_last is None or shown[2] != shown_last[2] or shown[0] != shown_last[0]
                    # 7h: a new step / picture shows at once; other changes (a new message) at most every 2 s - log lines alone never redraw
                    if step_moved or time.time() - drawn_at >= 2.0:
                        view, log_name, steps, notes, finished = view_new, log_name_new, steps_new, notes_new, fin_new
                        shown_last, changed = shown, True
                    else:
                        last = None                                 # look again next round
            
            if changed or img is None:                              # 7g/7h: redraw ONLY when what is shown changed
                img = compose(view, log_name, steps, notes, finished)
                drawn_at = time.time()
            if mode == "cv2":
                if changed or first:
                    cv2.imshow("Pipeline 5", img)
                    first = False
                cv2.waitKey(400)
                try:
                    if cv2.getWindowProperty("Pipeline 5", cv2.WND_PROP_VISIBLE) < 1:   # closed with its X: reopen
                        mode = open_window(cv2) or mode
                        first = True
                except cv2.error:
                    pass
            else:
                if changed:
                    tmp = dash + ".tmp.png"
                    cv2.imwrite(tmp, img)
                    os.replace(tmp, dash)
                time.sleep(0.4)
    finally:
        if viewer:
            viewer.terminate()


if __name__ == "__main__":
    sys.exit(main())
