#!/usr/bin/env python3
"""p4_make_core.py - makes pipeline 4 independent of pipeline 3 (run by install_pipeline4.sh; pipeline 3 is only READ):
  pipeline4/core/*   = COPIES of your installed pipeline 3 files (~/nero_pipeline_launch/pipeline3/*.py), unchanged except:
      p3_mtc.py      the gripper opens only as wide as pipeline 4's grasp step says (~/p3_runs/p4_grasp_latest.json; fully open if missing,
                     older than 30 min, or P4_GRIPPER_WIDTH=off)
      p3_go.py       runs pipeline 4's run script, points to p4_execute.sh
  pipeline4/p4_run.sh = a copy of run_pick_and_place3.sh using core/, with ONE added step: after the poses are in base_link, the wrist camera's
                     3D obstacle boxes go into MoveIt (/p4/wrist_obstacles; P4_WRIST=off removes them instead) - so the plan sees BOTH cameras.
Every change must match EXACTLY ONCE and every copied file must have the version pipeline 4 was written against; otherwise nothing is written.
"""
import hashlib, os, shutil, sys

P4_MAKE_CORE_VERSION = "p4-make-core-v1"
ROOT = os.path.expanduser(os.environ.get("P4_ROOT", "~/nero_pipeline_launch"))

VERSIONS = {          # file in pipeline3/ -> version string it must contain
    "add_table_box.py": 'P3_TABLE_BOX_VERSION = "p3-tablebox-v1"', "p3_calib_check.py": 'P3_CALIB_CHECK_VERSION = "p3-calibcheck-v4"',
    "p3_fingers.py": 'P3_FINGERS_VERSION = "p3-fingers-v2"', "p3_grasp_view.py": 'P3_GRASP_VIEW_VERSION = "p3-graspview-v1"',
    "p3_grasp_watch.py": 'P3_GRASP_WATCH_VERSION = "p3-graspwatch-v1"', "p3_graspmolmo_node.py": 'P3_NODE_VERSION = "p3-node-v8"',
    "p3_graspmolmo_worker.py": 'P3_WORKER_VERSION = "p3-worker-v1"', "p3_jaw_fix.py": 'P3_JAW_FIX_VERSION = "p3-jawfix-v8"',
    "p3_jobs.py": 'P3_JOBS_VERSION = "p3-jobs-v1"', "p3_logfilter.py": 'P3_LOGFILTER_VERSION = "p3-logfilter-v1"',
    "p3_mtc.launch.py": 'P3_MTC_LAUNCH_VERSION = "p3-mtclaunch-v1"', "p3_mtc.py": 'P3_MTC_VERSION = "p3-mtc-v3"',
    "p3_objects.py": 'P3_OBJECTS_VERSION = "p3-objects-v1"', "p3_obstacles.py": 'P3_OBSTACLES_VERSION = "p3-obstacles-v6"',
    "p3_picture.py": 'P3_PICTURE_VERSION = "p3-picture-v10"', "p3_scene.py": 'P3_SCENE_VERSION = "p3-scene-v2"',
    "p3_tablez.py": 'P3_TABLEZ_VERSION = "p3-tablez-v4"', "p3_go.py": 'GO_VERSION = "p3-go-v6"',
    "p3_geometry.py": None, "worker_client.py": None,
}
RUN_VERSION = 'P3_RUN_VERSION="p3-run-v8"'

GRIPPER_HELPER = '''
P4_MTC_VERSION = "p4-mtc-v1"   # PIPELINE 4 core copy: the gripper opens only as wide as the grasp step says (p4_make_core.py)


def _p4_open_width():
    """-> (opening in m, why) from pipeline 4's grasp step, or (None, why) = fully open."""
    import json as _j, os as _o, time as _t
    if _o.environ.get("P4_GRIPPER_WIDTH", "on") == "off":
        return None, "P4_GRIPPER_WIDTH=off"
    path = _o.path.expanduser("~/p3_runs/p4_grasp_latest.json")
    try:
        d = _j.load(open(path))
    except Exception:
        return None, "no %s (is p4_grasp running?)" % path
    age = _t.time() - float(d.get("unix_time", 0))
    if age > 1800:
        return None, "the grasp step's width is %.0f min old" % (age / 60)
    w = float(d.get("open_width_m", 0))
    if not 0.03 <= w <= 0.10:
        return None, "width %.3f m is outside 0.03-0.10" % w
    return w, "pipeline 4 grasp step of %s" % d.get("time")


def _p4_gripper_goal(stage, what):
    w, why = _p4_open_width()
    if w is None:
        stage.setGoal("gripper_open")
        print(f"PIPELINE 4: gripper {what}: FULLY open (0.100 m) - {why}")
        return
    try:
        stage.setGoal({"gripper": w})
        print(f"PIPELINE 4: gripper {what}: {w:.3f} m wide - {why}")
    except Exception as e:
        stage.setGoal("gripper_open")
        print(f"PIPELINE 4: gripper {what}: could not set {w:.3f} m ({e}); FULLY open instead")
'''

WRIST_STEP = '''call_checked /pose_transform/transform_all

# PIPELINE 4: the wrist camera's 3D obstacle boxes (the arm is at its start pose and still), so MoveIt plans with BOTH cameras
echo ""
if [ "${P4_WRIST:-on}" != off ]; then
echo "=== 2b/3: wrist camera obstacles: 3D boxes at their real height, into MoveIt ==="
call_checked /p4/wrist_obstacles
else
echo "=== 2b/3: P4_WRIST=off: wrist obstacle boxes removed (top camera only) ==="
call_checked /p4/clear_wrist_obstacles
fi
fi
'''

EDITS = {
    "p3_mtc.py": [
        ('P3_MTC_VERSION = "p3-mtc-v3"\n', 'P3_MTC_VERSION = "p3-mtc-v3"\n' + GRIPPER_HELPER),
        ('    open_gripper.setGoal("gripper_open")\n', '    _p4_gripper_goal(open_gripper, "open")\n'),
        ('    release_gripper.setGoal("gripper_open")\n', '    _p4_gripper_goal(release_gripper, "release")\n'),
    ],
    "p3_go.py": [
        ('ap.add_argument("--run_script", default=os.path.expanduser("~/nero_pipeline_launch/run_pick_and_place3.sh"))',
         'ap.add_argument("--run_script", default=os.path.expanduser("~/nero_pipeline_launch/pipeline4/p4_run.sh"))   # PIPELINE 4'),
        ('To execute it:  bash ~/nero_pipeline_launch/pipeline3/p3_do.sh', 'To execute it:  bash ~/nero_pipeline_launch/pipeline4/p4_execute.sh'),
    ],
}
RUN_EDITS = [
    (RUN_VERSION + "\n", RUN_VERSION + '\nP4_RUN_VERSION="p4-run-v1"   # PIPELINE 4: core/ copies + the wrist obstacle step (p4_make_core.py)\necho "p4_run $P4_RUN_VERSION"\n'),
    ('"$HOME/nero_pipeline_launch/pipeline3/p3_picture.py"', '"$HOME/nero_pipeline_launch/pipeline4/core/p3_picture.py"'),
    ('"$HOME/nero_pipeline_launch/pipeline3/p3_logfilter.py"', '"$HOME/nero_pipeline_launch/pipeline4/core/p3_logfilter.py"'),
    ('ros2 launch "$HOME/nero_pipeline_launch/pipeline3/p3_mtc.launch.py"', 'ros2 launch "$HOME/nero_pipeline_launch/pipeline4/core/p3_mtc.launch.py"'),
    ("call_checked /pose_transform/transform_all\nfi\n", WRIST_STEP),
]


def _apply(name, s, edits, problems):
    for old, new in edits:
        n = s.count(old)
        if n != 1:
            problems.append("%s: expected 1 match of %r, found %d" % (name, old.strip()[:70], n))
        else:
            s = s.replace(old, new)
    return s


def make(root=ROOT, write=True):
    src_dir, out_core = os.path.join(root, "pipeline3"), os.path.join(root, "pipeline4", "core")
    files, problems, record = {}, [], []
    for f, ver in VERSIONS.items():
        try:
            s = open(os.path.join(src_dir, f)).read()
        except OSError as e:
            problems.append("cannot read pipeline3/%s: %s" % (f, e)); continue
        if ver and ver not in s:
            problems.append("pipeline3/%s is not the expected version (%s)" % (f, ver)); continue
        record.append("%s  %s" % (hashlib.md5(s.encode()).hexdigest(), f))
        files[f] = _apply(f, s, EDITS.get(f, []), problems)
    try:
        run = open(os.path.join(root, "run_pick_and_place3.sh")).read()
        if RUN_VERSION not in run:
            problems.append("run_pick_and_place3.sh is not the expected version (%s)" % RUN_VERSION)
        record.append("%s  run_pick_and_place3.sh" % hashlib.md5(run.encode()).hexdigest())
        run = _apply("run_pick_and_place3.sh", run, RUN_EDITS, problems)
    except OSError as e:
        problems.append("cannot read run_pick_and_place3.sh: %s" % e)
    if problems:
        return False, problems
    if write:
        if os.path.isdir(out_core):
            shutil.rmtree(out_core)
        os.makedirs(out_core)
        for f, s in files.items():
            p = os.path.join(out_core, f)
            open(p, "w").write(s)
            os.chmod(p, 0o755)
        open(os.path.join(out_core, "COPIED_FROM_PIPELINE3.txt"), "w").write(
            "# %s: copies of these pipeline 3 files (md5 of the ORIGINALS); edited: p3_mtc.py (gripper width), p3_go.py (run script)\n" % P4_MAKE_CORE_VERSION
            + "\n".join(record) + "\n")
        p = os.path.join(root, "pipeline4", "p4_run.sh")
        open(p, "w").write(run)
        os.chmod(p, 0o755)
    return True, ["core/ (%d files copied from pipeline3/)" % len(files), "p4_run.sh"]


if __name__ == "__main__":
    ok, info = make()
    print(P4_MAKE_CORE_VERSION)
    for line in info:
        print(("  wrote " if ok else "  PROBLEM: ") + line)
    sys.exit(0 if ok else 1)
