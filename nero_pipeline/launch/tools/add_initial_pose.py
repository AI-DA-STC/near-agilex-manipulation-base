#!/usr/bin/env python3
"""
add_initial_pose.py - adds the named arm pose "initial" and makes the pipeline start from it.   (system python, no ROS needed)

It makes three edits, each with a timestamped backup (<file>.bak_initial_<time>), and can be run again safely:
  1. agx_arm.srdf.xacro   adds <group_state name="initial" group="arm"> (7 joints) after the pose_90deg block of the nero (joint7) section.
                          That is what makes "initial" appear in RViz (MotionPlanning > Planning > Goal State), like home and pose_90deg.
  2. move_to_pose_90deg.py  the start-pose script: replaces the single hard-coded pose by a table of named poses and a START_POSE switch
                          (default "initial"; START_POSE=pose_90deg brings the old behaviour back with no edit).
  3. run_pick_and_place.sh  step 0's wording only (the script it calls is unchanged in name).
The URDF is not touched: it describes the robot's shape, not named poses.
    /usr/bin/python3 add_initial_pose.py            (then rebuild agx_arm_moveit and restart, see the instructions)
    /usr/bin/python3 add_initial_pose.py --dry-run  (show what would change, write nothing)
    /usr/bin/python3 add_initial_pose.py --joint4 2.08   (gives joint4 margin from its 2.14 limit: the captured 2.13998 is AT the limit; the gripper moves about 1.8 cm)
"""
import argparse, os, re, shutil, sys, time
import xml.etree.ElementTree as ET

# joint1..7 of "initial", read from /feedback/joint_states of the real arm
INITIAL = [1.7453292519943296e-05, -1.043968692080408, -5.235987755982989e-05, 2.1399831024552873,
           6.981317007977319e-05, 0.002513274122871835, 0.43399357180090997]
POSE_90DEG = [-3.49e-05, -0.0063, -5.24e-05, 1.5674, -0.0759, 0.0026, 0.0403]       # as in the existing script

ap = argparse.ArgumentParser()
ap.add_argument("--srdf", default="~/agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/config/agx_arm.srdf.xacro")
ap.add_argument("--script", default="~/nero_pipeline_launch/scripts/move_to_pose_90deg.py")
ap.add_argument("--run", default="~/nero_pipeline_launch/run_pick_and_place.sh")
ap.add_argument("--dry-run", action="store_true")
ap.add_argument("--joint4", type=float, help="override joint4 in radians, e.g. 2.08: joint4 of the captured pose is AT its upper limit (2.14)")
a = ap.parse_args()
if a.joint4 is not None:
    INITIAL[3] = a.joint4
STAMP = time.strftime("%Y%m%d_%H%M%S")
changed = []


def load(path):
    p = os.path.expanduser(path)
    if not os.path.isfile(p):
        sys.exit("file not found: %s" % p)
    return p, open(p, encoding="utf-8").read()


def save(p, old, new, what):
    if new == old:
        print("  already up to date: %s" % p); return
    changed.append(p)
    if a.dry_run:
        print("  WOULD change %s  (%s)" % (p, what)); return
    shutil.copy2(p, "%s.bak_initial_%s" % (p, STAMP))
    open(p, "w", encoding="utf-8").write(new)
    print("  changed %s  (%s); backup: %s.bak_initial_%s" % (p, what, os.path.basename(p), STAMP))


# ------------------------------------------------------------------ 1. SRDF
p, text = load(a.srdf)
block = '    <group_state name="initial" group="arm">\n' + "".join(
    '      <joint name="joint%d" value="%r" />\n' % (i + 1, v) for i, v in enumerate(INITIAL)) + "    </group_state>"
pat = lambda name: re.compile(r'<group_state\b(?=[^>]*\bname="%s")(?=[^>]*\bgroup="arm")[^>]*>.*?</group_state>' % name, re.S)
new = text
existing = [m for m in pat("initial").finditer(text) if 'name="joint7"' in m.group(0)]
if existing:
    m = existing[0]; new = text[:m.start()] + block.lstrip() + text[m.end():]          # update the 7-joint block in place
    what = "updated the existing 'initial' state"
else:
    anchors = [m for m in pat("pose_90deg").finditer(text) if 'name="joint7"' in m.group(0)]
    if not anchors:
        sys.exit("could not find the 7-joint (joint7) group_state 'pose_90deg' for group 'arm' in %s - nothing changed" % p)
    m = anchors[0]; new = text[:m.end()] + "\n" + block + text[m.end():]
    what = "added the 'initial' state after pose_90deg"
try:
    ET.fromstring(new)                                                                   # must still be well-formed XML
except ET.ParseError as e:
    sys.exit("the edited SRDF would not be well-formed XML (%s) - nothing changed" % e)
print("1. SRDF"); save(p, text, new, what)

# ------------------------------------------------------------------ 2. start-pose script
p, text = load(a.script)
new = text
if "START_POSE" in text:                                                                 # table exists: just refresh the "initial" values
    new = re.sub(r'(    "initial": )\[[^\]]*\](,)', lambda m: m.group(1) + "[" + ", ".join("%.6f" % v for v in INITIAL) + "]" + m.group(2), text, count=1)
if "START_POSE" not in text:
    old_line = re.search(r'^POSE_90DEG = \[[^\]]*\]\s*$', text, re.M)
    if not old_line:
        sys.exit("could not find the 'POSE_90DEG = [...]' line in %s - nothing changed" % p)
    table = ('# Named start poses (joint1..7, radians). Choose with the environment variable START_POSE; the default is "initial".\n'
             '#   START_POSE=pose_90deg ./run_pick_and_place.sh     brings the old start pose back without editing anything.\n'
             'POSES = {\n    "pose_90deg": %s,\n    "initial": [%s],      # captured from the real arm: clear of the top camera\'s view\n}\n'
             'START_POSE = os.environ.get("START_POSE", "initial")\n'
             'if START_POSE not in POSES:\n    sys.exit("START_POSE=%%s is not one of %%s" %% (START_POSE, list(POSES)))\n'
             'POSE_90DEG = POSES[START_POSE]   # (variable name kept so the rest of the script is unchanged: it holds the CHOSEN start pose)'
             % (POSE_90DEG, ", ".join("%.6f" % v for v in INITIAL)))
    new = text[:old_line.start()] + table + text[old_line.end():]
    new = new.replace('print("Already at pose_90deg - no movement needed.")', 'print(f"Already at {START_POSE} - no movement needed.")')
    new = new.replace('print(f"Not at pose_90deg (current: {node.current_positions}) - moving there now...")', 'print(f"Not at {START_POSE} (current: {node.current_positions}) - moving there now...")')
    new = new.replace('print("Reached pose_90deg.")', 'print(f"Reached {START_POSE}.")')
    new = new.replace('print("FAILED to reach pose_90deg - check the arm_controller is running.")', 'print(f"FAILED to reach {START_POSE} - check the arm_controller is running.")')
compile(new, p, "exec")                                                                  # syntax check before writing
print("2. start-pose script"); save(p, text, new, "named poses + START_POSE (default initial)")

# ------------------------------------------------------------------ 3. run script wording
p, text = load(a.run)
new = text.replace('echo "=== 0/5: Moving arm to pose_90deg ==="', 'echo "=== 0/5: Moving arm to the start pose (${START_POSE:-initial}) ==="').replace(
    '{ echo "pose_90deg move failed"; exit 1; }', '{ echo "start-pose move failed"; exit 1; }')
print("3. run_pick_and_place.sh"); save(p, text, new, "step 0 wording")

print("\nnothing was written (dry run)." if a.dry_run else ("\n%d file(s) changed." % len(changed) if changed else "\nnothing to do."))
