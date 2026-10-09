#!/usr/bin/env python3
"""
Save the arm's CURRENT pose as a named state in the SRDF.

  1. Move the arm to the pose you want (RViz Plan & Execute, or however) and let it stop.
  2. Run (conda off):  /usr/bin/python3 capture_home.py <srdf.xacro> [state_name] [--initial <initial_positions.yaml>]
     state_name defaults to "home". Joints are matched by NAME, so the odd
     order of /control/joint_states doesn't matter.
"""
import os, re, shutil, sys

ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]
TOPIC = os.environ.get("JOINT_STATES_TOPIC", "/control/joint_states")


def rewrite_state(text, state, values):
    pat = (r'(<group_state\b(?=[^>]*\bname="%s")(?=[^>]*\bgroup="arm")[^>]*>)(.*?)(</group_state>)'
           % re.escape(state))
    m = None
    for cand in re.finditer(pat, text, re.S):          # file may hold a 6-joint (piper) and a 7-joint (nero) block
        if 'name="joint7"' in cand.group(2):
            m = cand
            break
    if not m:
        raise SystemExit(f'no 7-joint <group_state name="{state}" group="arm"> found in that file')
    body = m.group(2)
    for j in ARM_JOINTS:
        tag = re.search(r'<joint\b[^>]*\bname="%s"[^>]*/?>' % j, body)
        if not tag:
            raise SystemExit(f"state '{state}' has no <joint name=\"{j}\"> line")
        new = re.sub(r'value="[^"]*"', 'value="%.6f"' % values[j], tag.group(0))
        body = body.replace(tag.group(0), new, 1)
    return text[:m.start(2)] + body + text[m.end(2):]


def rewrite_initial(text, values):
    for j, v in values.items():
        text, n = re.subn(r'(?m)^(\s*%s:\s*)[-+0-9.eE]+' % j, lambda m: m.group(1) + "%.6f" % v, text)
        if n == 0:
            raise SystemExit(f"initial_positions file has no '{j}:' line")
    return text


def read_pose():
    import rclpy
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = rclpy.create_node("capture_home")
    got, seen = {}, {"names": None}
    def cb(msg):
        seen["names"] = list(msg.name)
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in ARM_JOINTS):
            got.update(d)
    node.create_subscription(JointState, TOPIC, cb, 10)
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if got:
            break
    node.destroy_node(); rclpy.shutdown()
    if not got:
        if seen["names"] is None:
            raise SystemExit(f"no messages at all on {TOPIC} - wrong topic name, or the arm driver isn't running. "
                             f"Find it with: ros2 topic list | grep -i joint   "
                             f"then run with JOINT_STATES_TOPIC=<that topic>")
        raise SystemExit(f"messages arrive on {TOPIC}, but the joint names are {seen['names']}, "
                         f"not {ARM_JOINTS} - tell me these names and I will map them")
    return {j: got[j] for j in ARM_JOINTS}


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    args = sys.argv[1:]
    initial = None
    if "--initial" in args:
        i = args.index("--initial")
        initial = os.path.expanduser(args[i + 1])
        del args[i:i + 2]
    path = os.path.expanduser(args[0])
    state = args[1] if len(args) > 1 else "home"
    vals = read_pose()
    print(f"Current pose -> '{state}':")
    for j in ARM_JOINTS:
        print(f"  {j}: {vals[j]:+.6f} rad")
    if input("Write this into the SRDF? [y/N]: ").strip().lower() != "y":
        raise SystemExit("not written")
    text = open(path).read()
    new = rewrite_state(text, state, vals)
    shutil.copy(path, path + ".bak_home")
    open(path, "w").write(new)
    print("written; backup:", path + ".bak_home")
    if initial:
        shutil.copy(initial, initial + ".bak_home")
        open(initial, "w").write(rewrite_initial(open(initial).read(), vals))
        print("also updated sim spawn pose:", initial, "(backup .bak_home)")
