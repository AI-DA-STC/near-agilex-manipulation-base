import ast, os, re, sys
BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src = open(os.path.join(BASE, "launch", "contactgraspnet.launch.py")).read(); ast.parse(src)
declared = set(re.findall(r'DeclareLaunchArgument\(\s*"(\w+)"', src)); used = set(re.findall(r'LaunchConfiguration\("(\w+)"\)', src))
node = open(os.path.join(BASE, "contact_graspnet_ros", "contact_graspnet_node.py")).read()
node_params = set(re.findall(r'declare_parameter\(\s*"(\w+)"', node)); passed = set(re.findall(r'^\s+"(\w+)": LaunchConfiguration', src, re.M))
ok = True
def check(c, m):
    global ok; print(("  PASS  " if c else "  FAIL  ") + m); ok = ok and c
check(used <= declared, "every LaunchConfiguration has a matching DeclareLaunchArgument (%s)" % sorted(used - declared))
check(passed <= node_params, "every parameter the launch file passes is declared by the node (%s)" % sorted(passed - node_params))
check({"use_worker", "forward_passes", "retries_if_empty"} <= passed, "the three new settings can be given on the launch command line")
sys.exit(0 if ok else 1)
