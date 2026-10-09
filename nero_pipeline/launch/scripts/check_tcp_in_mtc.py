import glob, os, re, yaml
f = max(glob.glob("/tmp/mtc_moveit_params_*.yaml"), key=os.path.getmtime)
def find(d):
    if isinstance(d, dict):
        for k, v in d.items():
            if k == "robot_description" and isinstance(v, str):
                return v
            r = find(v)
            if r: return r
urdf = find(yaml.safe_load(open(f)))
m = re.search(r'<joint[^>]*name="tcp_joint".*?</joint>', urdf or "", re.S)
print(f); print(m.group(0) if m else "tcp_joint not found")
