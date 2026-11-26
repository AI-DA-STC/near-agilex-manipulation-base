#!/usr/bin/env python3
"""Checks that the installed pipeline 5 is complete, unmodified and independent of pipeline 4 (no ROS needed).
Run: /usr/bin/python3 ~/nero_pipeline_launch/pipeline5/tests/test_p5_bundle.py"""
import glob, hashlib, os, re, subprocess, sys
HERE = os.path.dirname(os.path.abspath(__file__)); P5 = os.path.abspath(os.path.join(HERE, ".."))
results = []


def check(name, ok, info=""):
    results.append(bool(ok)); print("%-5s %s %s" % ("PASS" if ok else "FAIL", name, info))


pys = sorted(glob.glob(os.path.join(P5, "*.py")) + glob.glob(os.path.join(P5, "core", "*.py")) + glob.glob(os.path.join(P5, "tests", "*.py")))
bad = []
for f in pys:
    try:
        compile(open(f).read(), f, "exec")
    except Exception as e:
        bad.append("%s: %s" % (os.path.basename(f), e))
check("every Python file compiles (%d)" % len(pys), not bad, "; ".join(bad))

local = {os.path.splitext(os.path.basename(f))[0] for f in pys}
missing = []
for f in pys:
    for m in re.findall(r"^\s*(?:import|from)\s+(p[345]_\w+|worker_client)", open(f).read(), re.M):
        if m not in local:
            missing.append("%s needs %s" % (os.path.basename(f), m))
check("pipeline modules imported are all inside pipeline5", not missing, "; ".join(missing))

shs = sorted(glob.glob(os.path.join(P5, "*.sh")))
bad = [os.path.basename(f) for f in shs if subprocess.run(["bash", "-n", f], capture_output=True).returncode != 0]
check("every shell script parses (%d)" % len(shs), not bad, bad)

man = open(os.path.join(P5, "core", "COPIED_FROM_PIPELINE4.txt")).read().split("# shipped:")[1].split()
pairs = list(zip(man[0::2], man[1::2]))
bad = [rel for md5, rel in pairs if hashlib.md5(open(os.path.join(P5, rel)).read().encode()).hexdigest() != md5]
check("copied files are exactly as shipped (%d, md5)" % len(pairs), not bad, bad)

run = open(os.path.join(P5, "p5_run.sh")).read()
check("p5_run.sh calls /p5/grasps, not Contact-GraspNet", "call_checked /p5/grasps" in run and "contact_graspnet" not in run)
check("p5_run.sh uses pipeline 5's wrist services", "/p5/wrist_obstacles" in run and "/p4/" not in run)
leaks = []
for f in pys + shs + [os.path.join(P5, "p5_clouds.rviz")]:  # code lines only (text after # ignored)
    for i, line in enumerate(open(f), 1):
        code = line.split("#")[0]
        if f.endswith("install_pipeline5.sh"):           # its one-time copy of ~/.p4_offsets.yaml is intended (documented there)
            continue
        if re.search(r"nero_pipeline_launch/pipeline4|p4_grasp_latest|[\"']/p4/|p4_offsets\.yaml|import p4_", code) and "tests/test_p5_bundle.py" not in f:
            leaks.append("%s:%d" % (os.path.relpath(f, P5), i))
check("nothing points back to pipeline 4", not leaks, leaks[:8])
st = open(os.path.join(P5, "start_pipeline5.sh")).read()
check("start: pose_transform listens to the grasp node", "grasps_topic:=/p5/grasp_poses" in st and '"/p5/grasp_poses"' in open(os.path.join(P5, "p5_grasp_node.py")).read())
check("start: no Contact-GraspNet launched", "contactgraspnet.launch" not in st and "p4_grasp" not in st)
fu = open(os.path.join(P5, "p5_fuse_node.py")).read()
check("fusion leaves out the mask the grasp step used (/p5/mask_source)", '("mask_source", "/p5/mask_source", Image, 10),' in fu
      and '"/p5/mask_source"' in open(os.path.join(P5, "p5_grasp_node.py")).read())
check("clean script also stops the SAM2 worker", "p5_sam2_worker" in open(os.path.join(P5, "p5_clean.sh")).read())
check("fusion leaves the gripper's checked corridor out of the wrist boxes", "p5_keepout.apply_latest(Pw" in fu and "import p5_keepout" in fu)
gn = open(os.path.join(P5, "p5_grasp_node.py")).read()
check("grasp node writes the corridor into its report", 'g_rep["keepout"] = p5_keepout.corridor(' in gn)
check("MTC copy: full approach or no plan", "APPROACH_MIN, APPROACH_MAX = 0.078, 0.08" in open(os.path.join(P5, "core", "p3_mtc.py")).read())
check("start: CAN link check before anything starts, arm feedback checked after launch", "can_activate.sh" in st
      and st.index("can_link") < st.index("p5_clean.sh\"") and "/feedback/joint_states" in st and "candump" not in st)
check("run: /p5/place right after GraspMolmo's place step, before the plate point", run.index("/graspmolmo/place") < run.index("call_checked /p5/place")
      < run.index("call_checked /target_pose/extract"))
check("place point flows from /p5/mask_target (target_pose + the fusion's landing zone)", "mask_topic:=/p5/mask_target" in st
      and '("mask_target", "/p5/mask_target", Image, 10)' in fu and '"/p5/mask_target"' in gn)
mtc = open(os.path.join(P5, "core", "p3_mtc.py")).read()
check("MTC copy saves the plan (p5_execute.sh runs it without re-planning) and can exit after planning", "_p5_save_plan(chosen_solution.toMsg(), phase_names)" in mtc
      and 'P5_PLAN_EXIT' in mtc)
ex, go = open(os.path.join(P5, "p5_execute.sh")).read(), open(os.path.join(P5, "p5_go.sh")).read()
check("p5_execute.sh / p5_go.sh: no EXECUTE prompt, no re-planning", "p5_exec_plan.py" in ex and "--last" not in ex and "read " not in ex + go
      and "p5_execute.sh" in go and "P5_PLAN_EXIT=1" in go)
check("start: no live window (8a); the pictures are deleted on Ctrl+C", 'start "p5_view"' not in st and "rm -rf /tmp/p5_view" in st)
check("grasp node puts the object in hand and the camera box into MoveIt", "SO.apply(self.scene, slices, cam)" in gn)
print("%s: %d/%d passed" % ("ALL OK" if all(results) else "FAILURES", sum(results), len(results)))
sys.exit(0 if all(results) else 1)
