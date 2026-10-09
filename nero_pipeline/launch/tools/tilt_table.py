"""
tilt_table.py - which candidate grasps passed the tilt cap and were handed to MTC?

Needs /tmp/grasp_debug.json, which launch/tools/grasp_watch.py writes after every detection.
    /usr/bin/python3 ~/nero_pipeline_launch/tools/tilt_table.py     (conda off)
Prints, for each raw Contact-GraspNet candidate: its tilt from straight down, whether it passes MAX_TILT_DEG
(read from mtc_pick_and_place.py) and whether it is among the ones given to MTC.
Note: "given to MTC" assumes the candidates are in the order the JSON stores them; with the tilt sort active
in mtc_pick_and_place.py the watcher prints the exact list of tilts it handed over instead.
"""
import json, math, re, os, sys
import numpy as np
js = json.load(open("/tmp/grasp_debug.json"))
src = os.path.expanduser("~/agx_arm_ws/src/nero_mtc_task/nero_mtc_task/mtc_pick_and_place.py")
cap = float(re.search(r"^MAX_TILT_DEG\s*=\s*([0-9.]+)", open(src).read(), re.M).group(1))
given = len(js["cands"]); n_pass = 0
print("raw candidates: %d   cap: %.0f deg   handed to MTC: %d" % (len(js["raw"]), cap, given))
print(" #   tilt(deg)  passes?   given to MTC?   hand-base xyz (m)")
for i, T in enumerate(js["raw"]):
    T = np.array(T); tilt = math.degrees(math.acos(float(np.clip(-T[2, 2], -1, 1))))
    ok = tilt <= cap; used = ok and n_pass < given; n_pass += ok
    print("%2d  %8.0f   %-7s   %-13s   (%.3f, %.3f, %.3f)" % (i, tilt, "yes" if ok else "NO", "YES" if used else "-", T[0, 3], T[1, 3], T[2, 3]))
