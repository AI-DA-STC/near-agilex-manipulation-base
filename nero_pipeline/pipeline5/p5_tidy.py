#!/usr/bin/env python3
"""p5_tidy.py - PIPELINE 5: keep the run folders from piling up (newest kept): GraspMolmo's run folders in ~/p3_runs (pictures, answers) 10,
~/p3_runs/go_logs 30, ~/p3_runs/p5_snaps 20 (+ their pictures), ~/p5_runs (wrist boxes) 10. P5_KEEP_RUNS overrides the 10."""
import glob
import os
import re
import shutil

P5_TIDY_VERSION = "p5-tidy-v1"


def prune(paths, keep):
    paths = sorted(paths, key=os.path.getmtime)
    gone = paths[:-keep] if keep > 0 else paths
    for p in gone:
        try:
            shutil.rmtree(p) if os.path.isdir(p) and not os.path.islink(p) else os.remove(p)
        except OSError:
            pass
    return len(gone)


def main(home=os.path.expanduser("~")):
    keep = int(os.environ.get("P5_KEEP_RUNS", "10"))
    r = os.path.join(home, "p3_runs")
    runs = [p for p in glob.glob(os.path.join(r, "*")) if os.path.isdir(p) and not os.path.islink(p) and re.match(r"^\d{8}_\d{6}", os.path.basename(p))]
    n = prune(runs, keep)
    n += prune(glob.glob(os.path.join(r, "go_logs", "*.txt")), 30)
    n += prune(glob.glob(os.path.join(r, "p5_snaps", "p5_*.npz")), 20)
    n += prune(glob.glob(os.path.join(r, "p5_snaps", "p5_*_replay.png")), 20)
    p5r = [p for p in glob.glob(os.path.join(home, "p5_runs", "*")) if os.path.isdir(p) and not os.path.islink(p)]
    n += prune(p5r, keep)
    if n:
        print("tidy: %d old run file(s)/folder(s) removed (newest kept)" % n)
    return n


if __name__ == "__main__":
    main()
