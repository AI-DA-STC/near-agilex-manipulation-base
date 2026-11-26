#!/bin/bash
# runs every pipeline 5 offline test (no ROS, no robot)
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; PY="${P5_TEST_PYTHON:-/usr/bin/python3}"; BAD=0
for t in test_p5_bundle.py test_p5_grasp_core.py test_p5_node.py test_p5_keepout.py test_p5_place.py test_p5_run.py test_p5_scene_objects.py test_p5_fuse.py test_p5_wrist.py; do
  out=$("$PY" "$D/$t" 2>&1); rc=$?
  echo "$(echo "$out" | tail -1)    [$t]"
  [ $rc = 0 ] || { BAD=1; echo "$out" | grep -E "FAIL|ERROR|Traceback|Error" | head -8 | sed 's/^/      /'; }
done
[ $BAD = 0 ] && echo "PIPELINE 5 OFFLINE TESTS: ALL OK" || { echo "PIPELINE 5 OFFLINE TESTS: FAILURES (above)"; exit 1; }
