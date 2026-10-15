#!/bin/bash
# run_offline_tests.sh - runs every offline test suite in this folder. No ROS, no GPU, no robot: ROS, MoveIt, torch and the models are replaced by small
# stand-ins, so these tests check OUR code and numbers, NOT MoveIt's planning and NOT the real models.
#     maintainer/run_offline_tests.sh                       (uses python3)
#     TEST_PYTHON=/path/to/python maintainer/run_offline_tests.sh
# The test interpreter needs numpy, opencv-python (cv2) and pillow. If a suite FAILs with ModuleNotFoundError: pip install --user numpy opencv-python pillow
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${TEST_PYTHON:-python3}"; export TEST_PYTHON="$PY"
FAIL=0; RAN=0
for d in "$HERE"/packages/*/offline_tests; do
  [ -f "$d/run_all.sh" ] || continue
  name="${d#$HERE/packages/}"; name="${name%/offline_tests}"; RAN=$((RAN+1))
  if out=$(cd "$d" && bash run_all.sh 2>&1); then echo "PASS  $name"; else echo "FAIL  $name"; echo "$out" | tail -12 | sed 's/^/      /'; FAIL=$((FAIL+1)); fi
done
for t in "$HERE"/future_work/*/tests/test_*.py; do
  [ -f "$t" ] || continue
  RAN=$((RAN+1)); name="future_work/${t#$HERE/future_work/}"
  if out=$("$PY" "$t" 2>&1); then echo "PASS  $name"; else echo "FAIL  $name"; echo "$out" | tail -12 | sed 's/^/      /'; FAIL=$((FAIL+1)); fi
done
find "$HERE" -name __pycache__ -type d -prune -exec rm -rf {} + 2>/dev/null
echo; echo "== $RAN suites run, $FAIL failed =="
[ "$FAIL" -eq 0 ]
