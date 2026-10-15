#!/bin/bash
# Offline tests (no ROS, no GPU, no real models: stand-in models simulate slow loading, no-detection prompts, crashes and timeouts).
# Needs Python 3 with numpy, opencv-python and pillow. Use TEST_PYTHON=/path/to/python to choose the interpreter.
cd "$(dirname "$0")" || exit 1
PY="${TEST_PYTHON:-python3}"; export TEST_PYTHON="$PY"; fail=0
for t in test_worker.py test_node_gsam2.py; do
  if timeout 250 "$PY" "$t" > /tmp/_gt.out 2>/dev/null; then echo "PASS  $t"; else echo "FAIL  $t"; tail -12 /tmp/_gt.out; fail=1; fi
done
[ $fail -eq 0 ] && echo "ALL TESTS PASSED" || echo "SOME TESTS FAILED"; exit $fail
