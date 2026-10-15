#!/bin/bash
# Offline tests (no ROS needed). TEST_PYTHON picks the interpreter (needs numpy).
cd "$(dirname "$0")" || exit 1
PY="${TEST_PYTHON:-python3}"; fail=0
for t in test_plate.py node_test.py; do
  if "$PY" "$t" > /tmp/_tf.out 2>&1; then echo "PASS  $t"; else echo "FAIL  $t"; tail -15 /tmp/_tf.out; fail=1; fi
done
[ $fail -eq 0 ] && echo "ALL TESTS PASSED" || echo "SOME TESTS FAILED"; exit $fail
