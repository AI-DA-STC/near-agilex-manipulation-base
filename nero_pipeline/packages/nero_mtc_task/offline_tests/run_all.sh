#!/bin/bash
# Offline tests for mtc_pick_and_place.py (no ROS, no robot: ROS and MoveIt are replaced by stand-ins in stubs/).
# They check the code and the numbers (stage structure, retiming, place-height geometry), NOT MoveIt's planning.
# Needs Python 3 with numpy. TEST_PYTHON=/path/to/python picks the interpreter.
cd "$(dirname "$0")" || exit 1
PY="${TEST_PYTHON:-python3}"; fail=0
for t in smoke_test.py retime_test.py; do
  if "$PY" "$t" > /tmp/_mtc_test.out 2>&1; then echo "PASS  $t"; else echo "FAIL  $t"; tail -15 /tmp/_mtc_test.out; fail=1; fi
done
[ $fail -eq 0 ] && echo "ALL TESTS PASSED" || echo "SOME TESTS FAILED"; exit $fail
