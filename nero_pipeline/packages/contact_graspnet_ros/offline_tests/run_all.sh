#!/bin/bash
# Offline tests (no ROS, no GPU, no real model: stand-ins simulate slow loading, empty predictions, crashes and timeouts; the worker's output
# is compared with the ORIGINAL contact_graspnet_inference.py on the same input). Needs Python 3 with numpy.
cd "$(dirname "$0")" || exit 1
PY="${TEST_PYTHON:-python3}"; export TEST_PYTHON="$PY"; fail=0
for t in test_recentre.py test_cgn_worker.py test_cgn_node.py test_launch.py; do
  if timeout 280 "$PY" "$t" > /tmp/_ct.out 2>/dev/null; then echo "PASS  $t"; else echo "FAIL  $t"; tail -14 /tmp/_ct.out; fail=1; fi
done
[ $fail -eq 0 ] && echo "ALL TESTS PASSED" || echo "SOME TESTS FAILED"; exit $fail
