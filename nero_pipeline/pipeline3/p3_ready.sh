#!/bin/bash
# PIPELINE 3, STEP 1b (once per session, in Terminal 2, after start_pipeline3.sh is up): is everything ready?
#   joint states of the arm, MoveIt's table box (exact, top at z = 0.000), GraspMolmo loaded.   Nothing moves.
P3_READY_VERSION="p3-ready-v5"
echo "p3_ready $P3_READY_VERSION"
source /opt/ros/humble/setup.bash 2>/dev/null; source "$HOME/agx_arm_ws/install/setup.bash" 2>/dev/null
BAD=0
TOPICS=$(ros2 topic list 2>/dev/null)
if echo "$TOPICS" | grep -qx "/feedback/joint_states"; then ARM="REAL"; JS=/feedback/joint_states; else ARM="SIMULATED"; JS=/control/joint_states; fi
echo "arm: $ARM (from ROS: /feedback/joint_states is published only by the real arm's driver)"
if timeout 8 ros2 topic echo "$JS" --once >/dev/null 2>&1; then echo "  ok        $JS is publishing"; else echo "  PROBLEM   no message on $JS within 8 s - is the arm running? (Terminal 1)"; BAD=1; fi
# the exact table box: your lab copy first, else the copy bundled with pipeline 3 (identical to the repo's scripts/add_table_box.py)
T=""
# pipeline 3's own copy FIRST: it reads MoveIt 2's scene correctly (the original prints a wrong centre and top face, see its world_centre)
for c in "$HOME/nero_pipeline_launch/pipeline3/add_table_box.py" "$HOME/nero_pipeline_launch/scripts/add_table_box.py" "$HOME/m2t2_tools/add_table_box.py"; do
    if [ -f "$c" ]; then T="$c"; break; fi           # in THIS order (ls would sort them alphabetically)
done
[ -n "$T" ] && echo "  table box tool: $T"
# v4: the REAL table height (the robot stands on a plate, the table is lower), shared by all pipeline 3 tools (p3_tablez.py)
TZ=$(/usr/bin/python3 -c "import sys; sys.path.insert(0, '$(dirname "$0")'); import p3_tablez as t; v, s = t.table_z(); print('%.4f|%+.1f|%s' % (v, 1000 * v, s))")
TZV=${TZ%%|*}; REST=${TZ#*|}; TZMM=${REST%%|*}; TZS=${REST#*|}
echo "  table     real table height: $TZMM mm  (from $TZS)"
if [ -n "$T" ]; then
    OUT=$(/usr/bin/python3 "$T" --top_z "$TZV" 2>&1); echo "$OUT" | tail -2 | sed 's/^/  table     /'
    WANT=$(printf 'z = %+.3f' "$TZV")
    echo "$OUT" | grep -q -- "$WANT" || { echo "  PROBLEM   the table box did not report its top at $WANT"; BAD=1; }
else
    echo "  PROBLEM   add_table_box.py not found (not in ~/nero_pipeline_launch/scripts, pipeline3/ or ~/m2t2_tools)"; BAD=1
fi
L="$HOME/nero_pipeline_logs/$(ls -t "$HOME/nero_pipeline_logs" | grep '^p3_' | head -1)"
printf "  ...       waiting for GraspMolmo to load (about a minute)"
for i in $(seq 1 60); do
    grep -q "worker ready" "$L/graspmolmo.log" 2>/dev/null && break
    if grep -q "worker start-up failed\|Traceback" "$L/graspmolmo.log" 2>/dev/null; then echo ""; echo "  PROBLEM   GraspMolmo failed to start: see $L/graspmolmo.log"; BAD=1; break; fi
    printf "."; sleep 5
done
echo ""
if ! ros2 node list 2>/dev/null | grep -qx "/p3_graspmolmo_node"; then
    echo "  PROBLEM   the GraspMolmo node is NOT running (the log may be from an earlier start): start/restart start_pipeline3.sh"; BAD=1
elif grep -q "worker ready" "$L/graspmolmo.log" 2>/dev/null; then echo "  ok        GraspMolmo loaded (node running)"; else [ "$BAD" = 1 ] || { echo "  PROBLEM   GraspMolmo not ready after 5 min: see $L/graspmolmo.log"; BAD=1; }; fi
echo ""
if [ "$BAD" = 0 ]; then
    echo "READY ($ARM arm). Next, every time you change the objects or the prompt:"
    echo "  bash ~/nero_pipeline_launch/pipeline3/p3_look.sh \"green cube\" \"plate\""
    [ "$ARM" = REAL ] && echo "(Terminal 3: type STOP without pressing Enter, and keep your hand near it.)"
    exit 0
fi
echo "NOT READY: fix the PROBLEM line(s) above first."
exit 1
