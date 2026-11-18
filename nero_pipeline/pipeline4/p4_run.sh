#!/bin/bash
call_checked() { out=$(ros2 service call "$1" std_srvs/srv/Trigger 2>&1); echo "$out"; if echo "$out" | grep -q "success=False"; then echo "!!! $1 FAILED - stopping so the previous detection is not reused"; exit 1; fi; }
# PIPELINE 3 (GraspMolmo + Contact-GraspNet). A copy of run_pick_and_place.sh (pipeline 1): the same steps and services, with step 1 = GraspMolmo
# instead of Grounded-SAM-2. Needs start_pipeline3.sh running. The step lines are written so p3_go.py can time each step.
#   run_pick_and_place3.sh            plan + execute exactly like pipeline 1 (MTC_EXECUTE from your environment)
#   run_pick_and_place3.sh plan       plan only          run_pick_and_place3.sh execute   plan and execute (MTC_EXECUTE=y)
#   ... plan --last / execute --last  re-plan with the grasps and place spot ALREADY in pose_transform (no new GraspMolmo / Contact-GraspNet):
#                                     a cheap retry after "no IK found" (MoveIt's IK search is random, the next try often works)
#
# Runs the full perception chain, then immediately runs task planning -
# combined into one script specifically to avoid the "topic published
# before the next node subscribed" timing gap that happens when these are
# run as separate manual commands with any delay between them.
#
# Requires start_pipeline.sh to already be running (all nodes up).
#
# Usage:
#   chmod +x run_pick_and_place.sh
#   ./run_pick_and_place.sh

set -e
P3_RUN_VERSION="p3-run-v8"
P4_RUN_VERSION="p4-run-v1"   # PIPELINE 4: core/ copies + the wrist obstacle step (p4_make_core.py)
echo "p4_run $P4_RUN_VERSION"
echo "run_pick_and_place3 $P3_RUN_VERSION"
case "${1:-}" in
    plan) unset MTC_EXECUTE ;;
    execute) export MTC_EXECUTE=y ;;
    "") ;;
    *) echo "usage: run_pick_and_place3.sh [plan|execute] [--last]"; exit 2 ;;
esac
LAST=0
case "${2:-}" in
    --last) LAST=1 ;;
    "") ;;
    *) echo "usage: run_pick_and_place3.sh [plan|execute] [--last]"; exit 2 ;;
esac

echo "=== 0: moving the arm to the start pose (${START_POSE:-initial}) ==="
/usr/bin/python3 "$HOME/nero_pipeline_launch/scripts/move_to_pose_90deg.py" || { echo "start-pose move failed"; exit 1; }

source /opt/ros/humble/setup.bash
source "$HOME/agx_arm_ws/install/setup.bash"

if [ "$LAST" = 1 ]; then
echo "=== reusing the latest perception: the grasps and the place spot already in pose_transform (no new GraspMolmo / Contact-GraspNet). The objects must not have moved. ==="
else
echo "# 1/4  GraspMolmo: grasp point and object region (about 25 s)"
call_checked /graspmolmo/segment

echo ""
echo "# 2/4  Contact-GraspNet: grasps on that object (GraspMolmo answers the place question and finds the obstacles meanwhile)"
call_checked /contact_graspnet/detect_grasps

echo ""
echo "# 3/4  place point and obstacles (into MoveIt's planning scene), then the plate point in 3D"
# v7: a time limit, so a GraspMolmo node that died can never make the run wait forever
rc=0; out=$(timeout "${P3_PLACE_TIMEOUT_S:-120}" ros2 service call /graspmolmo/place std_srvs/srv/Trigger 2>&1) || rc=$?; echo "$out"
if [ "$rc" = 124 ]; then
    echo "!!! /graspmolmo/place did not answer within ${P3_PLACE_TIMEOUT_S:-120} s - the GraspMolmo node has probably stopped. Its log: ~/nero_pipeline_logs/<latest p3_...>/graspmolmo.log"
    exit 1
fi
if echo "$out" | grep -q "success=False"; then echo "!!! /graspmolmo/place FAILED - stopping so the previous detection is not reused"; exit 1; fi
call_checked /target_pose/extract

echo ""
echo "=== 2/3: transforming both poses into base_link ==="
call_checked /pose_transform/transform_all

# PIPELINE 4: the wrist camera's 3D obstacle boxes (the arm is at its start pose and still), so MoveIt plans with BOTH cameras
echo ""
if [ "${P4_WRIST:-on}" != off ]; then
echo "=== 2b/3: wrist camera obstacles: 3D boxes at their real height, into MoveIt ==="
call_checked /p4/wrist_obstacles
else
echo "=== 2b/3: P4_WRIST=off: wrist obstacle boxes removed (top camera only) ==="
call_checked /p4/clear_wrist_obstacles
fi
fi

echo ""
echo "=== 3/3: what MoveIt received: the grasp and the place spot (picture opens) ==="
set +e
/usr/bin/python3 "$HOME/nero_pipeline_launch/pipeline4/core/p3_picture.py"
PIC_RC=$?
set -e
if [ "$PIC_RC" = 3 ]; then
    echo "STOP: a fingertip of a grasp MoveIt may use would come too close to the table (see the FINGERS line above). Nothing was planned."
    echo "      Usual cause: the object is low and the grasp goes 6 mm below its top (CGN_DEPTH). Re-run p3_look.sh, or start with a smaller CGN_DEPTH."
    exit 1
elif [ "$PIC_RC" != 0 ]; then
    echo "(the picture could not be made - the run continues)"
fi

echo ""
# v6: pipeline 3's own MTC (a copy of pipeline 2's): the release yaw follows the grasp and the SMOOTHEST solution is executed, so the wrist
# no longer turns ~180 deg. It tries the relay's grasps in order (the picture's G1 first; the others only if G1 cannot be planned).
# P3_MTC=pipeline1 uses pipeline 1's MTC instead.
export P2_GRASP_FALLBACKS="${P2_GRASP_FALLBACKS:-9}"
if [ "${P3_MTC:-p3}" = pipeline1 ]; then MTC_NAME="pipeline 1's MTC"; else MTC_NAME="pipeline 3's MTC: release yaw follows the grasp, smoothest solution"; fi
echo "=== MoveIt Task Constructor: plan$([ "${MTC_EXECUTE:-}" = y ] && echo " AND EXECUTE") ($MTC_NAME) ==="
# known harmless MoveIt/OMPL noise ('carrot' attach messages, planner defaults) is hidden; P3_LOG_FILTER=0 shows everything
if [ "${P3_LOG_FILTER:-1}" = 0 ]; then LOG_FILTER=(cat); else LOG_FILTER=(/usr/bin/python3 "$HOME/nero_pipeline_launch/pipeline4/core/p3_logfilter.py"); fi
if [ "${P3_MTC:-p3}" = pipeline1 ]; then
    ros2 launch nero_mtc_task mtc_pick_and_place.launch.py arm_type:=nero effector_type:=agx_gripper tcp_offset:="[0.037, 0, -0.0235, -1.5708, 0, -1.5708]" 2>&1 | "${LOG_FILTER[@]}"
else
    ros2 launch "$HOME/nero_pipeline_launch/pipeline4/core/p3_mtc.launch.py" arm_type:=nero effector_type:=agx_gripper tcp_offset:="[0.037, 0, -0.0235, -1.5708, 0, -1.5708]" 2>&1 | "${LOG_FILTER[@]}"
fi
