#!/bin/bash
# Installs PIPELINE 3 (GraspMolmo + Contact-GraspNet) into ~/nero_pipeline_launch, next to pipeline 1. Pipeline 1's own files and pipeline 2
# (~/m2t2_tools) are not touched.
# Usage:  bash ~/Downloads/install_pipeline3.sh        (pipeline3_bundle.zip must be in ~/Downloads; the newest copy is used)
DEST="${DEST:-$HOME/nero_pipeline_launch}"
ZIP=$(ls -t "$HOME"/Downloads/pipeline3_bundle*.zip 2>/dev/null | head -1)
if [ -z "$ZIP" ]; then echo "STOP: no pipeline3_bundle*.zip in ~/Downloads. Download pipeline3_bundle.zip from the chat first."; exit 1; fi
if [ ! -f "$DEST/start_pipeline.sh" ]; then echo "STOP: $DEST/start_pipeline.sh not found (pipeline 1's launch folder). Set DEST=... if it lives elsewhere."; exit 1; fi
TMP=$(mktemp -d)
if ! unzip -o -q "$ZIP" -d "$TMP"; then echo "STOP: could not unzip $ZIP"; exit 1; fi
mkdir -p "$DEST/pipeline3"
cp "$TMP"/pipeline3/* "$DEST/pipeline3/"
cp "$TMP/start_pipeline3.sh" "$TMP/run_pick_and_place3.sh" "$DEST/"
chmod +x "$DEST/start_pipeline3.sh" "$DEST/run_pick_and_place3.sh" "$DEST"/pipeline3/*.py "$DEST"/pipeline3/*.sh
rm -rf "$TMP"
echo "installed from: $ZIP"
echo "into:           $DEST (start_pipeline3.sh, run_pick_and_place3.sh, pipeline3/)"

BAD=0
bad() { BAD=$((BAD + 1)); }
ver() {  # file, regex of the tag line, expected text
    local got; got=$(grep -o -m1 -e "$2" "$DEST/$1" 2>/dev/null)
    if [ -z "$got" ]; then printf "  MISSING   %-34s\n" "$1"; bad
    elif [ "$got" = "$3" ]; then printf "  ok        %-34s %s\n" "$1" "$got"
    else printf "  WRONG     %-34s %s   (expected: %s)\n" "$1" "$got" "$3"; bad; fi
}
echo ""; echo "== versions"
ver start_pipeline3.sh                 'P3_START_VERSION="[^"]*"'     'P3_START_VERSION="p3-start-v3"'
ver run_pick_and_place3.sh             'P3_RUN_VERSION="[^"]*"'       'P3_RUN_VERSION="p3-run-v8"'
ver pipeline3/p3_graspmolmo_node.py    'P3_NODE_VERSION = "[^"]*"'    'P3_NODE_VERSION = "p3-node-v8"'
ver pipeline3/p3_graspmolmo_worker.py  'P3_WORKER_VERSION = "[^"]*"'  'P3_WORKER_VERSION = "p3-worker-v1"'
ver pipeline3/p3_picture.py            'P3_PICTURE_VERSION = "[^"]*"'   'P3_PICTURE_VERSION = "p3-picture-v10"'
ver pipeline3/p3_logfilter.py          'P3_LOGFILTER_VERSION = "[^"]*"' 'P3_LOGFILTER_VERSION = "p3-logfilter-v1"'
ver pipeline3/add_table_box.py         'P3_TABLE_BOX_VERSION = "[^"]*"' 'P3_TABLE_BOX_VERSION = "p3-tablebox-v1"'
ver pipeline3/p3_calib_check.py        'P3_CALIB_CHECK_VERSION = "[^"]*"' 'P3_CALIB_CHECK_VERSION = "p3-calibcheck-v4"'
ver pipeline3/p3_obstacles.py          'P3_OBSTACLES_VERSION = "[^"]*"'  'P3_OBSTACLES_VERSION = "p3-obstacles-v6"'
ver pipeline3/p3_jobs.py               'P3_JOBS_VERSION = "[^"]*"'       'P3_JOBS_VERSION = "p3-jobs-v1"'
ver pipeline3/p3_tablez.py             'P3_TABLEZ_VERSION = "[^"]*"'     'P3_TABLEZ_VERSION = "p3-tablez-v4"'
ver pipeline3/p3_scene.py              'P3_SCENE_VERSION = "[^"]*"'      'P3_SCENE_VERSION = "p3-scene-v2"'
ver pipeline3/p3_mtc.py                'P3_MTC_VERSION = "[^"]*"'        'P3_MTC_VERSION = "p3-mtc-v3"'
ver pipeline3/p3_mtc.launch.py         'P3_MTC_LAUNCH_VERSION = "[^"]*"' 'P3_MTC_LAUNCH_VERSION = "p3-mtclaunch-v1"'
ver pipeline3/p3_objects.py            'P3_OBJECTS_VERSION = "[^"]*"'    'P3_OBJECTS_VERSION = "p3-objects-v1"'
ver pipeline3/p3_grasp_watch.py        'P3_GRASP_WATCH_VERSION = "[^"]*"' 'P3_GRASP_WATCH_VERSION = "p3-graspwatch-v1"'
ver pipeline3/p3_grasp_view.py         'P3_GRASP_VIEW_VERSION = "[^"]*"'  'P3_GRASP_VIEW_VERSION = "p3-graspview-v1"'
ver pipeline3/p3_fingers.py            'P3_FINGERS_VERSION = "[^"]*"'    'P3_FINGERS_VERSION = "p3-fingers-v2"'
ver pipeline3/p3_jaw_fix.py            'P3_JAW_FIX_VERSION = "[^"]*"'   'P3_JAW_FIX_VERSION = "p3-jawfix-v8"'
ver pipeline3/p3_go.py                 'GO_VERSION = "[^"]*"'         'GO_VERSION = "p3-go-v6"'
ver pipeline3/p3_ready.sh               'P3_READY_VERSION="[^"]*"'       'P3_READY_VERSION="p3-ready-v5"'
for f in pipeline3/p3_geometry.py pipeline3/worker_client.py pipeline3/p3_clean.sh pipeline3/p3_look.sh pipeline3/p3_do.sh; do
    if [ -s "$DEST/$f" ]; then printf "  ok        %s\n" "$f"; else printf "  MISSING   %s\n" "$f"; bad; fi
done

echo ""; echo "== your start_pipeline.sh vs start_pipeline3.sh: ONLY GraspMolmo / mask_topic / the start guard / echo lines should differ"
diff "$DEST/start_pipeline.sh" "$DEST/start_pipeline3.sh" | grep '^[<>]' | cut -c1-150
echo ""; echo "== your run_pick_and_place.sh vs run_pick_and_place3.sh: ONLY step 1, the step titles and the plan/execute argument should differ"
diff "$DEST/run_pick_and_place.sh" "$DEST/run_pick_and_place3.sh" | grep '^[<>]' | cut -c1-150

echo ""; echo "== what pipeline 3 reuses from pipeline 1"
for f in scripts/move_to_pose_90deg.py scripts/add_table_collision.py scripts/launch_arm.sh; do
    if [ -e "$DEST/$f" ]; then printf "  ok        %s\n" "$f"; else printf "  MISSING   %s\n" "$f"; bad; fi
done
if [ -e /opt/ros/humble/setup.bash ] && [ -e "$HOME/agx_arm_ws/install/setup.bash" ]; then
    for pkg in contact_graspnet_ros target_pose_ros pose_transform_ros nero_mtc_task; do
        if ( source /opt/ros/humble/setup.bash; source "$HOME/agx_arm_ws/install/setup.bash"; ros2 pkg prefix "$pkg" >/dev/null 2>&1 ); then printf "  ok        ROS package %s\n" "$pkg"; else printf "  MISSING   ROS package %s\n" "$pkg"; bad; fi
    done
else
    echo "  MISSING   /opt/ros/humble/setup.bash or ~/agx_arm_ws/install/setup.bash"; bad
fi

echo ""; echo "== GraspMolmo"
if conda env list 2>/dev/null | awk '{print $1}' | grep -qx graspmolmo || [ -x "$HOME/miniconda3/envs/graspmolmo/bin/python" ]; then echo "  ok        conda env graspmolmo"; else echo "  MISSING   conda env graspmolmo"; bad; fi
if [ -d "$HOME/.cache/huggingface/hub/models--allenai--GraspMolmo" ]; then echo "  ok        GraspMolmo weights"; else echo "  MISSING   GraspMolmo weights in ~/.cache/huggingface/hub"; bad; fi
if /usr/bin/python3 -B -m py_compile "$DEST"/pipeline3/*.py 2>/dev/null; then echo "  ok        pipeline3 python files compile"; else echo "  WRONG     a pipeline3 python file does not compile"; bad; fi
find "$DEST/pipeline3" -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null
if pgrep -f "gm_worker" >/dev/null 2>&1; then echo "  NOTE      pipeline 2's GraspMolmo worker is running: stop pipeline 2 first (only one GraspMolmo fits in memory)"; fi

echo ""
if [ "$BAD" -eq 0 ]; then
    echo "READY. The three steps:  1) start_pipeline3.sh + p3_ready.sh   2) p3_look.sh \"<object>\" \"<place>\"   3) p3_do.sh"
    exit 0
fi
echo "NOT READY: $BAD item(s) above say MISSING or WRONG. Send me this output."
exit 1
