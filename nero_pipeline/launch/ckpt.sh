#!/bin/bash
# usage: ./ckpt.sh save <name>   |   ./ckpt.sh restore <name>   |   ./ckpt.sh list
D=~/nero_checkpoints
FILES=(
  ~/agx_arm_ws/src/nero_mtc_task/nero_mtc_task/mtc_pick_and_place.py
  ~/agx_arm_ws/src/nero_mtc_task/launch/mtc_pick_and_place.launch.py
  ~/nero_pipeline_launch/run_pick_and_place.sh
  ~/nero_pipeline_launch/start_pipeline.sh
  ~/nero_pipeline_launch/mode.sh
  ~/nero_pipeline_launch/scripts/launch_arm.sh
  ~/nero_pipeline_launch/scripts/add_table_collision.py
  ~/nero_pipeline_launch/scripts/move_to_pose_90deg.py
)
case "$1" in
  save)    mkdir -p $D/$2; for f in "${FILES[@]}"; do [ -f "$f" ] && cp "$f" "$D/$2/$(echo "$f" | tr '/' '_')"; done; echo "saved: $2" ;;
  restore) for f in "${FILES[@]}"; do s="$D/$2/$(echo "$f" | tr '/' '_')"; [ -f "$s" ] && cp "$s" "$f"; done
           echo "restored: $2 - now rebuild:"
           echo "rm -rf ~/agx_arm_ws/build/nero_mtc_task ~/agx_arm_ws/install/nero_mtc_task; cd ~/agx_arm_ws && colcon build --symlink-install --packages-select nero_mtc_task && source install/setup.bash" ;;
  list)    ls $D ;;
  *) echo "usage: $0 save|restore|list <name>" ;;
esac
