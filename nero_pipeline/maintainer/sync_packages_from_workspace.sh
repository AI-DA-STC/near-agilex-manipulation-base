#!/bin/bash
# sync_packages_from_workspace.sh  -  run ON THE LAB MACHINE, after collect_from_workspace.sh.
# Refreshes packages/<package>/ with the files that are REALLY installed in the ROS workspace (~/agx_arm_ws/src/<package>/), so the snapshot in this
# folder is what runs, not a copy somebody made by hand. It only READS the workspace and only ADDS or OVERWRITES files inside packages/; it never deletes.
# It skips build products, caches, backups and the offline_tests/ folders (those exist only in this repository).
#     maintainer/sync_packages_from_workspace.sh
# Overridable:  WS (ROS workspace, default ~/agx_arm_ws)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${WS:-$HOME/agx_arm_ws}"
PKGS="gsam2_ros contact_graspnet_ros target_pose_ros pose_transform_ros nero_mtc_task"
NEW=0; UPD=0; SAME=0; MISS=0
for p in $PKGS; do
  src="$WS/src/$p"
  if [ ! -d "$src" ]; then echo "MISSING   $src  (package not found in the workspace)"; MISS=$((MISS+1)); continue; fi
  echo "== $p =="
  while IFS= read -r f; do
    rel="${f#$src/}"; dest="$HERE/packages/$p/$rel"
    if [ ! -f "$dest" ]; then mkdir -p "$(dirname "$dest")"; cp -p "$f" "$dest"; echo "NEW       packages/$p/$rel"; NEW=$((NEW+1))
    elif cmp -s "$f" "$dest"; then SAME=$((SAME+1))
    else cp -p "$f" "$dest"; echo "UPDATED   packages/$p/$rel   (workspace copy differed; review with: git diff)"; UPD=$((UPD+1)); fi
  done < <(find "$src" -type f \( -name __pycache__ -o -name '*.pyc' -o -name '*.bak*' -o -name '*.orig' -o -name '*.new' -o -name '*.npy' -o -name '*.log' -o -name '*.swp' -o -name '*~' \) -prune -o \
                   -type f -not -path '*/.git/*' -not -path '*/build/*' -not -path '*/install/*' -not -path '*/log/*' -not -path '*/offline_tests/*' -not -path '*.egg-info/*' -print | sort)
done
echo; echo "== helper scripts that the nodes start from outside the package (installed copies must match the package copies) =="
cmpinst() {   # cmpinst INSTALLED PACKAGE-RELATIVE
  if [ ! -f "$1" ]; then echo "absent    $1  (not installed on this machine)"; return; fi
  if cmp -s "$1" "$HERE/packages/$2"; then echo "same      $1"; else echo "DIFFERS   $1  vs packages/$2   (diff: diff $1 $HERE/packages/$2)"; fi
}
cmpinst "$HOME/gsam2_ros_scripts/grounded_sam2_worker.py"          gsam2_ros/scripts/grounded_sam2_worker.py
cmpinst "$HOME/gsam2_ros_scripts/grounded_sam2_inference.py"       gsam2_ros/scripts/grounded_sam2_inference.py
cmpinst "$HOME/contact_graspnet_ros_scripts/contact_graspnet_worker.py"     contact_graspnet_ros/scripts/contact_graspnet_worker.py
cmpinst "$HOME/contact_graspnet_ros_scripts/contact_graspnet_inference.py"  contact_graspnet_ros/scripts/contact_graspnet_inference.py
echo; echo "== Summary =="; echo "new: $NEW   updated: $UPD   unchanged: $SAME   packages missing on this machine: $MISS"
echo "Next: run maintainer/verify_repo.sh and read 'git status' / 'git diff --stat'."
