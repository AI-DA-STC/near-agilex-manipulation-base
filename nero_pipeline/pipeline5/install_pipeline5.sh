#!/bin/bash
# PIPELINE 5 installer: copies this folder to ~/nero_pipeline_launch/pipeline5 and runs every offline test. Reads/changes nothing of pipeline 3/4,
# except ONE optional first-time copy: your camera offsets ~/.p4_offsets.yaml -> ~/.p5_offsets.yaml (only if the p5 file does not exist yet).
P5_INSTALL_VERSION="p5-install-v13e"
echo "install_pipeline5 $P5_INSTALL_VERSION"
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; DST="$HOME/nero_pipeline_launch/pipeline5"
if [ "$SRC" != "$DST" ]; then
  rm -rf "$DST.new" && mkdir -p "$DST.new" && cp -r "$SRC"/. "$DST.new"/ || { echo "STOP: copy failed"; exit 1; }
  rm -rf "$DST" && mv "$DST.new" "$DST"
fi
find "$DST" -name "*.sh" -o -name "*.py" | xargs chmod +x
mkdir -p "$HOME/p3_runs/p5_snaps"
if [ ! -f "$HOME/.p5_offsets.yaml" ]; then
  if [ -f "$HOME/.p4_offsets.yaml" ]; then cp "$HOME/.p4_offsets.yaml" "$HOME/.p5_offsets.yaml"; echo "camera offsets: copied ~/.p4_offsets.yaml -> ~/.p5_offsets.yaml (pipeline 5 uses only its own file from now on):"
  else echo "camera offsets: no ~/.p5_offsets.yaml and no ~/.p4_offsets.yaml -> offsets 0 (pipeline 4 used wrist_offset_z: -0.014)"; fi
fi
[ -f "$HOME/.p5_offsets.yaml" ] && sed 's/^/    /' "$HOME/.p5_offsets.yaml"
[ -f "$HOME/.p5_gripper.yaml" ] && echo "gripper overrides in ~/.p5_gripper.yaml:" && sed 's/^/    /' "$HOME/.p5_gripper.yaml"
echo "versions:"; cat "$DST/VERSIONS.txt" | sed 's/^/    /'
bash "$DST/tests/run_tests.sh"
