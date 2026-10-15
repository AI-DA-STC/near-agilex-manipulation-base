# Our five ROS packages (snapshot)

`nero_mtc_task`, `gsam2_ros`, `contact_graspnet_ros`, `target_pose_ros`, `pose_transform_ros`.

These are the working copies from the lab machine (`~/agx_arm_ws/src/`). They are refreshed by `maintainer/sync_packages_from_workspace.sh`, so they hold what is really installed. On that machine they
were plain folders, not git repositories, so this snapshot is how the exact code that produced the results is preserved.
Build output, caches and backups were left out.

To use them:

```bash
cp -r packages/* ~/agx_arm_ws/src/
cd ~/agx_arm_ws && colcon build --symlink-install --packages-select gsam2_ros contact_graspnet_ros target_pose_ros pose_transform_ros nero_mtc_task
source install/setup.bash
```

If you also keep these packages in their own repositories, treat those as the authoritative history and this folder as the snapshot that
matches the documentation. Model weights are not included (see `docs/02_install_and_environment.md`, sections 2.6 and 2.7).

## What is new here (2026-10-14)

| Package | Added |
|---|---|
| `gsam2_ros` | `gsam2_ros/worker_client.py`, `scripts/grounded_sam2_worker.py`; the node uses a persistent worker with automatic fallback ([docs/11](../docs/11_perception_workers_frames_recentring.md) section 1) |
| `contact_graspnet_ros` | `worker_client.py`, `recentre.py`, `scripts/contact_graspnet_worker.py`; `forward_passes`, `retries_if_empty`, `recentre_*` parameters; poses labelled with the depth `camera_info` frame |
| `target_pose_ros` | `plate_geometry.py` (centre and height estimates); plate pose labelled with the depth `camera_info` frame |
| `nero_mtc_task` | The 2026-10-12 update of `mtc_pick_and_place.py` ([docs/06](../docs/06_tuning_reference.md) section 5.4) |

The nodes start helper scripts from outside the package (`~/gsam2_ros_scripts/`, `~/contact_graspnet_ros_scripts/`): copy `scripts/*.py` there ([docs/12](../docs/12_current_run_sequence.md) part 1 does it).

## Offline tests

Every package except `pose_transform_ros` has an `offline_tests/` folder with tests that need no ROS, GPU or robot (stand-ins replace them). They check our code and numbers, **not** MoveIt's planning or the real models.
Run all of them: `bash maintainer/run_offline_tests.sh` (needs python3 with numpy, opencv-python and pillow). The `offline_tests/` folders exist only in this repository: the sync script does not copy them to or from the workspace.
