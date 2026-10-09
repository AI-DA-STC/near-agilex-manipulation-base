# Our five ROS packages (snapshot)

`nero_mtc_task`, `gsam2_ros`, `contact_graspnet_ros`, `target_pose_ros`, `pose_transform_ros`.

These are the working copies from the lab machine (`~/agx_arm_ws/src/`), copied when this repository was assembled. On that machine they
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
