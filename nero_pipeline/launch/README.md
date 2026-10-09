# Launch folder

On the lab machine this folder is used as `~/nero_pipeline_launch` (all scripts use absolute paths under it):

```bash
ln -s <repo>/nero_pipeline/launch ~/nero_pipeline_launch
chmod +x ~/nero_pipeline_launch/*.sh ~/nero_pipeline_launch/scripts/*.sh
```

| File | What it does |
|---|---|
| `start_pipeline.sh` | Starts the arm (simulation, or real when `REAL_ARM=1`), the table collision object, the top camera, the calibration publisher and all perception nodes. Per-node logs in `~/nero_pipeline_logs/<timestamp>/`. Ctrl+C stops everything it started |
| `run_pick_and_place.sh` | Step 0 start-pose move, steps 1 to 4 perception (stops if a step answers `success=False`), step 5 MTC planning. `MTC_EXECUTE=y` also executes |
| `stop_arm.sh` | Software stop for the real arm (alias `STOP`), [docs/05](../docs/05_run_real_arm.md) |
| `ckpt.sh` | `save <name>`, `restore <name>`, `list`: named snapshots of the working state so a known-good state can be restored (the header of the script says exactly what it stores) |
| `scripts/launch_arm.sh` | Simulation arm: MoveIt demo with the corrected gripper frame and `ExecuteTaskSolutionCapability` |
| `scripts/launch_arm_real.sh` | Real arm: driver, wait for feedback, MoveIt with `follow:=true` |
| `scripts/add_table_collision.py` | Adds the table box to the planning scene (top face at `TABLE_TOP_Z`, default -0.10 m). Run with `/usr/bin/python3` |
| `scripts/move_to_pose_90deg.py` | Moves the arm to the start pose `pose_90deg`; skips if already there. Reads `JOINT_STATES_TOPIC`, takes `HOME_MOVE_SECONDS` |
| `scripts/approach_monitor.py` | Prints, at the moment the gripper closes and releases, how far the fingers point below/above horizontal and the `tcp_link` height |
| `scripts/check_tcp_in_mtc.py` | Checks that MTC's own robot model contains the corrected `tcp_joint` (the bug in [docs/07](../docs/07_development_log.md) stage 4) |
| `tools/` | Diagnostic helpers, see `tools/README.md` |

Environment variables read by these scripts are listed in `docs/05_run_real_arm.md` (section 6) and `docs/06_tuning_reference.md`.

Files that are collected from the lab machine by `maintainer/collect_from_workspace.sh` (they contain our final hand edits):
`start_pipeline.sh`, `run_pick_and_place.sh`, `ckpt.sh`, `scripts/add_table_collision.py`, `scripts/move_to_pose_90deg.py`,
`scripts/approach_monitor.py`, `scripts/check_tcp_in_mtc.py`.
