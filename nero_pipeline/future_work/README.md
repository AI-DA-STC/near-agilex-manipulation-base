# Future work

Things that are **built but not in use**, or designed but not built. Nothing in this folder is needed to run the pipeline, and nothing here has been used on the lab machine.

| Item | State | Where |
|---|---|---|
| Prompt question at the start of every run (`y` keeps the last prompts, `s:red mug t:blue tray` changes them) | **Built and tested with stand-ins** (27 checks); **not installed or used** on the lab machine | [`ask_prompts/`](ask_prompts/README.md) |
| Several roll angles per grasp candidate (a compact object can be grasped at any roll), more candidates, or a geometric top-down candidate for boxes | Idea only. `launch/tools/ik_probe.py` already sweeps the roll angle and shows which rolls MoveIt can reach | [docs/09](../docs/09_known_issues_and_next_steps.md) items 20 and 21 |
| A guard that refuses an unregistered depth image (instead of resizing the mask) | Idea only | [docs/09](../docs/09_known_issues_and_next_steps.md) item 22 |
| Table box built exactly (top face where you say, front of the arm only) as part of `start_pipeline.sh` | The tool exists (`launch/scripts/add_table_box.py`); it is not called by the start script | [docs/09](../docs/09_known_issues_and_next_steps.md) item 17 |

The full list with priorities is in [docs/09](../docs/09_known_issues_and_next_steps.md).
