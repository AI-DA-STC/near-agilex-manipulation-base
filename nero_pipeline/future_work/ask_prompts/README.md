# Ask for the prompts at the start of every run

**Status: built and tested with stand-ins; not installed and not used on the lab machine.** It was written on 2026-10-14 and the operator decided to move on to other work first.

## What it does

`ask_prompts.py` is called at the very start of `run_pick_and_place.sh` (before the arm moves). It shows the last prompts and waits for one answer:

```
=== Prompts for Grounded-SAM-2 ===
  source (pick):  a green cube.
  target (place): plate.
Enter or y = keep these.   To change:  s:<source> t:<target>   e.g.  s:red mug t:blue tray   (or only s:red mug)
>
```

| Answer | Effect |
|---|---|
| Enter, `y`, `yes` | keep the last prompts |
| `s:red mug t:blue tray` | change both (either order; `s:` = source/pick, `t:` = target/place) |
| `s:red mug` or `t:blue tray` | change only that one; the other stays |
| anything else | explained; asked again (3 tries, then the last prompts are kept) |

* Prompts are lowercased and given a trailing `.` (the gsam2 node does not add the period; Grounding DINO expects it). Symbols other than letters, digits, spaces and `, ' / ( ) . _ -` are refused.
* It sets the prompts on the running node with `ros2 param set /gsam2_segmentation_node source_prompt ...` (and `target_prompt`), and remembers the last prompts in `~/.nero_last_prompts.json`, also across restarts of the pipeline (after a restart the node forgets them; `y` puts the remembered ones back).
* It asks **only in a real terminal**. Scripts, loops and pipes skip it silently and keep working. `ASK_PROMPTS=0 ./run_pick_and_place.sh` skips it on purpose (the node's prompts stay untouched); `ASK_PROMPTS=force` asks even when stdin is not a terminal (used by the tests).
* If the node cannot be reached it prints a clear message and exits with an error, so `run_pick_and_place.sh` stops **before** the arm moves.

## Install (not done yet)

```bash
F=~/nero_pipeline_launch/run_pick_and_place.sh
cp "$F" "$F.bak_prompts_$(date +%H%M%S)"
cp ~/near-agilex-manipulation-base/nero_pipeline/future_work/ask_prompts/ask_prompts.py ~/nero_pipeline_launch/scripts/ask_prompts.py
grep -q ask_prompts.py "$F" || sed -i '0,/^set -e$/s@^set -e$@set -e\n\n# ask for the source/target prompts (skipped when not interactive or ASK_PROMPTS=0)\n/usr/bin/python3 "$HOME/nero_pipeline_launch/scripts/ask_prompts.py" || exit 1@' "$F"
bash -n "$F" && echo "run_pick_and_place.sh syntax OK"
grep -n -B1 -A1 "ask_prompts" "$F"
```

The block adds one line after `set -e` and is safe to run twice. On the repository's copy of the run script it was the only change (checked with `diff`).

## Tests

```bash
python3 future_work/ask_prompts/tests/test_ask_prompts.py      # needs only Python 3; uses a fake ros2 in a temporary folder
```

They cover the answer parsing, remembering across restarts, bad answers, end of input, skipping when not interactive and an unreachable node. They do **not** run against the real node.

## Things to know before using it

* The tuning of this pipeline (re-centring, closing depth 0.006 m) was done for a roughly 5 cm box ([docs/11](../../docs/11_perception_workers_frames_recentring.md) sections 3 and 8). Another object may need other values.
* The plate-height logic assumes a flat target with a floor, like a plate; a deep bowl or a tray needs a look first.
* If Grounded-SAM-2 does not find the object, the run stops at step 1 (`Source (...) failed`) and the arm does not move.
