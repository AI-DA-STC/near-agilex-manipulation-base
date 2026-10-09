# Push checklist (for the person who maintains this folder)

Do these in order. Nothing here changes your workspace except the git commands you run yourself.
Parts written in angle brackets, such as `<package>` or `<umbrella url>`, are placeholders: replace them with the real value before running the command.

## 0. Before you start

* Be on the lab machine, with the pipeline **stopped**. The arm does not need to be on.
* The folder you are reading is called `nero_pipeline/`. Keep it as one folder; do not rename files inside it (the documents link to each other).

## 1. Collect the final files and the environment snapshot

```bash
cd <wherever nero_pipeline/ is>
bash maintainer/collect_from_workspace.sh
```

Read the output top to bottom:

| Output word | Meaning | What to do |
|---|---|---|
| `NEW` / `UPDATED` | A file was copied from your workspace into this folder | Nothing; `git diff` later shows what changed |
| `same` | Already identical | Nothing |
| `MISSING <path>` | The file does not exist at that path on your machine | If it should exist, find it (`find ~ -name <file>`) and copy it by hand to the destination listed in `launch/README.md`; if it never existed, tell the supervisor in your message |
| `DIFFERS ... NOT copied` | Your lab copy of a tool differs from the tested reference in this folder | Run the `diff` command it prints and decide which is right. The reference copies were tested; the lab copy may be older or contain an edit |
| `uncommitted changes in <name>` | A repository in the workspace has local edits | If `<name>` is one of **our** packages: commit and push it (section 3). If it is a third-party repository (`agx_arm_ros`, ...): the patch in `patches/` is how we preserve the edits |

## 2. Verify the folder

```bash
bash maintainer/verify_repo.sh
python3 maintainer/check_docs.py
```

Both must end with `0 FAIL` / no problems. `WARN` lines are acceptable if you understand them (a missing `usable.sort(` line is a FAIL, not a WARN: it is the edit that made the grasps work, so your script must contain it).

Then **read three things yourself** and correct them if the facts have changed:
1. The status table in `README.md`.
2. The "Last known" values in `docs/06_tuning_reference.md` against the numbers `verify_repo.sh` printed.
3. The state at the end of `docs/07_development_log.md`.

## 3. Push our own packages first

Our packages are separate repositories (submodules of the umbrella repository). For each of
`gsam2_ros`, `contact_graspnet_ros`, `target_pose_ros`, `pose_transform_ros`, `nero_mtc_task`:

```bash
cd ~/agx_arm_ws/src/<package>
git status                    # what changed
git remote -v                 # where it will go
git checkout -b <feature>-<yourname>-dev      # team convention: [feature_name]-[yourname]-dev
git add -A && git status      # check nothing unwanted is staged (no build/, no *.npy, no logs)
git commit -m "<package>: <what changed and why, one line>"
git push -u origin <feature>-<yourname>-dev
```

Suggested messages (one commit per concern keeps the history readable):

| Package | Changes since the last push |
|---|---|
| `nero_mtc_task` | Cartesian approach; continuous place cone; `retime_solution` and direct `execute_task_solution`; candidate tilt filter, sort and `to_tcp_pose`; `GRASP_OFFSET_*`; hang-on-failure fix |
| `contact_graspnet_ros` | `forward_passes=5`; saves `/tmp/last_depth.npy` and `/tmp/last_segmap.npy` on every detection |
| `gsam2_ros` | source/target prompts; `box_threshold` |
| `pose_transform_ros`, `target_pose_ros` | (only if they changed; check `git status`) |

If a package has no remote yet, create the repository on GitHub first, then `git remote add origin <url>`.

## 4. Push this folder to the umbrella repository

```bash
cd ~/near-agilex-manipulation-base
git status                                  # start clean
git checkout -b nero-pipeline-docs-<yourname>-dev
cp -r <path to>/nero_pipeline .             # or unzip here; the folder must end up at <umbrella>/nero_pipeline/
# update the submodule pointers to the commits you just pushed:
git submodule status                        # find the submodule directories
#   for each of our packages:  cd <submodule dir> && git checkout <feature>-<yourname>-dev && cd -
git add nero_pipeline <submodule dirs>
git status                                  # review once more
git commit -m "Add nero_pipeline: launch scripts, tools, documentation and environment snapshot"
git push -u origin nero-pipeline-docs-<yourname>-dev
```

Then open a pull request into `main` on GitHub. In the description paste the **Status at a glance** table from `nero_pipeline/README.md`.
Add one line to the umbrella repository's own `README.md` linking to `nero_pipeline/README.md`.

## 5. Safety audit before you push

```bash
cd <umbrella or nero_pipeline>
git ls-files | xargs -I{} du -k {} 2>/dev/null | sort -rn | head -5       # nothing unexpectedly large
git grep -n -i -E "password|passwd|token|secret|api[_-]?key" -- . ':!docs' | head            # should print nothing sensitive
git ls-files | grep -E "\.(npy|log|bak|orig)$|__pycache__" || echo "no scratch files tracked"
```

## 6. Prove it works from a clean clone

On a different directory (or machine):

```bash
git clone --recurse-submodules <umbrella url> /tmp/clonetest && cd /tmp/clonetest
git checkout nero-pipeline-docs-<yourname>-dev
bash nero_pipeline/maintainer/verify_repo.sh
```

It must print `0 FAIL`. Then follow [docs/02](../docs/02_install_and_environment.md) on a spare machine if one is available, and note anything that did not work in [docs/09](../docs/09_known_issues_and_next_steps.md).

## 7. Licence and privacy

* **Licence.** A public repository with no licence is readable but **not** open source: by default nobody is allowed to reuse or modify it. The licence does not have to be a new file:
  files inside a repository are covered by the licence at its root. Check the umbrella repository:

  ```bash
  ls ~/near-agilex-manipulation-base/LICENSE*
  ```

  If a file is listed, nothing to do. If not, add one on GitHub (Add file, Create new file, name it `LICENSE`, click "Choose a license template"; MIT is a common permissive choice).
  Third-party components (AgileX packages, Contact-GraspNet, Grounded-SAM2, ...) keep their own licences.
* **Privacy (the repository is public).** `collect_from_workspace.sh` already removes your home directory path, the host name and the conda prefix from the generated files, and
  `verify_repo.sh` **fails** if your user name appears anywhere in the folder (step 2). If it lists a file, replace `/home/<you>` with `$HOME` in that file and run it again.
* The development log is candid on purpose (it lists wrong turns). Keep it that way; it saves the next person time.

## 8. What to tell the supervisor

Send the link to `nero_pipeline/README.md` and say: *read the Status table, then [docs/07](../docs/07_development_log.md) and [docs/09](../docs/09_known_issues_and_next_steps.md); run [docs/04](../docs/04_run_simulation.md) in simulation first; do not move the real arm before reading [docs/05](../docs/05_run_real_arm.md).*
