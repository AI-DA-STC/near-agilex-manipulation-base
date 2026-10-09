# 10. Grasp-quality investigation

**Problem.** Contact-GraspNet gives a good set of grasps on its own sample scenes but, on our top-down camera, many proposals are on the wrong place:
thin "sticks poking into" the object, grasps on one side of it, grasps beside it. The real arm then closes the gripper beside the object.
**Outcome (2026-10-08): fixed by candidate selection, not by changing the network.** The operator reports that grasps now work well and credits sorting the candidates
most top-down first (see [docs/07](07_development_log.md) stage 12 and section 7 below). This document is kept as the record of how the problem was approached, what was ruled out,
and what to try first if it comes back. Hypotheses H1 to H4 in section 4 were not needed in the end and remain untested.

## 1. Where a grasp comes from (what to suspect)

```
color + aligned depth (640x480) --> gsam2 mask --> Contact-GraspNet (depth, mask, K) --> grasps in camera frame
   --> pose_transform (TF from calibration) --> to_tcp_pose (frame/finger conversion) --> filter + sort --> MTC --> arm
```

A bad grasp can come from the **network output** (this investigation), the **conversion** (`to_tcp_pose`), the **calibration** or the **execution**.
Experiments 1 to 10 isolate the network side; experiment 12 isolates the rest.

## 2. Tools and how to reproduce

| Task | Command |
|---|---|
| Detect (no arm motion) | `ros2 service call /gsam2/segment std_srvs/srv/Trigger` then `/contact_graspnet/detect_grasps`, `/target_pose/extract`, `/pose_transform/transform_all` |
| Save the run | `D=~/grasp_runs/run_$(date +%Y%m%d_%H%M%S); mkdir -p $D; cp /tmp/last_depth.npy /tmp/last_segmap.npy $D/` (the files are overwritten on every detection) |
| View the grasps (Open3D) | in `conda activate contact_graspnet`, folder `~/contact_graspnet_pytorch`: `python visualize_grasps_from_saved.py --depth_npy $D/last_depth.npy --segmap_npy $D/last_segmap.npy --K "[488.77835083, 0, 317.80917358, 0, 488.77835083, 216.84880066, 0, 0, 1]" --ckpt_dir ~/contact_graspnet_pytorch/checkpoints/contact_graspnet --forward_passes 5` |
| Numbers per grasp | `python ~/nero_pipeline_launch/tools/grasp_report.py` (same environment; prints the contact point of the 12 best grasps along and across the object) |
| Compare against the sample scenes | `python contact_graspnet_pytorch/inference.py --np_path="test_data/7.npy" --local_regions --filter_grasps` (from `~/contact_graspnet_pytorch`) |
| Whole-scene prediction | a copy of the viewer with `--local_regions` and `--filter_grasps` defaulting to false (`cp visualize_grasps_from_saved.py viz_global.py`, edit the two defaults) |
| Which grasps MTC receives | `launch/tools/grasp_watch.py` + `grasp_view.py`, and `launch/tools/tilt_table.py` (prints tilt of every candidate and whether it was handed to MTC) |

`/tmp/last_depth.npy` and `/tmp/last_segmap.npy` are written by our change to `contact_graspnet_ros` (it saves the depth in metres and the mask it used on every call).

## 3. Experiment record

| # | Experiment | Result | What it rules in or out |
|---|---|---|---|
| 1 | Shapes and intrinsics of the data the network receives | Color and depth both 640x480; depth and color `camera_info` identical (`fx = fy = 488.78`, `cx = 317.81`, `cy = 216.85`); no "mask shape" resize warning in the node log | The mask is not resized or misaligned |
| 2 | Mask drawn over the depth image | The outline sits on the object's depth blob | Mask geometry is fine |
| 3 | Mask statistics (carrot) | 1139 mask pixels, 100% with valid depth, median depth 0.848 m, spread 2.6 cm | Depth is complete inside the mask |
| 4 | Depth quality (green cube) | Table noise RMS **2.0 mm**; cube height above the table plane **40.9 mm** (max 49.9 mm); 874 mask pixels; object depth 832 mm | The depth is metric and clean; a 4 cm cube appears 4 cm tall |
| 5 | Network on its bundled sample scene (`test_data/7.npy`) | Good poses, widths match the objects | The weights, GPU setup and code are working |
| 6 | "Thin gripper" appearance | The sample script discards the predicted openings and draws every grasp at a fixed width of 8 cm; our viewer draws the **predicted** opening | Not a model defect |
| 7 | Whole-scene prediction on our cube (no mask, no local regions) | Grasps everywhere on the scene **except on the cube** | The network does not "see" a graspable object at the cube's location in our data |
| 8 | Different object (a solid green cube instead of a carrot) | Same behaviour (grasps off or thin) | Not object shape |
| 9 | Training view range (Contact-GraspNet training configuration) | Camera elevation 30 to 150 degrees (90 = straight top-down), distance 0.9 to 1.3 m | The top-down camera angle is inside the training range. (An earlier claim that it was outside was wrong) |
| 10 | Point density on objects | Sample scenes about **102** points per cm^2 (median over 6 scenes, 1280x720); ours about **35** points per cm^2 (640x480 at 0.83 m). A 4 cm cube top face has about 870 points | Our objects are about 3 times sparser than what the network was shown in the samples |
| 11 | Candidate order and tilt | 12 raw candidates, tilts 6 to 51 degrees (all under the 55-degree cap); the 6-degree and 12-degree ones ranked 9th and 10th and were not handed to MTC | Selection issue, fixed by sorting by tilt ([docs/06](06_tuning_reference.md)). Does not explain bad proposals |
| 12 | Grasp watcher: does the arm go where the generated grasp says? | Tool built and tested on synthetic data; **no recorded result on the real arm yet** | Pending |

## 4. Hypotheses, ranked

| # | Hypothesis | For | Against / unknown | Test |
|---|---|---|---|---|
| H1 | Point **density** is too low for the network's fixed-radius neighbourhoods | Measured 3x lower density; the network works on the denser sample scenes | Not demonstrated | **Upsample the depth and mask by 2x** (about 140 points per cm^2, similar to the samples) and view again: `python launch/tools/upsample.py 2 /tmp/last_depth.npy /tmp/last_segmap.npy "<K>"` then the viewer on `/tmp/up_depth.npy`, `/tmp/up_segmap.npy` with the printed `K_up` (about `[977.56, 0, 636.12, 0, 977.56, 433.70, 0, 0, 1]`). **Ready, not yet run (no result recorded)** |
| H2 | A **single view of a small object** shows only its top face; the network proposes grasps on the visible surface | Grasps appear on the arm-facing side; sticks | The sample scenes are single-view too | A view with more visible surface (arm camera, closer) or a bigger object |
| H3 | **Checkpoint/noise mismatch**: the original repository ships variants trained for clean or strongly noisy depth; the port ships one | Our depth is clean (2 mm) | We do not know which variant the port's weights are | Obtain the other variants (original repository) and compare |
| H4 | **Object scale**: objects of 4 cm are small relative to the training objects | Plausible | Not tested | Try a larger object (about 8 to 10 cm) |
| H5 | A fault **after** the network (conversion or calibration) is partly responsible for "beside" | The viewer cannot show calibration errors; the cloud and the grasps share one frame | The "sticks" are visible in the viewer itself, so not everything is downstream | Experiment 12 |

## 5. Proposed work (also in [docs/09](09_known_issues_and_next_steps.md))

1. Run H1 (upsample test). If positive, upsample inside `contact_graspnet_ros` before inference.
2. Run experiment 12 with real execution.
3. If neither helps, replace the network for simple objects with a **geometry-based grasp generator** (centre, long axis and top surface from the mask and depth; approach straight down; jaws across the minor axis; flip variant) published in the existing grasp message format, so the rest of the pipeline is unchanged.
4. AnyGrasp when available.

## 6. Facts from the official Contact-GraspNet repository (read 2026-10-08)

* Pretrained weights are provided; training is only for retraining and needs a GPU with at least 24 GB, 64 GB RAM and the Acronym dataset. **We do not train.**
* Segmentation is recommended for object-wise grasps (`--local_regions --filter_grasps`), which is what we use.
* Confidence thresholds can be tuned with `--arg_configs TEST.second_thres:0.19 TEST.first_thres:0.23`; this changes how many proposals appear, not their direction.
* The network is trained for the Franka Panda gripper (8 cm opening); our gripper opens to 10 cm, but the pipeline ignores the predicted opening.

## 7. Outcome and lessons

* **What fixed it:** sorting the usable candidates most top-down first before cutting to `MAX_CANDIDATES_IN_TASK` (operator's attribution; there was no controlled on/off comparison).
* **Why it was missed for so long:** the viewer showed all candidates and the raw sticks, so the good grasps were visible, but the planner only receives the first six by score.
  Everything we examined (depth quality, mask, intrinsics, density, training views, calibration) was a plausible but unnecessary suspect. Experiments 1 to 10 are still useful: they establish that the inputs are healthy.
* **Order of checks next time:** (1) which candidates does MTC receive and what are their tilts (`launch/tools/grasp_watch.py`, `tilt_table.py`); (2) does the arm follow the generated grasp (watcher at the moment of closing);
  (3) only then the inputs and the network.
* **Open and optional:** a controlled comparison with the sort switched off; whether the 2x upsampling test (H1) improves proposal quality anyway; whether the near-vertical grasps are also the best-scoring after upsampling.
