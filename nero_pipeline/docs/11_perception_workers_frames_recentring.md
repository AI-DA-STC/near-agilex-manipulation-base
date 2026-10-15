# 11. Perception speed, camera frames, grasp re-centring and the table box (2026-10-12 to 2026-10-14)

This document records one working period in which the perception chain became fast, a wrong coordinate-frame label was found and fixed, the grasps were re-centred on the
object, and the table collision box was examined. It is written so that someone who was not there can see **what was changed, why, what the evidence was, and what is still unproven**.

Every claim carries one of four labels:

| Label | Meaning |
|---|---|
| **Measured** | A number printed by one of our tools or by the nodes on the lab machine, copied into this document |
| **Operator report** | Said by the person running the arm; no number was recorded |
| **Inferred** | Follows from reading the code or from geometry; nobody measured it |
| **Not tested** | Built, and checked only with stand-ins (see section 9), or not checked at all |

## 0. Summary

| # | Change | Why | Evidence | State |
|---|---|---|---|---|
| 1 | Persistent workers for `gsam2_ros` and `contact_graspnet_ros` (section 1) | Every call started a new process and loaded the models again: about 20 s per prompt | **Measured:** one `/gsam2/segment` call 52 s, now 5.4 s as seen from `ros2 service call` (2.8 s inside the node) | In use on the lab machine |
| 2 | Grasp and plate poses are labelled with the frame the depth data really lives in (section 2) | They were labelled `topcam_depth_optical_frame`, but with `depth_registration:=true` the data is in the colour frame: every grasp and the plate were about 3.3 cm off sideways | **Measured:** horizontal shift of the grasps from the object 3.3 cm to 0.5 cm; the same plate moved 3.3 cm in `base_link` y | Fixed. **Operator report:** real-arm pick and place worked well afterwards |
| 3 | Optional re-centring of grasps (section 3) | For a 5 cm box Contact-GraspNet proposed narrow rim pinches, none close to the middle | **Measured:** no grasp within 1.5 cm of the top-face middle; openings 0.9 to 2.5 cm | Parameter `recentre_on_object`; switched on in the lab `start_pipeline.sh` |
| 4 | `forward_passes` 5 and `retries_if_empty` 2 (section 3.4) | One pass found no grasps on the test cube | **Measured:** 0 grasps in all 15 attempts of five calls (3 attempts each) at 1 pass, and in 3 more probe runs; 2 to 9 grasps per call at 5 passes | Launch-argument default 5 |
| 5 | Exact table box tool, `launch/scripts/add_table_box.py` (section 4) | The box built by `add_table_collision.py` is 0.2 m thick, so its top face is not where its comment says | **Inferred** from the code; the live value was not recorded | In use on the lab machine (**operator report 2026-10-14**: the tool is in the start sequence and table collision worked on the real arm) |
| 6 | Closing depth 0.006 m below the object top (section 3.3) | Fingertips would reach the table at 0.020 m | **Inferred** from the finger geometry in the docs | Default in the lab `start_pipeline.sh`; not measured on the arm |
| 7 | Start pose `initial` instead of `pose_90deg` (section 5) | The folded-up arm at `pose_90deg` blocked the top camera | **Measured:** joint values read from the real arm; forward-kinematics checks | **Applied on the lab machine** (operator report 2026-10-14): the pipeline starts from `initial` every time |
| 8 | Intermittent `pregrasp IK ... no IK found` (section 6) | Contact-GraspNet draws grasps at random; MTC tries one orientation per candidate | **Measured:** it also failed with no table in the scene | **Open** (re-running the plan-only is the workaround) |

---

## 1. Persistent workers

### 1.1 The problem (measured)

`/gsam2/segment` ran **two separate Python processes** (one per prompt) through `conda run`. Timing of one such process with `launch/tools/grounded_sam2_inference_timed.py`
(four identical runs on the lab machine, GPU in use, `DEVICE = cuda`):

| Stage | Seconds |
|---|---|
| Python start and imports (torch, transformers, sam2) | 4.7 |
| Load Grounding DINO | 4.6 |
| Grounding DINO detection (real work) | 1.4 |
| Load the SAM2 checkpoint (large) | 2.5 |
| SAM2 image encoder (real work) | 1.5 |
| SAM2 mask and writing files | 0.1 |
| **Total inside the script** | **14.8** |

`conda run` adds about 5 s, so the node measured about 20 s per prompt (five calls: 19.4 to 20.0 s). A call therefore took **52 s**. Only about 3 s of each 20 s was actual work.
`/contact_graspnet/detect_grasps` had the same pattern: **20.0 s** for a call that found nothing (16.6 s inside the node's subprocess).

### 1.2 The design

Each node now starts **one** helper process when it starts (in a background thread), which loads its models once and then answers requests.

* **File protocol, not pipes.** Requests and replies are JSON files in a temporary "spool" directory (`req_<id>.json`, `res_<id>.json`, `ready.json`, `quit`).
  Reason: `conda run` is unreliable about passing stdin and stdout through, files do not depend on it.
* **Safety.** The worker exits when its parent process is gone, when the spool directory disappears, or when a `quit` file appears. A dead worker is restarted by the next request.
  **On any worker problem (cannot start, died, timeout, error reply) the node logs it and uses the old one-process-per-call method for that call**, so a worker failure cannot stop the pipeline.
* **Live switch.** `ros2 param set /gsam2_segmentation_node use_worker false` (and the same for `/contact_graspnet_node`) restores the old behaviour without a restart.
* **gsam2.** Both prompts go in one request and share one run of SAM2's image encoder (the expensive part).
* **Contact-GraspNet.** `forward_passes` is the network batch size, so it is fixed when the worker starts. `retries_if_empty` (default 2) re-runs the prediction when it returns no grasps, which is cheap with the model loaded.
  The worker's output was compared with the original inference script on the same input (with a stand-in model) and is identical.

### 1.3 Files

| File | Role |
|---|---|
| `packages/gsam2_ros/gsam2_ros/worker_client.py`, `packages/contact_graspnet_ros/contact_graspnet_ros/worker_client.py` | Starts and talks to the worker (same code, no ROS imports) |
| `packages/gsam2_ros/scripts/grounded_sam2_worker.py` | The Grounded-SAM-2 worker (runs in the `grounded_sam2` conda environment) |
| `packages/contact_graspnet_ros/scripts/contact_graspnet_worker.py` | The Contact-GraspNet worker (runs in the `contact_graspnet` conda environment) |
| the two node files | `use_worker`, `worker_script`, `worker_timeout_sec` parameters and the fallback |

The nodes start the **helper scripts from outside the package**: by default `~/gsam2_ros_scripts/` and `~/contact_graspnet_ros_scripts/` (the same folders that already held the one-shot inference scripts).
On a new machine copy them there:

```bash
mkdir -p ~/gsam2_ros_scripts ~/contact_graspnet_ros_scripts
cp ~/agx_arm_ws/src/gsam2_ros/scripts/*.py ~/gsam2_ros_scripts/
cp ~/agx_arm_ws/src/contact_graspnet_ros/scripts/*.py ~/contact_graspnet_ros_scripts/
```

`maintainer/sync_packages_from_workspace.sh` reports `DIFFERS` when an installed helper is not the same as the package copy.

### 1.4 Result (measured)

| | Before | After |
|---|---|---|
| gsam2 worker ready after node start | n/a | 17.6 s (model load 7.4 s, warm-up 2.3 s), once, in the background |
| `/gsam2/segment`, inside the node | about 45 s | 2.8 s (2.74 s inside the worker) |
| `/gsam2/segment`, as printed by `time ros2 service call` | 52 s | 5.4 s and 6.3 s (the command itself adds about 2.6 s) |
| Contact-GraspNet worker ready | n/a | 10.6 s (model load 0.4 s) |
| `/contact_graspnet/detect_grasps` | 20.0 s (found nothing) | about 2.4 s per prediction at `forward_passes` 1; 7.3 s for 3 attempts |

Memory was not a problem: 61 GiB total, 45 GiB available with both workers resident (the Jetson shares memory between CPU and GPU).
Expected log lines: `worker ready after N s`, `segmentation took X s (persistent worker; ...)`, `Contact-GraspNet took X s (persistent worker; ...; prediction attempts used: K)`.

**Pitfall (check which file ROS really loads).** A stale copy in `build/` or `install/` can shadow the source ([docs/02](02_install_and_environment.md) section 2.8). This asks Python which file it will load and whether it has the new code:

```bash
conda deactivate
source ~/agx_arm_ws/install/setup.bash
for p in contact_graspnet_ros.contact_graspnet_node:_grasp_frame target_pose_ros.target_pose_node:_pose_frame gsam2_ros.gsam2_segmentation_node:Gsam2Worker; do
  mod=${p%%:*}; key=${p#*:}
  file=$(/usr/bin/python3 -c "import importlib.util; s = importlib.util.find_spec('$mod'); print(s.origin if s else '')" 2>/dev/null)
  if [ -z "$file" ]; then echo "  MISSING  $mod cannot be found by Python (is the workspace sourced and built?)"
  elif grep -q "$key" "$file" 2>/dev/null; then echo "  ok       $mod -> $file"
  else echo "  OLD      $mod loads an OLD file: $file (remove build/ and install/ of that package and rebuild)"; fi
done
```

(With `--symlink-install` the Python files are normally *not* copied into `install/`, so looking for copies there reports false alarms; ask Python instead.)

---

## 2. The camera-frame label

### 2.1 Symptom

In the 3D viewer ([launch/tools/README.md](../launch/tools/README.md)) the grasps closed **beside** the object in the point cloud, by a roughly constant sideways amount.

### 2.2 What was ruled out (all measured)

| Suspect | Test | Result |
|---|---|---|
| Mask and depth image not aligned | `mask_depth_check.py` slides the mask over the depth image | Overlap 0.83 as is, best 0.87 at a shift of (-1, +2) px, about 4 mm: **aligned** |
| Contact-GraspNet opens too wide, pushing the closing centre sideways | `cgn_openings.py` | The openings are **narrow** (median 0.9 to 1.3 cm on a 5 cm cube), the opposite |
| The object seen from the side (it was at the far left of the image) | The cube was moved under the camera | Same rim pinches; viewpoint was not the cause of those (section 3) |
| Hand-eye calibration | Reasoning, then the measurement below | A calibration error moves the grasps **and** the cloud together, so it cannot cause an offset between them; **unless the two use different frames** |

### 2.3 Cause

* The camera is started with `depth_registration:=true`: the depth image is re-mapped onto the **colour** image. Its data therefore lives in `topcam_color_optical_frame`.
  (Consistent with the camera info: with aligned depth the intrinsics are the colour ones, `fx = fy = 488.8, cx = 317.8, cy = 216.8`; see the corrections list in [docs/07](07_development_log.md).)
* `contact_graspnet_ros` and `target_pose_ros` labelled their output poses with a **fixed** parameter `camera_frame = topcam_depth_optical_frame`, and `pose_transform_ros` moved them to `base_link` with the TF of **that** frame.
* The 3D viewer asked the depth `camera_info` for its frame and drew the cloud with that. After the fix the nodes log
  `grasp poses are labelled with frame 'topcam_color_optical_frame' (taken from the depth camera_info)`, so the camera does report the colour frame.
* The TF between the two optical frames contains the distance between the two sensors, so labelling colour-frame data as depth-frame data placed everything off by about that distance, the same for every pose.

**Evidence (measured).** Closing points of the candidates handed to MTC, relative to the object's middle in the same cloud, `pregrasp_check.py`:

| Run | Mean shift (cm) | In the camera frame (x right, y down, z forward) |
|---|---|---|
| depth-frame label, before re-centring | dx -1.1, dy +3.2 (3.4 cm) | (-3.2, +1.1, +1.7) |
| depth-frame label, with re-centring | dx -0.4, dy +3.2 (3.3 cm) | (-3.3, +0.4, +2.4) |
| **colour-frame label (fix)** | **dx -0.5, dy -0.1 (0.5 cm)** | **(0.0, +0.5, +2.6)** |

The same plate, with the same camera-frame position `(0.101, 0.148, 0.873)`, gave `Target in base_link: (-0.518, -0.131, ...)` before the fix and `(-0.518, -0.164, 0.007)` after: a 3.3 cm shift in y, the same as the grasps.

### 2.4 The fix

`camera_frame` is now **empty by default = automatic**: the node uses the `frame_id` of the depth `camera_info` message. An explicit name still wins; if no `camera_info` has arrived yet it falls back to the old value.
Both nodes log the frame they use once. Launch files default the argument to empty.

### 2.5 Check that the depth really is registered

```bash
/usr/bin/python3 - << 'PYEOF'
import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
rclpy.init(); n = Node("kcheck"); got = {}
topics = [("/topcam/depth/image_raw", Image), ("/topcam/depth/camera_info", CameraInfo)]
for name, typ in topics:
    n.create_subscription(typ, name, lambda m, name=name: got.setdefault(name, m), qos_profile_sensor_data)
t0 = time.time()
while len(got) < len(topics) and time.time() - t0 < 15: rclpy.spin_once(n, timeout_sec=0.3)
for name, typ in topics:
    m = got.get(name)
    if m is None: print(name, "NOT RECEIVED"); continue
    print(name, "frame=%s" % m.header.frame_id, "%dx%d" % (m.width, m.height), "" if typ is Image else "fx=%.1f cx=%.1f cy=%.1f" % (m.k[0], m.k[2], m.k[5]))
PYEOF
```

* **Good:** depth `640x480`, `fx=488.8 cx=317.8 cy=216.8`.
* **Bad:** depth `640x400`, `fx=476.9 cx=322.5 cy=200.2` (**measured once, on 2026-10-13, after a pipeline restart**): the camera came up **without** registration. Do not pick anything; unplug and replug the camera's USB cable and start again.
  When the depth is 640x400 and the mask is 640x480, `contact_graspnet_node.py` resizes the mask to the depth shape (nearest neighbour). That is not a valid mapping between two different cameras, and the segment lands on the wrong pixels.
  A guard that refuses instead of resizing is open work ([docs/09](09_known_issues_and_next_steps.md)).

### 2.6 Caveats

* The hand-adjusted calibration ([docs/03](03_calibration.md)) was tuned while the grasps carried this label, so it may have absorbed part of the offset. **Operator report (2026-10-14):** with the corrected grasp, place and table collision the real arm worked well.
  No measurement was recorded; a ruler or touch test ([docs/09](09_known_issues_and_next_steps.md) item 2) would give one.
* Nothing in this section was caused by `pose_transform_ros`; it uses the frame in each message header and was not changed.

---

## 3. Re-centring the grasps

### 3.1 Why (measured)

For the 5 cm test cube (top face 4.8 x 4.6 cm, 4.8 cm tall) the grasps proposed by Contact-GraspNet were **rim pinches**, not grasps of the body:

| Scene | Grasps | Within 1.5 cm of the top-face middle | Closest | Predicted openings | Tilt of the closest 15 |
|---|---|---|---|---|---|
| cube at the far left of the image | 26 | 0 | 1.8 cm | 0.9 to 2.5 cm (median 0.9) | 23 to 61 degrees |
| cube under the camera | 8 | 0 (none within 2 cm) | 2.4 cm | 0.9 to 1.7 cm (median 1.3) | 25 to 54 degrees |

The closing centres sat 2.4 to 3.0 cm from the middle, which is the cube's own edge (half width 2.4 cm). An open gripper centred on an edge has half of itself off the object.
The openings repeat in steps of about 0.8 cm (0.9, 1.7, 2.5), which does not look like a continuous width estimate; the reason was not found.

### 3.2 What it does

With `recentre_on_object` the node keeps every grasp's **orientation** but moves its closing centre (hand base + 0.1034 m along the approach axis) onto the **middle of the object's top face**,
`recentre_depth_below_top_m` below the top surface. Grasps whose closing centre is more than `recentre_max_offset_m` sideways from that middle are dropped. MTC then picks among the rest by tilt as before.
The top face is the set of masked points within 1.2 cm of the nearest 5 % to the camera. The code is `contact_graspnet_ros/recentre.py` (no ROS imports).

Assumptions: the camera looks roughly down at the table (camera z is "down"); the object is compact (box, cube, can). It is a heuristic, not part of Contact-GraspNet.

**Result (measured, simulation):** `dx -0.5, dy -0.1, dz -2.6` cm for all six candidates handed to MTC, tilts 15 to 26 degrees, all six pregrasp poses kinematically reachable (`pregrasp_check.py`, optimistic: no collisions).

### 3.3 Closing depth and the fingertips (inferred)

The finger span along the approach axis is 0.062 to 0.138 m from `tcp_link` ([launch/tools/README.md](../launch/tools/README.md), `grasp_view.py`), and the grasp centre is 0.1034 m from the hand base, so the fingertips reach about **3.5 cm beyond the closing centre**.
For a 4.8 cm cube closed 2.0 cm below the top, the closing centre is about 2.9 cm above the table and the tips about 0.6 to 1.6 cm **below** the table surface. The default depth was therefore lowered to **0.006 m**
(tips about 1 cm above the table by the same estimate). Nobody measured this on the arm; check it with `pregrasp_check.py` (its `FINGER TIPS` block) and by watching the first descent.

### 3.4 Parameters (`contact_graspnet_ros`)

See [docs/06](06_tuning_reference.md) section 5. `forward_passes` is **5 by default through the launch argument** (the committed node never passed it and the inference script's default is 1; see the corrections list in [docs/07](07_development_log.md)).

---

## 4. The table collision box

### 4.1 What the code builds (inferred)

`launch/scripts/add_table_collision.py` creates a box with `dimensions = [2.0, 2.0, 0.2]` whose centre is at `TABLE_TOP_Z - 0.005`. Its comment says the box is 0.01 m thick and its top face is at `TABLE_TOP_Z`.
With 0.2 m thickness the **top face is at `TABLE_TOP_Z + 0.095`**: +0.065 for `TABLE_TOP_Z = -0.03`, -0.005 for the default -0.10. The live scene was **not** read back at the time, so the real value during earlier sessions is unknown.
The script's own note says a box with its top at 0 overlapped the robot's base (which occupies space around the origin) and made every IK check fail; a 2 m x 2 m box centred on the base can never avoid it.

### 4.2 The tool

`launch/scripts/add_table_box.py` shows, sets or removes the `table_surface` object with the **top face exactly where you say** and a footprint **in front of the arm only** (default x from -1.10 to -0.20, y +-0.70, 0.05 m thick), so it cannot touch the base:

```bash
/usr/bin/python3 ~/nero_pipeline_launch/scripts/add_table_box.py --show          # what MoveIt really holds, nothing changes
/usr/bin/python3 ~/nero_pipeline_launch/scripts/add_table_box.py --top_z 0.0     # replace the box (REMOVE, then ADD)
```

Tested against stand-in ROS and MoveIt modules (replacement order, printed top face, warning when the footprint covers the base). Not tested against a live MoveIt except through the sessions below.

### 4.3 What was shown on the lab machine

* **Collision checking is alive (measured).** With the box deliberately too high (top at +0.12 m) MTC failed with `pregrasp IK 0-flip (0/1): eef in collision: gripper_base - table_surface`.
  This shows the gripper base has collision geometry. It does **not** show that the finger links do, so the fingertip clearance rests on section 3.3.
* **The real table height is z = 0 in `base_link` (operator report).** The forward-kinematics model puts the shoulder (joint 2) axis at z = 0.138 m, so the table is where the shoulder is 13.8 cm above it.
  The camera sees the table at z of about +0.008 (the cube's lowest points +0.007 to +0.010, the plate floor +0.007): within 1 cm, so the calibration height is fine.
* **Not modelled:** the cube and the plate are not in the planning scene; MoveIt does not protect them.
* **Operator report (2026-10-14):** with the corrected box, grasp and place the real arm worked well.

---

## 5. The start pose `initial`

The folded-up `pose_90deg` blocked the top camera. A new named pose was read from `/feedback/joint_states` of the real arm:

| joint1 | joint2 | joint3 | joint4 | joint5 | joint6 | joint7 |
|---|---|---|---|---|---|---|
| 0.000017 | -1.043969 | -0.000052 | 2.139983 | 0.000070 | 0.002513 | 0.433994 |

* **joint4 is at its upper limit** (2.14 rad in `launch/tools/nero_fk.py`, the URDF limit rounded inward; the pose has 2.13998). `--joint4 2.08` gives margin and moves the gripper about 1.8 cm.
  Forward kinematics (**measured** with our model): the gripper is almost above the base, at about (-0.01, 0, 0.42) m.
* `launch/tools/add_initial_pose.py` makes three edits with backups: the named state `initial` in `agx_arm.srdf.xacro` (so it appears in RViz), a table of named poses with a `START_POSE` switch in
  `launch/scripts/move_to_pose_90deg.py` (`START_POSE=pose_90deg` restores the old start without editing anything), and the wording of step 0 in `run_pick_and_place.sh`. MTC itself is unchanged: it plans from wherever the arm is.
* The start move is a **straight line in joint space without collision checking**. A geometric proxy (link centre lines, **not** a collision check) put the closest non-adjacent arm parts 0.229 m apart along home to `initial`.
  The first time on the real arm, go there once with RViz Plan & Execute (collision-checked), then let step 0 find it already there.
* The driver needs the arm at `home` when it starts; return to `home` (RViz Plan & Execute) before stopping the pipeline.

**Operator report (2026-10-14):** `add_initial_pose.py` was applied on the lab machine, so the pipeline starts from `initial` every time (`START_POSE` defaults to `initial` there). The script and the SRDF in use, including which joint-4 value was chosen (2.13998 as captured, or 2.08 with margin), are in the files that `maintainer/collect_from_workspace.sh` brings into this folder (`launch/scripts/move_to_pose_90deg.py`, `config_snapshots/agx_arm_moveit/agx_arm.srdf.xacro`);
if `verify_repo.sh` still warns about `START_POSE` or `name="initial"` after collecting, the lab copies were not picked up.

---

## 6. Planning notes

* **`pregrasp IK 0-flip (0/1): no IK found` is intermittent (measured).** In simulation it appeared with the table at +0.008 and at 0.0, **and with no table in the scene**; one no-table run found a plan.
  It is not caused by the table. The six candidates handed to MTC are the six most top-down grasps of a random draw, and **the MTC task tries only the `flip` orientation of each**
  (the `("orig", candidate)` variant is commented out on purpose: the operator wants one gripper direction). A draw where none of the six flipped orientations is reachable from the start pose fails. Nothing moves when it fails; run the plan-only again.
  `launch/tools/ik_probe.py` asks MoveIt's `/compute_ik` about the candidates (original, flipped, a roll sweep, collisions on and off) in seconds. Its sweep gives the idea for future work: try several roll angles ([docs/09](09_known_issues_and_next_steps.md)).
* **A successful plan-only never exits.** After `Found N solution(s).` the node publishes the plan to RViz and then idles in a `time.sleep` loop (by design); press Ctrl+C. A failed plan prints `Failing stage(s):` and exits.
  Any loop or script that runs the plan-only must look for `Found N solution(s)` or `Failing stage` and stop the node itself.
* **`eef in collision: <link> - <object>` is a real collision**, named; `no IK found` is the solver finding nothing (or every solution colliding elsewhere).

---

## 7. Tools added

Details and usage are in [launch/tools/README.md](../launch/tools/README.md).

| Tool | Question it answers |
|---|---|
| `pregrasp_check.py` | Where do the candidates close relative to the object, is that a fixed shift or scatter, where does the camera put the table, do the fingertips clear it, are the pregrasp poses reachable |
| `cgn_openings.py`, `cgn_probe.py` | Why does Contact-GraspNet find nothing or pinch rims: opening widths, contact points, variants (filter off, grown mask, more passes) |
| `mask_depth_check.py` | Is the mask lined up with the depth image |
| `ik_probe.py` | Which candidates can MoveIt's IK reach from the start pose, and which roll angles |
| `add_table_box.py` (in `launch/scripts/`) | Show or set the table box exactly |
| `add_initial_pose.py` | Add the named pose `initial` and the start-pose switch |
| `demo_view.py` | One window with the segmentation result (pick and place target) for demos and recordings |
| `grounded_sam2_inference_timed.py` | Per-stage timing of one Grounded-SAM-2 run |
| `plate_center_check.py` | A picture of where each estimate puts the plate centre |

---

## 8. What is verified and what is not

| Statement | State |
|---|---|
| Worker timings (section 1.4) | Measured on the lab machine |
| Fallback to the old method, live switch, crash and timeout recovery | **Not tested** on the real models; tested with stand-in models ([section 9](#9-offline-tests)) |
| Frame fix removes the sideways shift | Measured (shift 3.3 to 0.5 cm in simulation) |
| Real arm works with the corrected grasp, place and table | Operator report |
| Closing depth 0.006 keeps the fingertips above the table | Inferred only |
| Finger links have collision geometry | **Not tested** (only `gripper_base` was seen in a collision message) |
| Cube and plate in the planning scene | Not modelled |
| Re-centring helps objects other than a roughly 5 cm box | **Not tested** |
| Start pose `initial` is collision-free from `home` | Geometric proxy only |

## 9. Offline tests

Each package has an `offline_tests/` folder, and `future_work/ask_prompts/tests/` has one more. They need no ROS, no GPU and no robot (ROS, MoveIt, torch and the models are replaced by small stand-ins), so they check **our code and numbers**, not MoveIt's planning and not the real models:

```bash
bash maintainer/run_offline_tests.sh                      # all suites; needs python3 with numpy, opencv-python and pillow
TEST_PYTHON=/path/to/python bash maintainer/run_offline_tests.sh
```

The commands to run the system as it is now are in [docs/12](12_current_run_sequence.md).
