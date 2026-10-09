# Tools

Helper and diagnostic tools. None of them is needed to run the pipeline; they exist to **look inside it**.
"Env" says which Python to use: *system* = `conda deactivate`, then `/usr/bin/python3` (ROS available);
*conda* = `conda activate contact_graspnet`. "Tested" says how it was checked.

| Tool | Purpose | Env | Tested |
|---|---|---|---|
| `grasp_watch.py` | **Read-only.** Listens to the candidate grasps, converts them exactly as MTC does (it imports the functions from `mtc_pick_and_place.py`), and when the gripper closes reads where `tcp_link` really is, finds the nearest candidate and prints whether the arm followed a generated grasp. Writes `/tmp/grasp_debug.json` and starts the viewer | system (needs the workspace sourced) | Run on the real arm by the operator (operator report: works); the selection, tilt sort and matching logic were also tested on synthetic poses |
| `grasp_view.py` | Open3D window for the watcher: cloud in `base_link`, raw candidates (grey stick from hand base to grasp centre, orange cross at the centre), the up-to-3 poses given to MTC (blue, cyan, yellow), the used one (green), the real closing pose (red). Refreshes when the JSON changes. Started by the watcher | conda (needs Open3D) | Run on the real arm by the operator (the Open3D window showed the cloud and the grasps); geometry also tested on synthetic scenes |
| `grasp_report.py` | Numbers per grasp: where each contact point lies relative to the object (cm) | conda, run from `~/contact_graspnet_pytorch` | Analysis part tested on synthetic grasps; model part not runnable off the Jetson |
| `tilt_table.py` | Tilt of every raw candidate, whether it passes the cap, whether it reached MTC | system | Tested on synthetic JSON |
| `upsample.py` | Doubles the resolution of a saved detection (density test, [docs/10](../../docs/10_grasp_quality_investigation.md)) | conda or any with numpy+OpenCV | 3D positions preserved to 0.13 mm (median) |
| `reach_check.py` + `nero_fk.py` | Can the arm point its gripper down at a target? Forward kinematics built from the URDF (validated against RViz TF to 5 decimals) plus random-restart IK with joint limits | system (numpy, scipy) | Reproduced the finding that a plate at 0.617 m is out of reach |
| `calib_adjust.awk` | Apply a pitch about the `base_link` y-axis and a translation to the calibration file (see below) | any | Verified against an independent matrix calculation; quaternion stays unit length |
| `capture_home.py` | Save the arm's current pose as a named state (`home`) in the SRDF and `initial_positions.yaml` | system | Used on the real arm |
| `find_down_interactive.py` | Tries candidate orientations one at a time on the arm; you answer `y` when the gripper really points straight down | system | Used on the arm; found `(1, 0, 0, 0)` |
| `find_real_down_orientation.py` | Variant that checks the finger link's own resulting orientation | system | Used during the frame investigation |
| `adjust_calib_yaw.py` | Rotate the calibration about the vertical axis by N degrees (keeps one backup, `.bak`, overwritten each run) and tells you how to reload it | **system** (needs `yaml`) | Used on the real calibration; re-tested on a sample file (translation unchanged, quaternion stays unit length) |

## Using the grasp watcher and viewer

```bash
# terminal W (conda off), BEFORE running the pipeline
source ~/agx_arm_ws/install/setup.bash
export JOINT_STATES_TOPIC=/feedback/joint_states          # real arm. For simulation: /control/joint_states
/usr/bin/python3 ~/nero_pipeline_launch/tools/grasp_watch.py
```

* Candidates appear after the perception steps of `run_pick_and_place.sh`; the Open3D window opens by itself (it needs the Python
  that has Open3D: default `~/miniconda3/envs/contact_graspnet/bin/python`, override with `GRASP_VIEW_PY`).
* Judge a raw candidate by its **orange cross** (where it closes), not by where its stick starts (the hand base is 10.3 cm behind).
* After the gripper closes the watcher prints one line, for example:
  `GRASP at x=... | nearest candidate #1: 6 mm away, approach axis off by 0.8 deg, -7 mm along the approach (negative = stopped short), jaw direction flipped 180 deg -> ARM FOLLOWED THE GENERATED GRASP`.
  A stop short of a few millimetres up to about 1 cm along the approach is normal (the straight-line approach accepts 7 to 8 cm).
  `DOES NOT MATCH ANY CANDIDATE` means the arm closed somewhere else.
* The watcher shows the used grasp only **after** the gripper closes, not during planning.

## Calibration helper `calib_adjust.awk`

Applies, to the camera pose in `base_link`: a rotation by `th` degrees about the **+y axis of `base_link`** through the pivot point
(`px`, `pz`) (in the x-z plane), then a translation (`dx`, `dy`, `dz`) in `base_link`.
Sign convention: a positive angle turns, in a side view with the arm base at the left and the workspace at the right, **anticlockwise**
(a point straight above the base tips toward the arm side). The calibration is read only when the publisher starts, so restart it afterwards ([docs/03](../../docs/03_calibration.md)).

```bash
C=~/.ros2/easy_handeye2/calibrations/nero_eye_to_hand_calib.calib
cp $C $C.bak_$(date +%H%M%S)
awk -v th=15 -v dx=-0.08 -v dy=0 -v dz=0 -v px=-0.45 -v pz=0 -f ~/nero_pipeline_launch/tools/calib_adjust.awk $C $C > $C.new && mv $C.new $C
```

Note: the formula was tested, but the final calibration values in use were set by hand ([docs/07](../../docs/07_development_log.md) stage 9).
