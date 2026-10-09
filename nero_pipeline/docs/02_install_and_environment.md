# 02. Install and environment

This document describes the machine the project was developed on and how to reproduce it. Exact versions of
everything are recorded by `maintainer/collect_from_workspace.sh` into `environments/` and
`docs/PACKAGE_VERSIONS.md`; those generated files are the authority when this text and a version disagree.

Order of work: **2.1 base system, 2.2 CAN adapter, 2.3 workspace and AgileX packages, 2.4 planners, 2.5 cameras,
2.6 conda environments, 2.7 model weights, 2.8 our packages, 2.9 launch folder, 2.10 calibration tools, 2.11 shell
setup**. Section 2.12 is a one-command-per-requirement checklist.

## 2.1 Base system

| Item | Reference value |
|---|---|
| Board / OS | NVIDIA Jetson, Ubuntu 22.04 |
| JetPack / CUDA | 6.2 / 12.6 |
| Kernel | `5.15.199-tegra` |
| ROS | ROS 2 Humble (`/opt/ros/humble`) |
| `ROS_DOMAIN_ID` | `7` (in `~/.bashrc`) |

Install ROS 2 Humble, MoveIt 2 and the usual build tools (`colcon`, `rosdep`) as for any Humble machine.

## 2.2 CAN adapter (USB `1d50:606f`)

The stock Jetson kernel does not ship the `gs_usb` driver, so we built it out of tree against the kernel headers.
This was done on 2026-09-14.

```bash
# 1. get the driver source that matches the kernel series (5.15)
mkdir -p ~/gs_usb_build && cd ~/gs_usb_build
curl -o gs_usb.c https://raw.githubusercontent.com/torvalds/linux/v5.15/drivers/net/can/usb/gs_usb.c
```

Create `~/gs_usb_build/Makefile` with this content (the two command lines must start with a **tab**):

```make
obj-m += gs_usb.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
```

```bash
# 2. build
cd ~/gs_usb_build && make
# If make complains about missing symbols/config, the Jetson's split kernel header package needs one extra step:
#   cd /usr/src/linux-headers-5.15.199-tegra-ubuntu22.04_aarch64/3rdparty/canonical/linux-jammy/kernel-source
#   sudo make modules_prepare
# and then retry make.
# (A compiler-version warning "the compiler differs from the one used to build the kernel" was harmless here.)

# 3. load it (modprobe cannot find it because it is not installed into /lib/modules)
sudo insmod ~/gs_usb_build/gs_usb.ko
lsmod | grep gs_usb
```

Bring up the interface (every session, until it is made persistent):

```bash
cd ~/agx_arm_ws/src/agx_arm_ros/scripts
bash find_all_can_port.sh                          # lists the USB port string, e.g. 1-4.4:1.0
bash can_activate.sh can_piper 1000000 "<usb_port>"   # names the interface can_piper, 1 Mbit/s
ip -details link show can_piper                    # expect: state UP ... bitrate 1000000
candump can_piper                                  # with the arm powered, frames must scroll
```

Notes:
* The interface is bound to a **USB port**. If you plug the adapter into a different port, run `find_all_can_port.sh`
  again and re-run `can_activate.sh` with the new port string.
* `lsmod | grep gs_usb` empty after a reboot means the module is not loaded: run the `insmod` line again.
  Making the module load automatically at boot was **not completed** (open item in [docs/09](09_known_issues_and_next_steps.md)). Do not rely on
  `/etc/modules` alone: it only works for modules that are installed in the kernel module tree.
* Seeing frames in `candump` proves the arm is powered and transmitting. It does not prove the driver can talk to it.

## 2.3 Workspace and AgileX packages

```bash
mkdir -p ~/agx_arm_ws/src && cd ~/agx_arm_ws/src
git clone -b ros2 --recurse-submodules https://github.com/agilexrobotics/agx_arm_ros.git
cd ~/agx_arm_ws && colcon build --symlink-install
source install/setup.bash
```

* `pyAgxArm` (the AgileX Python SDK, used by the driver) must be importable from the **system** Python:
  `python3 -c "import pyAgxArm"` (with conda deactivated).
* We modified some configuration files of `agx_arm_moveit`. Apply them from `config_snapshots/agx_arm_moveit/` or with
  `patches/agx_arm_ros_local_changes.patch` (generated):
  * `config/agx_arm.srdf.xacro`: custom named states `home` and `pose_90deg` (in the `arm_type == 'nero'` block)
  * `config/initial_positions.yaml`: the pose the **simulation** starts in (our `home`)
  * `config/joint_limits.yaml`: planning limits (see [docs/06](06_tuning_reference.md); the velocity edit has no effect on the executed speed)

```bash
cd ~/agx_arm_ws/src/agx_arm_ros && git apply --check /path/to/patches/agx_arm_ros_local_changes.patch && git apply /path/to/patches/agx_arm_ros_local_changes.patch
```

## 2.4 Planners

* **Remove CHOMP**: `sudo apt remove ros-humble-moveit-planners-chomp`.
  CHOMP segfaulted at `ChompOptimizer::initialize()` (confirmed with `gdb`) and the plugin loader preferred it over OMPL.
* Keep the MoveIt packages at one consistent version: `sudo apt full-upgrade` fixed a mismatch
  (`libchomp_motion_planner.so.2.5.9` against `libmoveit_planning_interface.so.2.5.10`).
* Pilz stays broken on this system (`libmoveit_planning_interface.so.2.5.9` missing). Our code still names a
  `pilz_industrial_motion_planner` pipeline, which falls back to OMPL with a warning on every run. This is expected.
* MTC with Python bindings must import: `python3 -c "from moveit.task_constructor import core, stages"`.

## 2.5 Cameras

Top camera (this exact command is used by `launch/start_pipeline.sh`):

```bash
ros2 launch orbbec_camera ob_camera.launch.py camera_name:=topcam \
  enable_depth:=true depth_width:=640 depth_height:=400 depth_fps:=30 depth_format:=Y12 \
  enable_color:=true color_width:=640 color_height:=480 color_fps:=30 color_format:=MJPG \
  enable_ir:=true ir_format:=Y10 enable_point_cloud:=true depth_registration:=true
```

`depth_registration:=true` is **required**: it aligns depth to the color image, so the depth and the
segmentation mask (made on the color image) have the same shape and geometry. Without it the mask has 480 rows and the
depth 400 and the grasp node has to resize the mask, which misaligns it. With registration both images are 640x480 and the
camera info of both streams is identical (`fx = fy = 488.78`, `cx = 317.81`, `cy = 216.85` on the reference camera).

## 2.6 Conda environments

Two environments, both Python 3.10, both with the **Jetson-specific PyTorch wheel** `torch 2.5.0a0+872d972e41.nv24.8`
(from NVIDIA's Jetson wheels for JetPack 6; never install torch from PyPI here): `grounded_sam2` and `contact_graspnet`.
Exact package lists are in `environments/`.

Put these two lines in each environment's activation script
(`$CONDA_PREFIX/etc/conda/activate.d/env_vars.sh`):

```bash
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$HOME/.local/lib/python3.10/site-packages/nvidia/cusparselt/lib:$LD_LIBRARY_PATH
export PYTHONNOUSERSITE=1
```

Version pins that fixed real breakages:

| Pin | Why |
|---|---|
| `pip install numpy==1.26.1 --force-reinstall` **after** installing `opencv-python` | `opencv-python` pulls a numpy that breaks the other libraries |
| `pip install "transformers==4.44.2"` | Newer versions reject the Jetson torch version string |

## 2.7 Model weights

* Contact-GraspNet (PyTorch port, `~/contact_graspnet_pytorch`): the weights ship with the port. Check
  `ls -lh ~/contact_graspnet_pytorch/checkpoints/contact_graspnet/checkpoints/` shows `model.pt`. **You do not need to train it.**
* Grounded-SAM2: checkpoint paths are arguments of `gsam2_ros/scripts/grounded_sam2_inference.py`.

## 2.8 Our packages

The five packages in the table of `docs/01_system_overview.md` section 2.2 must each end up at
`~/agx_arm_ws/src/<package_name>` and be built:

```bash
cd ~/agx_arm_ws && colcon build --symlink-install --packages-select gsam2_ros contact_graspnet_ros target_pose_ros pose_transform_ros nero_mtc_task
source install/setup.bash
```

`docs/PACKAGE_VERSIONS.md` (generated) lists the remote URL and commit of every package in the workspace.

**Build pitfall (hit several times):** with `--symlink-install`, `build/<pkg>/` can contain a *plain copy* of a launch
file or Python module, and `install/` points at that copy, so edits in `src/` do nothing until you rebuild. After editing
`nero_mtc_task` always do:

```bash
rm -rf ~/agx_arm_ws/build/nero_mtc_task ~/agx_arm_ws/install/nero_mtc_task
cd ~/agx_arm_ws && colcon build --symlink-install --packages-select nero_mtc_task && source install/setup.bash
```

## 2.9 Launch folder

```bash
ln -s <this repo>/nero_pipeline/launch ~/nero_pipeline_launch
chmod +x ~/nero_pipeline_launch/*.sh ~/nero_pipeline_launch/scripts/*.sh
```

All scripts use absolute paths under `$HOME/nero_pipeline_launch`. The helper tools in `launch/tools/` are run from there or by
full path.

## 2.10 Calibration tools

`easy_handeye2` and `aruco_opencv` (workspace packages), `rqt_image_view`. See `docs/03_calibration.md`.

## 2.11 Shell setup (`~/.bashrc`)

```bash
export ROS_DOMAIN_ID=7
alias STOP='~/nero_pipeline_launch/stop_arm.sh'     # only for the real arm
```

Do not leave `REAL_ARM`, `JOINT_STATES_TOPIC`, `HOME_MOVE_SECONDS`, `TABLE_TOP_Z`, `ARM_WAIT` exported in a shell you
also use for simulation runs: `unset REAL_ARM TABLE_TOP_Z JOINT_STATES_TOPIC HOME_MOVE_SECONDS ARM_WAIT`.

## 2.12 Verification checklist

Run each command (ROS ones with `conda deactivate`); the comment after it is what you must see.

```bash
echo $ROS_DOMAIN_ID                                                    # 7
python3 -c "from moveit.task_constructor import core, stages; print('ok')"   # ok   (MTC Python bindings)
python3 -c "import pyAgxArm; print('ok')"                              # ok   (AgileX SDK, system Python)
dpkg -l | grep -c moveit-planners-chomp                                # 0    (CHOMP removed)
ros2 pkg list | grep -c -E "gsam2_ros|contact_graspnet_ros|target_pose_ros|pose_transform_ros|nero_mtc_task"   # 5
lsmod | grep gs_usb                                                    # one line (module loaded)
ip -details link show can_piper | grep -o -E "state UP|bitrate 1000000"   # both words
timeout 3 candump can_piper | head -3                                  # frames (arm powered)
ls ~/contact_graspnet_pytorch/checkpoints/contact_graspnet/checkpoints/model.pt   # the file exists
conda run -n contact_graspnet python -c "import torch; print(torch.__version__, torch.cuda.is_available())"   # 2.5.0a0+872d972e41.nv24.8 True
bash <this repo>/nero_pipeline/maintainer/verify_repo.sh               # no FAIL lines
```

## 2.13 What this guide does not cover (for a second machine)

The system works on the lab machine. These are only gaps in the **written installation instructions**, relevant when someone sets up another machine.
They are steps that were done during early sessions and were **not recorded as scripts**:

| Gap | What exists instead |
|---|---|
| Installing Grounded-SAM2 and the Contact-GraspNet PyTorch port from source | The upstream READMEs, plus the pins in section 2.6 and the frozen package lists in `environments/` (generated). Recreating the environments from `environments/*.yml` is the most reliable route, but the Jetson PyTorch wheel must be installed from NVIDIA's wheels, not from the YAML |
| Building or installing MoveIt Task Constructor with the Python bindings | Not recorded. `environments/ros2_pkg_list.txt` and `environments/system.txt` (generated) show where the lab machine has it (`ros2 pkg prefix moveit_task_constructor_core`) |
| Installing the Orbbec ROS 2 driver and `easy_handeye2`, `aruco_opencv` | Their upstream READMEs; versions in `docs/PACKAGE_VERSIONS.md` (generated) |
| Making `gs_usb` load at boot | Open item ([docs/09](09_known_issues_and_next_steps.md), item 11) |
| Early-session decisions between Sep 14 and Sep 24 | Summarised in [docs/07](07_development_log.md) stages 0 to 2 from the session records; fine detail of those weeks is not reproduced here |

If you set up a second machine, please write down every step you take and add it here.

## 2.14 Our packages are included

The five packages of section 2.8 are included as plain folders in [`packages/`](../packages/README.md), a snapshot of the lab machine's working copies taken
when this repository was assembled (they were not under version control there). `cp -r packages/* ~/agx_arm_ws/src/` puts them where section 2.8 expects them.
