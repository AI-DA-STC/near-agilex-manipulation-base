# gsam2_ros

ROS 2 bridge to Grounded-SAM-2: text prompt in, 2D segmentation mask out.
First stage of the manipulation perception pipeline - see
[Pipeline](#pipeline) below for how this connects to grasp detection.

Grounded-SAM-2's dependencies (torch, torchvision, transformers) live in a
dedicated `grounded_sam2` conda env, fully isolated from the ROS 2 system
Python. This package's ROS node calls into that env as a subprocess rather
than importing anything from it directly.

## Pipeline

```
source_prompt ("carrot.") + top-down camera RGB
target_prompt ("plate.")  +
        |
        v
  Grounding DINO  -->  2D bounding box, one per prompt
        |
        v
      SAM 2        -->  2D pixel mask, one per prompt
        |                (this repo's output: /gsam2/mask_source, /gsam2/mask_target)
        v
  contact_graspnet_ros consumes ONLY /gsam2/mask_source
  (+ depth + camera intrinsics) to generate grasp poses.
  /gsam2/mask_target is never grasped - its pose is read
  directly off its own point cloud downstream (centroid/
  top-surface), no grasp planner involved.
```

**gsam2_ros is purely 2D** - Grounding DINO and SAM 2 never touch depth or
3D geometry. Two independent inference calls run per `/gsam2/segment`
trigger (one per prompt, same captured frame), each producing its own 2D
mask; everything 3D happens downstream.

---

## Setup (one-time)

### 1. Conda environment (Jetson-specific PyTorch build)

```bash
conda create -n grounded_sam2 python=3.10 -y
conda activate grounded_sam2
export PYTHONNOUSERSITE=1

export TORCH_INSTALL=https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
python3 -m pip install numpy==1.26.1
python3 -m pip install --no-cache $TORCH_INSTALL
```

> **Do not** `pip install torch` directly - generic PyPI wheels are built
> for x86_64 and have no GPU support on Jetson's ARM64 CUDA build. Use the
> Jetson-specific wheel above, matched to JetPack 6.2 / CUDA 12.6.

Required libraries this wheel expects on the system path (install once):
```bash
sudo apt-get install -y cuda-cupti-12-6
wget https://raw.githubusercontent.com/pytorch/pytorch/5c6af2b583709f6176898c017424dc9981023c28/.ci/docker/common/install_cusparselt.sh
export CUDA_VERSION=12.6
bash ./install_cusparselt.sh

mkdir -p ~/miniconda3/envs/grounded_sam2/etc/conda/activate.d
cat >> ~/miniconda3/envs/grounded_sam2/etc/conda/activate.d/env_vars.sh << 'EOF'
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$HOME/.local/lib/python3.10/site-packages/nvidia/cusparselt/lib:$LD_LIBRARY_PATH
export PYTHONNOUSERSITE=1
EOF
conda deactivate && conda activate grounded_sam2
```

### 2. torchvision, built from source (matched to the torch build above)

```bash
pip install "setuptools<81" typing_extensions
sudo apt-get install -y cuda-toolkit-12-6   # provides nvcc + CUDA dev headers
pip install --no-cache-dir --no-build-isolation 'git+https://github.com/pytorch/vision.git@v0.20.0'
```

### 3. Grounded-SAM-2 itself

```bash
cd ~/near-agilex-manipulation-base/external/Grounded-SAM-2
pip install -e .
pip install --no-build-isolation -e grounding_dino
pip install opencv-python supervision pycocotools tqdm
pip install numpy==1.26.1 --force-reinstall   # opencv-python drags in numpy 2.x - re-pin after

cd checkpoints && bash download_ckpts.sh && cd ..
cd gdino_checkpoints && bash download_ckpts.sh && cd ..
```

### 4. Required compatibility patch: pin `transformers`

Newer `transformers` releases misparse Jetson's torch version string
(`2.5.0a0+872d972e41.nv24.8` gets read as "less than 2.5", rejecting a
build that's actually current):
```bash
pip install "transformers==4.44.2"
```
This older version's `post_process_grounded_object_detection` uses
`box_threshold`/`text_threshold` as separate arguments instead of a single
`threshold` - `scripts/grounded_sam2_inference.py` in this repo is already
written against this signature. If you ever upgrade `transformers`, check
that function's signature hasn't changed back first.

### 5. Install this package

```bash
mkdir -p ~/gsam2_ros_scripts
cp scripts/grounded_sam2_inference.py ~/gsam2_ros_scripts/

cd ~/agx_arm_ws/src
cp -r gsam2_ros .
cd ~/agx_arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select gsam2_ros
source install/setup.bash
```

---

## Usage (every session)

**1. Launch the top-down camera** (see `nero-cam-config`)

**2. Launch this node**
```bash
ros2 launch gsam2_ros groundedsam2.launch.py \
  camera_topic:=/topcam/color/image_raw \
  source_prompt:="carrot." \
  target_prompt:="plate." \
  box_threshold:=0.4 \
  text_threshold:=0.3
```
Wait for `Ready. Call /gsam2/segment...`.

**3. Trigger segmentation**
```bash
ros2 service call /gsam2/segment std_srvs/srv/Trigger
```
Runs two independent inference calls against the same captured frame - one
for `source_prompt`, one for `target_prompt` - and checks the response for
both detections' labels and confidence scores. Results publish to:
```bash
ros2 topic echo /gsam2/mask_source       # mono8, source object (this is what contact_graspnet_ros consumes)
ros2 topic echo /gsam2/mask_target       # mono8, target/placement object - no grasp planning applied to this one
ros2 topic echo /gsam2/annotated_source  # bgr8, box + mask overlay
ros2 topic echo /gsam2/annotated_target  # bgr8, box + mask overlay
```

To change which objects are source/target, update the prompts and call
again:
```bash
ros2 param set /gsam2_segmentation_node source_prompt "red cup."
ros2 param set /gsam2_segmentation_node target_prompt "blue plate."
ros2 service call /gsam2/segment std_srvs/srv/Trigger
```

## Notes

- **Triggered, not continuous.** Each call grabs the single most recent
  camera frame and runs full inference twice (source + target) - several
  seconds total on Jetson - this is not a per-frame live overlay, matching
  the intended use (segment once per pick attempt).
- Each of `source_prompt`/`target_prompt` returns only its **single
  highest-confidence detection** per call. For scenes with multiple
  candidate objects of the same type, this picks the most confident one,
  not all of them.
- If confidence scores come back low/borderline for a real object, try
  rephrasing the prompt ("a carrot." vs "carrot.") before lowering the
  threshold - a very low threshold risks false positives on other objects.
- `box_threshold`/`text_threshold` apply to both the source and target
  inference calls equally - there's currently no way to set different
  thresholds per role. Not usually an issue in practice, but worth knowing
  if one object is consistently harder to detect than the other.
- Next stage: [`contact_graspnet_ros`](https://github.com/rarlyx/contact_graspnet_ros) consumes `/gsam2/mask_source` only.
