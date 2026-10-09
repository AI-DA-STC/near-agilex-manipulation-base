# contact_graspnet_ros

ROS 2 bridge to Contact-GraspNet (PyTorch port by elchun): segmented 2D
mask + depth in, 6-DoF grasp poses out. Second stage of the manipulation
perception pipeline - see [Pipeline](#pipeline) below.

Same conda-env-as-subprocess isolation pattern as
[`gsam2_ros`](https://github.com/rarlyx/gsam2_ros) - see that repo for the
first stage this one depends on.

## Pipeline

```
gsam2_ros /gsam2/mask_source (2D)  -----\
                                    +--> extract_point_clouds()  -->  3D point cloud, cropped to just the object
/topcam/depth/image_raw   --------/         (depth + K lift the
/topcam/depth/camera_info -------/           2D mask into 3D)
                                                    |
                                                    v
                                        predict_scene_grasps()
                                                    |
                                                    v
                                    6-DoF grasp poses (/contact_graspnet/grasps)
                                    in the CAMERA'S OWN optical frame
```

**The only input this stage takes from `gsam2_ros` is the mask** - nothing
else about the segmentation (labels, confidence scores, the annotated
image) is used here. Depth and camera intrinsics come directly from the
camera, independent of segmentation.

**Critical limitation: output poses are in the camera frame, not
`base_link`.** MoveIt cannot use `/contact_graspnet/grasps` poses directly.
They must first be transformed through the top camera's `eye_on_base`
extrinsic calibration - see
[`nero-camera-calibration`](https://github.com/rarlyx/nero-camera-calibration)
for that calibration, done separately and already completed for the top
Orbbec. That transform step is not implemented in this package; it belongs
in whichever node consumes this topic next (the task-planning / MoveIt
Task Constructor stage).

---

## Setup (one-time)

### 1. Conda environment

Reuses the exact same Jetson PyTorch wheel proven working for
`gsam2_ros` - no need to re-derive it:

```bash
conda create -n contact_graspnet python=3.10 -y
conda activate contact_graspnet
export PYTHONNOUSERSITE=1

export TORCH_INSTALL=https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
python3 -m pip install numpy==1.26.1
python3 -m pip install --no-cache $TORCH_INSTALL

mkdir -p ~/miniconda3/envs/contact_graspnet/etc/conda/activate.d
cat >> ~/miniconda3/envs/contact_graspnet/etc/conda/activate.d/env_vars.sh << 'EOF'
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$HOME/.local/lib/python3.10/site-packages/nvidia/cusparselt/lib:$LD_LIBRARY_PATH
export PYTHONNOUSERSITE=1
EOF
conda deactivate && conda activate contact_graspnet
```

### 2. torchvision

```bash
pip install "setuptools<81" typing_extensions
pip install --no-cache-dir --no-build-isolation 'git+https://github.com/pytorch/vision.git@v0.20.0'
```

No custom CUDA compilation is needed beyond torchvision itself. Unlike
the original TensorFlow implementation of Contact-GraspNet, this PyTorch
port uses pure PyTorch ops for PointNet++ (no `nvcc`-compiled kernels) -
meaningfully simpler than `gsam2_ros`'s setup, no additional `-dev`
header hunting past this point.

### 3. Install the port itself

```bash
pip install opencv-python scipy trimesh pyyaml open3d pyrender freetype-py tqdm matplotlib
git clone https://github.com/elchun/contact_graspnet_pytorch.git ~/contact_graspnet_pytorch
cd ~/contact_graspnet_pytorch
pip install --no-build-isolation -e .
```
Model weights ship in the repo (`checkpoints/contact_graspnet/`) - no
manual download needed.

Verify standalone, headless from ROS, before continuing:
```bash
python contact_graspnet_pytorch/inference.py --np_path="test_data/*.npy" --local_regions --filter_grasps
```
Opens an Open3D GUI window showing the bundled test scene with grasp
markers (close it to finish) - confirms the model loads and runs
correctly before wrapping it for live camera input.

### 4. Install this package

```bash
mkdir -p ~/contact_graspnet_ros_scripts
cp scripts/contact_graspnet_inference.py ~/contact_graspnet_ros_scripts/

cd ~/agx_arm_ws/src
cp -r contact_graspnet_ros .
cd ~/agx_arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select contact_graspnet_ros
source install/setup.bash
```

---

## Usage (every session)

**Prerequisite: `gsam2_ros` must be running** and the top camera live with
**`depth_registration:=true`** set on its launch (required - see
[Camera alignment](#camera-alignment) below).

**1. Launch this node**
```bash
ros2 launch contact_graspnet_ros contactgraspnet.launch.py \
  depth_topic:=/topcam/depth/image_raw \
  camera_info_topic:=/topcam/depth/camera_info \
  mask_topic:=/gsam2/mask_source \
  depth_scale_to_meters:=0.001
```
Check `depth_scale_to_meters` for your camera:
`ros2 topic echo /topcam/depth/image_raw --once | grep encoding` -
`16UC1` (millimeters) -> `0.001`; `32FC1` (already meters) -> `1.0`.

**2. Run the full pipeline**
```bash
# Segment the object of interest first
ros2 service call /gsam2/segment std_srvs/srv/Trigger

# Then detect grasp poses on that segmented region
ros2 service call /contact_graspnet/detect_grasps std_srvs/srv/Trigger
```
Check the response for the number of grasps found and the best score.
Full pose list publishes to:
```bash
ros2 topic echo /contact_graspnet/grasps
```

## Camera alignment

Grounded-SAM-2's mask is derived from the color image; Contact-GraspNet
needs depth and the mask to share the same resolution and pixel alignment.
Enable `depth_registration:=true` on the Orbbec launch so the driver
properly aligns depth to the color frame at the pixel level (a plain
resize would stretch dimensions to match without correcting actual
spatial alignment). This node also resizes defensively as a fallback if
shapes still mismatch, logging a warning when it does - treat that
warning as a signal to check the camera launch args, not as normal
operation.

## Visualizing a result

`scripts/visualize_grasps_from_saved.py` re-runs the pipeline on saved
`depth.npy`/`segmap.npy`/`rgb.npy` files and opens Contact-GraspNet's own
Open3D viewer (real point cloud, real colors, grasp poses rendered as
gripper markers) - useful for visually sanity-checking a result outside
the ROS/headless flow. See the script's docstring for usage.
