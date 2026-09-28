# M4.4 Isaac ROS FoundationPose (ROS 2 Humble)

This chapter uses NVIDIA Isaac ROS 3.2 FoundationPose for 6D object pose
estimation. The separate native NVlabs/PyTorch scaffold is taught in M4.5.

## Current result

**PARTIAL overall, with the official FP32/252 single-frame example passed.**
The official Mustard bag produced a nonempty `vision_msgs/Detection3DArray`
on `/output`, with finite position and a unit quaternion. The separate
FP32/42 adaptation also passed. Physical RGB-D camera acceptance remains open.

The first idle build of NVIDIA's FP32 score profile (min/opt/max 1/1/252)
failed **inside the container** because a 2190 MB TensorRT tactic found only
1405 MB available. Repeating the same builder profile with the Jetson host's
TensorRT 10.3 succeeded in 213 seconds. `--skipInference` only skipped the
build command's benchmark; the container separately deserialized the resulting
`score_trt_engine.plan` at shape 252. The build and load logs are
`score_trtexec_252_host_fp32.log` and `score_trtexec_252_load_max.log` under
the model asset directory. The FP32
refine and SyntheticaDETR grasp FP16 engines also passed `trtexec`.
The separate score plan `score_trt_engine_42_fp32.plan` was built with FP32
min/opt/max 1/1/42 and passed `trtexec` and maximum-shape deserialization.

## Run the Mustard demonstration

Run from the course M4 `code/` directory. Set the container and host asset
locations before starting; the runner stages the course launch files into the
container for this invocation.

```bash
# NVIDIA FP32/252 single-frame example (default mode)
export M4_CODE_ROOT="$HOME/mobile-robot-full-stack-course/docs/M04-AI-Vision-And-Edge-Acceleration/code"
export ISAAC_ROS_CONTAINER=m4-isaacros-foundationpose
export ISAAC_ROS_HOST_ASSET_ROOT="$HOME/isaac_ros_assets"
export HOST_MODEL_ROOT="$ISAAC_ROS_HOST_ASSET_ROOT/models/foundationpose"
cd "$M4_CODE_ROOT"
M44_MODE=official ./scripts/m4/run_m4_4_isaacros_quickstart.sh

# Separate FP32/42 adaptation
M44_MODE=adapted ./scripts/m4/run_m4_4_isaacros_quickstart.sh
```

The official mode builds a missing 252-profile plan using host TensorRT,
then selects NVIDIA's launch fragment and `score_trt_engine.plan`. The adapted mode selects this
project's `config/foundationpose_42.yaml`, `launch/m4_4_foundationpose_42.launch.py`
and independent 42-profile engine. `max_hypothesis: 42` only limits the grid;
the adapted launch also fixes `z_0`, reducing orientation coverage so the
sampled hypotheses fit. Runtime ROS parameters confirmed this config, angle
constraint and score engine path. Do not connect the 42-profile plan to the
default 252-candidate graph.

The runner loops the one-frame bag, waits up to 240 seconds for a valid pose,
and kills its own launch and bag process groups on exit. The observed ROS topic
is `/output`; the FoundationPose node remaps its internal output to that topic.

## View the Mustard image and pose

From a **graphical terminal on the Jetson desktop** (a local display or remote
desktop session), use the visual runner:

```bash
cd "$M4_CODE_ROOT"
M44_MODE=official ./scripts/m4/run_m4_4_isaacros_visual.sh
```

This reuses the official quickstart and keeps its graph and looping bag active
for 120 seconds after `valid_pose`. RViz opens with its 3D view focused on the
recorded Mustard position. The RGB frame is in RViz's left **Camera** dock;
click the arrow at the far left if the dock is hidden, then drag the Camera
dock border upward to make the frame larger. Set `M44_VIEW_SECONDS=300` for a
longer inspection. The optional desktop entry in this directory runs the same
script after `M4_CODE_ROOT` is exported. The viewer runs in a temporary container made from the installed
Isaac ROS container (`m44-foundationpose-viewer:local`) and mounts the host X11
socket directly; this avoids a physical-display GLX stall seen with the earlier
TCP display bridge. The runner removes the viewer container on exit, and the
quickstart stops its own launch and bag process groups, including when the
launching terminal is interrupted. A second launch while the viewer is active
is rejected. It requires a desktop `DISPLAY` and X11
authorization. A plain SSH shell with no desktop cannot display the window.
A single-frame bag is still not live video.

## Physical integration gate

The single-frame bag cannot establish frame rate, accuracy over multiple
views or physical-camera performance. Orbbec Gemini 2 is not connected.
Production input needs calibrated, aligned RGB/depth and an object-instance
mask. The official quickstart converts RT-DETR detections to a binary mask;
M4.3's semantic mask is not an instance mask. No project pose/TF adapter is
accepted yet.

Versioned documentation:
https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html

## Technical path

FoundationPose consumes an RGB image, aligned depth, `CameraInfo`, and a
single-object instance mask. It generates global pose hypotheses, refines
them with a TensorRT refine engine, ranks them with a TensorRT score engine,
and publishes a `vision_msgs/Detection3DArray`. The refine and score stages
are different models: tracking can reuse the previous pose and mainly run
refinement, while first-frame estimation must search and score hypotheses.

The official ROS inputs are `pose_estimation/image`,
`pose_estimation/depth_image`, `pose_estimation/camera_info`, and
`pose_estimation/segmentation`. This project's launch remaps them to
`rgb/image_rect_color`, `depth_image`, `rgb/camera_info`, and `segmentation`;
the observed absolute output is `/output`. A 4.3 semantic class mask is not a
replacement for the target instance mask.

## Jetson resource and performance notes

Isaac ROS 3.2 documents FP32 TensorRT engines for FoundationPose on TensorRT
10.3+ because of FP16 precision loss, and calls for about 7.5 GB of free GPU
memory during conversion. The official score profile is min/opt/max
`1/1/252`; the separate project adaptation is `1/1/42` and uses a narrower
orientation sampling grid. Do not connect the 42-profile plan to the default
252-candidate graph.

The official Isaac ROS 3.2 AGX Orin 720p benchmark is about 1.54 FPS for pose
estimation. The official README reports tracking above 120 FPS on Jetson Orin;
these numbers describe different stages and are not a physical-camera result
from this project. This Jetson has a valid single-frame Mustard pose, but no
Orbbec Gemini 2, continuous sequence, ground truth, or accepted physical FPS.

## Applications and limits

The output can feed grasping, mobile-robot approach/avoidance, AR/MR overlay,
or inventory/inspection. Each consumer still needs calibrated camera
extrinsics, TF conversion, a reliable instance mask, and a task-specific
quality gate. A nonempty message with a unit quaternion proves message
validity, not pose accuracy.

## Source references

- [FoundationPose paper, arXiv:2312.08344](https://arxiv.org/html/2312.08344)
- [Isaac ROS 3.2 FoundationPose documentation](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html)
- [NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation release-3.2](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation/tree/release-3.2)
- [NVlabs/FoundationPose](https://github.com/NVlabs/FoundationPose)
