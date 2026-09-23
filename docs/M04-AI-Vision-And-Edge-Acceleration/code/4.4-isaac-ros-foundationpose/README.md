# M4.4 Isaac ROS FoundationPose (ROS 2 Humble)

This chapter uses NVIDIA Isaac ROS 3.2 FoundationPose for 6D object pose
estimation. The separate native NVlabs/PyTorch scaffold is taught in M4.5.
The only runtime status source is [`../PROJECT_STATUS.md`](../PROJECT_STATUS.md).

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
`/home/seeed/workspace/isaac_ros_assets/models/foundationpose/`. The FP32
refine and SyntheticaDETR grasp FP16 engines also passed `trtexec`.
The separate score plan `score_trt_engine_42_fp32.plan` was built with FP32
min/opt/max 1/1/42 and passed `trtexec` and maximum-shape deserialization.

## Run the Mustard demonstration

Use the Jetson repository at `/home/seeed/workspace/ros2_bev`. The runner
checks for active M4.1 and M4.3 inference nodes before using the GPU. The
Isaac ROS container and NGC Mustard assets must already be present.

```bash
# NVIDIA FP32/252 single-frame example (default mode)
M44_MODE=official modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_4_isaacros_quickstart.sh

# Separate FP32/42 adaptation
M44_MODE=adapted modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_4_isaacros_quickstart.sh
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
is `/output`; the FoundationPose node logs that remapping. The official run
`20260923-112313-14976-official` recorded `frame_id=tf_camera`, position
(metres) `[-0.4350625575, 0.1339290440, 0.7972502112]`, quaternion xyzw
`[0.7753970849, -0.3331845536, 0.3022323303, -0.4431738174]` and norm
`1.0`. Its launch, bag and pose logs are under
`/home/seeed/workspace/isaac_ros_assets/m4_4_logs/`. The earlier adapted
run `20260923-100715-14266-adapted` remains separately recorded there.

## Physical integration gate

The single-frame bag cannot establish frame rate, accuracy over multiple
views or physical-camera performance. Orbbec Gemini 2 is not connected.
Production input needs calibrated, aligned RGB/depth and an object-instance
mask. The official quickstart converts RT-DETR detections to a binary mask;
M4.3's semantic mask is not an instance mask. No project pose/TF adapter is
accepted yet.

Versioned documentation:
https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html
