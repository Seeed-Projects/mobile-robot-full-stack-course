# M4.4 Isaac ROS FoundationPose (ROS 2 Humble)

This chapter uses NVIDIA Isaac ROS 3.2 FoundationPose for 6D object pose
estimation. The separate native NVlabs/PyTorch scaffold is taught in M4.5.
The only runtime status source is [`../PROJECT_STATUS.md`](../PROJECT_STATUS.md).

## Current result

**PARTIAL overall.** The project-owned **FP32, maximum 42-candidate Mustard
adaptation passed**: the official one-frame RGB-D bag produced a nonempty
`vision_msgs/Detection3DArray` on `/output`, with finite position and a unit
quaternion. This is not a pass for NVIDIA's 252-candidate configuration or a
physical RGB-D camera.

The idle rebuild of NVIDIA's FP32 score profile (min/opt/max 1/1/252, no
additional builder flags) failed because a 2190 MB TensorRT tactic found only
1405 MB available. See
`/home/seeed/workspace/isaac_ros_assets/models/foundationpose/score_trtexec_official_fp32.log`.
The FP32 refine engine and SyntheticaDETR grasp FP16 engine passed `trtexec`.
The separate score plan `score_trt_engine_42_fp32.plan` was built with FP32
min/opt/max 1/1/42 and passed `trtexec` and maximum-shape deserialization.

## Run the Mustard demonstration

Use the Jetson repository at `/home/seeed/workspace/ros2_bev`. The runner
checks for active M4.1 and M4.3 inference nodes before using the GPU. The
Isaac ROS container and NGC Mustard assets must already be present.

```bash
# 42-candidate adaptation that passed the single-frame test
M44_MODE=adapted modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_4_isaacros_quickstart.sh

# NVIDIA 252-candidate profile; currently blocked at score-engine build
M44_MODE=official modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_4_isaacros_quickstart.sh
```

The official mode selects NVIDIA's launch fragment and
`score_trt_engine.plan` with a 252 maximum. The adapted mode selects this
project's `config/foundationpose_42.yaml`, `launch/m4_4_foundationpose_42.launch.py`
and independent 42-profile engine. `max_hypothesis: 42` only limits the grid;
the adapted launch also fixes `z_0`, reducing orientation coverage so the
sampled hypotheses fit. Runtime ROS parameters confirmed this config, angle
constraint and score engine path. Do not connect the 42-profile plan to the
default 252-candidate graph.

The runner loops the one-frame bag, waits up to 240 seconds for a valid pose,
and kills its own launch and bag process groups on exit. The observed ROS topic
is `/output`; the FoundationPose node logs that remapping. For an accepted
2026-09-23 run, the verifier recorded `frame_id=tf_camera`, position (metres)
`[-0.4713481963, 0.0929617882, 0.8295211196]`, quaternion xyzw
`[0.2184645543, -0.3925129918, 0.0745772909, 0.8903061370]` and norm
`1.0`. Run `20260923-100715-14266-adapted` has the launch, bag and pose logs
under `/home/seeed/workspace/isaac_ros_assets/m4_4_logs/`.

## Physical integration gate

The single-frame bag cannot establish frame rate, accuracy over multiple
views or physical-camera performance. Orbbec Gemini 2 is not connected.
Production input needs calibrated, aligned RGB/depth and an object-instance
mask. The official quickstart converts RT-DETR detections to a binary mask;
M4.3's semantic mask is not an instance mask. No project pose/TF adapter is
accepted yet.

Versioned documentation:
https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html
