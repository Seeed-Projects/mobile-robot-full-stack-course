# M4.4 Isaac ROS FoundationPose (ROS 2 Humble)

This chapter uses NVIDIA's Isaac ROS 3.2 implementation of FoundationPose for
6D object pose estimation. It is distinct from the native NVlabs/PyTorch
scaffold in the physical `4.4-foundationpose/` directory, now taught in M4.5.

## Current result

- AGX Orin / JetPack 6.2.1 / TensorRT 10.3: the separate
  `m4-isaacros-foundationpose` container has
  `ros-humble-isaac-ros-foundationpose 3.2.14` and examples 3.2.5.
- Official NGC FoundationPose 3.2.0 Mustard mesh, texture, interface JSON and
  one-frame RGB-D rosbag are in `/home/seeed/workspace/isaac_ros_assets`.
- Official FoundationPose `1.0.0_onnx` refine and score models are present.
  The FP32 refine engine built and passed `trtexec`; the score engine build
  failed at the official 252-candidate max profile both during M4.3 operation
  and in an idle retry because its required tactic had too little available
  device memory. No valid pose output has been observed.
- The official `foundationpose` launch fragment also requires a SyntheticaDETR
  grasp RT-DETR engine. Its `1.0.0_onnx` model parsed and its FP16 plan passed
  `trtexec`. This is an engine check, not end-to-end pose acceptance.

## Reproduce the official example

Run only when the managed M4.1 and M4.3 inference nodes have ended through
their owning Hub. The runner checks this before starting a GPU build. The
default SyntheticaDETR grasp engine is already present. Run from the
repository root:

```bash
modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_4_isaacros_quickstart.sh
```

The runner builds missing FoundationPose engines with FP32 defaults and a
limited TensorRT builder footprint, launches NVIDIA's `foundationpose`
fragment, loops the official bag, and waits for one message on
`/isaac_ros_examples/output`. The bag contains only three messages (one each
for RGB, depth and camera info), so it cannot support a frame-rate claim.

## Project integration contract

The official graph uses RT-DETR detections converted to a binary mask for the
Mustard example. Production input needs an object-instance mask aligned to
RGB/depth and calibrated camera info. M4.3's semantic mask is not an instance
mask. The official pose topic and message type must be observed before an
adapter publishes the project's `/perception/object_pose` and TF; no adapter
is claimed to work yet. An attached RGB-D camera and a target-specific mesh
are separate physical acceptance gates.

Versioned documentation:
https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html
