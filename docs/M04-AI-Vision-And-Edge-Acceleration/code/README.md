# M04 · Code (AI Vision and Edge Acceleration)

Curated, git-safe source code for M04 chapters 4.1–4.5 plus the shared infrastructure
they build on. This folder is the **published copy** referenced by the M04 chapters
and is also the learner's runnable checkout after cloning the course repository.

The Jetson tree was used as the validation source; the course `code/` tree is the
portable teaching entry point. Scripts derive their paths from their own location,
while external model assets are supplied through environment variables.

All changes in this course project live under `docs/M04-AI-Vision-And-Edge-Acceleration/code/`.

## Layout

```text
code/
  README.md
  4.1-yolo-object-detection/           # chapter 4.1
    README.md
    ros2/bev_detection/                # ROS 2 ament_cmake package (C++ / TensorRT)
  4.2-multi-object-tracking/           # chapter 4.2
    README.md
    ros2/bev_tracking/                 # ROS 2 ament_python package (ByteTrack)
  4.3-semantic-segmentation/           # chapter 4.3
    README.md
    ros2/bev_segmentation/             # ROS 2 ament_cmake package (C++ / TensorRT)
  4.5-native-foundationpose/           # chapter 4.5 native NVlabs route
    README.md
    ros2/bev_pose/                     # ROS 2 ament_python package (FoundationPose)
  4.4-isaac-ros-foundationpose/        # chapter 4.4 Isaac ROS runbook
    README.md
    config/foundationpose_42.yaml      # adapted hypothesis cap
    launch/m4_4_foundationpose_42.launch.py
  common/                              # shared infrastructure
    README.md
    ros2/bev_interfaces/               # FrameSet / SyncStats / SystemStatus messages
    ros2/m4_demo_bringup/              # demo orchestrator + web preview
  models/m4/                           # model metadata (binaries are gitignored)
    detection/   labels/coco.names, README.md
    segmentation/ labels/labels.json, README.md, LICENSE.md
    pose/        object.yaml
  scripts/
    setup_workspace.sh                 # assembles ros2_ws/src from the chapter sources
    m4/                                # demo runners, model export, verification, shared lib
    regression/                        # regression gates
```

In this course copy, `scripts/setup_workspace.sh` symlinks the published packages into
`ros2_ws/src`, and the same scripts can be run from the checkout after cloning. The
Jetson path is kept only in the provenance table as the validation source.

## Provenance

| Chapter folder | Source | Detail |
|---|---|---|
| `4.1-yolo-object-detection/` | Jetson `<Jetson IP>` | `/home/seeed/workspace/ros2_bev/modules/m04-ai-vision-and-edge-acceleration/4.1-yolo-object-detection/` |
| `4.2-multi-object-tracking/` | Jetson | same module root, `4.2-multi-object-tracking/` |
| `4.3-semantic-segmentation/` | Jetson | same module root, `4.3-semantic-segmentation/` |
| `4.5-native-foundationpose/` | Jetson | same module root, `4.4-foundationpose/` |
| `4.4-isaac-ros-foundationpose/` | Jetson | same module root, `4.4-isaac-ros-foundationpose/` |
| `common/ros2/m4_demo_bringup/` | Jetson | same module root, `common/ros2/m4_demo_bringup/` |
| `common/ros2/bev_interfaces/` | Jetson | `/home/seeed/workspace/ros2_bev/modules/common/ros2/bev_interfaces/` |
| `models/m4/` | Jetson | same module root, `models/m4/` (tracked metadata only) |
| `scripts/` | Jetson | same module root, `scripts/` |

The earlier broad snapshot came from branch `main` at
`cd3b6698425eb301fc1b9809d81f5d96f16da2f1`; the latest M4.4 update
copies selected files from Jetson commit
`6683e08dca3edb7137494d7f84fccc93f5826623` on 2026-09-23.

Jetson access: `ssh seeed@<Jetson IP>` (the working configuration used an SSH alias
`j50-robotics`).

> **Note**: replace `<Jetson IP>` with your Jetson's actual IP. Find it by running
> `hostname -I` on the Jetson; do not reuse a fixed address.

### Re-syncing

Sync only from a clean, tested Jetson commit. The source commit and selected paths are
the manifest; engines, ONNX exports, CAD, `output/` and local configuration are excluded:

```bash
ssh seeed@<Jetson IP> \
  'git -C /home/seeed/workspace/ros2_bev archive <verified-sha> <selected-paths>' \
  > /tmp/m4-source.tar
```

Extract to a temporary directory, map the paths to the course `code/` layout, and inspect
`git diff` before committing. Never copy this folder
back to the Jetson.

## Excluded from this copy

Jetson `build/`, `install/`, `log/` and `output/`, internal requirement/Agent files,
the runtime repository's root scripts and local configuration, course `ros2_ws/{build,install,log}`,
`__pycache__/`, and every model binary — `*.engine`, `*.onnx`, `*.onnx.data`, `*.obj`,
`*.step` — which the repository `.gitignore` already covers.

The published source snapshot contains no model binaries or tracked symlinks.

## Prerequisites

- NVIDIA Jetson (Seeed reComputer J501 / AGX Orin), JetPack 6.x, Ubuntu 22.04
- ROS 2 Humble, `colcon`
- CUDA 12.6 and TensorRT 10.3 (`trtexec` at `/usr/src/tensorrt/bin/trtexec`)
- Python 3.10, `numpy`, OpenCV (`cv2`), `PyYAML`
- A camera source. The demos use a GMSL front camera exposed as a V4L2 node
  (`/dev/video0`); `CAMERA_SOURCE=test` substitutes a synthetic publisher.

```bash
sudo apt install -y ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-vision-opencv \
  ros-$ROS_DISTRO-sensor-msgs-py ros-$ROS_DISTRO-message-filters python3-numpy
pip3 install --user supervision==0.27.0
```

## The M4 stack at a glance

```text
camera (V4L2 /dev/video0)
      |
      v
/perception/cameras/front/image          <- sensor_msgs/Image
      |
      v
bev_detection (YOLO11n TensorRT)  ------> /perception/detections
      |                                          |
      |                                          v
      |                              bev_tracking (ByteTrack)
      |                                          |
      |                                          v
      |                                  /perception/tracks
      v
/perception/demo/m4_x                    <- visualization (m4_demo_bringup)

bev_segmentation ----------------------> /perception/semantic_mask
                                         /perception/drivable_mask

bev_pose (+ Orbbec RGB-D) ------------> /perception/object_pose (+ TF)
```

Every topic carries the source image's `header.stamp` and `frame_id` unchanged. The only
exceptions are noted per chapter below.

## Build

```bash
cd docs/M04-AI-Vision-And-Edge-Acceleration/code
./scripts/setup_workspace.sh                  # creates ros2_ws/src/<pkg> symlinks

export PATH=/usr/local/cuda/bin:$PATH         # REQUIRED: nvcc for the CUDA kernels
export CUDACXX=/usr/local/cuda/bin/nvcc
source /opt/ros/humble/setup.bash

cd ros2_ws
colcon build --symlink-install --packages-select \
  bev_interfaces bev_detection bev_tracking bev_segmentation bev_pose m4_demo_bringup
```

`setup_workspace.sh --clean` removes `ros2_ws/{build,install,log}`. The script derives the
module root from its own location, so it works from a checkout of this folder unchanged.

> `bev_interfaces` lives in `common/ros2/` and is a hard build dependency of
> `bev_detection` — `camera_adapter_node` uses its `FrameSet` message.

## Model preparation

Model binaries are **not** committed: they are target-specific, and the segmentation
checkpoint's licence does not permit redistribution. Each chapter's `models/m4/…/README.md`
names the expected file, its source and the build command.

| Chapter | Artifact | Produce with |
|---|---|---|
| 4.1 | `engines/yolo11n_fp16.engine` | `models/m4/detection/README.md` |
| 4.3 | `onnx/segformer_b0.onnx` + `engines/segformer_b0_fp16.engine` | `scripts/m4/export_segformer.sh`, `scripts/m4/build_segformer_engine.sh` |
| 4.4 | none yet — chapter is blocked | see 4.4 below |

## 4.1 — YOLO object detection (TensorRT)

Training → ONNX → TensorRT engine → ROS 2 detection topic, at ~30 Hz on Orin.

```bash
ros2 launch bev_detection yolo.launch.py            # detection only
ros2 launch bev_detection m4_detection.launch.py    # + legacy GMSL frameset bridge

./scripts/m4/run_m4_1_demo.sh                       # auto camera resolution
CAMERA_SOURCE=test ./scripts/m4/run_m4_1_demo.sh    # synthetic, no hardware
./scripts/m4/run_m4_1_benchmark.sh
```

Contract: `header` copied from the input image; empty frames still publish an empty
`Detection2DArray` (4.2 depends on this); `Detection2D.id` is deliberately left `""` for
4.2 to fill; boxes are in **original image pixels**; `SensorDataQoS` throughout.

Details, measured numbers and known debt: [`4.1-yolo-object-detection/README.md`](4.1-yolo-object-detection/README.md).

## 4.2 — ByteTrack multi-object tracking

```bash
./scripts/m4/run_m4_2_demo.sh
```

`/perception/detections` → `/perception/tracks`, with the track id written into
`Detection2D.id` as a string (not into `hypothesis.id`). The node touches no camera and
runs no detector.

Its 30 pytest tests only pass with plugin autoload disabled:

```bash
cd ros2/bev_tracking && source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q     # -> 30 passed
```

Details: [`4.2-multi-object-tracking/README.md`](4.2-multi-object-tracking/README.md).

## 4.3 — SegFormer semantic segmentation

```bash
export HF_ENDPOINT=https://hf-mirror.com   # huggingface.co has no route from some Jetsons
./scripts/m4/export_segformer.sh           # checkpoint -> ONNX (static 1x3x512x1024)
./scripts/m4/build_segformer_engine.sh     # ONNX -> FP16 engine
./scripts/m4/generate_labels_json.sh       # id2label -> labels.json
./scripts/m4/run_m4_3_demo.sh
```

Without the engine the node **throws in its constructor** — there is no ONNX or PyTorch
fallback. `test_engine_smoke.cpp` skips when the engine is absent and gtest scores a skip
as a pass, so the suite is split into two CTest labels:

```bash
colcon test --packages-select bev_segmentation --ctest-args -L unit          # no artifacts needed
colcon test --packages-select bev_segmentation --ctest-args -L engine_gate   # RED until the engine exists
```

Two things in this chapter are worth reading even if you do not run it: the
**inverse-letterbox** restoration (`unletterbox_mask()`, and why the previous
`nearest_neighbor_resize` was wrong for any source that is not exactly 2:1), and the
**PyTorch ↔ TensorRT parity harness** — torch and the `tensorrt` bindings are importable
from different interpreters only, so the check is split across two processes joined over
raw buffers.

Details: [`4.3-semantic-segmentation/README.md`](4.3-semantic-segmentation/README.md).

## 4.4 — Isaac ROS FoundationPose

The official Isaac ROS 3.2 packages, Mustard quickstart data, FoundationPose
ONNX models, FP32 refine engine and score engines are present on Jetson. The
official FP32/252 Mustard graph and separate FP32/42 adaptation both produced
valid single-frame poses; physical RGB-D acceptance remains open. Use
[`4.4-isaac-ros-foundationpose/README.md`](4.4-isaac-ros-foundationpose/README.md)
and `scripts/m4/run_m4_4_isaacros_quickstart.sh` for this chapter.

## 4.5 — Native NVlabs FoundationPose

The native route is pinned to NVlabs commit
`a1b694b83e633c2cb6115b9063d940a687759392`. Its minimum MVP uses the official
recorded Mustard RGB-D sequence, calls `register` on the first frame and
`track_one` on the following frames, then writes pose matrices, annotated images
and a JSON timing report.

```bash
bash scripts/m4/run_m4_5_native_mvp.sh --frames 8
```

Read [`4.5-native-foundationpose/README.md`](4.5-native-foundationpose/README.md) for the
runtime layout, output files, ROS 2 topic contract and CAD-mesh path. The
runtime layout, output files, ROS 2 topic contract and CAD-mesh path.

## Common — bev_interfaces + m4_demo_bringup

Everything that is not algorithm code: the demo orchestrator, the visualizers, the web
preview server, and the shared process-lifecycle shell library.

```bash
./scripts/m4/run_m4_web_hub.sh          # unified web preview on :8080
```

A single aiohttp server on port 8080 serves `/healthz`, `/stream` (MJPEG), `/signaling`
(WebRTC), `/api/demos` and `/m4/{1,2,3,4,hub}`. Transport is selected automatically
`h264 → mjpeg → vp8`; hardware H.264 (`nvv4l2h264enc`) is the normal result. Frames live in
a single-slot buffer that drops stale frames — there are no unbounded queues.

The supervisor owns every child process by PID **and** `/proc/<pid>/stat` start-tick, so a
recycled PID is never signalled; there is no blanket `pkill` on this path.

Details: [`common/README.md`](common/README.md).

## Troubleshooting

| Symptom | Cause / fix |
| --- | --- |
| `No CMAKE_CUDA_COMPILER could be found` | `export PATH=/usr/local/cuda/bin:$PATH` before building |
| `ros2 pkg prefix` returns *Package not found* | re-run `scripts/setup_workspace.sh`, then `source ros2_ws/install/setup.bash` |
| `Cannot open engine file: ...` on node start | the chapter's model artifact is missing; there is no fallback path |
| Camera busy / no `/perception/cameras/front/image` | another demo still owns `/dev/video0`; run `scripts/m4/cleanup_demo_residual.sh --list` then `--yes` |
| Python tests fail with `ModuleNotFoundError: _pytest.scope` | run pytest with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` (see 4.2) |
| `setup_workspace.sh` dies with `<pkg>: unbound variable` | you are on bash 3.2 (macOS's default), which has no associative arrays. The script needs bash 4+ — run it on the Jetson, or with a newer bash (`brew install bash`) |
| Demo exits at preflight | a required model or binary is missing; the message names the exact path |
| `m4_2_demo.launch.py` / `m4_3_demo.launch.py` raise `invalid condition expression` | known bug: `IfCondition('$(eval ...)')` is not valid ROS 2 launch syntax. The demos themselves work via `m4_all_demo.launch.py` (what `run_m4_web_hub.sh` uses) |

Per-chapter run instructions and prerequisites are documented in each chapter README.
