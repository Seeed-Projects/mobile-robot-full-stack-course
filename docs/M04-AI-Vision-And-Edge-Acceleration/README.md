# M04 AI 视觉与边缘加速 / AI Vision and Edge Acceleration

Turn raw camera input into deployable edge inference pipelines on NVIDIA Jetson —
detection, tracking, semantic segmentation and 6D object pose, each accelerated with
TensorRT and wired into ROS 2.

## 章节 / Chapters

| Chapter | Topic | ROS 2 package | Status | 教程 |
| --- | --- | --- | --- | --- |
| 4.1 | YOLO object detection (training → ONNX → TensorRT → ROS 2) | `bev_detection` | **PASS** — verified on hardware | 待发布 |
| 4.2 | Multi-object tracking (ByteTrack) | `bev_tracking` | **PASS** — verified on hardware | 待发布 |
| 4.3 | Semantic segmentation + drivable area (SegFormer TensorRT) | `bev_segmentation` | **VERIFIED** — parity, geometry and live topics all measured | 待发布 |
| 4.4 | 6D pose estimation (FoundationPose + Orbbec Gemini 2) | `bev_pose` | **BLOCKED** — no backend, no camera | 待发布 |
| — | Shared infrastructure (demo orchestrator, web preview, interfaces) | `m4_demo_bringup`, `bev_interfaces` | — | — |
| 4.5 | Isaac ROS / model optimization | — | out of scope | — |

中英文教程（`README_en_US.md` / `README_zh_CN.md`）随代码逐个章节发布。代码现在就可以
构建和运行 —— 见下方 **代码 / Code**。

Full evidence table — what is verified, what is blocked, and the measurements behind both:
[PROJECT_STATUS.md](./PROJECT_STATUS.md).

## 代码 / Code

- [M04 Code — chapter 4.1–4.4 source (YOLO TensorRT, ByteTrack, SegFormer, FoundationPose, web demo)](./code/README.md)

代码以 **Jetson 上的课程仓库为唯一可编辑源**，本目录是其发布快照。改动只从 Jetson 流向
这里，不反向。`code/scripts/setup_workspace.sh` 把各章源码软链接成一个 colcon 工作区，
每个包只有一份可编辑副本。

## 硬件 / Hardware

- NVIDIA Jetson AGX Orin 32GB (reComputer J501)
- Seeed GMSL 1×4 camera board (MAX96724 deserializer × MAX96717 serializer) exposing the
  front camera as a V4L2 node (`/dev/video0`); native 1920×1536, driven at 1920×1080@30 here
- Orbbec Gemini 2 RGB-D — chapter 4.4 only, **not currently attached**

## 软件 / Software

| Component | Version |
| --- | --- |
| JetPack / L4T | 6.2.1 / R36.4.4 |
| Ubuntu | 22.04 |
| ROS 2 | Humble (RMW: `rmw_cyclonedds_cpp`) |
| CUDA | 12.6 |
| TensorRT | 10.3.x |
| Python | 3.10 |

## 共享架构 / Shared architecture

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