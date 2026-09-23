# M04 AI 视觉与边缘加速 / AI Vision and Edge Acceleration

Turn raw camera input into deployable edge inference pipelines on NVIDIA Jetson —
detection, tracking, semantic segmentation and 6D object pose, each accelerated with
TensorRT and wired into ROS 2.

## 章节 / Chapters

| Chapter | Topic | ROS 2 package | Status | 教程 |
| --- | --- | --- | --- | --- |
| 4.1 | YOLO object detection (training → ONNX → TensorRT → ROS 2) | `bev_detection` | **PASS** — verified on hardware | [中文](./4.1_YOLO_Object_Detection_and_TensorRT_Deployment/README_zh_CN.md) / [English](./4.1_YOLO_Object_Detection_and_TensorRT_Deployment/README_en_US.md) |
| 4.2 | Multi-object tracking (ByteTrack) | `bev_tracking` | **PASS** — verified on hardware | [中文](./4.2_Multi-Object_Tracking/README_zh_CN.md) / [English](./4.2_Multi-Object_Tracking/README_en_US.md) |
| 4.3 | Semantic segmentation + drivable area (SegFormer TensorRT) | `bev_segmentation` | **VERIFIED** — parity, geometry and live topics all measured | [中文](./4.3_Semantic_Segmentation_and_Drivable_Area_Analysis/README_zh_CN.md) / [English](./4.3_Semantic_Segmentation_and_Drivable_Area_Analysis/README_en_US.md) |
| 4.4 | Isaac ROS FoundationPose 6D pose and acceleration | Isaac ROS 3.2 container | **PARTIAL** — FP32 42-candidate Mustard adaptation passed; official 252-candidate and physical RGB-D gates open | [中文](./4.4_Isaac_ROS_FoundationPose_and_Acceleration/README_zh_CN.md) / [English](./4.4_Isaac_ROS_FoundationPose_and_Acceleration/README_en_US.md) |
| — | Shared infrastructure (demo orchestrator, web preview, interfaces) | `m4_demo_bringup`, `bev_interfaces` | — | — |
| 4.5 | Native NVlabs FoundationPose 6D pose | legacy `bev_pose` scaffold | **BLOCKED** — native inference and physical RGB-D acceptance pending | [中文](./4.5_Native_FoundationPose_6D_Pose/README_zh_CN.md) / [English](./4.5_Native_FoundationPose_6D_Pose/README_en_US.md) |

中英文教程（`README_en_US.md` / `README_zh_CN.md`）已按章节发布，见上表「教程」列。代码现在
就可以构建和运行 —— 见下方 **代码 / Code**。

Full evidence table — what is verified, what is blocked, and the measurements behind both:
[PROJECT_STATUS.md](./PROJECT_STATUS.md).

## 代码 / Code

- [M04 Code — chapter 4.1–4.5 source and Isaac ROS quickstart](./code/README.md)

代码以 **Jetson 上的课程仓库为唯一可编辑源**，本目录是其发布快照。改动只从 Jetson 流向
这里，不反向。`code/scripts/setup_workspace.sh` 把各章源码软链接成一个 colcon 工作区，
每个包只有一份可编辑副本。

## 硬件 / Hardware

- NVIDIA Jetson AGX Orin 32GB (reComputer J501)
- Seeed GMSL 1×4 camera board (MAX96724 deserializer × MAX96717 serializer) exposing the
  front camera as a V4L2 node (`/dev/video0`); native 1920×1536, driven at 1920×1080@30 here
- Orbbec Gemini 2 RGB-D — physical pose acceptance for chapters 4.4/4.5, **not currently attached**

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
