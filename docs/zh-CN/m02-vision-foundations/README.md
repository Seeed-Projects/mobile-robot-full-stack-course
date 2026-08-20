# M2：视觉系统基础（中文教程）

> **实机环境**：reComputer Robotics J501（AGX Orin 32GB），L4T 36.4.4 / JetPack 6.2.1
>
> **Host PC**：Ubuntu 22.04.5 LTS，x86_64

## 模块目标

构建感知层的相机管线与几何基础：打通 GMSL2 与深度相机两类视觉传感器的接入链路（含相机成像理论与 ISP 调优），掌握单目/双目/多传感器标定方法，完成多相机 BEV 拼接，为后续 SLAM 与 AI 视觉模块提供可复用的采集与标定资产。

- **前置要求**：完成 M1（开发环境基线与 ROS 2 Humble 基础）
- **模块产物**：多相机采集链路、camera_info 与外参标定资产、BEV 拼接 ROS 2 节点
- **对应英文模块目录**：[`modules/m02-vision-foundations/`](../../../modules/m02-vision-foundations/)

## 章节导航

| 章节 | 主题 | 内容概要 |
| --- | --- | --- |
| 2.1 | [相机成像基础与 GMSL2 多相机接入](<./2.1-gmsl2-multi-camera/2.1 相机成像基础与 GMSL2 多相机接入.md>) | 相机接口选型（CSI/GMSL2/USB）、Bayer 与去马赛克原理、Jetson ISP 管线、设备树 overlay 配置、nvargus-daemon 与 v4l2-ctl 采集、ROS 2 相机节点（Argus RAW 路径与 ISX031 YUV 路径）、GMSL2 协议与 MAX96712 解串器、PoC 供电、FAKRA 连接与 I2C 枚举、链路诊断与多相机同步 |
| 2.2 | [深度相机与 3D 视觉感知](<./2.2-depth-camera-3d-vision/2.2 深度相机与 3D 视觉感知.md>) | 深度感知原理、RealSense/Orbbec/ZED 2i 选型、**ZED 2i 实机验证（UVC 采集、ZED SDK 深度/点云、视差公式交叉验证）**、librealsense 安装与 ROS 2 集成、深度滤波、Open3D 点云与 RANSAC 平面分割、精度评估（实机内容为此前 ZED 2i 在位时的记录，当前实机已拆下 ZED 2i，复现需另备深度相机） |
| 2.3 | [相机标定：内外参与联合标定](<./2.3-camera-calibration/2.3 相机标定：内外参与联合标定.md>) | 针孔模型与坐标变换、畸变模型、张正友标定法、camera_calibration 实操、重投影误差、双目标定与 Kalibr 多传感器标定、**鱼眼相机标定（cv2.fisheye，适用实机 H190XA 190° 镜头）** |
| 2.4 | [多相机 BEV 拼接与环视感知基础](<./2.4-bev-stitching/2.4 多相机 BEV 拼接与环视感知基础.md>) | BEV/IPM 原理、安装参数到外参的计算、分辨率与覆盖范围权衡、加权融合与接缝优化、TF2/URDF 坐标系、BEV 拼接 ROS 2 节点与质量评估 |

## 建议学习顺序

2.1 → 2.2 → 2.3 → 2.4：先掌握相机成像理论与采集软件栈、打通 GMSL2 多相机链路，再扩展到深度相机，然后通过标定建立几何基础，最后用标定成果完成多相机 BEV 拼接。每章均提供完整的操作步骤、预期输出和常见问题。

## 说明

- 各章配图存放在对应章节目录的 `images/` 中，文件名带章节编号（如 `2.1.2_bayer_pattern.png`），便于检索；2.1 章主要配图已就绪，仍有少量缺图处文中以"待配图"标注；2.2/2.3/2.4 配图已就绪。
- 各章命令均可在 J501 上直接复现。当前实机相机配置为 4 路 SG3S-ISX031C-GMSL2F（H190XA 190° 超广角，YUV 路径）；2.2 章深度相机（ZED 2i）已拆下，其内容为在位时实测记录。
- 课程整体规划见 [课程大纲](../../syllabus.md) 与 [模块依赖图](../../module-map.md)。
