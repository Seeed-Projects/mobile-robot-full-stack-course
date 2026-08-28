# 2.1 CSI 相机接入与 ISP 调优（IMX219 / IMX477）

## 课程目标

让 J501 正确识别并输出高质量的 CSI 图像流，理解 ISP 管线。

## 硬件清单

- J501
- IMX219 / IMX477 CSI 相机
- MIPI 排线

## 前置基础

M1.1（硬件接口识别）。

## 理论要点

- Jetson ISP 管线：`nvarguscamerasrc → nvvidconv → appsink`。
- 设备树覆盖（DTBO）原理与 `jetson-io.py` 配置。
- 曝光、白平衡、增益的实时调参。

## 实践内容

- 通过 `v4l2-ctl` 与 `gst-launch-1.0` 拉取 CSI 视频流。
- 使用 `nvarguscamerasrc` 获取经过 ISP 处理的 NV12 图像。
- 编写 GStreamer Python 绑定，实现 1080p@60fps 采集。

## 产出物

CSI 相机采集 Python 类库 + ISP 参数调优指南。

