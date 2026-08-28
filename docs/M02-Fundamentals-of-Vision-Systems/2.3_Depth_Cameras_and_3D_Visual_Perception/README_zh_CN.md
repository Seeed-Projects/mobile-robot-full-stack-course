# 2.3 深度相机与 3D 视觉感知

## 课程目标

接入深度相机，获取 RGB-D 数据并理解深度标定。

## 硬件清单

- Orbbec Gemini 335Lg（GMSL2）或 Intel RealSense D455（USB3）
- J501

## 前置基础

M2.1（相机基础）。

## 理论要点

- 结构光 / 立体视觉 / ToF 深度原理对比。
- 深度图与彩色图的外参对齐（Registration）。
- 点云生成：RGB-D → PointCloud2（PCL / Open3D）。

## 实践内容

- 通过 USB3 或 GMSL2 接入深度相机，发布 ROS2 PointCloud2 话题。
- 使用 Open3D 实时可视化点云流。
- 深度图滤波与空洞填充（Bilateral Filter、Temporal Filter）。

## 产出物

RGB-D 点云发布节点 + 可视化 RViz2 配置。

