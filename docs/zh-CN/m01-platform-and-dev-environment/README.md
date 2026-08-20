# M1：平台入门与开发环境（中文教程）

> **实机环境**：reComputer Robotics J501（AGX Orin 32GB），L4T 36.4.4 / JetPack 6.2.1
>
> **Host PC**：Ubuntu 22.04.5 LTS，x86_64

## 模块目标

完成 J501 硬件与软件基线的搭建，建立可复现的 Jetson 开发环境，为后续所有模块（视觉、雷达、SLAM、导航、机械臂）提供统一的开发基础。

- **前置要求**：无
- **模块产物**：一套可复现的、基于 Jetson 的开发环境基线
- **对应英文模块目录**：[`modules/m01-platform-and-dev-environment/`](../../../modules/m01-platform-and-dev-environment/)

## 章节导航

| 章节 | 主题 | 内容概要 |
| --- | --- | --- |
| 1.1 | [J501 硬件平台解析与接口概览](<./1.1-j501-hardware/1.1 J501 硬件平台解析与接口概览.md>) | 理解 J501 作为机器人主控的硬件架构，掌握关键接口的选型与接线 |
| 1.2 | [JetPack 6.2 系统刷机与基础配置](<./1.2-jetpack-flashing/1.2 JetPack 6.2 系统刷机与基础配置.md>) | 完成 J501 的系统初始化，验证 CUDA/cuDNN/TensorRT，建立稳定的开发基线 |
| 1.3 | [容器化开发环境与远程工具链](<./1.3-container-dev-env/1.3 容器化开发环境与远程工具链.md>) | 建立可复现、可迁移的容器化开发环境，支持远程开发与调试 |
| 1.4 | [机器人软件中间件 ROS 2 Humble 快速上手](<./1.4-ros2-humble/1.4 机器人软件中间件 ROS 2 Humble 快速上手.md>) | 双端安装 ROS 2 Humble，编写发布者/订阅者节点，实现 J501 与 Host PC 跨机通信 |

## 建议学习顺序

1.1 → 1.2 → 1.3 → 1.4 严格递进：先认识硬件，再刷系统建基线，然后搭容器化开发环境，最后接入 ROS 2 中间件。每章均提供完整的操作步骤、预期输出和常见问题。

## 说明

- 各章配图存放在对应章节目录的 `images/` 中，文件名带章节编号（如 `1.1.1_product_overview.png`），便于检索。
- 1.4 章的跨机通信演示视频（约 59MB）因体积较大未随仓库提交，已托管为外链：`https://files.seeedstudio.com/1.4.5.3_cross_machine_demo.mp4`。
- 课程整体规划见 [课程大纲](../../syllabus.md) 与 [模块依赖图](../../module-map.md)。
