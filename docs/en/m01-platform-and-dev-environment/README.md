# M1: Platform and Development Environment (English Tutorial)

> **Target Hardware**: reComputer Robotics J501 (AGX Orin 32GB), L4T 36.4.4 / JetPack 6.2.1
>
> **Host PC**: Ubuntu 22.04.5 LTS, x86_64

## Module Objective

Bring up the J501 hardware and software baseline and build a reproducible Jetson development environment that provides a unified development foundation for all later modules (vision, LiDAR, SLAM, navigation, manipulation).

- **Prerequisites**: None
- **Deliverable**: A reproducible Jetson-based development environment baseline
- **Corresponding module directory**: [`modules/m01-platform-and-dev-environment/`](../../../modules/m01-platform-and-dev-environment/)

## Chapter Navigation

| Chapter | Topic | Overview |
| --- | --- | --- |
| 1.1 | [J501 Hardware Platform Overview and Interfaces](<./1.1-j501-hardware/1.1 J501 Hardware Platform Overview and Interfaces.md>) | Understand the hardware architecture of the J501 as a robot main controller; master the selection and wiring of key interfaces |
| 1.2 | [JetPack 6.2 System Flashing and Basic Configuration](<./1.2-jetpack-flashing/1.2 JetPack 6.2 System Flashing and Basic Configuration.md>) | Initialize the J501 system, verify CUDA/cuDNN/TensorRT, and establish a stable development baseline |
| 1.3 | [Containerized Development Environment and Remote Toolchain](<./1.3-container-dev-env/1.3 Containerized Development Environment and Remote Toolchain.md>) | Build a reproducible and portable containerized development environment with remote development and debugging support |
| 1.4 | [Robotics Middleware ROS 2 Humble Quick Start](<./1.4-ros2-humble/1.4 Robotics Middleware ROS 2 Humble Quick Start.md>) | Install ROS 2 Humble on both machines, write publisher/subscriber nodes, and achieve cross-machine communication between the J501 and the Host PC |

## Recommended Learning Path

1.1 → 1.2 → 1.3 → 1.4 in strict order: first get familiar with the hardware, then flash the system to establish the baseline, then build the containerized development environment, and finally bring up the ROS 2 middleware. Every chapter provides complete steps, expected outputs, and FAQs.

## Notes

- Images for each chapter are stored in the `images/` directory of the corresponding chapter, with chapter numbers in the filenames (e.g. `1.1.1_product_overview.png`) for easy reference.
- The cross-machine communication demo video of chapter 1.4 (~59MB) is too large to commit to the repository and is hosted externally: `https://files.seeedstudio.com/1.4.5.3_cross_machine_demo.mp4`.
- For the overall course plan, see the [Syllabus](../../syllabus.md) and the [Module Map](../../module-map.md).
