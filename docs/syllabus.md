# Syllabus

## Course Positioning

This course is a system-level mobile robotics curriculum built around `reComputer J501 (Jetson AGX Orin)` and organized around the pipeline:

`Perception -> Cognition -> Decision -> Actuation -> Deployment`

## Modules

| Module | Directory | Hours | Summary |
| --- | --- | ---: | --- |
| M1 | `modules/m01-platform-and-dev-environment/` | 4 | Hardware platform, JetPack, containers, ROS 2 bring-up |
| M2 | `modules/m02-vision-foundations/` | 5 | Cameras, calibration, depth, BEV |
| M3 | `modules/m03-lidar-and-sensor-fusion/` | 4 | LiDAR, IMU, GPS, CAN-FD, synchronization, EKF |
| M4 | `modules/m04-ai-vision-and-edge-acceleration/` | 5 | Detection, tracking, segmentation, pose, TensorRT, Isaac ROS |
| M5 | `modules/m05-3d-reconstruction-and-slam/` | 5 | Visual SLAM, LiDAR SLAM, multi-sensor SLAM, 3DGS, semantic maps |
| M6 | `modules/m06-localization-navigation-and-planning/` | 4 | Nav2, planning, dynamic avoidance, multi-robot basics |
| M7 | `modules/m07-vision-language-navigation/` | 3 | Language-grounded navigation and semantic routing |
| M8 | `modules/m08-ptz-and-active-vision/` | 2 | PTZ control, active tracking, coordinated sensing |
| M9 | `modules/m09-manipulation-and-grasping/` | 4 | MoveIt 2, hand-eye calibration, grasp planning, teleoperation |
| M10 | `modules/m10-capstone-projects/` | 4 | End-to-end project integration |
| M11 | `modules/m11-deployment-and-productization/` | 3 | Optimization, production pipelines, monitoring, OTA |

## Capstones

- `projects/p01-autonomous-inspection-robot/`
- `projects/p02-bev-reconstruction-cart/`
- `projects/p03-vln-scene-reconstruction/`
- `projects/p04-semantic-grasping-robot/`

## Expected Final Demo

A full-stack mobile robot that can:

1. navigate to a target area,
2. perceive and localize the target object,
3. reason over semantic instructions,
4. perform robotic grasping,
5. report success as an integrated system.

