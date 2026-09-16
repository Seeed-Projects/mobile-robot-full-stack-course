# Jetson 6-Camera Pure-Vision BEV Smart Vehicle

Six hardware-synchronized RGB cameras → Neural BEV (BEVDet) → 3D detection → tracking →
occupancy/free-space → collision risk → ROS2 → vehicle control interface, running
end-to-end on a NVIDIA Jetson AGX Orin with TensorRT FP16.

## What it does

- Acquires 6 synchronized camera streams (`front`, `front_left`, `front_right`, `back`, `back_left`, `back_right`)
- Runs **BEVDet**-family neural BEV perception through **TensorRT 10.x FP16** (no PyTorch at runtime)
- Produces metric 3D detections in `base_link` (x forward, y left, z up)
- Tracks objects (Kalman + Hungarian), builds a V1-hybrid occupancy grid, computes
  collision risk (TTC over a vehicle-footprint safety corridor), and emits
  `safe/warning/danger/emergency` states with a SAFE STOP vehicle interface

## Hardware

- NVIDIA Jetson AGX Orin 32GB (Seeed reComputer J501), aarch64, GPU sm_87
- 6 × RGB cameras (target: GMSL2, hardware-synchronized, HFOV ≈ 90–120°, with inter-camera overlap)

## Software

- JetPack 6.2.1 (L4T R36.4.x), Ubuntu 22.04
- CUDA 12.6, TensorRT 10.3, cuDNN 9.x
- ROS2 Humble, C++17, CMake (ament_cmake), OpenCV, Eigen3, yaml-cpp

## Architecture

```
6 × RGB Cameras → driver/adapter → timestamp sync (FrameSet)
        → GPU preprocessing → BEVDet TensorRT FP16 (BEV view transform)
        → 3D Detection (+ Occupancy, V1 hybrid)
        → Tracker → Collision Checker (TTC) → Vehicle Interface (CAN, safe stop)
```

Packages: `bev_interfaces`, `bev_camera`, `bev_camera_sync`, `bev_preprocessor`,
`bev_perception`, `bev_tracker`, `bev_occupancy`, `bev_collision`,
`bev_vehicle_interface`, `bev_visualization`, `bev_system_monitor`, `bev_bringup`.

## Quick start

```bash
scripts/check_environment.sh          # Phase 0 environment gate
ros2 launch bev_bringup perception.launch.py   # perception stack (Phase 2+)
ros2 launch bev_bringup vehicle.launch.py      # full system incl. vehicle interface

# 多路 GMSL 相机调试启动器(自适应探测, 窗口排列命名, 映射落盘)
python3 tools/camera_probe_gui.py      # 打开全部相机, 拖好窗口按 s 保存映射
python3 tools/camera_probe_gui.py --probe-only    # 无 GUI 探测

# 远程调试: Jetson 无需接显示器, 浏览器打开网页看全部相机实时画面
python3 tools/camera_web_viewer.py    # 网页(MJPEG)拖拽排序+命名; 保存映射, 同一落盘链
```

## Current status

- [x] Phase 0: environment validation + TensorRT 10.3 BEVDet engine gate — **PASSED** (see docs/PHASE0_REPORT.md)
- [x] Phase 1: nuScenes offline validation — **PASSED** (see docs/PHASE1_REPORT.md)
- [x] Phase 2: ROS2 nuScenes pipeline — **PASSED** (see docs/PHASE2_REPORT.md)
- [ ] Phase 3: six physical cameras
- [ ] Phase 4: real calibration + BEV inference
- [ ] Phase 5: tracking
- [ ] Phase 6: occupancy
- [ ] Phase 7: collision + vehicle loop

## Documentation

- [ARCHITECTURE](docs/ARCHITECTURE.md)
- [ENVIRONMENT](docs/ENVIRONMENT.md)
- [INSTALL](docs/INSTALL.md)
- [CALIBRATION](docs/CALIBRATION.md)
- [DATASET](docs/DATASET.md)
- [MODEL_DEPLOYMENT](docs/MODEL_DEPLOYMENT.md)
- [ROS_INTERFACE](docs/ROS_INTERFACE.md)
- [PERFORMANCE](docs/PERFORMANCE.md)
- [TROUBLESHOOTING](docs/TROUBLESHOOTING.md)
- [TEST_REPORT](docs/TEST_REPORT.md)
- [PHASE0_REPORT](docs/PHASE0_REPORT.md)
- [TRT10_PORTING](docs/TRT10_PORTING.md)