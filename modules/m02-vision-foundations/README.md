# M2: Vision Foundations

## Focus

Build the camera pipeline and geometric foundations for perception.

## Topics

- Camera imaging foundations and GMSL2 multi-camera input
- RGB-D and 3D visual perception
- Intrinsic and extrinsic calibration
- BEV stitching and surround-view basics

## Deliverable

Multi-camera acquisition and calibration assets that later SLAM and AI modules can reuse.

## Code

- [`code/ros2_ws/src/j501_vision/`](code/ros2_ws/src/j501_vision/) — ROS 2 Python package used by the M2 tutorials. The same source is embedded in the zh/en tutorials. Contents:
  - 2.1: `argus_camera.py`, `argus_camera_node.py`, `gmsl2_camera_node.py` (real-hardware tested), `gmsl2_diag.py`, `sync_eval.py`, `launch/argus_camera.launch.py`
  - 2.2: `zed_uvc_capture.py`, `zed_sdk_validate.py`, `depth_filter.py`, `pointcloud_viz.py`, `plane_segment.py`, `depth_accuracy.py`, `launch/realsense.launch.py` (unverified, no hardware)
  - 2.3: `mono_calibrate.py`, `stereo_calibrate.py`, `handeye_calibrate.py`, `fisheye_calibrate.py`, `yuyv2png.py`, `config/aprilgrid_6x6.yaml`, `config/camchain.yaml`
  - 2.4: `ipm_calculator.py`, `bev_stitcher.py`, `bev_stitch_node.py`, `bev_quality.py`, `launch/bev_stitch.launch.py`, `urdf/j501_robot.urdf.xacro`
  - Known gaps: Kalibr/IMU calibration not run on hardware; calibration scripts untested on the J501 (see tutorial notes).

## Tutorials

- English tutorial (published): [`docs/en/m02-vision-foundations/`](../../docs/en/m02-vision-foundations/) — Camera imaging foundations and GMSL2 multi-camera input, depth cameras and 3D vision, camera calibration, multi-camera BEV stitching.
- Chinese tutorial (published): [`docs/zh-CN/m02-vision-foundations/`](../../docs/zh-CN/m02-vision-foundations/) — 同一内容的中文版本。

