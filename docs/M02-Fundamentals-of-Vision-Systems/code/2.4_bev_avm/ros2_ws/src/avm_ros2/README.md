# AVM ROS2 software phase

This package publishes the calibrated four-camera surround view without a
motor, CAN, chassis, SLAM, or motion-control dependency.

Build:

```bash
cd /home/seeed/workspace/ros2_bev/ros2_ws
source /opt/ros/humble/setup.bash
source /home/seeed/ros2_ws/install/setup.bash
colcon build --packages-select avm_ros2 --symlink-install
```

The calibration web service and ROS camera driver need exclusive access to the
same V4L2 devices. Stop `calib_web.py` before starting the driver:

```bash
ros2 launch avm_ros2 avm_rviz.launch.py start_camera_driver:=true
```

Outputs:

```text
/avm/bev/metric/image     4x4m metric ground-plane BEV
/avm/bev/surround/image  non-metric bowl observation view
/avm/bev/valid_mask      mono8 trustworthy metric pixels
/avm/bev/status          compact JSON status
/avm/bev/image           compatibility alias of metric/image
/avm/bev/coverage        compatibility alias of valid_mask
/avm/bev/camera_mask     final owner, 0 unknown and 1..4 cameras
```

Only `/avm/bev/metric/image` and `/avm/bev/valid_mask` may feed the local map.
The bowl output is observation-only and must not be used for measurement.
