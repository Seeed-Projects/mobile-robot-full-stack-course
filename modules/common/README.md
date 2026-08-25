# Code

Use this directory for runnable or reusable technical assets.

## Subdirectories

- `ros2_ws/`: ROS 2 workspace and packages
- `docker/`: dev containers and runtime container assets
- `scripts/`: helper scripts for setup, calibration, evaluation, and export
- `configs/`: shared YAML, launch, and runtime configs
- `notebooks/`: exploratory notebooks that should later be converted to reusable assets
## Integrated Drivers

- [DM-H65 Python SDK](../../robot_drivers/README.md): root-level SocketCAN single-motor and differential-drive control, CLI diagnostics, Jetson services, and a remote chassis WebUI. This integration is Python-only and does not include the C++ implementation.
