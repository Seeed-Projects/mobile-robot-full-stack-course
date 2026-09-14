# calibration/ (placeholder)

Source: `ros2_bev/calibration/` on the remote Jetson.

The original `ros2_bev` checkout keeps four empty placeholder subdirectories here:

- `extrinsics/`
- `intrinsics/`
- `validation/`
- `vehicle/`

They carry no source code (all files were empty on the device), so they are
represented by this README instead of tracked empty directories.

For actual calibration data and tooling, see the sibling chapter folder:

- `code/2.3_camera_calibration/` — `j501_avm_calib`, `calib_web.py`, `run_calib_web.sh`, `camera_probe_gui.py`, tests, and sample outputs.