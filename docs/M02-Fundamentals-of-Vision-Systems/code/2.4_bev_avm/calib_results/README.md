# calib_results/ (runtime output directory)

At runtime, `calib_web.py` and `avm_ros2` read calibration products from
`/home/seeed/workspace/ros2_bev/calib_results/` on the Jetson (set by the
`J501_AVM_CALIB_RESULTS_DIR` environment variable in `run_calib_web.sh`).

Expected runtime products:

- `front.json` / `back.json` / `left.json` / `right.json` — per-camera intrinsics (`K / D / D_inv / rms / image_size / model`).
- `extrinsics.json` — ground-plane homographies `H` per camera plus seam/book LUT references.
- `camera_info/*.yaml` — ROS 2 `CameraInfo` with `distortion_model: equidistant` (kept in `j501_avm_calib/config/camera_info/`).
- `*.intr_samples.json` — raw intrinsic-capture samples used by `tests/test_calib_web_regressions.py`.

Large `.npz` backup/working files (`seam_lut*.npz`, `backups/`) are intentionally
excluded from this repository.

Sample calibrated outputs are provided once, under
`code/2.3_camera_calibration/calib_results_sample/` (see its README) to avoid
duplicating them here.