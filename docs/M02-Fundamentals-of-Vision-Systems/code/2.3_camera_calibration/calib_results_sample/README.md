# calib_results_sample/

Small, curated sample calibration outputs (no large `.npz` seam LUTs or backup
copies). Source: `/home/seeed/workspace/ros2_bev/calib_results/` on the Jetson.

| File | What it is |
|---|---|
| `front.json` / `back.json` / `left.json` / `right.json` | Per-camera intrinsics result (`K`, `D`, `D_inv`, `rms`, `image_size`, `model` = `equidistant`). |
| `extrinsics.json` | Ground-plane homography `H` per camera (and seam/book references) consumed by `avm_ros2`. |

The matching ROS 2 `CameraInfo` YAML files live in
`j501_avm_calib/config/camera_info/{front,back,left,right}.yaml`
(`distortion_model: equidistant`).

Produced by the 2.3 workflow: `python3 calib_web.py` (or `bash run_calib_web.sh`)
at `http://<jetson-ip>:8090`, order `/intrinsics` then `/extrinsics`.