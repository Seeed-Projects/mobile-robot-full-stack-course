# M02 · Code (Fundamentals of Vision Systems)

Curated, git-safe source code for M02 chapters 2.1–2.4. This folder is the
**local copy** referenced by the four chapters; the canonical upstream sources
are listed under [Provenance](#provenance).

All changes in this course project live under `docs/M02-Fundamentals-of-Vision-Systems/code/`.

## Layout

```text
code/
  README.md
  2.1_gmsl2/
    gmsl4_start.sh                     # 4-camera GMSL FSYNC start script (Feishu attachment)
  2.2_depth_camera/
    pointcloud_utils/                  # ROS 2 ament_python package (RGB-D → PointCloud2)
  2.3_camera_calibration/
    j501_avm_calib/                    # ROS 2 fisheye calib library + pipeline (imported by calib_web.py)
    calib_web.py                       # headless web calibration台 (intrinsics → extrinsics → BEV preview)
    camera_probe_gui.py                # CamGrabber dependency of calib_web.py
    run_calib_web.sh                   # one-shot launcher for calib_web.py (:8090)
    tests/                             # test_calib_web_regressions.py + fixtures
    calib_results_sample/              # small sample outputs (no .npz/backups)
  2.4_bev_avm/
    ros2_ws/src/                       # avm_ros2 + bev_* stack + autoware_msgs + bevdet_vendor
    tools/  scripts/  tests/  config/  # BEV tooling from ros2_bev root
    calibration/  calib_results/       # runtime placeholders (README only)
    docs/  README.md  LICENSE
```

## Provenance

| Chapter folder | Source | Detail |
|---|---|---|
| `2.1_gmsl2/gmsl4_start.sh` | Feishu attachment | Media token `NDZVbwEJBo057cxySo3cpaq5nZc`, from doc <https://seeedstudio.feishu.cn/wiki/YiGow5u7QiEifnkVGhycLgMonMg> |
| `2.2_depth_camera/pointcloud_utils/` | GitHub | `git clone --depth 1 https://github.com/zibochen6/Mobile_Robot_Code.git` (only the `pointcloud_utils` ROS package kept; `fisheye_avm_ros/` and nested `.git` removed) |
| `2.3_camera_calibration/j501_avm_calib/` | Jetson `<Jetson IP>` | `/home/seeed/ros2_ws/src/j501_avm_calib` |
| `2.3_camera_calibration/calib_web.py`, `camera_probe_gui.py` | Jetson | `/home/seeed/workspace/ros2_bev/tools/` |
| `2.3_camera_calibration/run_calib_web.sh` | Jetson | `/home/seeed/workspace/ros2_bev/scripts/` |
| `2.3_camera_calibration/tests/` | Jetson | `/home/seeed/workspace/ros2_bev/tests/` |
| `2.3_camera_calibration/calib_results_sample/` | Jetson | `/home/seeed/workspace/ros2_bev/calib_results/` (curated) |
| `2.4_bev_avm/…` | Jetson | `/home/seeed/workspace/ros2_bev/` root: `ros2_ws/src/*`, `tools/`, `scripts/`, `tests/`, `config/`, `calibration/`, `calib_results/`, `docs/`, `README.md`, `LICENSE` |

Jetson access: `sshpass -p seeed ssh -o StrictHostKeyChecking=no seeed@<Jetson IP>`
(SSH alias `j50-robotics`).

> **Note**: replace `<Jetson IP>` with your Jetson's actual IP. Find it by running `hostname -I` on the Jetson; do not reuse a fixed address.

## Excluded from this copy

`build/`, `install/`, `log/`, `.venv-tools/`, `models/` (ONNX 291 MB + engine 147 MB),
`datasets/` (nuScenes + bags), `backups/`, `__pycache__/`, `.pytest_cache/`,
`*.pyc`, nested `.git/` directories, and any single file > 5 MB
(`*.npz`, `*.onnx`, `*.engine`, `*.db3`, `*.bag`, large assets).

The repo `.gitignore` already covers `models/`, `datasets/`, `build/`, `install/`,
`log/`, `__pycache__/`, `*.pyc`, `.pytest_cache/`, and the large-asset globs.

## Prerequisites

- NVIDIA Jetson (Seeed reComputer J5012 / AGX Orin), JetPack 6.x, Ubuntu 22.04
- ROS 2 Humble, `colcon`
- Python 3, `numpy`, OpenCV (`cv2`), `PyYAML`
- Media/v4l2 tooling for 2.1: `v4l-utils`, `media-ctl`

Per chapter:

```bash
# 2.2 pointcloud_utils
sudo apt install -y ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-vision-opencv \
  ros-$ROS_DISTRO-sensor-msgs-py ros-$ROS_DISTRO-message-filters python3-numpy
```

## 2.1 — GMSL2 multi-camera start

```bash
# copy the script to the Jetson (from this repo checkout)
scp code/2.1_gmsl2/gmsl4_start.sh seeed@<Jetson IP>:~/
chmod +x gmsl4_start.sh

# 2x2 HDMI mosaic preview (+ diagnostics)
bash gmsl4_start.sh preview

# HDMI preview + RTSP relay at rtsp://<Jetson-IP>:8554/gmsl4
bash gmsl4_start.sh stream

# preflight / verify only, then stop / status
bash gmsl4_start.sh verify
bash gmsl4_start.sh stop
bash gmsl4_start.sh status
```

Modes: `preview | stream | verify | stop | status [seconds]`.
For a non-SG3S camera model, edit the embedded Python constant
`WIDTH, HEIGHT, FPS = 1920, 1536, 30` (≈ line 155) to the actual resolution/frame
rate before running.

## 2.2 — pointcloud_utils (RGB-D → PointCloud2)

```bash
# copy the validated package into the Jetson workspace
cp -r code/2.2_depth_camera/pointcloud_utils ~/ros2_ws/src/     # or scp -r to seeed@<Jetson IP>:~/ros2_ws/src/

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select pointcloud_utils
source install/setup.bash

# hand-written RGB-D synchronizer node (publishes /camera/points)
ros2 run pointcloud_utils rgbd_to_pointcloud --ros-args -p pixel_stride:=1

# Open3D point-cloud viewer
ros2 run pointcloud_utils open3d_viewer
```

`rgbd_to_pointcloud` parameters: `registered_depth_topic` (default
`/camera/depth/image_raw`), `color_topic` (`/camera/color/image_raw`),
`color_camera_info_topic` (`/camera/color/camera_info`), `output_topic`
(`/camera/points`), `pixel_stride` (2), `sync_slop_sec` (0.03),
`depth_unit_m` (0.001).

> Note: this GitHub repo ships only the two example nodes above. The chapter's
> convenience launcher `start_orbbec_rviz.sh` and `orbbec_rviz.launch.py` are
> **not present** in the current upstream repo (they were referenced but not
> committed there). The Orbbec `orbbec_camera` driver itself comes from the
> official `OrbbecSDK_ROS2` wrapper, built separately — see chapter 2.2 for that
> install step.

## 2.3 — j501_avm_calib + calib_web.py (web calibration)

```bash
# deploy (from this repo checkout) to the Jetson's runtime layout
#   j501_avm_calib/      -> ~/ros2_ws/src/j501_avm_calib
#   calib_web.py, camera_probe_gui.py -> ~/workspace/ros2_bev/tools/
#   run_calib_web.sh     -> ~/workspace/ros2_bev/scripts/

cd ~/ros2_ws
colcon build --packages-select j501_avm_calib --symlink-install
source install/setup.bash

# launch the headless web calibration UI (browser at http://<Jetson-IP>:8090)
cd ~/workspace/ros2_bev
bash scripts/run_calib_web.sh          # == python3 tools/calib_web.py --port 8090
```

Web workflow order: open `http://<Jetson IP>:8090` → `/intrinsics` →
`/extrinsics` (→ optional `/seam`, `/bev` preview). Products are written to
`J501_AVM_CALIB_RESULTS_DIR` (default `/home/seeed/workspace/ros2_bev/calib_results/`):
`{front,back,left,right}.json`, `extrinsics.json`, and
`camera_info/*.yaml` (`distortion_model: equidistant`).

`j501_avm_calib` also exposes a full ROS 2 pipeline (GUI intrinsics, extrinsic
wizard, evaluation) — see its bundled `README.md` for those commands.

## 2.4 — avm_ros2 (BEV surround view)

```bash
# deploy ros2_ws/src/* into ~/workspace/ros2_bev/ros2_ws/src/ on the Jetson
# (avm_ros2, bev_bringup, bev_camera_sync, bev_interfaces, bev_perception,
#  bev_preprocessor, bev_system_monitor, bev_visualization, bevdet_vendor, autoware_msgs)

cd ~/workspace/ros2_bev/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash       # for j501_avm_calib interfaces
colcon build --packages-select avm_ros2 --symlink-install
source install/setup.bash

# stop calib_web.py first (they share V4L2 devices)
ros2 launch avm_ros2 avm_rviz.launch.py start_camera_driver:=true
```

Verify topics (metric/image is the measurement ground-plane BEV):

```text
/avm/bev/metric/image   /avm/bev/valid_mask   /avm/bev/surround/image
/avm/bev/camera_mask    /avm/bev/status       /avm/bev/image
```

Only `/avm/bev/metric/image` and `/avm/bev/valid_mask` may feed the local map;
`surround/image` is observation-only.

## Full BEV stack (bev_* packages, optional)

The `bev_bringup` / `bevdet_vendor` TensorRT path (`perception.launch.py`) needs
the excluded `models/` assets (ONNX/engine) and a full CUDA/TensorRT toolchain,
so it is source-only here. See `2.4_bev_avm/README.md` and `docs/` for the
upstream README and phase reports.