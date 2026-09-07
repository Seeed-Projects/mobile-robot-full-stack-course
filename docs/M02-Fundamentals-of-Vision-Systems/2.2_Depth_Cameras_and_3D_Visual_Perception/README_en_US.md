# 2.2 Depth Cameras and 3D Visual Perception

#### Learning Objectives

After completing this section, you will be able to turn a depth camera into a reliable 3D perception input. You will compare depth-sensing technologies, start the official ROS 2 driver to obtain registered RGB-D data and colored point clouds, understand how depth pixels become 3D points through coordinate transforms, and validate the result in RViz2. Optional extensions include implementing your own synchronized point-cloud node, Open3D visualization, and depth filtering.

| Stage | Task | Verifiable result |
| --- | --- | --- |
| Integration | Start Gemini 2 with its official ROS 2 driver | RGB, depth, and CameraInfo topics are available |
| Registration | Enable depth-to-color registration | Depth edges align with the color image |
| Point cloud | Enable the official colored point cloud and inspect it in RViz2 | Coordinate frame, colors, and geometry are correct |
| Advanced | Understand back-projection, synchronization, and filtering | You can explain and modify your own point-cloud node |

#### Hardware List

| ![test.jpg](./images/E7XWb0DdxoJI3Nx9lpvcDjQqnvc.jpg)<br><br>Orbbec Gemini 2 | ![test.jpg](./images/OqhFb6jirov5EdxRaemcz4xInrh.jpg)<br><br>reComputer Robotics J5012 |
| --- | --- |

- **Depth camera:** Orbbec Gemini 2 with USB 3, active-stereo infrared depth sensing, a built-in six-axis IMU, RGB-D hardware synchronization, and depth registration.
- **Compute platform:** J501 development board, or an equivalent x86/ARM platform capable of running ROS 2 Humble or later.
- **Cable:** USB 3.0 data cable between Gemini 2 and the compute platform.

Official references include the Gemini 2 specifications and the OrbbecSDK_ROS2 Wrapper v2 documentation. Driver parameters and topic names may change between releases; during the exercise, treat the current official documentation and the output of `ros2 topic list -t` on your device as authoritative.

#### Prerequisites

- ROS 2 fundamentals: nodes, topics, message types, and `ros2 launch`.
- Linear algebra fundamentals: homogeneous coordinates and composition of rotation matrices and translation vectors.

---

#### Theory

##### 1. Comparing Depth-Sensing Technologies

Most consumer and industrial depth cameras use one of the following three approaches. Each involves tradeoffs among accuracy, range, resistance to lighting interference, cost, and power consumption.

![image.png](./images/YD6qbV0X0o2phQxcYGCc5Nt0npg.png)

| Technology | Principle | Advantages | Limitations |
| --- | --- | --- | --- |
| Structured Light | Projects a known coded infrared dot or stripe pattern, measures its deformation on object surfaces, and triangulates depth from disparity. | High short-range accuracy; works well on low-texture surfaces; relatively low power. | Vulnerable to strong ambient light; multiple devices may interfere; range is typically ≤ 3 m. |
| Stereo Vision | Uses two infrared or visible-light cameras to measure binocular disparity. Stereo algorithms such as SGBM or BM generate a disparity map, which is converted to depth using the baseline and focal length. | Active IR texture improves low-texture matching; relatively long range; usually performs well indoors and in partially outdoor environments. | Difficult matching on textureless surfaces; computationally expensive; accuracy falls with distance. |
| Time of Flight (ToF) | Emits modulated infrared light and directly estimates each pixel's distance from phase shift or round-trip travel time. | No stereo matching; insensitive to texture; high frame rates and compact modules are possible. | Susceptible to multipath interference; ordinary short-range accuracy; lower SNR in direct sunlight; invalid pixels can still occur. |

> **Hardware used here:** Orbbec Gemini 2 uses active-stereo IR. Two infrared cameras triangulate depth from disparity, while projected IR texture assists matching on low-texture surfaces. The camera includes a six-axis IMU and supports RGB-D hardware synchronization and hardware D2C registration. Strong sunlight, reflective surfaces, occlusion, and long range can still reduce depth quality.

##### 2. Registering Depth and Color Images

A depth camera normally contains separate depth and RGB imaging units. Their optical centers do not coincide, so a fixed extrinsic transform—rotation (R) and translation (t)—exists between them. Directly overlaying depth and color images causes pixel misalignment; depth registration aligns the two streams.

The goal is to map every depth pixel into the color-camera coordinate system so that each depth value corresponds to an RGB pixel.

Core transformation:

1. For each depth pixel \((u_d,v_d)\), use depth \(Z\) and depth intrinsics \(K_d\) to back-project a 3D point \(P_d=(X_d,Y_d,Z_d)\) in the depth-camera frame.
1. Transform the point into the color-camera frame using the depth-to-color extrinsics: \(P_c=R_{d2c}P_d+t_{d2c}\).
1. Project \(P_c\) onto the color image using color intrinsics \(K_c\), producing \((u_c,v_c)\).
1. Write \(Z_c=P_c.z\) into \((u_c,v_c)\) in the registered depth image. Mark unfilled pixels invalid with 0 or NaN.

![Depth-to-color registration first back-projects into the depth-camera frame, transforms into the RGB-camera frame with the extrinsics, and finally projects onto RGB pixels.](./images/HMKzbKkdjoKwpqxuEbtcYsdqnVc.png)

*Depth-to-color registration first back-projects into the depth-camera frame, transforms into the RGB-camera frame with the extrinsics, and finally projects onto RGB pixels.*

> Extrinsic-calibration accuracy directly affects registration quality. Factory calibration is normally sufficient. Recalibrate with Kalibr or MATLAB Stereo Camera Calibrator after replacing a lens or if the mechanical structure deforms.

##### 3. Point-Cloud Generation: RGB-D to PointCloud2

A point cloud is a collection of discrete 3D points. Every point contains at least \((x,y,z)\) and may also contain color, normals, intensity, or other attributes. Converting RGB-D data into a point cloud is a per-pixel back-projection of the depth image.

For a registered depth pixel \((u,v)\) with depth \(Z\) and camera matrix \(K=[[f_x,0,c_x],[0,f_y,c_y],[0,0,1]]\):

\[
X=(u-c_x)Z/f_x
\]

\[
Y=(v-c_y)Z/f_y
\]

\[
Z=Z
\]

Depth is normally expressed in meters.

![Every valid registered depth pixel becomes a 3D point in the camera frame through the intrinsics and depth Z; all points together form the colored point cloud.](./images/UcrobxIayobtXBxpxwjcvX8inOd.png)

*Every valid registered depth pixel becomes a 3D point in the camera frame through the intrinsics and depth (Z); all points together form the colored point cloud.*

Important fields in a ROS 2 `PointCloud2` message:

- `header.frame_id`: the point-cloud coordinate frame, normally an optical camera frame such as `camera_color_optical_frame`.
- `height` and `width`: cloud organization. An organized cloud retains the image's 2D structure; an unorganized cloud has `height=1`.
- `fields`: commonly `x,y,z` as `FLOAT32` plus packed `rgb` as `FLOAT32`, or separate `r,g,b` as `UINT8`.
- `point_step` and `row_step`: byte strides for one point and one row; `data` stores the raw bytes.

Common libraries:

- **PCL:** the de facto C++ point-cloud library; `pcl_conversions` converts between `pcl::PointCloud<pcl::PointXYZRGB>` and ROS 2 messages.
- **Open3D:** friendly Python/C++ APIs for rapid prototyping and visualization.
- **depth_image_proc:** a ROS 2 package that converts depth images to point clouds through launch configuration without custom code.

---

#### Exercises

> **Reference environment:** reComputer Mini J501 with Orbbec Gemini 2, running JetPack 6.2.1. Check your dependencies against this environment; configuration may differ on other systems.

![Reference setup](./images/JjnybbyrzoBERbx54AXcBsoAnKd.jpeg)

##### Task 1: Connect the Depth Camera and Publish a ROS 2 PointCloud2 Topic

**Goal:** connect the camera, start the official driver, obtain registered RGB-D data, and publish it as `sensor_msgs/PointCloud2`.

**Step A: Hardware Connection and Driver Installation**

Connect the Orbbec Gemini 2 through USB 3.

![image.png](./images/SfXKbO7rMowr41xKmNkcimVYnGe.png)

1. Connect Gemini 2 to a USB 3.0 port on the J501. USB 2.0 does not provide enough bandwidth for simultaneous depth and color streams.
1. Install Orbbec SDK v2 for ARM64 and configure the udev rules:

*Install Orbbec SDK v2*

```bash
wget https://github.com/orbbec/OrbbecSDK_v2/releases/download/v2.4.11/OrbbecSDK_v2.4.11_202508040936_058db73_linux_aarch64.zip && unzip OrbbecSDK_v2.4.11_202508040936_058db73_linux_aarch64.zip && cd OrbbecSDK_v2.4.11_202508040936_058db73_linux_aarch64/shared/ && sudo chmod +x ./install_udev_rules.sh && sudo ./install_udev_rules.sh && sudo udevadm control --reload-rules && sudo udevadm trigger && cd .. && ./build_examples.sh && ./setup.sh
```

![Orbbec SDK build result](./images/Y1RQbTmUVoXrolxMXHycd9Mlnqh.png)

Build and install the Orbbec ROS 2 driver from source:

1. Clone `OrbbecSDK_ROS2` and install its dependencies.
1. Install its udev rules, build the workspace, and verify device detection.

*Build and install the Orbbec ROS 2 driver*

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src && git clone https://github.com/orbbec/OrbbecSDK_ROS2.git && sudo apt install libgflags-dev nlohmann-json3-dev ros-$ROS_DISTRO-image-transport ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport ros-$ROS_DISTRO-image-publisher ros-${ROS_DISTRO}-camera-info-manager ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs ros-${ROS_DISTRO}-backward-ros libdw-dev && cd ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts && sudo bash install_udev_rules.sh && sudo udevadm control --reload-rules && sudo udevadm trigger && cd ~/ros2_ws/ && colcon build --packages-select orbbec_camera --cmake-args -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/usr/lib/cmake/opencv4 && source ./install/setup.bash && ls /dev/video*
```

![Orbbec ROS 2 build result](./images/IZvubaxhBoSDmdx0TINcsPNwnfc.png)

![Detected devices](./images/Z0vEbrE23ohPOVxd7LWcde1wnMc.png)

**Step B: Start the Camera Driver**

Start Gemini 2 with depth registration and colored point-cloud output:

```bash
ros2 launch orbbec_camera gemini2.launch.py \
  depth_registration:=true \
  enable_point_cloud:=true \
  enable_colored_point_cloud:=true
```

Verify the topics:

```bash
ros2 topic list | grep camera
# Check the color, depth, and point-cloud topics
ros2 topic hz /camera/depth_registered/points
ros2 topic info /camera/depth_registered/points
ros2 topic echo /camera/color/camera_info --once
```

> With `depth_registration:=true`, the depth image is registered to the color-camera frame, and every point in `/camera/depth_registered/points` corresponds directly to an RGB pixel. With registration disabled, the cloud is expressed in the depth-camera frame and color requires an additional lookup mapping.

![Point-cloud topic output](./images/ZnNWbPAMzowF7dxSgttcDrHqntc.png)

**Step C (Advanced): Clone and Launch the Custom Point Cloud with RViz2**

The repository's `pointcloud_utils` package synchronizes RGB, D2C-registered depth, and color-camera intrinsics; publishes `/camera/points` in real time; and opens the included RViz2 layout. By default, the Orbbec Color and Orbbec Depth panels are expanded, and the node generates a complete colored cloud over a valid range of 0.2–8.0 m.

> Complete Steps A and B first and confirm that the official `/camera/depth_registered/points` topic works. The custom node does not replace the camera driver; it converts registered RGB-D data into an editable `PointCloud2`. The included one-click script is the recommended way to start the camera, custom node, and RViz2 together.

**Step C1: Clone and Build**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/zibochen6/Mobile_Robot_Code.git pointcloud_utils
cd pointcloud_utils

# If you received the validated course package, you may copy it directly:
# cp -r /path/to/validated/pointcloud_utils ~/ros2_ws/src/

sudo apt update
sudo apt install -y ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-vision-opencv \
  ros-$ROS_DISTRO-sensor-msgs-py ros-$ROS_DISTRO-message-filters python3-numpy

cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select pointcloud_utils
source install/setup.bash
```

> If `~/ros2_ws/src/pointcloud_utils` already exists, do not clone it again. Run `git pull --ff-only` inside it, or replace it with the validated course version, then rebuild `pointcloud_utils`.

**Step C2: Start Orbbec, the Custom Cloud, and RViz2**

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

# Recommended one-click script
~/ros2_ws/install/pointcloud_utils/share/pointcloud_utils/scripts/start_orbbec_rviz.sh

# Equivalent launch command:
# ros2 launch pointcloud_utils orbbec_rviz.launch.py
```

*Default parameters*

```bash
# Launch/node defaults:
# pixel_stride:=1
# min_depth_m:=0.2
# max_depth_m:=8.0
# sync_mode:=latest

# Reduce density if Jetson CPU load is too high:
# ros2 run pointcloud_utils rgbd_to_pointcloud --ros-args -p pixel_stride:=2
```

*Acceptance checks*

```bash
ros2 topic hz /camera/points --window 30
ros2 topic echo /camera/points --once --field header.frame_id
ros2 topic echo /camera/points --once --field fields
```

![RViz2 result](./images/Gk2UbmJ9do9jNtxT0xYcFatbnKb.png)

Success criteria: `/camera/points` publishes continuously; `header.frame_id` is `camera_color_optical_frame`; and the fields contain `x`, `y`, `z`, and `rgb`. RViz2 should open with:

- Orbbec Color and Orbbec Depth image panels expanded rather than collapsed into title bars.
- The central PointCloud2 display subscribing to `/camera/points`, Fixed Frame set to `camera_color_optical_frame`, and Color Transformer set to RGB8.
- Valid cloud coverage from nearby objects to approximately 8 m, rather than a small cluster immediately in front of the camera.

##### Implementing It Yourself: Two Essential Pieces

1. RGB and depth frames must be paired. The node must not compute a cloud whenever either stream produces an arbitrary frame. Gemini 2's registered D2C streams use `sync_mode:=latest` by default to pair the most recent frames; use `strict_stamp` only after confirming stable timestamps.

```python
self.depth_sub = self.create_subscription(
    Image, depth_topic, self.depth_callback, qos_profile_sensor_data)
self.color_sub = self.create_subscription(
    Image, color_topic, self.color_callback, qos_profile_sensor_data)

# Gemini 2 RGB/depth timestamps may have a fixed offset.
# For D2C-registered streams, pair the latest frames by default.
self.latest_depth = msg
self.latest_color = msg
self.maybe_publish()
```

2. Back-project pixels with the intrinsics. For each valid depth \(Z\), color-camera intrinsics give \(X=(u-c_x)Z/f_x\) and \(Y=(v-c_y)Z/f_y\). The node first verifies that RGB, depth, and CameraInfo resolutions agree; otherwise it stops publishing to avoid generating a geometrically incorrect cloud.

```python
sampled_z = z[::stride, ::stride]
valid = (
    np.isfinite(sampled_z)
    & (sampled_z >= min_depth)
    & (sampled_z <= max_depth)
)
v, u = np.mgrid[0:z.shape[0]:stride, 0:z.shape[1]:stride]

points['x'] = (u[valid] - cx) * sampled_z[valid] / fx
points['y'] = (v[valid] - cy) * sampled_z[valid] / fy
points['z'] = sampled_z[valid]
points['rgb'] = (r << 16) | (g << 8) | b
```

The default tuning point is `pixel_stride:=1`, `min_depth_m:=0.2`, and `max_depth_m:=8.0`. Increase `pixel_stride` to 2 or 4 only if CPU load is excessive. If the cloud appears as a tiny cluster next to the camera, inspect the depth image's actual valid range before enabling hole filling.

##### Gemini 2 / Jetson AGX Orin Validation Record

Validated on a Jetson AGX Orin Developer Kit with JetPack 6.2.1 and ROS 2 Humble; Gemini 2 USB ID `2bc5:0670`; OrbbecSDK_ROS2 commit `8e7cad2` (2026-08-07). Both the wrapper and `pointcloud_utils` built successfully from source.

**SDK boundary:** the source-built wrapper includes and uses OrbbecSDK 2.9.3 from its installation directory; no external SDK `LD_LIBRARY_PATH` is needed. The earlier SDK installation steps remain for existing environment setups, but do not source another SDK environment script in the same terminal. Run only the current wrapper's udev installation script.

| Item | Measured result |
| --- | --- |
| D2C input | `/camera/depth/image_raw` and `/camera/color/image_raw` are both 1280×720; depth encoding is `16UC1`. |
| Frames | Color CameraInfo, official colored cloud, and custom cloud all use `camera_color_optical_frame`. |
| Official colored cloud | `/camera/depth_registered/points` runs at about 20–23 Hz; `rgb` is a packed `FLOAT32` field at offset 16. |
| Custom cloud | `/camera/points` defaults to `pixel_stride:=1`, `min_depth_m:=0.2`, `max_depth_m:=8.0`, and `sync_mode:=latest`; fields are `x/y/z/FLOAT32` and `rgb/UINT32`, with `point_step=16`. |

Why not rely on the topic name alone? On this device, the D2C-registered depth stream is still named `/camera/depth/image_raw`; no `depth_registered/image_raw` topic exists. The reproducible check is that depth and RGB resolutions match and that back-projection uses the color CameraInfo. If validation fails, the node stops publishing rather than generating an incorrect cloud.

The following RViz2 layout was captured after using the one-click launcher. Color and Depth panels are expanded on the right, with the custom colored cloud in the center. A complete QMainWindow State is included so the image panels do not start as collapsed title bars. Fixed Frame is `camera_color_optical_frame`, and Color Transformer is RGB8.

![RViz2 after one-click startup: Color and Depth panels are expanded, with the custom colored point cloud in the center](./images/Jvplb8awWoQH9GxmGaZckpvQnzf.jpg)

*RViz2 after one-click startup: Color and Depth panels are expanded, with the custom colored point cloud in the center.*

> If Color and Depth still appear as title bars, the included `rviz/pointcloud.rviz` already contains a complete QMainWindow State. Source the workspace again and start it with `start_orbbec_rviz.sh` or `ros2 launch pointcloud_utils orbbec_rviz.launch.py`; enabling the displays alone will not restore the layout.

> For a quick test without building a package, source `/opt/ros/$ROS_DISTRO/setup.bash` and run `python3 rgbd_to_pointcloud.py`. Use the package's one-click script for formal acceptance so that the corrected `pointcloud.rviz` layout is loaded as well.

##### Task 2: Visualize the Point-Cloud Stream in Open3D

**Optional advanced task:** RViz2 is the required validation tool. Use Open3D only when you plan to continue with downsampling, segmentation, or fusion in Python. Availability depends on the combination of Python, Ubuntu, and CPU architecture; installation failure does not affect acceptance of this section.

Implementation notes:

- Store points and colors in `open3d.geometry.PointCloud`.
- Parse a complete frame in the ROS 2 callback and update geometry in the visualization thread; never assume that `rgb` is stored at a fixed byte offset.
- Isolate the ROS 2 callback and visualization thread with a lock or `copy.deepcopy`.

*open3d_pointcloud_viewer.py*

```python
import threading
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class PointCloudViewer(Node):
    def __init__(self):
        super().__init__('open3d_viewer')
        self.declare_parameter('pointcloud_topic', '/camera/depth_registered/points')
        topic = self.get_parameter('pointcloud_topic').value
        self.sub = self.create_subscription(PointCloud2, topic, self.pc_callback, qos_profile_sensor_data)
        self.lock = threading.Lock()
        self.latest_xyz = None
        self.latest_colors = None
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='RGB-D PointCloud', width=960, height=720)
        self.geometry = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.geometry)

    def pc_callback(self, msg):
        names = {field.name for field in msg.fields}
        if not {'x', 'y', 'z'} <= names:
            self.get_logger().error('Point cloud is missing x/y/z fields', throttle_duration_sec=2.0)
            return
        # Humble returns a structured ndarray; the official utility honors
        # fields, offsets, and point_step while parsing.
        data = point_cloud2.read_points(msg, skip_nans=True)
        if data.size == 0:
            return
        xyz = np.column_stack((data['x'], data['y'], data['z'])).astype(np.float64)
        valid = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)
        xyz = xyz[valid]
        if xyz.size == 0:
            return

        colors = None
        if 'rgb' in names:
            raw = np.asarray(data['rgb'][valid])
            packed = raw.view(np.uint32) if np.issubdtype(raw.dtype, np.floating) else raw.astype(np.uint32)
            colors = np.column_stack(((packed >> 16) & 255, (packed >> 8) & 255, packed & 255)) / 255.0
        elif {'r', 'g', 'b'} <= names:
            colors = np.column_stack((data['r'][valid], data['g'][valid], data['b'][valid])) / 255.0

        with self.lock:
            self.latest_xyz = xyz
            self.latest_colors = colors

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            with self.lock:
                if self.latest_xyz is not None:
                    self.geometry.points = o3d.utility.Vector3dVector(self.latest_xyz)
                    if self.latest_colors is not None:
                        self.geometry.colors = o3d.utility.Vector3dVector(self.latest_colors)
                    self.vis.update_geometry(self.geometry)
            self.vis.poll_events()
            self.vis.update_renderer()
        self.vis.destroy_window()

def main(args=None):
    rclpy.init(args=args)
    viewer = PointCloudViewer()
    try:
        viewer.run()
    finally:
        viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Save the file as `open3d_pointcloud_viewer.py`, either under the same package's `scripts/` directory or as a standalone script. For package use, add this entry point to `setup.py`:

```python
entry_points={
    "console_scripts": [
        "rgbd_to_pointcloud = pointcloud_utils.scripts.rgbd_to_pointcloud:main",
        "open3d_viewer = pointcloud_utils.scripts.open3d_pointcloud_viewer:main",
    ],
},
```

Install dependencies and run:

```bash
# Create an isolated environment first; system-site-packages reuses ROS 2 packages.
sudo apt install -y python3-venv
cd ~/ros2_ws
python3 -m venv --system-site-packages .venv-open3d
source .venv-open3d/bin/activate
python -m pip install --only-binary=:all: open3d
python -c "import open3d as o3d; print(o3d.__version__)"

# Run only after the previous command succeeds. A desktop, display, or X11 forwarding is required.
python ~/ros2_ws/src/pointcloud_utils/pointcloud_utils/scripts/open3d_pointcloud_viewer.py \
  --ros-args -p pointcloud_topic:=/camera/depth_registered/points
```

**Jetson validation result:** PyPI provides an Open3D 0.19.0 wheel for Linux x86_64, but not Linux ARM64; this device also encountered a TLS EOF while accessing PyPI. Do not make this command part of the required path or silently compile Open3D from source on Jetson. If installation fails, complete acceptance with the validated RViz2 configuration or a real-time PointCloud2 rendering.

Confirm that a point-cloud topic is active:

```bash
ros2 topic list | grep points

# Official colored cloud: /camera/depth_registered/points
# Custom node output: /camera/points
# Open3D subscribes to the official cloud by default; switch without editing code:
python open3d_pointcloud_viewer.py --ros-args \
  -p pointcloud_topic:=/camera/points
```

> Open3D requires a graphical display. On a headless J501, use `ssh -X user@DEVICE_IP`, attach a monitor, or start VNC before running the script.

> For a quick quality check, use RViz2 as described in Task 1. Open3D is useful when overlaying processing results such as downsampling, plane segmentation, or clustering in the same window.

##### Task 3: Depth Filtering and Hole Filling

> Choose filters according to the downstream task. Obstacle avoidance and visualization may favor continuity and smoothness; measurement, grasping, and mapping must retain the original depth and valid-pixel mask. Every filled hole is an estimate, not a measurement.

**Goal:** understand noise and holes in raw depth images, implement bilateral and temporal filters, and produce smoother and more complete depth maps.

Common problems:

- **Noise:** random depth variation, especially in low-texture or distant regions.
- **Holes:** pixels with depth 0 or NaN, caused by IR absorption, specular reflection, range limits, failed stereo matching, or occlusion.
- **Flying pixels:** isolated erroneous depths near discontinuous object boundaries.

**Method 1: Bilateral filtering—edge-preserving spatial denoising**

A bilateral filter combines spatial proximity with value similarity, smoothing flat regions while preserving object edges.

```python
import cv2
import numpy as np

def bilateral_filter_depth(depth_uint16, d=9, sigma_color=50, sigma_space=50):
    """
    Apply a bilateral filter to a uint16 depth image.
    Convert to float32 first; sigma values must match the depth unit and scale.
    """
    depth_f = depth_uint16.astype(np.float32)
    mask = depth_uint16 > 0
    filtered = cv2.bilateralFilter(depth_f, d, sigma_color, sigma_space)
    result = np.where(mask, filtered, 0).astype(np.uint16)
    return result

# Example
# depth_raw = cv2.imread('depth_raw.png', cv2.IMREAD_UNCHANGED)
# depth_filtered = bilateral_filter_depth(depth_raw, d=9, sigma_color=80, sigma_space=80)
```

The example restores invalid pixels to zero afterward, but the bilateral filter itself does not know that zero means invalid. It is suitable for local visualization smoothing when few holes are present. For depth maps with many holes, prefer the camera's spatial filter or an explicitly mask-aware algorithm, and retain the original depth.

**Method 2: Temporal filtering—suppressing frame-to-frame jitter**

Temporal filtering uses depth consistency across adjacent frames. An exponential moving average (EMA) or median filter at each pixel effectively suppresses jitter, particularly in static or slowly moving scenes.

```python
import numpy as np

class TemporalFilter:
    def __init__(self, alpha=0.3, max_diff=50):
        """
        alpha: smoothing coefficient; lower is smoother but adds latency (0–1).
        max_diff: per-frame depth-change threshold in mm. Larger changes are
                  treated as motion and are not smoothed.
        """
        self.alpha = alpha
        self.max_diff = max_diff
        self.accumulated = None

    def apply(self, depth_uint16):
        if self.accumulated is None:
            self.accumulated = depth_uint16.astype(np.float32).copy()
            return depth_uint16.copy()

        current = depth_uint16.astype(np.float32)
        valid = depth_uint16 > 0
        diff = np.abs(current - self.accumulated)
        stable = valid & (diff < self.max_diff)
        self.accumulated[stable] = (
            self.alpha * current[stable] +
            (1 - self.alpha) * self.accumulated[stable]
        )
        new_valid = valid & ~stable
        self.accumulated[new_valid] = current[new_valid]
        result = self.accumulated.copy()
        result[~valid] = self.accumulated[~valid]
        return result.astype(np.uint16)

# Example
# temporal_filter = TemporalFilter(alpha=0.3, max_diff=50)
# for frame in depth_stream:
#     smoothed = temporal_filter.apply(frame)
```

Temporal filtering adds latency, and filling invalid pixels from history may create ghosts. Use a shorter history window—or only the current valid mask—for moving objects, grasping, and fast obstacle avoidance.

**Method 3: Hole filling**

Strategies for invalid depth pixels include:

- **Nearest-neighbor/inpainting:** use `cv2.inpaint()` on a normalized visualization image; do not treat its output as faithful depth measurements.
- **Morphological closing:** dilation followed by erosion to fill small holes and smooth boundaries.
- **Multi-frame accumulation:** fill current holes with valid values from previous frames, together with temporal filtering.

```python
import cv2
import numpy as np

def fill_small_holes_for_visualization(depth_uint16, max_hole_area=9):
    """Fill only tiny connected zero-depth regions.
    Return both the filled visualization and original validity mask.
    Never use the filled data for dimensional measurement, grasp poses,
    or high-accuracy mapping.
    """
    valid_mask = depth_uint16 > 0
    holes = (~valid_mask).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(holes, connectivity=8)
    dilated = cv2.dilate(depth_uint16, np.ones((3, 3), np.uint8))
    filled = depth_uint16.copy()

    for label in range(1, count):
        area = stats[label, cv2.CC_STAT_AREA]
        if area <= max_hole_area:
            filled[labels == label] = dilated[labels == label]
    return filled, valid_mask

# depth_visual, original_valid_mask = fill_small_holes_for_visualization(depth_raw)
```

> Hole filling introduces estimated values. Use it cautiously in accuracy-sensitive tasks such as 3D reconstruction and robot grasping. Retain both the original and filled depth images for different downstream modules.

---

#### Deliverables

1. **Official colored-cloud validation (required):** provide Gemini 2 launch parameters, the actual cloud topic, an RViz2 screenshot, and checks of the cloud `frame_id` and CameraInfo. For the advanced path, also include the `start_orbbec_rviz.sh` or `orbbec_rviz.launch.py` command and a screenshot showing expanded Color/Depth panels with cloud coverage of approximately 0.2–8.0 m.
1. **RViz2 configuration:** a directly loadable `.rviz` file with Fixed Frame set to the cloud's actual `header.frame_id`; PointCloud2 subscribed to the correct topic with RGB8 Color Transformer; and Grid and TF axes enabled.
1. **Depth-filtering report:** screenshots comparing raw depth, bilateral filtering, temporal filtering, and hole filling, including pseudo-colored depth and corresponding point clouds, plus a brief discussion of parameter effects.

---

#### Questions and Extensions

1. Why does depth quality from structured-light and active-stereo cameras deteriorate in sunlight? Analyze projector power relative to ambient IR noise.
1. Which TF transforms are required to change a cloud's `frame_id` from `camera_color_optical_frame` to `base_link`? Draw the TF tree.
1. How do increasing `sigma_color` and `sigma_space` affect smoothing and edge preservation?
1. Use PCL's VoxelGrid to downsample the cloud and compare point counts and visual quality.
1. Explore NVIDIA Isaac ROS nvblox or Open3D TSDFVolume. How could single-frame RGB-D clouds be fused into a global 3D reconstruction?
