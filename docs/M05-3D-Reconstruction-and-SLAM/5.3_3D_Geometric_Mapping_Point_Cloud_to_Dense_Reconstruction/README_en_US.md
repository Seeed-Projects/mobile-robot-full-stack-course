# 5.3 3D Geometric Mapping: From Point Cloud to Dense Reconstruction

In the previous lesson, we used RTAB-Map to implement RGB-D SLAM, completing robot motion estimation, loop closure and global map optimization. However, for autonomous navigation and robotic arm manipulation, knowing one's own position is not enough. The robot also needs to understand **which spaces are occupied by obstacles, how far away the obstacles are, and how to use this information for motion planning.** To this end, we need to combine the depth data captured by an RGB-D camera with the pose information provided by SLAM to build a 3D map that describes the geometric structure of the environment and obstacle distances.

This lesson starts from the 3D point cloud, introduces the fundamentals of **Voxel, TSDF, ESDF and Mesh** in turn, and uses **Isaac ROS Nvblox** to implement GPU-accelerated 3D mapping on the Jetson.

---

## 1. From SLAM Maps to 3D Geometric Maps

**SLAM map:** An example of an RTAB-Map map with an optimized trajectory, focusing on robot poses and the global consistency of the map.

**3D geometric map:** An example of Nvblox 3D surface and distance field visualization in RViz, focusing on environment shape and obstacle distances.

The SLAM system learned earlier mainly solves localization and map consistency problems, while the geometric mapping in this lesson further describes the shapes and spatial distribution of objects in the environment. The connection between the two can be summarized as:

![The complete pipeline from an RGB-D camera to a 3D geometric map: SLAM pose + depth fusion → TSDF → Mesh / ESDF](./images/m5-3-rgbd-to-3d-map-pipeline.png)

In this pipeline, RTAB-Map provides spatial poses, while Nvblox builds the geometric map from depth data and poses. They do not replace each other; they take on different tasks.

---

## 2. Core Representations of 3D Spatial Maps

### 2.1 Sparse Point Clouds and Dense Point Clouds

![Comparison of sparse and dense point clouds: differences in density, coverage and geometric detail](./images/m5-3-sparse-vs-dense-pointcloud.png)

A point cloud is the most intuitive 3D geometric representation. Each point usually contains XYZ coordinates and may also contain RGB color information.

| | Sparse point cloud | Dense point cloud |
|-|-|-|
| Data source | Image feature points | A large number of valid depth pixels |
| Point density | Low | High |
| Main use | SLAM localization and feature matching | 3D space reconstruction |
| Geometric detail | Incomplete | Relatively rich |

For example, a 640×480 depth image contains 307,200 pixels. If all pixels have valid depths, up to about 307 thousand 3D points can theoretically be back-projected. However, a point cloud itself is only a set of discrete coordinates. If the robot wants to perform efficient collision checking and obstacle distance computation, a more structured spatial representation is needed.

### 2.2 Voxel: Dividing 3D Space into Small Cubes

> Figure: an example of 3D space voxelization. The space is discretized into many small cubes, and each voxel can hold local map information.

A voxel can be understood as a 3D version of a pixel. A 2D image is made of pixels, while a 3D voxel map is made of small cubes with volume. Each voxel can record occupancy state, distance, color or semantic information. The voxel resolution determines both the geometric detail of the map and its resource consumption. As the voxel resolution increases, the theoretical data volume grows rapidly. Therefore, a balance must be struck between map accuracy and the computational resources of the Jetson GPU.

### 2.3 Three Core Maps: Occupancy, TSDF and ESDF

![Comparison of three map representations: Occupancy, TSDF and ESDF](./images/m5-3-occupancy-tsdf-esdf.png)

**Occupancy Grid: where are the obstacles?**

Each grid cell or voxel records a free, occupied or unknown state, and is suitable for judging whether a space is traversable.

**TSDF: where is the object surface?**

TSDF (Truncated Signed Distance Field) mainly describes signed distances near the object surface and fuses multiple depth observations. In the figure above, the black dashed line represents the object surface, and the surrounding grid cells use signed distance values to express the space on both sides of the surface.

**ESDF: how far is it to an obstacle?**

ESDF (Euclidean Signed Distance Field) describes the Euclidean distance from a position in space to the nearest obstacle surface.

The key difference between TSDF and ESDF: TSDF distances are mainly represented within a finite truncated region around the object surface, making it better suited for surface reconstruction; ESDF focuses on the distance from a spatial position to the nearest obstacle, making it better suited for safety distance queries and motion planning.

### 2.4 Mesh: Recovering 3D Space as a Continuous Surface

![Nvblox 3D space reconstruction result: the Mesh surface extracted from TSDF](./images/m5-3-nvblox-mesh-reconstruction.png)

*Image source: https://forums.developer.nvidia.com/t/isaac-ros-nvblox-2-1-with-live-realsense-camera/346703*

The TSDF and ESDF introduced above are both voxel-based distance field representations. However, if we want to display a complete 3D room on screen, merely displaying voxels is usually not the most intuitive approach. Therefore, we can further extract a Mesh (triangle mesh) from the TSDF.

A common surface extraction algorithm is Marching Cubes. It finds the zero-crossing positions of the surface from the distance values of the surrounding voxels and builds triangles. In this way, the robot can obtain a more intuitive 3D geometric surface model.

---

## 3. Isaac ROS Nvblox: GPU-Accelerated 3D Mapping on the Jetson

In the previous sections, we learned about 3D map representations such as point clouds, Voxel, TSDF, ESDF and Mesh. Among them, TSDF can fuse depth observations and reconstruct object surfaces, while ESDF can further provide distance information between the robot and obstacles. So how do we apply these theories to a real robot system?

In M5.2, we already obtained real-time camera poses with RTAB-Map. Next, by combining the depth images captured by the RGB-D camera, we can fuse spatial observations from different viewpoints into a unified 3D map.

![Isaac ROS Nvblox core workflow: perception → SLAM/localization → GPU 3D geometric mapping → outputs](./images/m5-3-nvblox-core-workflow.png)

To implement this process, this lesson uses NVIDIA Isaac ROS Nvblox. It can leverage the Jetson GPU to accelerate depth fusion, TSDF map updates, Mesh reconstruction and ESDF computation, taking the robot from trajectory estimation to real-time 3D spatial mapping.

### 3.1 The Core Workflow of Nvblox

Isaac ROS Nvblox is a 3D reconstruction and robot spatial map computation component provided by NVIDIA. It receives depth images, camera intrinsics and pose information, updates the 3D map with the GPU, and provides both Mesh visualization and ESDF distance maps.

![Nvblox 3D mapping result: warehouse environment reconstruction, color image and depth mask image](./images/m5-3-nvblox-warehouse-mapping.png)

Relationship with RTAB-Map from the previous lesson: RTAB-Map can correct the historical trajectory through loop closure, but when Nvblox performs real-time incremental fusion, it does not automatically rebuild the already fused TSDF map from new historical poses. Therefore, the experiment below first uses the continuous `odom` coordinate frame as the Nvblox mapping frame to verify real-time local 3D reconstruction. Re-fusion and long-term maintenance of large-scale global maps are left as further research topics.

---

## 4. Hands-on: Real-Time 3D Mesh Reconstruction with Nvblox

### 4.1 Checking the Base Environment

First, confirm on the Jetson host that ROS 2 and RTAB-Map are available:

```bash
source /opt/ros/humble/setup.bash

echo $ROS_DISTRO
ros2 pkg prefix rtabmap_launch
ros2 pkg prefix rtabmap_odom
ros2 pkg prefix orbbec_camera
```

![Checking the ROS 2 and RTAB-Map package paths in a terminal](./images/m5-3-ros2-env-check.png)

If all packages can be located, the environment satisfies the requirements of the later experiments. Otherwise, refer to the earlier content to configure the software runtime environment.

### 4.2 Installing Isaac ROS Nvblox

Prepare the workspace on the Jetson device:

```bash
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src

git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
cd ~/workspaces/isaac_ros-dev
./src/isaac_ros_common/scripts/run_dev.sh
```

> If your Jetson device does not have Docker, refer to the [Beginner's Guide](https://github.com/Seeed-Projects/reComputer-Jetson-for-Beginners/blob/main/3-Basic-Tools-and-Getting-Started/3.7-Docker/README.md) to install Docker.

![Isaac ROS development container creation: image pull and container startup logs](./images/m5-3-isaac-ros-container.png)

Install Nvblox inside the container:

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y ros-humble-isaac-ros-nvblox
```

![Querying the Nvblox package path inside the container](./images/m5-3-nvblox-pkg-prefix.png)

### 4.3 Starting the Camera and SLAM

This stage needs four terminals, forming the following runtime structure:

![Four-terminal runtime structure: the Gemini 2 camera and RTAB-Map SLAM feed the Nvblox container, which outputs RViz / Mesh / ESDF](./images/m5-3-four-terminal-architecture.png)

**Terminal 1: start the Orbbec Gemini 2**

Run on the Jetson host:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch orbbec_camera gemini2.launch.py \
    enable_color:=true \
    enable_depth:=true \
    depth_registration:=true \
    align_mode:=HW \
    enable_frame_sync:=true \
    depth_colorizer_mode:=none \
    enable_point_cloud:=false \
    enable_ir:=false
```

Check the camera data:

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/depth/image_raw --once --field header
ros2 topic echo /camera/depth/image_raw --once --field encoding
```

Nvblox supports floating-point depth in meters, as well as 16-bit depth images in OpenNI format in millimeters. But the corresponding CameraInfo must contain the correct camera intrinsics.

**Terminal 2: start RTAB-Map**

Follow the M5.2 SLAM configuration and turn off the visualization windows:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch rtabmap_launch rtabmap.launch.py \
    visual_odometry:=true \
    depth:=true \
    frame_id:=camera_link \
    vo_frame_id:=odom \
    odom_topic:=/odom \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/depth/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=true \
    qos:=2 \
    publish_tf_odom:=true \
    publish_tf_map:=true \
    database_path:=$HOME/slam_maps/m5_3_gemini2.db \
    rtabmap_viz:=false \
    rviz:=false
```

Check the TF:

```bash
ros2 run tf2_ros tf2_echo \
  odom camera_link
```

You also need to confirm that the optical coordinate frame corresponding to the depth image `header.frame_id` can be transformed to `odom` through TF.

### 4.4 Configuring the Interface Between Gemini 2 and Nvblox

NVIDIA officially provides examples such as RealSense and Isaac Sim, but this lesson uses the Gemini 2, so a custom topic mapping must be set up.

![Gemini 2 and Nvblox interface configuration: depth image, CameraInfo and TF/pose feed into Nvblox](./images/m5-3-gemini2-nvblox-interface.png)

First create the Launch directory:

```bash
mkdir -p ~/workspaces/isaac_ros-dev/m5_3_launch
```

Then create `gemini2_nvblox.launch.py` with the following example configuration:

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    nvblox = ComposableNode(
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        name='nvblox_node',

        remappings=[
            ('camera_0/depth/image',
             '/camera/depth/image_raw'),

            ('camera_0/depth/camera_info',
             '/camera/depth/camera_info'),

            ('camera_0/color/image',
             '/camera/color/image_raw'),

            ('camera_0/color/camera_info',
             '/camera/color/camera_info'),
        ],

        parameters=[{
            'global_frame': 'odom',
            'num_cameras': 1,
            'voxel_size': 0.05,
            'mapping_type': 'static_tsdf',
            'esdf_mode': '2d',

            'use_depth': True,
            'use_color': True,
            'use_lidar': False,

            'use_tf_transforms': True,
            'use_topic_transforms': False,
            'input_qos': 'SENSOR_DATA',

            'map_clearing_frame_id': 'camera_link',
            'map_clearing_radius_m': 0.0,

            'esdf_slice_bounds_visualization_attachment_frame_id':
                'camera_link',

            'static_mapper.esdf_slice_min_height': 0.0,
            'static_mapper.esdf_slice_max_height': 1.2,
            'static_mapper.esdf_slice_height': 0.5,
        }]
    )

    container = ComposableNodeContainer(
        name='nvblox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[nvblox],
        output='screen'
    )

    return LaunchDescription([container])
```

Among these, the core parameters are:

| Parameter | Meaning |
|-|-|
| `global_frame` | The coordinate frame of the 3D map |
| `voxel_size` | Voxel edge length, 5 cm in this example |
| `mapping_type` | Static TSDF mapping |
| `esdf_mode` | 2D ESDF distance slice |
| `use_tf_transforms` | Obtain poses through TF |
| `map_clearing_radius_m` | 0 disables radius-based map clearing |

Parameter names and meanings can be found in NVIDIA's official [Nvblox ROS Parameters](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/parameters.html).

There are two points that need to be verified with actual testing.

First, the Gemini 2's aligned depth image must match the CameraInfo provided to Nvblox. If the depth image published by the driver is aligned to the RGB coordinate frame, the CameraInfo must be verified against the actual image size, intrinsics and frame_id; do not blindly reuse the original depth intrinsics.

Second, the 2D ESDF is sliced by default along the height axis of the map coordinate frame. If the Z axis of `odom` is not correctly aligned with the true vertical direction, the distance slice may not match the real horizontal ground. The camera orientation and coordinate definition must be confirmed before using it for real obstacle avoidance.

> This custom Launch is a Gemini 2 adaptation example written from the official interface, and has not yet been verified in a real J501 + Gemini 2 environment.

### 4.5 Starting Nvblox

**Terminal 3: enter the Isaac ROS container on the Jetson host**

```bash
cd ~/workspaces/isaac_ros-dev

./src/isaac_ros_common/scripts/run_dev.sh
```

Inside the container, run:

```bash
source /opt/ros/humble/setup.bash

ros2 launch /workspaces/isaac_ros-dev/m5_3_launch/gemini2_nvblox.launch.py
```

If the node starts successfully, check whether it can subscribe to the RGB-D data:

```bash
ros2 node list
ros2 topic info /camera/depth/image_raw -v
ros2 topic list | grep nvblox
```

Under normal conditions, Mesh and ESDF topics related to Nvblox appear.

Note: since the camera and RTAB-Map run on the host while Nvblox runs in a Docker container, both sides must be able to communicate over ROS 2 DDS. If the container cannot discover the camera topics, check the ROS_DOMAIN_ID, the network mode and the DDS configuration.

### 4.6 Observing the 3D Reconstruction in RViz

**Terminal 4: start RViz2 on the Jetson**

```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

Then configure the visualization content in the RViz window.

**Global Options → Fixed Frame: `odom`**

**Add → By display type → nvblox_rviz_plugin → NvbloxMesh**

**Topic: `/nvblox_node/mesh`**

![Configuring the NvbloxMesh display in RViz: Fixed Frame and Topic settings](./images/m5-3-nvblox-mesh-rviz-config.png)

Under normal conditions, once Nvblox receives the depth image and a valid camera pose, the 3D surface of the environment appears in RViz. Below is the result we obtained by running it on a Jetson AGX Thor:

![Real-time Nvblox 3D Mesh reconstruction result on the Jetson AGX Thor](./images/m5-3-nvblox-rviz-mesh-result.png)

We have already seen the 3D shape of objects through the Mesh. Next, we observe the ESDF further to verify whether the 3D geometric map can provide obstacle distance information.

Back in RViz, with the Mesh already added, click `Add` and add a `PointCloud2` display. Set the Topic to:

```bash
/nvblox_node/static_esdf_pointcloud
```

We suggest temporarily turning off `NvbloxMesh` and displaying only the ESDF, so that the two maps can be distinguished.

![Mesh and ESDF overlaid in RViz: comparison of surface and distance information](./images/m5-3-nvblox-esdf-rviz.png)

Here we can observe that the Mesh expresses obstacle surfaces, while the ESDF expresses the robot's distance to obstacles.

---

## 5. Summary

In this lesson, starting from RGB-D depth data, we introduced 3D map representations such as point clouds, Voxel, Occupancy Grid, TSDF, ESDF and Mesh, and used Isaac ROS Nvblox to implement real-time 3D geometric reconstruction on the Jetson.

Through this experiment, we successfully ran Nvblox on a Jetson AGX Thor and observed a real-time 3D Mesh in RViz. Next, by checking camera-motion-map linkage, ESDF distance verification and map export, we can further inspect the completeness and outputs of the 3D mapping pipeline. From the map representation perspective, TSDF is mainly used for depth fusion and surface reconstruction, Mesh intuitively expresses the 3D geometry of the environment, and ESDF further provides the distance to the nearest obstacle, laying the foundation for obstacle avoidance and motion planning. At this point, the robot can not only estimate its own motion, but also describe the 3D spatial structure around it.

---

## References

1. [NVIDIA Isaac ROS Nvblox documentation](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_nvblox/index.html)
2. [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2)
3. [Robbyant/lingbot-map](https://github.com/Robbyant/lingbot-map)
4. [Seeed Wiki: deploy_nvblox_jetson_agx_orin](https://wiki.seeedstudio.com/deploy_nvblox_jetson_agx_orin)
