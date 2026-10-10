# 5.2 Tightly Coupled 3D SLAM Mapping with Inertial Odometry

A 2D occupancy grid can already support navigation in simple environments, but it only represents obstacle contours on one horizontal slice. For real-world 3D obstacles such as tables and stairs, a 2D grid makes it hard for the robot to perceive the environment accurately. That is why 3D SLAM based on 3D LiDAR becomes necessary: once the robot enters a more complex 3D environment, it needs a metric map built from 3D point clouds.

This lesson is the advanced follow-up to Lesson 5.1. It extends 2D planar mapping into 3D mapping and introduces tightly coupled LiDAR-inertial odometry. On `reComputer Robotics J501` with Livox MID-360, it deploys Fast-LIO2 and reuses the sensor bring-up already validated in M3.

## Learning Objectives

- understand common 3D LiDAR mapping routes and the basic principle of Fast-LIO2
- understand how IMU odometry works and what information it provides
- independently complete MID-360 driver build, network setup, and point-cloud confirmation under ROS 2
- deploy and launch Fast-LIO2 on J501, then finish one slow indoor mapping run
- save trajectory and PCD artifacts, and judge whether the map is usable by checking geometric consistency, revisit error, and runtime stability

## Prerequisites

- Lesson 5.1 completed, with a reusable 2D occupancy grid in hand
- M3.1 completed: MID-360 can publish LiDAR and IMU topics over the dedicated Ethernet link
- basic concepts of timestamps, frames, and covariance; enough to follow the related M3.2 / M3.4 content
- Ubuntu 22.04 + ROS 2 Humble already available on the board; this lesson installs the driver stack and Fast-LIO2 starting from Livox-SDK2

---

# Part A. From 2D Occupancy Grids to 3D Maps

## 5.2.1 3D SLAM VS 2D SLAM

2D SLAM can often treat one laser sweep as an almost simultaneous planar observation. 3D LiDAR mapping does not work that way.

A spinning or scanning 3D LiDAR keeps scanning the surrounding environment over a non-zero time window. If the robot translates or rotates during that window, earlier returns come back while later beams are still being emitted, and the robot pose used as the reference has already changed. As a result, a single-frame point cloud becomes distorted. Without deskewing, obstacles deform; straight walls and floors may bend, and mapping quality drops.

Therefore, modern 3D LiDAR mapping usually needs at least these pieces:

1. a high-rate IMU prior
2. per-point timestamps, or at least intra-scan timing
3. a state estimator that keeps updating while the robot moves
4. a map structure that expands as the LiDAR moves

On the J501 + MID-360 setup, those conditions are already in place:

- a point cloud with per-point `offset_time`
- an IMU near 200 Hz
- under a reasonable configuration, J501 can carry a Fast-LIO2-class realtime mapping load

![M5.2-1.png](./images/M5.2-1.png)

## 5.2.2 3D LiDAR Mapping

After long development, the common engineering implementations of 3D LiDAR mapping mainly include LiDAR odometry, LiDAR-inertial odometry (LIO), and mapping systems that further combine backend optimization.

An earlier representative route is LOAM. It usually extracts geometric features such as edges and planes from the point cloud, then estimates sensor motion through scan-to-scan or scan-to-map matching. The basic idea is to separate high-rate motion estimation from lower-rate map optimization. That architecture also influenced many later LiDAR mapping systems.

With the introduction of the IMU, LiDAR-inertial odometry (LIO) gradually became an important realtime mapping approach for mobile robots. These methods combine IMU prediction with LiDAR observation: the IMU provides a high-rate motion prior and is used for scan deskewing; LiDAR observations further correct the system attitude, position, velocity, and bias.

On top of that, the Fast-LIO family further adopts a tightly coupled LiDAR-inertial state-estimation framework and achieves realtime localization and mapping at relatively low compute cost. Fast-LIO2 further improves point-cloud processing and map matching, and can use raw point clouds directly for state estimation. While keeping realtime performance, it also offers good accuracy and robustness. Therefore, this section uses **Fast-LIO2** as the example and introduces the deployment and basic usage of a 3D LiDAR-inertial mapping system.

The goal of this section is to complete a repeatable 3D LiDAR mapping workflow on J501. Fast-LIO2 uses a tightly coupled LiDAR-inertial odometry framework. It can directly use the MID-360 point cloud and built-in IMU to perform realtime state estimation, point-cloud deskewing, and incremental map construction, while keeping a relatively low compute load on a Jetson platform. This section mainly focuses on Fast-LIO2 deployment and basic operation. Full loop closure, backend graph optimization, visual fusion, and dense reconstruction will be covered in later advanced material.

In Fast-LIO2, LiDAR and IMU data enter an iterated error-state Kalman filter (iEKF) together for state estimation. First, the IMU predicts the system attitude, position, velocity, and bias at a high rate, and uses the predicted short-term motion to deskew the laser scan. Then the deskewed point cloud is matched against the current incremental map to build LiDAR observation residuals, and the system state is corrected through iterative updates. After state estimation is finished, the aligned point cloud is added to the incremental map and becomes the reference for the next scan match.

---

# Part B. Practice: Deploy Fast-LIO2 on the ROS 2 Humble Stack

This section uses J501 and Livox MID-360 as the hardware platform and completes a repeatable 3D LiDAR mapping workflow. The overall deployment chain is:

**Install Livox-SDK2 → build `livox_ros_driver2` → configure networking and verify the point cloud → build Fast-LIO2 → configure `mid360.yaml` → launch mapping and save results.**

## 5.2.1 Course Environment

The experiments in this section are based on the following hardware and software environment:

| Item | Course value |
| --- | --- |
| OS | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| LiDAR | Livox MID-360 |
| Mapping algorithm | Fast-LIO2 |
| Workspace | `~/ros2_ws` |
| Fast-LIO repository | `https://github.com/Ericsii/FAST_LIO_ROS2` |
| Example host wired IP | `192.168.1.50/24` |
| MID-360 IP rule | usually `192.168.1.1xx`, where the last two digits come from the last two digits of the LiDAR SN |

> **Network note:** if the host IP is not `192.168.1.50`, replace every later network-related setting with the real address, and keep the host IP, the host address in `MID360_config.json`, and the MID-360 IP consistent.

## 5.2.2 Install Livox-SDK2

Livox-SDK2 is the low-level software development kit for connecting Livox LiDARs. This section first builds and installs the SDK so that later `livox_ros_driver2` compilation has the required dependency.

### 1. Clone the repository

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
```

### 2. Build and install

```bash
cd ~/Livox-SDK2
mkdir build && cd build
cmake .. && make -j
sudo make install
```

### 3. Verify the installation

After the SDK is installed, you can use the official sample for a basic check. This step does not require the LiDAR to be connected. If the device is not connected, the program may print device-connection messages, but as long as the SDK starts normally, the build and install process is usually complete.

```bash
find ~/Livox-SDK2 -name "mid360_config.json"
cd ~/Livox-SDK2/build/samples/livox_lidar_quick_start
./livox_lidar_quick_start /home/$USER/Livox-SDK2/samples/livox_lidar_quick_start/mid360_config.json
```

If the actual install path or username differs, replace the path in the command with the corresponding local path.

## 5.2.3 Download and Build `livox_ros_driver2`

After the SDK is installed, continue configuring the Livox ROS 2 driver in the ROS 2 workspace.

### 1. Create the workspace

If the system does not have a workspace yet, run:

```bash
mkdir -p ~/ros2_ws/src
```

### 2. Clone the driver repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

### 3. Prepare the ROS 2 `package.xml`

The `livox_ros_driver2` repository provides `package_ROS1.xml` and `package_ROS2.xml` by default. Copy the ROS 2 version into the standard `package.xml`:

```bash
cd ~/ros2_ws/src/livox_ros_driver2
cp package_ROS2.xml package.xml
```

Without this file, the ROS 2 workspace cannot recognize the package in the standard way, and later builds will fail.

### 4. Use the repository build script

`livox_ros_driver2` should be built with the repository-provided `build.sh`, not by running `colcon build` directly in the workspace:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src/livox_ros_driver2
./build.sh humble
cd ~/ros2_ws
source install/setup.bash
```

Ordinary warning messages during the build usually do not affect the final result. The key is to confirm that the build finishes normally.

## 5.2.4 Configure Networking and Verify the MID-360 Point Cloud

Fast-LIO2 needs both the MID-360 LiDAR point cloud and IMU data. Therefore, before launching the mapping algorithm, first confirm that host-to-LiDAR networking works and that the ROS 2 driver can publish a stable point cloud.

### 1. Configure the host wired NIC

In the system network settings, set the wired NIC connected to MID-360 to manual IPv4. For example:

| Item | Example |
| --- | --- |
| Address | `192.168.1.50` |
| Netmask | `255.255.255.0` |
| Gateway | leave empty |

### 2. Configure `MID360_config.json`

Open the driver config file:

```bash
nano ~/ros2_ws/src/livox_ros_driver2/config/MID360_config.json
```

Make sure the host IP in the config file matches the actual wired NIC address. For example, if the host uses `192.168.1.50`, the corresponding address in the config file should also be `192.168.1.50`.

The default MID-360 IP usually follows the form `192.168.1.1xx`, where the last two digits correspond to the last two digits of the LiDAR SN. For example, if the SN ends with `23`, the corresponding IP is usually:

```text
192.168.1.123
```

In real use, follow the device’s current network configuration.

### 3. Launch the driver and verify the point cloud

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

Under normal conditions, RViz should show a stable MID-360 point cloud. Confirm that the cloud publishes continuously and does not show obvious frame drops, jumps, or whole-cloud drift.

![Screenshot from 2026-09-24 15-35-42.png](./images/Screenshot%20from%202026-09-24%2015-35-42.png)

Only after this step is complete should you move on to Fast-LIO2 deployment.

## 5.2.5 Install and Build Fast-LIO2

### 1. Install dependencies

```bash
sudo apt install -y libeigen3-dev libpcl-dev
sudo apt install -y ros-humble-pcl-conversions ros-humble-pcl-ros
```

### 2. Clone the Fast-LIO2 ROS 2 repository

Early official Fast-LIO versions mainly targeted ROS 1. This section uses the community-maintained ROS 2 Humble version:

```text
https://github.com/Ericsii/FAST_LIO_ROS2
```

Clone with the repository submodules:

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git
```

If you did not use `--recursive` during clone, initialize the submodules manually:

```bash
cd ~/ros2_ws/src/FAST_LIO_ROS2
git submodule update --init --recursive
```

### 3. Build Fast-LIO2

The repository lives under the workspace `src` directory, so return to the `~/ros2_ws` root and run `colcon build`:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select fast_lio --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

To keep later experiment results reproducible, record the current Git commit:

```bash
cd ~/ros2_ws/src/FAST_LIO_ROS2
git rev-parse --short HEAD
```

## 5.2.6 Configure Fast-LIO2 Parameters

First create the map-saving directory:

```bash
mkdir -p ~/maps/fast_lio
```

Then open the Fast-LIO2 config file for MID-360:

```bash
nano ~/ros2_ws/src/FAST_LIO_ROS2/config/mid360.yaml
```

Confirm these parameters:

```yaml
extrinsic_est_en: false

map_file_path: "/home/username/maps/fast_lio/mid360_current.pcd"

pcd_save:
    pcd_save_en: true
    interval: -1
```

Where:

- `extrinsic_est_en: false`: disable online extrinsic estimation and use the fixed extrinsic from the config file for the first deployment.

- `map_file_path`: specify where the map file is saved; an absolute path is recommended.

- `pcd_save_en: true`: enable PCD map saving.

- `interval: -1`: save the final map according to the current configuration.

The LiDAR and IMU topics used by Fast-LIO2 are usually already set in the config file as:

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/livox/imu"
```

If `livox_ros_driver2` has not changed the default topic names, these usually do not need adjustment.

## 5.2.7 Launch Fast-LIO2 Mapping

Fast-LIO2 mapping needs the Livox driver and the Fast-LIO2 node running at the same time. Launch them in two separate terminals.

### Terminal A: start the Livox driver

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### Terminal B: start Fast-LIO2

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

If you also want to start RViz, use:

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=true
```

Note that the point-cloud verification stage uses `rviz_MID360_launch.py`, while formal mapping uses `msg_MID360_launch.py`.

### First mapping tips

For the first run, follow this approach:

1. keep the device still for about 5–10 seconds after launch so the IMU state can initialize fully.

2. start with slow translation and gentle turns.

3. prefer indoor scenes with clear geometric structure such as walls, pillars, and door frames.

4. after a stretch of motion, return to a place with distinctive structure and observe whether revisit drift is obvious.

### Check system status

Use the following commands to check the main topic rates:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | rg -i 'livox|odom|path|cloud'
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
```

![simplescreenrecorder-2026-10-09_16.17.21 (1).gif](./images/simplescreenrecorder-2026-10-09_16.17.21%20%281%29.gif)

## 5.2.8 Save Trajectory and Map

If `pcd_save_en` is already enabled in `mid360.yaml`, call the map-saving service after mapping:

```bash
source ~/ros2_ws/install/setup.bash
ros2 service list | rg map_save
ros2 service call /map_save std_srvs/srv/Trigger {}
ls -lh ~/maps/fast_lio/
```

## 5.2.9 Optimize IMU Parameters to Improve Mapping Accuracy

Fast-LIO2 uses the IMU as an important input for high-rate state prediction. IMU noise parameters affect the estimation of motion states and bias, which can lead to drift and inaccurate localization. Therefore, before dynamic mapping experiments, you can first perform a static noise calibration of the MID-360 built-in IMU.

This section uses an Allan Variance tool under ROS 2 Humble to analyze IMU data. The tool reads long-duration static IMU data from a ROS 2 bag, computes Allan Deviation, and obtains parameters such as noise density and random walk for the gyroscope and accelerometer. The ROS 2 version can process `rosbag2` data directly and does not need conversion into a ROS 1 bag.

### 1. Get the Allan Variance tool

Enter the workspace and clone the ROS 2 version:

```bash
cd ~/ros2_ws/src
git clone https://github.com/cwha0212/allan_variance_ros2.git
```

Install the related dependencies:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the tool:

```bash
colcon build --packages-select allan_variance_ros
source install/setup.bash
```

### 2. Check MID-360 IMU data

Launch the Livox driver:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Confirm that IMU data is publishing normally:

```bash
ros2 topic hz /livox/imu
```

Inspect a single message:

```bash
ros2 topic echo /livox/imu --once
```

Under normal conditions, a `sensor_msgs/msg/Imu` message should contain:

- `angular_velocity`: 3-axis angular velocity;

- `linear_acceleration`: 3-axis linear acceleration;

- `header.stamp`: message timestamp.

### 3. Collect static IMU data

Place the J501 and MID-360 on a stable platform and keep the whole device completely still. During recording, avoid touching the device or introducing other obvious mechanical vibration.

Record the IMU data:

```bash
mkdir -p ~/imu_calib

ros2 bag record \
    -o ~/imu_calib/mid360_imu_static \
    /livox/imu
```

For Allan Variance analysis, long-duration static recording is recommended. Short recordings can only support a quick look at noise characteristics and cannot estimate low-frequency random walk and bias stability well. In practice, follow the tool requirements and collect several hours of data. Recording about 2 hours is recommended; longer recordings usually produce more stable estimates of low-frequency parameters.

After recording finishes, stop with `Ctrl+C`.

Inspect the bag:

```bash
ros2 bag info ~/imu_calib/mid360_imu_static
```

Confirm that it contains:

```text
/livox/imu
```

### 4. Run Allan Variance analysis

If the recorded data has out-of-order timestamps, first clean the data:

```bash
ros2 run allan_variance_ros cookbag.py \
    --input ~/imu_calib/mid360_imu_static \
    --output ~/imu_calib/mid360_imu_cooked
```

Then run the Allan Variance computation:

```bash
ros2 run allan_variance_ros allan_variance \
    ~/imu_calib/mid360_imu_cooked \
    ~/ros2_ws/src/allan_variance_ros/config/realsense_d425i.yaml
```

In real use, choose a config file from the tool repository that matches the MID-360 IMU data rate. The tool outputs Allan Deviation data and generates the corresponding CSV file.

Then run the analysis script:

```bash
ros2 run allan_variance_ros analysis.py \
    --data allan_variance.csv
```

The analysis results mainly include:

| Parameter | Meaning |
| --- | --- |
| Angle Random Walk (ARW) | gyroscope angle random walk |
| Gyroscope Bias Instability | gyroscope bias instability |
| Gyroscope Random Walk | gyroscope random walk |
| Velocity Random Walk (VRW) | accelerometer velocity random walk |
| Accelerometer Bias Instability | accelerometer bias instability |
| Accelerometer Random Walk | accelerometer random walk |

Different regions of the Allan Deviation curve correspond to different IMU noise types. Use the MID-360 calibration result as the actual parameters, and do not directly reuse default parameters from another IMU.

### 5. Apply the results to Fast-LIO2

After calibration, check the IMU noise-related parameters in Fast-LIO2 according to the output, and adjust them following the parameter definitions of the current FAST_LIO_ROS2 version.

Edit:

```bash
nano ~/ros2_ws/src/FAST_LIO_ROS2/config/mid360.yaml
```

Then relaunch Fast-LIO2:

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch fast_lio mapping.launch.py \
    config_file:=mid360.yaml \
    rviz:=true
```

First run a static test. Keep the MID-360 completely still and observe `/Odometry` together with the trajectory change in RViz.

```bash
ros2 topic hz /Odometry
```

If clear continuous drift still appears while the device is stationary, further check the IMU timestamps, noise parameters, LiDAR-IMU extrinsic, and whether the point cloud and IMU data are healthy.

## 5.2.10 Dynamic Moving LiDAR Mapping Verification

After IMU parameter calibration, move into a real motion environment and verify Fast-LIO2 with dynamic mapping. This section completes an indoor loop mapping run by moving the J501 + MID-360 combination, with a focus on localization stability and map consistency during translation, turning, and revisiting the same area.

### 1. Launch the Livox driver

Open the first terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Confirm that LiDAR and IMU data are normal:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

### 2. Launch Fast-LIO2

Open the second terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch fast_lio mapping.launch.py \
    config_file:=mid360.yaml \
    rviz:=true
```

After launch, first keep the device still for about 5–10 seconds so the system can finish initial state estimation.

### 3. Start dynamic mapping

Choose an indoor environment with clear geometric structure for testing, for example:

- walls;

- door frames;

- pillars;

- corridors;

- fixed structures such as desks and chairs.

When moving the device, follow this order:

1. first perform slow straight-line motion and observe whether the point cloud stays stable;

2. keep a relatively gentle angular velocity when passing corners;

3. complete a continuous motion along a corridor or through a room;

4. return to a previously visited area and observe whether the same structure overlaps well;

5. after finishing one loop, return to the start and check overall map consistency.

During the test, try to avoid fast rotation, sudden acceleration, and strong vibration. For the first experiment, prioritize smooth motion first, then gradually increase speed and turning amplitude.

![simplescreenrecorder-2026-10-09_16.17.21 (2).gif](./images/simplescreenrecorder-2026-10-09_16.17.21%20%282%29.gif)

### 4. Observe the mapping result

In RViz, focus on the following:

**Live point cloud**

Observe whether static structures such as walls and door frames remain clear. If clear stretching, ghosting, or overall misalignment appears during motion, first check IMU data, timestamps, and the extrinsic configuration.

**Odometry trajectory**

Observe whether the motion trajectory corresponding to `/Odometry` stays continuous. Under normal conditions, the trajectory should change smoothly with device motion and should not jump frequently.

**Map consistency**

When the device revisits an already mapped area, observe whether the same structure overlaps well with the existing map. This process can be used as an intuitive check of accumulated drift.

### 5. Save the experiment data

After one complete loop, save the 3D map:

```bash
ros2 service call /map_save std_srvs/srv/Trigger {}
```

Check the map file:

```bash
ls -lh ~/maps/fast_lio/
```

It is also recommended to save a ROS 2 bag for later repeated analysis:

```bash
mkdir -p ~/maps/fast_lio

ros2 bag record \
    -o ~/maps/fast_lio/mid360_dynamic_test \
    /livox/lidar \
    /livox/imu \
    /Odometry \
    /path \
    /cloud_registered \
    /tf \
    /tf_static
```

If the topic names on the actual system differ, adjust them according to:

```bash
ros2 topic list
```

### 6. Check the mapping result

After the experiment, check the result from these aspects:

| Check item |  | Observation |
| --- | --- | --- |
| Point-cloud quality |  | whether walls and main structures stay clear, and whether clear smear appears during motion |
| Trajectory continuity |  | whether the motion trajectory is continuous without obvious jumps |
| Map consistency |  | whether revisited point clouds overlap well |
| Accumulated drift |  | whether an obvious position offset appears near the start after completing the loop |
| Realtime performance |  | whether clear delay or data backlog appears during mapping |

If clear drift appears during mapping, recheck the parameters with the IMU calibration result from the previous section, and further confirm the LiDAR-IMU extrinsic and sensor timestamps.

![录屏 2026-10-10 09-40-30.gif](./images/录屏%202026-10-10%2009-40-30.gif)

### Experiment results

After dynamic mapping, keep at least the following results:

- RViz screenshots from the Fast-LIO2 run;

- a ROS 2 bag of the full motion process;

- the saved PCD 3D map;

- `/Odometry` or `/path` trajectory data;

- the IMU calibration result and the final `mid360.yaml` used.

These data can later serve as the basis for map-quality analysis, parameter tuning, and comparisons across algorithms.

### Experiment goals of this section

After finishing this section, you should be able to complete the following workflow independently:

**Static IMU data collection → Allan Variance parameter analysis → Fast-LIO2 parameter configuration → dynamic loop mapping → map and trajectory saving.**

At this point, the basic 3D LiDAR mapping workflow for J501 + MID-360 + Fast-LIO2 is complete. Later sections can build on this and further introduce loop-closure optimization, multi-sensor fusion, and 3D map reconstruction.

## 5.2.11 Lesson Summary

This section completed the full deployment flow from Livox-SDK2 and ROS 2 driver configuration to Fast-LIO2 3D LiDAR mapping. On top of the earlier 2D mapping baseline, it further introduced tightly coupled LiDAR-IMU 3D state estimation, so the system can output a motion trajectory and a 3D point-cloud map in realtime on the J501 platform.

The core goal of this section is to establish a stable and repeatable **MID-360 + Fast-LIO2** mapping environment. After deployment, you should be able to independently complete sensor bring-up, network configuration, mapping launch, and saving of map and trajectory data.

## Next Step

Once Fast-LIO2 can run stably, you can further expand in these directions:

- add a camera on top of LIO and move into multi-sensor fusion mapping;

- introduce loop closure and backend optimization to improve global consistency in large environments;

- continue from the 3D map into environment reconstruction and semantic fusion.

Before entering later content, first make sure the MID-360 and Fast-LIO2 deployment flow can run stably and repeatedly.

## References

- Xu, W., Zhang, F., et al. Fast-LIO / Fast-LIO2.

- Zhang, J., Singh, S. LOAM: Lidar Odometry and Mapping in Real-time.

- Shan, T., et al. LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping.

- [Fast-LIO ROS 2](https://github.com/Ericsii/FAST_LIO_ROS2)

- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)

- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
