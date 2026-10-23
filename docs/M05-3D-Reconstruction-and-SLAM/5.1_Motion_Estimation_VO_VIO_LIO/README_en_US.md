# 5.1 Robot Motion Estimation: Visual and LiDAR-Inertial Odometry

![RTAB-Map software interface: 3D trajectory visualization for robot motion estimation](./images/m5-1-rtabmap-motion-estimation-ui.png)

*Image source: https://limo-agx.github.io/navigation_rtabmap.html*

In the previous lessons, we learned how to connect a Camera, a LiDAR and an IMU to the Jetson, and how to acquire images, point clouds and inertial measurements. However, once the robot starts moving, acquiring sensor data alone is not enough. The robot also has to answer a very basic question: how far did I move just now? In which direction? What is my current position and orientation?

This lesson starts from this question and introduces the fundamentals of robot motion estimation. The learning objectives of this lesson are:

1. **Understand 6DoF poses**: how a robot expresses position and rotation;
2. **Master odometry principles**: how VO, VIO and LIO work;
3. **Know typical algorithms**: RTAB-Map, ORB-SLAM3, FAST-LIO2;
4. **Complete a ROS 2 hands-on lab**: output odometry in real time and observe motion.

## 1. Robot Motion Estimation

For a robot to move autonomously in a real environment, it first has to solve a basic problem: **how does the robot know how far it has moved and where it is currently heading?** For humans, we can judge our own motion through vision, body sensations and other cues. Robots do not have such natural abilities. They must rely on sensors such as cameras, IMUs and wheel encoders to continuously estimate their own motion state. This process is called **robot motion estimation**. Motion estimation is an important foundation for robot localization, mapping, path planning and autonomous navigation. Below, we use an RGB-D camera as an example to understand how a robot infers its own motion from sensor data.

### 1.1 Inferring Robot Motion from Environmental Changes

Suppose we mount an RGB-D camera on a robot. At time $t_0$, the camera sees a table; after the robot moves a distance to the right, it sees the same table again at time $t_1$. Although the position of the table in the world coordinate frame has not changed, its position in the camera image has changed. By analyzing these visual changes, we have an opportunity to infer the camera's own motion.

![Static scene example for motion estimation: the robot's position relative to the same table at times t₀ and t₁](./images/m5-1-motion-estimation-static-scene.png)

This reveals the core idea of visual motion estimation: **infer the camera's own motion by observing how the static environment changes relative to the camera.**

A typical motion estimation process consists of three steps:

1. **Find common scene features**: in the two frames at `t₀` and `t₁`, identify repeatedly observable feature points such as table corners and object edges, and establish correspondences.
2. **Build spatial geometric constraints**: combined with camera intrinsics and depth information, convert the matched image features into points in 3D space, and analyze their positional relationships in the two camera coordinate frames.
3. **Compute the relative pose transform**: find the rotation and translation that best explain these spatial correspondences, yielding the camera's relative motion from `t₀` to `t₁`.

Finally, we use a relative pose transform ΔT to describe this motion.

Note that the robot does not directly measure its absolute position in the world here. Instead, through the geometric relationship between two observations, it estimates **how much it moved and rotated relative to the previous moment**. This is the basic idea of Visual Odometry (VO).

### 1.2 From Relative Motion to a 6DoF Pose

By comparing two consecutive frames, we can estimate how the camera moved. But how do we describe this motion mathematically? For a robot moving in 3D space, knowing the distance traveled is not enough. We also need to describe both its position and its orientation. Therefore, a complete 3D rigid-body pose usually contains six degrees of freedom (6DoF).

Three translational degrees of freedom (Translation) describe the position change along the three coordinate axes: translation along the X axis, translation along the Y axis, and translation along the Z axis; three rotational degrees of freedom (Rotation) describe the orientation change about the three axes: roll, pitch and yaw.

Therefore, **6DoF = 3 translational degrees of freedom + 3 rotational degrees of freedom**.

![6DoF pose composition: 3 translational and 3 rotational degrees of freedom](./images/m5-1-6dof-pose.png)

In SLAM and robotics, a 4×4 homogeneous transformation matrix is commonly used to describe the position and orientation of a 3D rigid body in a unified way:

$$T = \begin{bmatrix}R & t \\0_{1 \times 3} & 1\end{bmatrix}$$

Where:

- **R** is a 3×3 rotation matrix describing the orientation in 3D space;
- **t** is a 3×1 translation vector describing the position;
- **T** belongs to the special Euclidean group SE(3), representing a complete 3D rigid-body transform.

Compared with storing position and Euler angles separately, the homogeneous transformation matrix makes coordinate conversion and composition of consecutive motions convenient.

Back to the RGB-D camera example. Suppose when the robot starts up, the camera is at the initial position `t₀`, and we temporarily define this position as the origin of a reference coordinate frame, with the initial pose $T_0 = I_{4 \times 4}$. When the robot moves to `t₁`, the visual motion estimation algorithm computes the relative pose transform ΔT₀ from `t₀` to `t₁`. The camera's new pose in the reference frame can then be expressed as $T_1 = T_0 \Delta T_0$. If the robot continues to move to `t₂`, then $T_2 = T_1 \Delta T_1$. This pattern generalizes to any pair of consecutive moments.

Pose updates are performed by right-multiplication. By continuously estimating the motion between consecutive moments and composing them in sequence, we obtain the robot's continuous poses relative to the initial position, as well as its trajectory. This is how **odometry** fundamentally works. If we also know the time interval between two consecutive frames, we can estimate the robot's average linear and angular velocities from the pose change.

> It is worth emphasizing that visual odometry directly estimates camera motion. To obtain the pose of the robot chassis itself, the fixed mounting transform between the camera and the chassis also has to be taken into account.

### 1.3 Why Does Motion Estimation Drift?

From the formula above, the robot's current pose depends on the motion estimates from all previous moments. In a real environment, however, every motion estimate can contain errors. The phenomenon in which continuously accumulated relative-motion errors gradually push the estimated trajectory away from the true trajectory is called **odometry drift**. There are many causes of error, for example:

- Blurred images or insufficient texture, leading to inaccurate feature matching;
- Measurement noise in the depth camera, making the 3D point positions less accurate;
- Moving objects in the scene interfering with the geometric estimation of the static environment;
- Fast rotation or vibration of the camera, making matching between consecutive frames difficult.

![Odometry drift illustration: the true trajectory (green) and the estimated trajectory (orange) gradually separate over motion](./images/m5-1-odometry-drift.png)

This also reveals an important limitation of relying on motion estimation alone: **motion estimation can tell the robot "how much I moved relative to the previous moment", but it cannot guarantee long-term positional accuracy from locally accumulated motion alone.**

Therefore, practical SLAM systems usually need additional mechanisms such as stable features in the map, historical observations, loop closure detection and global optimization to constrain and correct the accumulated pose errors. Let us first establish a basic understanding: **motion estimation solves the relative-motion problem between consecutive moments; odometry obtains a trajectory by composing these motion estimates in sequence; and SLAM further exploits the environment map and multi-moment observations to improve the consistency of long-term localization and mapping.**

## 2. The Difference Between VO, VIO and LIO

A robot can estimate its own motion with different sensors, but these techniques are not exactly the same.

<table>
  <tr>
    <td width="45%">
      <img src="./images/m5-1-vo-feature-matching.png" alt="Visual Odometry: ORB feature extraction and brute-force matching diagram" />
    </td>
    <td>
      <strong>VO — Visual Odometry</strong><br/>
      Estimates camera motion from feature positions, photometric information and scene geometry in consecutive images.<br/>
      Suitable for RGB-D, stereo or environments with sufficient visual texture.
    </td>
  </tr>
  <tr>
    <td width="45%">
      <img src="./images/m5-1-vio-sensor-rig.png" alt="Visual-Inertial Odometry: sensor rig combining a camera and an IMU" />
    </td>
    <td>
      <strong>VIO — Visual-Inertial Odometry</strong><br/>
      Uses Camera + IMU. Vision provides environmental constraints while the IMU provides high-frequency inertial prediction.<br/>
      Suitable for robots moving quickly and for scenarios that need smooth pose estimation.
    </td>
  </tr>
  <tr>
    <td width="45%">
      <img src="./images/m5-1-lio-livox-mid360.png" alt="LiDAR-Inertial Odometry: Livox MID-360 point cloud and motion pose visualization" />
    </td>
    <td>
      <strong>LIO — LiDAR-Inertial Odometry</strong><br/>
      Uses LiDAR + IMU, estimating motion through geometric matching of 3D point clouds and inertial constraints.<br/>
      Independent of ambient lighting, but may degrade in environments with uniform geometry.
    </td>
  </tr>
</table>

**Comparison of typical algorithms:**

| Algorithm | Technical approach | Key features | Role in this lesson |
|-|-|-|-|
| ORB-SLAM3 | Feature-based SLAM, can fuse IMU | Loop closure, relocalization, multi-map | Conceptual understanding |
| OpenVINS | MSCKF filter-based VIO | Real-time visual-inertial estimation | Extension |
| RTAB-Map Odometry | RGB-D / Stereo odometry | Complete ROS 2 interface, easy to experiment with | Core practice |
| FAST-LIO2 | Tightly coupled LiDAR-IMU | Iterated EKF and ikd-Tree | Extended experiment |
| LIO-SAM | LiDAR-IMU factor graph | Odometry and graph optimization | Conceptual understanding |

In this lesson, we choose Fast-LIO2 as the motion estimation algorithm. Using the Livox MID-360 point cloud and IMU data, it achieves real-time LiDAR-Inertial Odometry (LIO). The experiment focuses on observing the robot pose, trajectory and accumulated drift, without covering full SLAM loop closure optimization.

## 3. How Does ROS 2 Represent Robot Motion?

Everything introduced so far is theory. In practical engineering, different motion estimation algorithms have different implementations, while modules in a robot system such as mapping, navigation and path planning all need the motion information output by these algorithms. How can the results of different algorithms be used uniformly by other modules? And how do we relate the positions of the camera, the LiDAR and the robot body itself?

To solve these problems, ROS 2 provides a standardized data communication and coordinate transform mechanism. We need to understand two core concepts:

- Odometry (odometry message): describes the robot's current position, orientation and motion velocities;
- TF2 (coordinate transform system): manages the positional and orientational relationships between the robot's various coordinate frames.

### 3.1 Odometry: A Unified Representation of Robot Motion State

Whether motion estimation is performed by RTAB-Map, FAST-LIO2 or wheel encoders, the result ultimately has to be provided to ROS 2. ROS 2 defines the standard message type `nav_msgs/msg/Odometry` for odometry information, which mainly contains the following data:

| Data | Meaning | Example |
|-|-|-|
| Position | The robot's current position | X=2m, Y=1m |
| Orientation | The robot's current orientation | Facing 90° |
| Linear Velocity | Linear velocity | 0.5 m/s |
| Angular Velocity | Angular velocity | 0.2 rad/s |
| Covariance | Uncertainty of the estimate | Pose and velocity covariance |

For example, when the robot moves forward, the motion estimation algorithm continuously updates this data and publishes it through a ROS 2 topic.

![ROS 2 odometry architecture: the motion estimation algorithm publishes standard Odometry messages on the /odom topic to RViz2, state estimation and navigation modules](./images/m5-1-ros2-odom-architecture.png)

In this way, other modules do not need to care about how the motion estimation algorithm is implemented internally. They only need to subscribe to the corresponding topic to obtain the robot's motion state, achieving functional decoupling.

### 3.2 TF2: Unifying the Coordinate Frames of the Robot's Parts

Odometry solves the problem of how motion data is delivered. But a real robot faces another issue: data from different sensors usually lives in different coordinate frames. For example, a camera detects an object 1 meter in front of the camera, but the robot's navigation system needs to know the object's position relative to the chassis. Because the camera and the chassis are mounted at different locations, a coordinate transform is required. In ROS 2, TF2 manages these coordinate relationships.

![Coordinate frames of a mobile robot: map, odom, base_link and camera_link / lidar_link](./images/m5-1-tf-coordinate-frames.png)

The figure above shows common coordinate frames of a mobile robot. The key point here is: Odometry tells us how the robot moves, while TF2 tells us with respect to which coordinate frame these motions and sensor observations are expressed.

### 3.3 Viewing Motion Information in ROS 2

After starting a motion estimation algorithm on the Jetson, we can inspect the results with a few simple commands.

1. **View the odometry message and its update rate:**

```bash
ros2 topic echo /odom --once
ros2 topic hz /odom
```

2. **View the coordinate transform**

Assuming the system has already published `odom → base_link`:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

This command continuously outputs the translation and rotation of the robot relative to the odometry coordinate frame.

3. **Visualize with RViz2**

![Visualizing the robot motion state in RViz2: Odometry and TF displays](./images/m5-1-rviz2-odometry.jpg)

In RViz2, set `Fixed Frame` to `odom`, add the Odometry and TF display components, and you can intuitively observe the robot's motion state and coordinate relationships. Note that different odometry algorithms may use different topic names and coordinate frame configurations; always rely on the actual runtime output.

## 4. Hands-on on the Jetson

We have introduced the fundamentals of robot motion estimation and how ROS 2 represents robot motion with Odometry and TF2. Next, we will use the Jetson and a Livox MID-360 LiDAR to deploy Fast-LIO2, estimating the robot's position and orientation in real time from the point cloud and IMU data, and observe the trajectory in RViz2.

This experiment focuses on three questions:

1. How does Fast-LIO2 receive LiDAR and IMU data?
2. How do we view the real-time position, orientation and trajectory through ROS 2?
3. During robot motion, how is odometry updated, and why does drift occur?

### 4.1 Installing Livox-SDK2

Livox-SDK2 is the low-level software development kit for connecting Livox LiDARs. First, download the SDK:

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
```

Enter the directory, build and install:

```bash
cd ~/Livox-SDK2
mkdir build && cd build

cmake ..
make -j
sudo make install
```

After installation, you can use the sample program provided by the SDK to check whether it starts properly:

```bash
cd ~/Livox-SDK2/build/samples/livox_lidar_quick_start
./livox_lidar_quick_start /home/$USER/Livox-SDK2/samples/livox_lidar_quick_start/mid360_config.json
```

If the SDK starts normally, the low-level software installation is basically complete. When no LiDAR is connected, device connection related messages may appear.

### 4.2 Installing the Livox ROS 2 Driver

After the SDK, configure the ROS 2 driver so that the MID-360 can publish point cloud and IMU data.

1. **Download the driver**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

2. **Build the driver**

```bash
cd ~/ros2_ws
colcon build --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
```

### 4.3 Configuring the MID-360 Network and Verifying Sensor Data

The MID-360 communicates with the Jetson over Ethernet. Before running the driver, configure the Jetson's wired network interface to the same subnet as the LiDAR. This experiment uses the following network configuration:

| Item | Example value |
|-|-|
| Jetson wired IP | 192.168.1.5 (static) |
| Subnet mask | 255.255.255.0 |
| Gateway | 192.168.1.1 |
| MID-360 IP | Default |

You also need to modify the LiDAR configuration file:

```bash
...
"lidar_configs" : [
  {
    "ip" : "192.168.1.3",
    ...
  }
]
...
```

After configuration, start the MID-360 driver:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

Under normal conditions, you should see the real-time updating 3D point cloud in RViz2.

![Real-time updating 3D point cloud in RViz2](./images/m5-1-rviz2-livox-pointcloud.png)

To further verify that the sensor data is being published, open a new terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
```

![Running ros2 topic list in a terminal to view published topics](./images/m5-1-ros2-topic-list-livox.png)

Check the point cloud and IMU topics:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

View an IMU message:

```bash
ros2 topic echo /livox/imu --once
```

Under normal conditions, the IMU message contains angular velocity `angular_velocity`, linear acceleration `linear_acceleration` and the timestamp `header.stamp`. Only continue with the Fast-LIO2 deployment after both the point cloud and IMU are published stably.

### 4.4 Installing and Configuring Fast-LIO2

Fast-LIO2 uses a tightly coupled LiDAR-inertial state estimation approach, combining the IMU's high-frequency motion prediction with point cloud geometric matching to continuously compute the sensor's position and orientation.

1. **Install dependencies**

```bash
sudo apt install -y libeigen3-dev libpcl-dev
sudo apt install -y ros-humble-pcl-conversions ros-humble-pcl-ros
```

2. **Download the Fast-LIO2 ROS 2 version**

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git
```

3. **Build Fast-LIO2**

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select fast_lio --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

4. **Configure the MID-360 parameters**

Open the Fast-LIO2 configuration file:

```bash
vim ~/ros2_ws/src/FAST_LIO_ROS2/config/mid360.yaml
```

Check the point cloud and IMU topics:

```yaml
common:
    lid_topic: "/livox/lidar"
    imu_topic: "/livox/imu"
```

Also check the LiDAR-IMU extrinsic configuration. For the MID-360 built-in IMU, the configuration must follow the actual coordinate relationship of the device and the driver. Do not casually use the extrinsics of other sensors.

For a first deployment, you may disable online extrinsic estimation:

```yaml
extrinsic_est_en: false
```

The algorithm will then use the fixed extrinsics from the configuration file.

### 4.6 Starting Fast-LIO2

After configuration, start the driver and the algorithm in two terminals.

**Terminal A: start the Livox driver**

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

![Livox driver startup terminal log](./images/m5-1-livox-driver-log.png)

**Terminal B: start Fast-LIO2**

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch fast_lio mapping.launch.py \
  config_file:=mid360.yaml \
  rviz:=true
```

After startup, first keep the MID-360 stationary for about 5–10 seconds so that the system completes the initial state estimation. Then move the LiDAR slowly and observe the real-time updating point cloud and trajectory in RViz2.

![Fast-LIO2 real-time updating point cloud and trajectory in RViz2](./images/m5-1-fastlio2-rviz2-trajectory.png)

### 4.7 Viewing Real-Time Odometry Data

In Section 3, we introduced how ROS 2 uses `nav_msgs/msg/Odometry` to represent the robot's motion state. Now we verify this mechanism with Fast-LIO2's actual output.

Open a third terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
```

Locate the `/Odometry` topic published by Fast-LIO2.

![Locating the /Odometry topic in the ros2 topic list output](./images/m5-1-ros2-topic-list-odometry.png)

First check its message type:

```bash
ros2 topic info /Odometry
```

Then view one real-time message:

```bash
ros2 topic echo /Odometry --once
```

Under normal conditions, you can see odometry fields such as `pose`, which contain the current position and orientation.

1. **Observe position changes**

```bash
ros2 topic echo /Odometry \
  --field pose.pose.position
```

Keep the LiDAR stationary and record the initial position. Then move the LiDAR slowly in a straight line and observe the changes in the X, Y and Z coordinates. Note that these coordinates describe the sensor's position relative to the reference frame, and do not necessarily correspond directly to the robot chassis's front/back/left/right directions.

2. **Observe orientation changes**

```bash
ros2 topic echo /Odometry \
  --field pose.pose.orientation
```

Rotate the LiDAR slowly and check whether the quaternion data changes. This corresponds to the three rotational degrees of freedom of the 6DoF pose introduced earlier.

3. **Check the odometry update rate**

```bash
ros2 topic hz /Odometry
```

If the rate information is continuously output, Fast-LIO2 is publishing odometry data.

### 4.8 Observing the Trajectory with RViz2

After verifying the data, we can observe the motion estimation result more intuitively in RViz2. Keep Fast-LIO2 running, check the point cloud and trajectory displays in RViz2, and configure the Odometry, Path and TF display components according to the actually published topics. For the first experiment, choose an indoor environment with clear geometric structure, such as walls, door frames, pillars and corridors.

Under normal conditions, Fast-LIO2 should continuously update the sensor pose as the device moves. If the trajectory jumps, the point cloud stretches, or obvious ghosting appears during motion, check the IMU data, timestamps, the LiDAR-IMU extrinsics, and whether fast rotation or strong vibration is occurring.

## 5. Summary

This lesson introduced the fundamentals of robot motion estimation, how VO, VIO, LIO and ROS 2 odometry work, and implemented real-time pose estimation and trajectory visualization with Fast-LIO2.

**Motion estimation can continuously track the robot's motion state, but it is hard to avoid long-term accumulated errors.** In the next lesson, we will learn SLAM techniques to build environment maps and perform global optimization.
