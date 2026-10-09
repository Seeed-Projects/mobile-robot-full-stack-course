# 5.1 Understanding SLAM and Building Your First Occupancy Grid Map

SLAM (Simultaneous Localization and Mapping) is a foundational technology for mobile-robot navigation and environment perception. While the robot moves, it must estimate both **its own pose** and a **map of the surrounding environment**. Only when the robot knows where it is, and builds a map that can be reused over time, can it move on to path planning, obstacle avoidance, relocalization, and autonomous navigation.

This lesson first explains the problem SLAM needs to solve and the main technical routes used in practice. Then, on `reComputer Robotics J501`, it uses `RPLIDAR A1` and **SLAM Toolbox** to complete a first 2D LiDAR SLAM workflow. This practice becomes the baseline for later M5 LiDAR SLAM content: start from a structurally simple and easy-to-validate 2D laser map, then move on to 3D LiDAR and LiDAR-inertial odometry.

![M5-1.jpg](./images/M5-1.jpg)

## Learning Objectives

After completing this lesson, you should be able to:

- explain the basic SLAM task in engineering language, and why pose and map must be estimated together;

- distinguish the main SLAM routes based on 2D LiDAR, 3D LiDAR, vision, and multi-sensor fusion;

- understand the roles of frames such as `map`, `odom`, `base_link`, and `laser` in a SLAM system;

- complete 2D LiDAR mapping on J501 with RPLIDAR A1 and SLAM Toolbox;

- save a reusable 2D occupancy grid map;

- load an existing map and perform map-based localization.

## Prerequisites

Before starting this lesson, make sure that:

- **M1**: J501 has been flashed, and Ubuntu 22.04 with ROS 2 Humble works normally;

- **M3.1**: RPLIDAR A1 is already integrated, can publish `/scan`, and shows a stable scan in RViz2;

- **M3.2 / M3.4**: the robot already has wheel odometry, and `odom -> base_link` has only one publisher;

- you already know the basic ROS 2 topic and TF workflow;

- you understand the basic relationship among `map`, `odom`, `base_link`, and `laser`.

---

# Part A. The SLAM Task

## 5.1.1 What Is SLAM

A single sensor usually provides only local information.

For example:

- a LiDAR tells the robot which obstacles are nearby right now;

- a camera tells the robot what is visible in the current field of view;

- wheel odometry estimates how far the robot moved from the previous moment;

- an IMU provides short-term angular-rate and acceleration changes.

None of these signals alone can directly answer two key questions:

1. **Where is the robot in the environment right now?**

2. **How should the observed structure be organized into a reusable map?**

SLAM jointly solves these two problems while the robot moves.

If the map were already perfectly accurate, the robot would only need to localize against it. If the robot pose were always perfectly accurate, mapping would also become simple. In a real robot, every sensor has noise, odometry accumulates error, and environment observations can be ambiguous. Therefore the robot must keep correcting both its pose and the map while it moves and observes.

From an engineering viewpoint, a SLAM system eventually needs to provide outputs such as:

| Output | Role |
| --- | --- |
| Robot pose | continuously track the robot position and orientation in the environment |
| Environment map | store the spatial structure already observed by the robot |
| Frame relationships | let navigation, planning, and perception modules share one spatial coordinate system |
| Estimation status and consistency information | judge whether localization is reliable and whether drift is becoming obvious |

For this course, you can first understand the whole SLAM system as:

```text
Sensors
  │
  ├── /scan
  ├── /odom
  ├── camera
  └── IMU
       │
       ▼
   SLAM / Localization
       │
       ├── Robot Pose
       ├── Map
       └── TF
             │
             ▼
      Nav2 / Planning / Perception
```

In other words, SLAM is not an isolated “map-drawing program.” It is a core link in the robot’s spatial state-estimation chain.

![M5-2.png](./images/M5-2.png)

---

## 5.1.2 The Core Process of SLAM

The core SLAM problem can be understood simply as:

> The robot moves while observing the environment with sensors, estimates its own location at the same time, and organizes those observations into a consistent map.

When the robot just starts moving, it only knows its initial pose and a small nearby portion of the environment. As motion continues, it obtains more and more observations.

If the robot relies on odometry alone, pose error keeps accumulating. Therefore SLAM also needs geometric structure in the environment as constraints. For example, after the robot leaves a room and later returns to the same room, the LiDAR may see similar walls, door frames, or pillars again. The system can then judge:

> “The place I am seeing now is likely a place I have already visited.”

That is **loop closure**.

With loop constraints, previously accumulated pose error can be corrected again so that the whole trajectory and map stay consistent.

Therefore, a typical SLAM system usually contains these core processes:

1. **Motion prediction (Predict)**  
   Predict the next robot pose from wheel odometry, IMU, or visual odometry.

2. **Environment observation (Observe)**  
   Use sensors such as LiDAR or cameras to obtain current environment information.

3. **Data association (Associate)**  
   Decide how the current observation corresponds to the existing map or historical observations.

4. **Pose update (Update)**  
   Correct the current robot pose according to sensor observations.

5. **Map update (Map Update)**  
   Add the new environment observation into the map.

6. **Loop correction (Loop Closure)**  
   When the robot revisits a known area, apply a global correction to the historical trajectory.

7. **Result publishing (Publish)**  
   Output the map, robot pose, and the TF / topics needed by other modules.

This process can be summarized as:

```text
Motion prior + environment observation
        │
        ▼
    Data association
        │
        ▼
    Pose update
        │
        ▼
    Map update
        │
        ▼
     Loop detection
        │
        ▼
   Globally consistent map
```

---

## 5.1.3 Basic Components of a SLAM System

Different SLAM algorithms are implemented differently, but from an engineering structure viewpoint, most systems can be broken into a few basic parts.

| Module | Main role | Common implementations |
| --- | --- | --- |
| Front-end | convert sensor data into motion or geometric constraints | scan matching, feature tracking, point-cloud matching, visual odometry |
| Back-end | optimize robot states according to constraints | filters, pose graphs, factor graphs, nonlinear optimization |
| Map | store environment spatial information | occupancy grids, point clouds, meshes, TSDF/ESDF |
| TF / Output | provide spatial relationships to other robot modules | `map -> odom`, `odom -> base_link`, trajectories, map files |

Among these, the front-end and back-end can first be understood as:

**The front-end is responsible for “what was seen and what motion happened”; the back-end is responsible for “how to adjust the whole trajectory to become more consistent under those constraints.”**

For 2D LiDAR SLAM:

```text
LaserScan
    │
    ▼
Scan Matching
    │
    ▼
Local Pose Estimate
    │
    ▼
Pose Graph / Optimization
    │
    ▼
Occupancy Grid Map
```

This structure also explains why SLAM is not simply “drawing laser points onto a map.” The hard part is deciding **where the current observation should be placed**, and how to keep the whole map consistent over long-term operation.

---

## 5.1.4 Main SLAM Routes

Which SLAM technology to use in practice mainly depends on the robot’s sensors, the structure of the environment, and the final map form that is needed.

Because this course will later focus on 2D LiDAR and 3D LiDAR, first build a basic understanding of the following four technical routes.

### 1) 2D LiDAR SLAM

**Input:** 2D laser scans, usually combined with wheel odometry.  
**Typical map:** a 2D occupancy grid.  
**Main use:** indoor mobile-robot navigation.

2D LiDAR SLAM is a typical entry path for indoor wheeled robots. It directly produces the 2D occupancy grids commonly used by navigation systems, so it is a natural baseline map for Nav2.

In environments with clear structure such as walls, corridors, and rooms, 2D LiDAR can provide stable geometric constraints while keeping computation relatively low, which makes it convenient to run on edge platforms such as J501.

Its limitation is also clear: the laser only observes one horizontal slice, so it cannot fully describe overhanging objects, ramps, or spatial structure at different heights. For environments with significant vertical change, 3D sensing is needed.

The **RPLIDAR A1 + SLAM Toolbox** setup used in this lesson belongs to this technical route.

---

### 2) 3D LiDAR SLAM

**Input:** 3D point clouds, usually combined with an IMU.  
**Typical map:** a 3D point cloud, a voxel map, or a further projected 2D map.  
**Main use:** large spaces, complex structure, and 3D environment modeling.

When the environment cannot be described well by a single 2D plane, 3D LiDAR is needed.

3D LiDAR can observe walls, pillars, furniture, ramps, and other non-planar structure. Combined with an IMU, it can also estimate short-term motion from high-rate inertial information and compensate for motion distortion during a laser scan.

Therefore, Lesson 5.2 continues from this lesson’s 2D LiDAR baseline and further uses **Livox MID-360 + Fast-LIO-style LiDAR-inertial odometry**.

3D SLAM is more capable, but system complexity also increases clearly. Time synchronization, IMU noise, LiDAR-IMU extrinsics, and motion deskewing all directly affect the final result.

---

### 3) Visual SLAM

**Input:** monocular, stereo, or RGB-D cameras.  
**Typical map:** feature points, keyframe maps, dense or semi-dense reconstruction.  
**Main use:** pose estimation, place recognition, and visual scene understanding.

Visual SLAM mainly uses texture and features in images for localization and mapping.

Compared with LiDAR, vision can obtain richer appearance information, so it can further support place recognition and semantic understanding. But visual systems are also more easily affected by lighting changes, motion blur, weak texture, and occlusion.

Among them, monocular visual SLAM also needs special attention to scale; stereo and RGB-D can provide more direct depth information.

Therefore, in real robot systems, visual SLAM is often combined with an IMU or other sensors.

---

### 4) Multi-sensor Fusion SLAM

**Input:** LiDAR + IMU, vision + IMU, or LiDAR + vision + IMU.  
**Goal:** improve system stability by exploiting complementarity among sensors.

Different sensors have different strengths:

- LiDAR can provide relatively stable geometric ranges;

- an IMU can provide high-rate short-term motion information;

- a camera can provide texture and appearance information;

- wheel odometry can provide chassis motion constraints.

Therefore, multi-sensor SLAM can use one sensor to make up for the weakness of another.

For example:

```text
LiDAR ────────┐
              ├──► LiDAR-Inertial SLAM
IMU ──────────┘

Camera ───────┐
              ├──► Visual-Inertial SLAM
IMU ──────────┘
```

But the more sensors you add, the higher the requirements become for **time synchronization, extrinsic calibration, frame definitions, and data quality**.

Therefore, this course follows a simple-to-complex learning path: first complete 2D LiDAR SLAM, then move into 3D LiDAR and IMU fusion.

---

## 5.1.5 How to Choose a SLAM Route

No single SLAM algorithm fits every robot. When choosing in practice, first consider what map the robot finally needs, and what characteristics the environment has.

| Robot need | Recommended starting point | Main reason |
| --- | --- | --- |
| Indoor differential-drive navigation | 2D LiDAR SLAM | can directly generate a navigation occupancy grid |
| Halls, campuses, semi-open spaces | 3D LiDAR / LIO | can retain more 3D geometric structure |
| Semantic inspection and place recognition | vision or vision fusion | images contain rich appearance information |
| Highly dynamic, GNSS-denied environments | LiDAR-inertial / visual-inertial | IMU can provide high-rate motion constraints |
| Product navigation oriented to Nav2 | 2D or layered maps | planners need clear free space and obstacle information |

This lesson does not try to cover every SLAM technology at once. Instead, it first establishes a baseline that can actually be run, validated, and debugged:

```text
RPLIDAR A1
    ↓
2D LaserScan
    ↓
SLAM Toolbox
    ↓
2D Occupancy Grid
    ↓
Nav2
```

After this chain is complete, move on to 3D LiDAR SLAM in the next lesson.

---

# Part B. Practice: Complete Your First 2D LiDAR Map on J501

This practice uses **SLAM Toolbox** under ROS 2 Humble to complete 2D LiDAR mapping on `reComputer Robotics J501`.

The final goal is not merely to see a “changing map” in RViz2, but to complete the full data chain:

```text
RPLIDAR A1
    │
    ▼
 /scan
    │
    ├──────────────┐
    ▼              ▼
TF / Odometry   SLAM Toolbox
                   │
                   ▼
                /map
                   │
                   ▼
              saved map
              PGM + YAML
```

## Practice Hardware and Software

| Item | Course baseline |
| --- | --- |
| Compute platform | reComputer Robotics J501 / J5011 |
| System | JetPack 6.x / Ubuntu 22.04 / aarch64 |
| ROS | ROS 2 Humble |
| 2D LiDAR | SLAMTEC RPLIDAR A1 |
| Chassis / odometry | DM-H65 differential chassis or equivalent wheel odometry |
| Mapping software | SLAM Toolbox |
| Visualization | RViz2 |

---

## 5.1.6 Bring Up RPLIDAR A1 and the TF Contract

The RPLIDAR A1 driver and basic scan verification flow were already completed in M3, so this lesson reuses them directly.

### Demonstration environment

| Item | Course-validated setup |
| --- | --- |
| Computer | reComputer Robotics J5011 |
| Operating system | Ubuntu 22.04.5 LTS, aarch64 |
| ROS 2 | Humble |
| LiDAR | SLAMTEC RPLIDAR A1 |
| Device node | `/dev/rplidar` |
| Scan topic | `/scan` |
| Laser frame | `laser` |

### Start and check the driver

```bash
source /opt/ros/humble/setup.bash

ros2 launch rplidar_ros rplidar_a1_launch.py \
  serial_port:=/dev/rplidar
```

Check `/scan`:

```bash
ros2 topic info /scan --verbose
ros2 topic hz /scan
ros2 topic echo /scan --once --field header
```

Under normal conditions, the following should be true:

- the message type is `sensor_msgs/msg/LaserScan`;

- timestamps keep increasing;

- the scan rate is basically stable;

- RViz2 shows a continuous and complete scan outline;

- there is no obvious jump or large-area dropout while the LiDAR spins.

---

### Build the minimum TF tree

2D SLAM at least needs the following frame relationships to be clear:

```text
map
└── odom
    └── base_link
        └── laser
```

Where:

- `base_link -> laser`: the static extrinsic between the robot chassis and the LiDAR;

- `odom -> base_link`: provided by wheel odometry or the state-estimation node from M3.4;

- `map -> odom`: provided by SLAM Toolbox according to the mapping result.

These three relationships need a clear division of responsibility. In particular, avoid having multiple nodes publish the same TF at the same time.

For example, you can use a static TF publisher for testing:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.20 --y 0.00 --z 0.15 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link \
  --child-frame-id laser
```

The parameters above are only an example. In real use, replace them with the actual mounting pose of the LiDAR on the robot.

Before mapping, check at least these four items:

1. when the robot moves forward, the `x` direction of `base_link` matches the actual forward direction;

2. when the robot turns left, yaw increases according to the ROS convention;

3. only one node in the system publishes `odom -> base_link`;

4. `/scan` and odometry use consistent and valid timestamps.

> **Image placeholder:** `images/tf_tree_map_odom_base_laser.png`  
> **Fill with:** a TF tree with publisher information, clearly showing the sources of `map -> odom`, `odom -> base_link`, and `base_link -> laser`.

---

## 5.1.7 Deploy SLAM Toolbox

### Install dependencies

```bash
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server
```

Create the course workspace:

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

ros2 pkg create \
  --build-type ament_python \
  j501_slam_2d \
  --dependencies rclpy slam_toolbox

mkdir -p ~/robot_ws/src/j501_slam_2d/{config,launch,maps,rviz}
```

---

### Create the SLAM Toolbox parameter file

Create:

```text
~/robot_ws/src/j501_slam_2d/config/slam_toolbox_online_async.yaml
```

Write:

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: false
    mode: mapping

    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan

    map_update_interval: 1.0
    resolution: 0.05
    max_laser_range: 12.0

    minimum_travel_distance: 0.10
    minimum_travel_heading: 0.10

    throttle_scans: 1
    transform_publish_period: 0.02

    map_start_at_dock: true

    use_scan_matching: true
    use_scan_barycenter: true

    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
```

This set of parameters is good enough as an initial indoor configuration.

During the course practice stage, do not start with heavy parameter tuning. First confirm:

```text
Sensors are healthy
    ↓
TF is correct
    ↓
Odometry is healthy
    ↓
SLAM can build a map
    ↓
A loop can be completed
```

Only after the basic chain is correct should you adjust scan-matching, loop-closure, and map-update parameters for a specific environment.

---

### Launch the system

**Terminal A: start the LiDAR**

```bash
source /opt/ros/humble/setup.bash

ros2 launch rplidar_ros rplidar_a1_launch.py \
  serial_port:=/dev/rplidar
```

**Terminal B: start TF and wheel odometry**

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x 0.20 --y 0.00 --z 0.15 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link \
  --child-frame-id laser
```

Then start the wheel-odometry node or the EKF node from M3.4.

**Terminal C: start SLAM Toolbox**

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash

ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file \
  ~/robot_ws/src/j501_slam_2d/config/slam_toolbox_online_async.yaml
```

Open RViz2:

```bash
rviz2
```

Set:

```text
Fixed Frame = map
```

And add:

- `/map`

- `/scan`

- TF

- Odometry

Under normal conditions, the map should gradually build as the robot moves.

> **Image placeholder:** `images/j501_2d_mapping_rviz.jpg`  
> **Fill with:** an RViz2 screenshot after J501 completes the first indoor loop, showing the occupancy grid, the current laser scan, and the robot pose.

---

## 5.1.8 Mapping, Saving, and Relocalization

### Mapping route

Do not start mapping with high-speed motion.

For the first loop, follow this approach:

1. start from a place with clear geometric features such as a wall corner, doorway, or corridor intersection;

2. move slowly in a straight line and avoid sudden acceleration;

3. keep turns smooth;

4. revisit areas that have already been covered when appropriate;

5. try to complete one closed loop;

6. after returning near the start, stay still for a few seconds and observe whether the loop can align correctly.

A recommended route can look like:

```text
      ┌──────────────┐
      │              │
      │              │
      │      ↑       │
      │      │       │
      └──────┴───────┘
             Start
```

Or use a simple figure-eight route.

The key is not the trajectory shape itself, but giving the robot enough overlapping environment observations.

---

### Save the 2D navigation map

After mapping is complete, use the Nav2 map-saving tool:

```bash
mkdir -p ~/robot_ws/src/j501_slam_2d/maps

ros2 run nav2_map_server map_saver_cli \
  -f ~/robot_ws/src/j501_slam_2d/maps/lab_2d
```

Under normal conditions, this generates:

```text
lab_2d.pgm
lab_2d.yaml
```

Where:

- `.pgm` stores the grid image;

- `.yaml` stores map metadata such as resolution, origin, and the image file name.

These two files are the 2D map that later Nav2 navigation can use directly.

---

### Save the SLAM pose graph

If you want to keep using SLAM Toolbox pose-graph information later, serialize it as well:

```bash
ros2 service call \
  /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/seeed/robot_ws/src/j501_slam_2d/maps/lab_2d_posegraph'}"
```

Change the path according to the actual username.

Keep this distinction in mind:

> **An occupancy grid map and a SLAM Toolbox pose graph are not the same kind of data.**

The former is mainly used for navigation. The latter stores pose-graph information from the SLAM process and can be used for later continued optimization or reloading SLAM state.

---

### Reload the map for localization

Create:

```text
~/robot_ws/src/j501_slam_2d/config/map_server.yaml
```

With content:

```yaml
map_server:
  ros__parameters:
    yaml_filename: "/home/seeed/robot_ws/src/j501_slam_2d/maps/lab_2d.yaml"
    topic_name: "map"
    frame_id: "map"
```

Start the map server:

```bash
source /opt/ros/humble/setup.bash

ros2 run nav2_map_server map_server \
  --ros-args \
  --params-file \
  ~/robot_ws/src/j501_slam_2d/config/map_server.yaml
```

Then manage its lifecycle:

```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

After opening RViz2, set Fixed Frame to:

```text
map
```

And use `2D Pose Estimate` to specify the robot’s current initial pose in the map.

Then move the robot slowly and observe whether the current `/scan` correctly overlaps the saved walls and obstacles.

The core acceptance criterion is not “the robot moved,” but:

> **The existing map can be reloaded, and the robot can re-establish the correct spatial correspondence inside that map.**

> **Image placeholder:** `images/saved_map_and_localization_lock.png`  
> **Fill with:** the saved `lab_2d.pgm` on the left, and on the right an RViz2 localization result where the current laser scan correctly overlaps the walls after the map is reloaded.

---

## 5.1.9 Acceptance Criteria and Common Failures

### Minimum acceptance criteria

Only after completing the following checks should the 2D SLAM baseline be considered ready:

1. the robot can complete about 1 m of straight-line motion with correct odometry direction;

2. after an in-place rotation of about 180°, the map does not show obvious overall stretch or distortion;

3. one indoor loop can be completed;

4. after returning to the start, revisited regions largely overlap;

5. a `PGM + YAML` map is saved successfully;

6. after reloading the map, the current laser scan can basically align with the walls in the map.

---

### Course deliverables

It is recommended to keep the following results:

```text
j501_slam_2d/
├── maps/
│   ├── lab_2d.pgm
│   ├── lab_2d.yaml
│   └── lab_2d_posegraph.*
│
├── config/
│   └── slam_toolbox_online_async.yaml
│
└── screenshots/
    ├── mapping.png
    └── localization.png
```

Also record:

- the actual `/scan` publish rate;

- the `/odom` publish rate;

- the TF tree;

- the `base_link -> laser` extrinsic;

- the SLAM Toolbox parameters used during mapping;

- at least one real failure and its debugging process.

---

### Common issues

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| `/scan` exists, but there is no map | incomplete TF or inconsistent frame names | check `map`, `odom`, `base_link`, and `laser` |
| the map moves abnormally with the robot as a whole | abnormal `odom -> base_link` publishing | check odometry direction, timestamps, and TF publishers |
| the map is clearly distorted during turns | large wheel-odometry error | prioritize calibrating track width, wheel diameter, and angular velocity |
| ghosting gradually appears in the map | weak scan matching or poor odometry quality | slow down and check `/scan` and `/odom` |
| the first loop looks fine, but closing fails at the start | insufficient environment features or motion that is too fast | increase environment overlap and reduce speed |
| localization immediately leaves the map after start | wrong initial pose | reset with `2D Pose Estimate` |
| the map shows an obvious scale problem | odometry scale or TF extrinsic error | check wheel diameter, track width, and LiDAR mounting parameters |
| J501 becomes laggy | RViz2 or other debug nodes create too much load | reduce visualization content and keep the mapping chain lean |

One debugging principle needs special attention:

> **If the map shows severe drift, do not first try to “force-tune” the result by changing SLAM Toolbox parameters.**

Instead, check layer by layer:

```text
/scan
  ↓
timestamps
  ↓
base_link -> laser
  ↓
odom -> base_link
  ↓
SLAM Toolbox
```

Because if the SLAM input data or TF is wrong, further tuning algorithm parameters usually only hides the problem instead of solving it.

---

# Lesson Summary

The core SLAM task is to estimate the robot **pose** and the environment **map** at the same time. The robot obtains motion and environment constraints from sensors such as odometry, IMU, LiDAR, or vision, then uses mechanisms such as scan matching, data association, and loop closure to keep the trajectory and map consistent.

This lesson first built the basic concept of SLAM and compared:

- 2D LiDAR SLAM;

- 3D LiDAR SLAM;

- visual SLAM;

- multi-sensor fusion SLAM.

Then, on `reComputer Robotics J501`, it used `RPLIDAR A1 + SLAM Toolbox` to complete a full 2D LiDAR mapping workflow:

```text
RPLIDAR A1
    ↓
/scan
    ↓
TF + Odometry
    ↓
SLAM Toolbox
    ↓
2D Occupancy Grid
    ↓
PGM + YAML
    ↓
Map-based Localization
```

After completing this lesson, you should at least have one 2D occupancy grid map that can be saved, reloaded, and used for later Nav2 navigation.

---

# Next Step

2D LiDAR can only observe one fixed-height plane.

Once the robot enters halls, campuses, warehouses, or environments with clear height variation, a single laser slice cannot fully describe walls, pillars, ramps, and other 3D structure.

Therefore, Lesson 5.2 continues from this lesson’s 2D SLAM baseline and moves further into **3D LiDAR SLAM**.

The next lesson uses:

```text
Livox MID-360
      +
IMU
      ↓
Fast-LIO
      ↓
3D Point Cloud Map
```

The focus is 3D LiDAR, IMU, motion deskewing, LiDAR-inertial odometry, and 3D map construction.

Keep the 2D map obtained in this lesson, because later you can combine 2D navigation with 3D perception into a more complete robot spatial-perception chain.

---

# References

- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

- [Nav2](https://docs.nav2.org/)

- Thrun, Burgard, Fox: *Probabilistic Robotics*

- Course prerequisite: [M3.1 LiDAR Integration and Point Cloud Preprocessing](../../M03-LiDAR-and-Multi-Sensor-Fusion/3.1_LiDAR_Integration_and_Point_Cloud_Preprocessing/README_en_US.md)

- Course prerequisite: [M3.4 EKF and State Estimation](../../M03-LiDAR-and-Multi-Sensor-Fusion/3.4_EKF_and_State_Estimation/README_en_US.md)
