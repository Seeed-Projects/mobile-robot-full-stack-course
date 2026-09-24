# 5.1 Getting started with SLAM and Building Your First Occupancy Grid Map

SLAM is a core capability for mobile-robot navigation and environment perception. It estimates the robot pose and builds a reusable spatial model of the surroundings at the same time. Without that joint estimate, a robot may sense local geometry, but it cannot maintain a consistent world frame for planning, relocalization, and repeated operation.

This lesson introduces the SLAM problem, the main sensing families used in practice, and a first 2D LiDAR mapping workflow on `reComputer Robotics J501` with RPLIDAR A1 and **SLAM Toolbox**. The conceptual framework comes first; the 2D exercise provides the first runnable baseline for the rest of M5.

> **Image placeholder:** `images/slam_big_picture_to_2d_lab.png`
> **Fill with:** a tech-style SLAM navigation visualization. A mobile robot moves through a scene, senses the surroundings with a camera and LiDAR, and follows a planned path.

## Learning Objectives

- Explain the SLAM task in engineering terms: what is estimated, why the estimates are coupled, and which outputs matter for a robot;
- Distinguish the major SLAM families used in practice: 2D/3D LiDAR, visual, and multi-sensor fusion;
- Locate this lesson within M5 and identify what later lessons add;
- Deploy a 2D LiDAR mapping stack on J501 with RPLIDAR A1 and SLAM Toolbox;
- Save a reusable occupancy map and reload it in localization mode.

## Prerequisites

- M1: J501 is flashed, ROS 2 Humble is available, and the workspace can be sourced;
- M3.1: RPLIDAR A1 publishes `/scan` and shows a clean ring in RViz2;
- M3.2 / M3.4: wheel odometry is available, and `odom -> base_link` has a single publisher;
- Basic TF knowledge: `map`, `odom`, `base_link`, and `laser`.

---

# Part A. The SLAM Task

## 5.1.1 Definition and Required Outputs

A single sensor observation answers only a local question. A laser scan describes nearby geometry at the current time. A camera frame describes appearance in the current image. Wheel odometry describes estimated motion since the previous update. Navigation requires two stronger results:

1. the robot pose in an environment that may already have been partially observed;
2. a spatial model of that environment that can be reused later.

SLAM estimates both quantities jointly. If the map were already perfect, the problem would reduce to localization. If the robot pose were already perfect, the problem would reduce to mapping. Neither condition holds on a real robot, so pose and map must be estimated together.

| Output                           | Role                                                                      |
| -------------------------------- | ------------------------------------------------------------------------- |
| Robot pose over time             | Track the robot while it moves                                            |
| A reusable map                   | Avoid rebuilding the environment on every run                             |
| Consistency between pose and map | Keep planning, obstacle checking, and relocalization in one spatial frame |
| Uncertainty / failure awareness  | Detect drift, ambiguity, and estimator failure                            |

For this course, the first system contract is:

```text
sensors                         SLAM / localization                 downstream use
/scan, /odom, camera, IMU  ->   pose + map (+ TF)            ->    Nav2, planning, semantics
```

> **Image placeholder:** `images/slam_task_pose_and_map.png`
> **Fill with:** a scientific-paper style diagram with one robot, partial wall observations, an uncertain pose distribution, and a growing map. Annotate "estimate pose", "update map", and the combined output "consistent world model".

## 5.1.2 From State Estimation to Practical Mapping

SLAM began as a state-estimation problem: can a moving robot build a map while localizing itself inside the map that is still under construction? Early filter-based formulations placed robot pose and map elements in one growing estimated state. Those systems showed that the problem was solvable and exposed constraints that remain relevant today:

- drift accumulates if the robot never revisits known structure;
- incorrect data association damages both pose and map;
- computation grows with map size;
- different sensors introduce different geometry, appearance, and failure modes.

Later practice moved from one large joint state toward more scalable structures such as particle-filter mapping and especially **pose graphs**. In a pose graph, robot poses are nodes and sensor constraints are edges. When the robot recognizes a previously visited place, a loop-closure constraint can correct the trajectory.

Modern SLAM systems therefore concentrate on four requirements:

- a motion prior that bridges successive observations;
- reliable association between new measurements and the current map;
- loop closure that reduces drift after place revisits;
- a map representation that can be stored and reused.

> **Image placeholder:** `images/slam_short_origin_to_modern_graph.png`
> **Fill with:** a scientific-paper style flowchart of the SLAM timeline: early joint-state filters -> practical indoor mapping -> modern pose-graph / multi-sensor SLAM.

## 5.1.3 Shared Estimation Loop

Most SLAM implementations follow the same estimation loop, even when their code organization differs.

1. **Predict** the next pose with a motion prior from wheel odometry, IMU, visual odometry, or a previous estimator.
2. **Observe** the environment with one or more exteroceptive sensors such as 2D/3D LiDAR or mono/stereo/RGB-D cameras.
3. **Associate** the new observation with the current map or with previous keyframes.
4. **Update** the pose estimate from that association.
5. **Update** the map with the newly aligned observation.
6. **Correct globally** when loop closure or place recognition provides an additional constraint.
7. **Publish** pose, map, and the frames required by the rest of the robot stack.

```text
motion prior + sensor observation
        -> data association
        -> local pose update
        -> map update
        -> optional loop closure
        -> reusable world model
```

| Layer              | Role                                           | Typical content                                               |
| ------------------ | ---------------------------------------------- | ------------------------------------------------------------- |
| Front-end          | Convert raw sensor data into constraints       | Scan matching, feature tracking, deskewing, odometry factors  |
| Back-end           | Optimize poses and map under those constraints | Filters, pose graphs, factor graphs, bundle adjustment        |
| Map representation | Store the environment in a usable form         | Occupancy grid, point cloud, mesh, TSDF/ESDF, semantic layers |
| System outputs     | Connect SLAM to the robot stack                | `map -> odom`, `/map`, trajectory, serialized map files       |

## 5.1.4 Major SLAM Families

Practical SLAM systems are shaped by sensing modality and map requirements. The main families below are the ones used throughout M5.

### 1) 2D LiDAR SLAM

**Inputs:** planar laser scan and motion prior.  
**Typical map:** occupancy grid.  
**Primary use:** indoor navigation with Nav2-style planners.

2D LiDAR SLAM is the standard entry point for indoor mobile robots. It constructs an occupancy grid that planning stacks can use with little additional conversion. In rooms and corridors with stable wall geometry, the representation is compact, the computational cost is moderate, and the mapping loop is easy to inspect in RViz. The limitation is structural: a single laser plane cannot represent overhangs, ramps, or vertical detail, and long self-similar corridors can weaken loop closure. For this course, 2D LiDAR SLAM provides the first navigation map, not a complete scene model.

### 2) 3D LiDAR SLAM

**Inputs:** 3D point clouds, often with IMU.  
**Typical map:** point-cloud map, sometimes later projected into 2D layers.  
**Primary use:** large spaces, uneven structure, and later reconstruction.

3D LiDAR SLAM becomes necessary when a single plane no longer describes the environment. The estimator operates on point clouds and is often tightly coupled with an IMU so that motion distortion can be compensated during scanning. The resulting metric map preserves pillars, slopes, furniture, and other non-planar structure. Lesson 5.2 therefore continues from the 2D baseline to MID-360 and Fast-LIO-style LIO. The cost is higher system complexity in timing, extrinsic calibration, and motion compensation, and many navigation stacks still project the 3D result into 2D or layered costmaps before planning.

### 3) Visual SLAM

**Inputs:** mono, stereo, or RGB-D cameras.  
**Typical map:** sparse landmarks, semi-dense maps, or denser reconstructions.  
**Primary use:** appearance, place recognition, and scene understanding.

Visual SLAM estimates pose from appearance rather than from range geometry alone. Images provide texture, place identity, and a direct path toward semantics, which makes vision especially useful when geometric structure is not distinctive. The complementary weakness is sensitivity to lighting changes, motion blur, and textureless regions, while monocular scale remains fragile. In this course, visual SLAM is introduced as a way to add appearance and meaning after a metric spatial backbone already exists.

### 4) Multi-sensor / tightly coupled SLAM

**Inputs:** LiDAR + IMU, vision + IMU, or LiDAR + vision + IMU.  
**Goal:** robustness by letting sensors cover each other's failure modes.

Multi-sensor SLAM combines the previous families so that each modality compensates for the limitations of the others. LiDAR-inertial estimation uses the IMU to bridge high-rate motion between geometric updates. Visual-inertial estimation stabilizes image tracking during fast motion. LiDAR-visual-inertial estimation joins metric structure, appearance, and short-term motion in one estimator. This is typically the most robust direction for deployed robots, but only after single-modality failure modes are understood. Extrinsic calibration, time synchronization, and frame conventions become first-class design constraints.

## 5.1.5 Selecting a SLAM Path

The first stack should be selected according to the robot task.

| Robot requirement                       | Preferred starting family                | Reason                                                     |
| --------------------------------------- | ---------------------------------------- | ---------------------------------------------------------- |
| Indoor differential-drive navigation    | 2D LiDAR SLAM                            | Occupancy maps integrate directly with planners            |
| Large halls, yards, semi-open structure | 3D LiDAR / LIO                           | Geometry remains usable when the environment is not planar |
| Semantic inspection / place recognition | Visual or vision-centric fusion          | Appearance carries identity and meaning                    |
| High dynamics / GNSS-denied operation   | LiDAR-inertial or visual-inertial fusion | IMU bridges motion between sparse updates                  |
| Product map for Nav2                    | Often 2D or projected layers             | Planners still require costmaps and explicit free space    |

---

# Part B. Practice: First 2D LiDAR Map on J501

The practice section implements the estimation loop with one maintained ROS 2 stack: **SLAM Toolbox**. The goal is a reusable occupancy map on J501, not a complete survey of every SLAM package.

### Practice hardware and software

| Item               | Course baseline                                                                                                                             |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------- |
| Compute platform   | reComputer Robotics J501 / J5011, JetPack 6.x, Ubuntu 22.04, aarch64                                                                        |
| ROS                | ROS 2 Humble                                                                                                                                |
| 2D LiDAR           | SLAMTEC RPLIDAR A1 from [M3.1](../../M03-LiDAR-and-Multi-Sensor-Fusion/3.1_LiDAR_Integration_and_Point_Cloud_Preprocessing/README_en_US.md) |
| Chassis / odometry | DM-H65 differential drive or equivalent wheel odometry publishing `nav_msgs/Odometry`                                                       |
| Mapping stack      | SLAM Toolbox                                                                                                                                |
| Visualization      | RViz2                                                                                                                                       |

## 5.1.6 Bring Up RPLIDAR A1 and the TF Contract

Reuse the validated workflow from M3.

### Demonstration environment

| Item                      | Measured course setup       |
| ------------------------- | --------------------------- |
| Computer                  | reComputer Robotics J5011   |
| OS                        | Ubuntu 22.04.5 LTS, aarch64 |
| ROS 2                     | Humble                      |
| LiDAR                     | SLAMTEC RPLIDAR A1          |
| Device node               | `/dev/rplidar`              |
| Scan topic                | `/scan`                     |
| Default frame from driver | `laser`                     |

### Start and verify the driver

```bash
source /opt/ros/humble/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py \
  serial_port:=/dev/rplidar

ros2 topic info /scan --verbose
ros2 topic hz /scan
ros2 topic echo /scan --once --field header
```

Expected indicators:

- message type `sensor_msgs/msg/LaserScan`
- non-zero, increasing timestamps
- approximately stable rate of 7–10 Hz
- a clean scan ring in RViz2

### Minimum TF tree

```text
map
└── odom
    └── base_link
        └── laser
```

- `base_link -> laser`: static extrinsic from mechanical measurement
- `odom -> base_link`: wheel odometry or M3.4 local EKF
- `map -> odom`: published by SLAM Toolbox

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.20 --y 0.00 --z 0.15 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id laser
```

Replace the values with the measured mounting offsets. Before mapping, also confirm that:

1. forward motion increases `x`;
2. left turn increases yaw;
3. only one node publishes `odom -> base_link`;
4. scan and odometry share a consistent time domain.

> **Image placeholder:** `images/tf_tree_map_odom_base_laser.png`
> **Fill with:** TF tree with publisher annotations for `map->odom`, `odom->base_link`, and `base_link->laser`.

## 5.1.7 Deploy SLAM Toolbox

### Install packages and create the package skeleton

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y ros-humble-slam-toolbox ros-humble-nav2-map-server

mkdir -p ~/robot_ws/src && cd ~/robot_ws/src
ros2 pkg create --build-type ament_python j501_slam_2d --dependencies rclpy slam_toolbox
mkdir -p ~/robot_ws/src/j501_slam_2d/{config,launch,maps,rviz}
```

Create `config/slam_toolbox_online_async.yaml`:

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

These parameters are a conservative indoor starting point. Complete one successful loop before tuning.

### Launch sequence

Terminal A, LiDAR:

```bash
source /opt/ros/humble/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/rplidar
```

Terminal B, static TF and odometry:

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x 0.20 --y 0.00 --z 0.15 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id laser

# start wheel odometry or M3.4 EKF here
```

Terminal C, SLAM Toolbox:

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file ~/robot_ws/src/j501_slam_2d/config/slam_toolbox_online_async.yaml
```

Open RViz2, set Fixed Frame to `map`, and display `/map`, `/scan`, TF, and odometry.

> **Image placeholder:** `images/j501_2d_mapping_rviz.jpg`
> **Fill with:** the first successful corridor loop on J501, with occupancy grid, current scan overlay, and robot footprint aligned.

## 5.1.8 Map, Save, and Relocalize

### Drive for topological coverage

1. Start at a distinctive corner.
2. Drive a slow rectangle or figure-eight with sufficient overlap.
3. Avoid aggressive spins while the map is still sparse.
4. Return to the start and pause to allow loop closure.

### Save the navigation map

```bash
mkdir -p ~/robot_ws/src/j501_slam_2d/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/robot_ws/src/j501_slam_2d/maps/lab_2d
```

This produces:

- `lab_2d.pgm`
- `lab_2d.yaml`

Optional pose-graph serialization:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/seeed/robot_ws/src/j501_slam_2d/maps/lab_2d_posegraph'}"
```

Replace `/home/seeed` with the actual user path.

### Reload for localization

Create `config/map_server.yaml`:

```yaml
map_server:
  ros__parameters:
    yaml_filename: "/home/seeed/robot_ws/src/j501_slam_2d/maps/lab_2d.yaml"
    topic_name: "map"
    frame_id: "map"
```

```bash
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_server --ros-args \
  --params-file ~/robot_ws/src/j501_slam_2d/config/map_server.yaml
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

In RViz2, set `2D Pose Estimate` near the true start pose before motion begins. The acceptance criterion is that the saved map can be reused without rebuilding the environment.

> **Image placeholder:** `images/saved_map_and_localization_lock.png`
> **Fill with:** left, saved `lab_2d.pgm`; right, localization-mode RViz view with the current scan aligned to the walls.

## 5.1.9 Acceptance Criteria and Common Failures

### Minimum checks

1. Straight-line odometry test over approximately 1.0 m.
2. In-place 180° rotation without shearing the map into a fan.
3. One complete indoor loop saved as `pgm/yaml`.
4. Visually tight overlap after returning to the start.
5. Map reload followed by a short drive without scan-to-wall divergence.

### Deliverables

- `lab_2d.pgm` and `lab_2d.yaml`
- screenshots from mapping mode and localization mode
- notes on LiDAR rate, odometry topic, and TF offsets
- one observed failure and the corresponding fix

### Common issues

| Symptom                                 | Likely cause                            | Action                                                |
| --------------------------------------- | --------------------------------------- | ----------------------------------------------------- |
| `/scan` exists but no map appears       | Frame names do not match the TF tree    | Check `base_frame`, `odom_frame`, and laser extrinsic |
| Map spirals during turns                | Incorrect angular odometry or wheelbase | Correct odometry before matcher tuning                |
| First pass is acceptable, revisit fails | Weak start features or excessive speed  | Restart at a distinctive corner and reduce speed      |
| Localization diverges immediately       | Poor initial pose                       | Reset the pose before moving                          |
| J501 hitching                           | Excessive debug visualization load      | Keep the bring-up minimal                             |

---

## Lesson Summary

SLAM jointly estimates robot **pose** and environment **map**. This lesson defined the estimation task, compared the main sensing families, and implemented a first 2D LiDAR mapping stack with SLAM Toolbox on J501. The expected result is a clear conceptual baseline for M5 and one occupancy map that later navigation modules can consume.

## Next Step

Lesson 5.2 remains on the LiDAR branch and moves from planar occupancy grids to 3D LiDAR mapping with Livox MID-360 and Fast-LIO-style estimation. Keep the 2D map. The next lesson adds the geometric structure that a single laser plane cannot observe.

## References

- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Map Server](https://docs.nav2.org/)
- Thrun, Burgard, Fox: *Probabilistic Robotics*
- Course prerequisite: [M3.1 LiDAR Integration and Point Cloud Preprocessing](../../M03-LiDAR-and-Multi-Sensor-Fusion/3.1_LiDAR_Integration_and_Point_Cloud_Preprocessing/README_en_US.md)
- Course prerequisite: [M3.4 EKF and State Estimation](../../M03-LiDAR-and-Multi-Sensor-Fusion/3.4_EKF_and_State_Estimation/README_en_US.md)
