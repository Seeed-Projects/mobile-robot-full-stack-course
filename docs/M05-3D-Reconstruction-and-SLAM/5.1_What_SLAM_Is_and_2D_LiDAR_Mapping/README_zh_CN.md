# 5.1 认识SLAM，构建你的第一张栅格地图

SLAM 是移动机器人导航与环境感知的核心技术。它在估计机器人位姿的同时，构建可复用的环境空间模型。没有这一联合估计，机器人或许能感知局部几何，却难以维持规划、重定位与重复运行所需的一致世界坐标系。

本课介绍 SLAM 问题本身、实践中常用的主要传感路线，以及在 `reComputer Robotics J501` 上基于 RPLIDAR A1 与 **SLAM Toolbox** 的第一套二维激光建图流程。先建立概念框架，再用二维实践为后续 M5 提供第一条可运行基线。

> **图片占位：** `images/slam_big_picture_to_2d_lab.png`
> **需要补充：** slam导航视觉效果图，科技风格，移动机器人在场景中移动，通过相机、激光雷达感知周围环境并沿一条规划的路径移动。

## 学习目标

- 用工程语言说明 SLAM 任务：估计什么、为何必须联合估计、哪些输出对机器人真正有用；
- 区分实践中常见的主要 SLAM 路线：二维/三维激光、视觉与多传感器融合；
- 明确本课在 M5 中的位置，以及后续课程将补充的内容；
- 在 J501 上部署基于 RPLIDAR A1 与 SLAM Toolbox 的二维激光建图流程；
- 保存可复用占用栅格，并在定位模式下重新加载。

## 前置条件

- M1：J501 已刷机，ROS 2 Humble 可用，工作空间可正常 source；
- M3.1：RPLIDAR A1 可发布 `/scan`，并在 RViz2 中显示干净扫描环；
- M3.2 / M3.4：已有轮式里程计，且 `odom -> base_link` 仅有一个发布者；
- 具备基本 TF 知识：`map`、`odom`、`base_link`、`laser`。

---

# Part A. SLAM 任务

## 5.1.1 定义与必要输出

单次传感器观测通常只回答局部问题。激光扫描描述当前时刻附近的几何，相机帧描述当前图像中的外观，轮式里程计描述自上一时刻以来的估计运动。导航需要两类更强的结果：

1. 机器人在可能已被部分观测过的环境中的位姿；
2. 可在后续运行中复用的环境空间模型。

SLAM 对二者进行联合估计。若地图已经完美，问题退化为定位；若机器人位姿已经完美，问题退化为建图。真实机器人通常两者都不具备，因此必须同时估计位姿与地图。

| 输出                   | 作用                                   |
| ---------------------- | -------------------------------------- |
| 随时间变化的机器人位姿 | 在运动过程中持续跟踪机器人             |
| 可复用地图             | 避免每次运行都从零重建环境             |
| 位姿与地图的一致性     | 使规划、避障与重定位共享同一空间坐标系 |
| 不确定性 / 失效感知    | 识别漂移、歧义与估计器失效             |

对本课程而言，第一份系统契约如下：

```text
sensors                         SLAM / localization                 downstream use
/scan, /odom, camera, IMU  ->   pose + map (+ TF)            ->    Nav2, planning, semantics
```

> **图片占位：** `images/slam_task_pose_and_map.png`
> **需要补充：** 科技论文风格，示意图包含一台机器人、局部墙面观测、不确定位姿分布与不断扩展的地图，并标注 “estimate pose”、“update map” 与输出 “consistent world model”。

## 5.1.2 从状态估计到可落地建图

SLAM 最初是一个状态估计问题：运动中的机器人，能否在仍在构建的地图中完成自定位，同时持续建图？早期滤波形式将机器人位姿与地图元素放入同一不断增长的估计状态。这些工作证明了问题可解，也暴露了至今仍然成立的约束：

- 若不重访已知结构，漂移会持续累积；
- 错误的数据关联会同时破坏位姿与地图；
- 计算代价随地图规模增长；
- 不同传感器会引入不同的几何、外观与失效模式。

后续实践逐渐从单一大型联合状态转向更可扩展的结构，例如粒子滤波建图，尤其是 **位姿图（pose graph）**。在位姿图中，机器人位姿是节点，传感器约束是边。当机器人再次识别已访问地点时，回环约束可校正轨迹。

因此，现代 SLAM 系统通常聚焦于四项要求：

- 用运动先验连接相邻观测；
- 将新测量与当前地图可靠关联；
- 在重访地点后通过回环降低漂移；
- 采用可存储、可复用的地图表示。

> **图片占位：** `images/slam_short_origin_to_modern_graph.png`
> **需要补充：** 科技论文流程图风格，展示slam发展的时间线：早期联合状态滤波 -> 可实用的室内建图 -> 现代位姿图 / 多传感器 SLAM。

## 5.1.3 共同估计闭环

多数 SLAM 实现的代码组织不同，但估计闭环基本一致。

1. **Predict**：用轮式里程计、IMU、视觉里程计或上一级估计器提供的运动先验预测下一时刻位姿。
2. **Observe**：用二维/三维激光或单目/双目/RGB-D 等外感受传感器观察环境。
3. **Associate**：将新观测与当前地图或历史关键帧关联。
4. **Update**：根据关联结果更新位姿估计。
5. **Update**：将新对齐的观测写入地图。
6. **Correct globally**：在出现回环或场景识别时进行全局校正。
7. **Publish**：发布位姿、地图，以及机器人其余模块所需坐标系。

```text
motion prior + sensor observation
        -> data association
        -> local pose update
        -> map update
        -> optional loop closure
        -> reusable world model
```

| 层级               | 作用                       | 典型内容                                    |
| ------------------ | -------------------------- | ------------------------------------------- |
| Front-end          | 将原始传感器数据转换为约束 | 扫描匹配、特征跟踪、去畸变、里程计因子      |
| Back-end           | 在约束下优化位姿与地图     | 滤波器、位姿图、因子图、光束法平差          |
| Map representation | 以可用形式存储环境         | 占用栅格、点云、网格、TSDF/ESDF、语义层     |
| System outputs     | 将 SLAM 接入机器人系统     | `map -> odom`、`/map`、轨迹、序列化地图文件 |

## 5.1.4 主要 SLAM 路线

实际 SLAM 系统由传感模态与地图需求共同塑造。以下是 M5 中持续使用的主要路线。

### 1) 二维激光 SLAM

**输入：** 平面激光扫描与运动先验。  
**典型地图：** 占用栅格。  
**主要用途：** 面向 Nav2 类规划器的室内导航。

二维激光 SLAM 是室内移动机器人的标准起点。它构建规划栈几乎可直接使用的占用栅格。在墙面几何稳定的房间与走廊中，该表示紧凑、计算代价适中，建图闭环也便于在 RViz 中检查。其限制是结构性的：单层激光平面无法表达悬空物、斜坡与垂直细节，长而相似的走廊也会削弱回环。对本课程而言，二维激光 SLAM 提供的是第一张导航地图，而不是完整场景模型。

### 2) 三维激光 SLAM

**输入：** 三维点云，常结合 IMU。  
**典型地图：** 点云地图，有时再投影为二维层。  
**主要用途：** 大空间、非平面结构与后续重建。

当单一平面已无法充分描述环境时，三维激光 SLAM 成为必要选择。估计器处理点云，并常与 IMU 紧耦合，以在扫描过程中补偿运动畸变。所得度量地图能够保留立柱、斜坡、家具及其他非平面结构。因此第 5.2 课将从二维基线继续进入 MID-360 与 Fast-LIO 风格的 LIO。代价是时间同步、外参标定与运动补偿的系统复杂度上升；而且许多导航栈在规划前仍会将三维结果投影为二维或分层代价地图。

### 3) 视觉 SLAM

**输入：** 单目、双目或 RGB-D 相机。  
**典型地图：** 稀疏路标、半稠密地图或更稠密重建。  
**主要用途：** 外观、地点识别与场景理解。

视觉 SLAM 主要依据外观而非单纯距离几何估计位姿。图像提供纹理、地点身份，以及通向语义的直接路径，因此在几何结构辨识度不足时尤其有价值。与之对应的弱点是对光照变化、运动模糊和弱纹理区域敏感，单目尺度也往往脆弱。在本课程中，视觉 SLAM 用于在已有度量空间骨架后补充外观与含义。

### 4) 多传感器 / 紧耦合 SLAM

**输入：** 激光 + IMU、视觉 + IMU，或激光 + 视觉 + IMU。  
**目标：** 通过传感器互补提升稳健性。

多传感器 SLAM 将前述路线组合起来，使各模态互相弥补不足。激光-惯性估计利用 IMU 在几何更新之间桥接高频运动；视觉-惯性估计在快速运动中稳定图像跟踪；激光-视觉-惯性估计则把度量结构、外观与短时运动纳入同一估计器。对实际部署而言，这通常是最稳健的方向，但前提是已经理解单模态失效模式。外参标定、时间同步与坐标系约定将成为一等设计约束。

## 5.1.5 选择 SLAM 路线

第一条技术路线应按机器人任务选择。

| 机器人需求             | 更合适的起点             | 原因                               |
| ---------------------- | ------------------------ | ---------------------------------- |
| 室内差速底盘导航       | 二维激光 SLAM            | 占用栅格可直接接入规划器           |
| 大厅、园区、半开放结构 | 三维激光 / LIO           | 环境非平面时几何仍然可用           |
| 语义巡检 / 地点识别    | 视觉或视觉主导融合       | 外观携带身份与含义                 |
| 高动态 / 无 GNSS 运行  | 激光-惯性或视觉-惯性融合 | IMU 可在稀疏更新之间桥接运动       |
| 面向 Nav2 的产品地图   | 常为二维或投影分层       | 规划器仍需要代价地图与明确自由空间 |

---

# Part B. 实践：在 J501 上完成第一张二维激光地图

本实践部分用一套可维护的 ROS 2 方案 **SLAM Toolbox** 实现前述估计闭环。目标是在 J501 上得到可复用的占用栅格，而不是遍历所有 SLAM 功能包。

### 实践硬件与软件

| 项目          | 课程基线                                                                                                                                   |
| ------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| 计算平台      | reComputer Robotics J501 / J5011，JetPack 6.x，Ubuntu 22.04，aarch64                                                                       |
| ROS           | ROS 2 Humble                                                                                                                               |
| 二维激光雷达  | SLAMTEC RPLIDAR A1，见 [M3.1](../../M03-LiDAR-and-Multi-Sensor-Fusion/3.1_LiDAR_Integration_and_Point_Cloud_Preprocessing/README_zh_CN.md) |
| 底盘 / 里程计 | DM-H65 差速底盘，或等价轮式里程计，发布 `nav_msgs/Odometry`                                                                                |
| 建图栈        | SLAM Toolbox                                                                                                                               |
| 可视化        | RViz2                                                                                                                                      |

## 5.1.6 拉起 RPLIDAR A1 与 TF 契约

复用 M3 中已验证的流程。

### 演示环境

| 项目           | 课程实测配置                |
| -------------- | --------------------------- |
| 计算机         | reComputer Robotics J5011   |
| 操作系统       | Ubuntu 22.04.5 LTS，aarch64 |
| ROS 2          | Humble                      |
| 激光雷达       | SLAMTEC RPLIDAR A1          |
| 设备节点       | `/dev/rplidar`              |
| 扫描话题       | `/scan`                     |
| 驱动默认坐标系 | `laser`                     |

### 启动并检查驱动

```bash
source /opt/ros/humble/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py \
  serial_port:=/dev/rplidar

ros2 topic info /scan --verbose
ros2 topic hz /scan
ros2 topic echo /scan --once --field header
```

预期现象：

- 消息类型为 `sensor_msgs/msg/LaserScan`
- 时间戳非零且持续递增
- 频率大致稳定在 7–10 Hz
- RViz2 中扫描环干净

### 最小 TF 树

```text
map
└── odom
    └── base_link
        └── laser
```

- `base_link -> laser`：由机械安装测得的静态外参
- `odom -> base_link`：轮式里程计或 M3.4 局部 EKF
- `map -> odom`：由 SLAM Toolbox 发布

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.20 --y 0.00 --z 0.15 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id laser
```

请将数值替换为实际安装偏置。建图前还需确认：

1. 前进时 `x` 增大；
2. 左转时 yaw 增大；
3. 仅有一个节点发布 `odom -> base_link`；
4. 扫描与里程计处于一致的时间域。

> **图片占位：** `images/tf_tree_map_odom_base_laser.png`
> **需要补充：** 带发布者标注的 TF 树，分别对应 `map->odom`、`odom->base_link` 与 `base_link->laser`。

## 5.1.7 部署 SLAM Toolbox

### 安装依赖并创建功能包骨架

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y ros-humble-slam-toolbox ros-humble-nav2-map-server

mkdir -p ~/robot_ws/src && cd ~/robot_ws/src
ros2 pkg create --build-type ament_python j501_slam_2d --dependencies rclpy slam_toolbox
mkdir -p ~/robot_ws/src/j501_slam_2d/{config,launch,maps,rviz}
```

创建 `config/slam_toolbox_online_async.yaml`：

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

上述参数是偏保守的室内起点。先完成一圈有效闭环，再进行调参。

### 启动顺序

终端 A，激光：

```bash
source /opt/ros/humble/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/rplidar
```

终端 B，静态 TF 与里程计：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x 0.20 --y 0.00 --z 0.15 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id laser

# 在此处启动轮式里程计或 M3.4 EKF
```

终端 C，SLAM Toolbox：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file ~/robot_ws/src/j501_slam_2d/config/slam_toolbox_online_async.yaml
```

打开 RViz2，将 Fixed Frame 设为 `map`，并显示 `/map`、`/scan`、TF 与里程计。

> **图片占位：** `images/j501_2d_mapping_rviz.jpg`
> **需要补充：** J501 首次成功绕走廊一圈的画面，占用栅格、当前扫描叠加与机器人 footprint 应保持对齐。

## 5.1.8 建图、保存与重定位

### 以拓扑覆盖为目标行驶

1. 从具有辨识度的墙角起步；
2. 以较低速度行驶矩形或 8 字轨迹，并保持足够重叠；
3. 当地图仍较稀疏时，避免剧烈原地旋转；
4. 返回起点后稍作停留，以便回环生效。

### 保存导航地图

```bash
mkdir -p ~/robot_ws/src/j501_slam_2d/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/robot_ws/src/j501_slam_2d/maps/lab_2d
```

将生成：

- `lab_2d.pgm`
- `lab_2d.yaml`

可选位姿图序列化：

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/seeed/robot_ws/src/j501_slam_2d/maps/lab_2d_posegraph'}"
```

请将 `/home/seeed` 替换为实际用户路径。

### 重新加载用于定位

创建 `config/map_server.yaml`：

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

在 RViz2 中，先用 `2D Pose Estimate` 将初始位姿设置到真实起点附近，再开始运动。验收标准是：保存后的地图可直接复用，而无需重新建图。

> **图片占位：** `images/saved_map_and_localization_lock.png`
> **需要补充：** 左右对比图。左为保存的 `lab_2d.pgm`；右为定位模式下当前扫描贴合墙面的 RViz 画面。

## 5.1.9 验收标准与常见故障

### 最低检查项

1. 约 1.0 m 的直线里程计测试；
2. 原地旋转 180°，地图不应被拉伸成扇形；
3. 完成一圈室内环路，并保存 `pgm/yaml`；
4. 返回起点后重叠关系目视紧致；
5. 重新加载地图后短距离行驶，扫描不应与墙面分离。

### 交付物

- `lab_2d.pgm` 与 `lab_2d.yaml`
- 建图模式与定位模式截图
- 激光频率、里程计话题与 TF 偏置记录
- 一次实际故障及其对应修复

### 常见问题

| 现象                   | 可能原因                 | 处理                                          |
| ---------------------- | ------------------------ | --------------------------------------------- |
| 存在 `/scan` 但无地图  | 坐标系名称与 TF 树不一致 | 检查 `base_frame`、`odom_frame` 与 laser 外参 |
| 转弯时地图呈螺旋       | 角速度里程计或轮距错误   | 先修正里程计，再调整匹配参数                  |
| 第一圈可接受，重访失败 | 起点特征不足或速度过快   | 换到辨识度更高的墙角重开，并降低速度          |
| 定位一开始即发散       | 初始位姿偏差过大         | 运动前重置位姿                                |
| J501 卡顿              | 调试可视化负载过高       | 保持启动链路精简                              |

---

## 课程小结

SLAM 联合估计机器人 **位姿** 与环境 **地图**。本课定义了估计任务，比较了主要传感路线，并在 J501 上用 SLAM Toolbox 实现了第一套二维激光建图流程。预期结果是：为 M5 建立清晰的概念基线，并得到一张后续导航模块可直接使用的占用栅格。

## 下一步

第 5.2 课仍属于激光路线，将从平面占用栅格进入三维激光建图，使用 Livox MID-360 与 Fast-LIO 风格估计。二维地图应予保留。下一课补充的，是单层激光平面无法观测到的几何结构。

## 参考

- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Map Server](https://docs.nav2.org/)
- Thrun, Burgard, Fox: *Probabilistic Robotics*
- 课程前置：[M3.1 激光雷达接入与点云预处理](../../M03-LiDAR-and-Multi-Sensor-Fusion/3.1_LiDAR_Integration_and_Point_Cloud_Preprocessing/README_zh_CN.md)
- 课程前置：[M3.4 EKF 与状态估计](../../M03-LiDAR-and-Multi-Sensor-Fusion/3.4_EKF_and_State_Estimation/README_zh_CN.md)
