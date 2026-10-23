# 5.2 完整 SLAM 系统：回环检测与全局地图优化

![RTAB-Map 优化前后地图对比：回环检测与全局优化显著降低误差累积](./images/m5-2-rtabmap-optimized-map.png)

*图片来源：https://learn-udacity.top/udrb552921/Robotics%20Software%20Engineer%20v5.0.0/Part%2006-Module%2001-Lesson%2004_GraphSLAM/18.%20RTAB-Map%20Optimization%20and%20Output.html*

在上一节课程中，我们介绍了机器人运动估计的基本原理，并实现了实时位姿估计。但是，我们也发现了一个问题：运动估计算法虽然能够持续计算机器人的位置和姿态，却无法完全避免误差累积。例如，一台机器人沿着办公室走廊移动一圈，最终回到起点。虽然机器人实际已经回到了原来的位置，但里程计估计的轨迹却可能无法闭合。

那么，机器人能否识别自己曾经到过的位置，并利用这一信息修正之前的运动轨迹？这正是本节课要介绍的 SLAM（Simultaneous Localization and Mapping，同步定位与建图）所要解决的核心问题之一。本节将重点学习回环检测（Loop Closure）、位姿图优化（Pose Graph Optimization），并在 Jetson J501 上使用 RTAB-Map 实现完整的 RGB-D SLAM 建图流程。

本节学习目标：

- **理解完整 SLAM 架构**：区分前端里程计与后端优化；
- **掌握回环检测**：理解机器人如何识别曾经到过的位置；
- **理解位姿图优化**：了解全局轨迹和地图如何被修正；
- **完成 RTAB-Map 实验**：实现回环检测、地图优化和结果验证。

## 一、从里程计到 SLAM

无论采用 VO、VIO 还是 LIO，每一次估计都可能存在误差。随着机器人不断移动，这些误差也会逐渐积累。这种漂移会带来一个直接的问题：机器人在不同时间经过同一位置时，可能会把同一个物体错误地记录在地图中的不同位置。

![闭合路径示意：估计轨迹（橙色）与真实起点（蓝色）无法精准重合](./images/m5-2-loop-closure-path.png)

为了减少这种累计误差，完整 SLAM 系统通常不会只依赖最近几帧的运动估计，而是将新的传感器观测与已经建立的地图进行比较。当机器人再次经过之前访问过的位置时，SLAM 有机会发现当前位置与历史位置之间存在关联。这个过程称为回环检测（Loop Closure）。

因此，相比于主要关注连续运动估计的里程计，完整 SLAM 不仅能够估计机器人运动，还能够利用历史观测修正轨迹，并维护全局一致性更好的环境地图。

## 二、完整 SLAM 系统的核心工作流程

一个典型的 SLAM 系统不仅需要通过前端里程计估计机器人运动，还需要记录历史观测、识别重复经过的场景，并利用这些信息优化机器人轨迹。整个过程可以概括为：

`运动估计` → `位姿图构建` → `回环检测` → `全局优化` → `地图更新`。

### 2.1 关键帧与位姿图构建

机器人在运动过程中，前端里程计会不断估计新的位姿。完整 SLAM 系统则需要进一步记录这些位姿之间的关系。为了避免保存所有传感器数据带来的计算负担，许多 SLAM 算法会选择具有代表性的观测，形成**关键帧（Keyframe）**或地图节点。这些节点及其相互关系可以组成一张**位姿图（Pose Graph）**。

![位姿图示例：T0–T4 节点通过里程计约束相连](./images/m5-2-pose-graph.png)

在位姿图中：

- 节点（Node）：表示机器人在某个时刻的位姿；
- 边（Edge）：表示两个位姿之间的相对运动约束。

最初，这些约束主要来自里程计。但由于里程计存在累计误差，仅靠相邻节点之间的约束，仍然无法保证整张地图的全局一致性。因此，SLAM 还需要利用历史观测建立额外的约束。

### 2.2 回环检测与约束建立

假设机器人从办公室出发，经过走廊后再次回到办公室。虽然机器人在两个时刻的里程计位置可能不同，但实际上它们对应的是同一个地方。回环检测（Loop Closure）的任务，就是发现这种历史位置与当前位置之间的关联。

以 RTAB-Map 为例，它可以利用视觉特征和词袋模型（Bag of Words）检索与当前图像相似的历史观测，再通过几何一致性检查确认是否真正发生回环。

整个过程主要包含三个步骤：

1. **场景检索**：从历史观测中寻找与当前场景相似的位置；
2. **几何验证**：利用特征匹配和几何关系排除错误匹配；
3. **回环约束**：在当前节点与历史节点之间建立新的位姿约束。

需要注意，回环检测不是简单地判断两张图像是否相似。例如，两个外观相似的走廊并不一定是同一个位置。如果错误地建立回环约束，反而可能导致整张地图发生畸变。因此，可靠的几何验证非常重要。

### 2.3 位姿图优化与地图更新

当回环检测成功后，SLAM 系统就在原有位姿图中增加了一条新的约束。此时可能出现一个矛盾：里程计认为机器人当前的位置距离起点还有一段距离，而回环检测却发现机器人已经回到了起点附近。

位姿图优化（Pose Graph Optimization）的作用，就是综合这些约束，重新计算一组更加一致的机器人位姿。

优化并不是简单地将终点移动到起点，而是综合里程计约束、回环约束及各自的不确定性，对相关节点的位姿进行联合调整。当位姿发生调整后，以这些位姿为基础生成的点云或其他地图数据也可以重新对齐，从而改善地图重影、墙面错位等问题。在 ROS 2 中，这种全局修正通常通过 `map → odom` 坐标变换体现。

这样，机器人既可以保留连续的局部运动估计，又能够利用 SLAM 获得经过修正的全局地图位置。

## 三、实操：使用 RTAB-Map 实现完整 SLAM

前面我们已经了解了完整 SLAM 系统的核心工作流程。接下来，我们将使用 Orbbec Gemini 2 相机与 RTAB-Map 算法，实现关键帧建图、回环检测和全局地图优化。

本节实验与上一课最大的区别是：上一课只关注相机如何运动，而本节将进一步利用历史观测修正运动轨迹，并构建具有全局一致性的环境地图。

![Orbbec Gemini 2 与 RTAB-Map 数据流：RGB-D 观测同时用于里程计与 SLAM 后端](./images/m5-2-rtabmap-gemini2-dataflow.png)

### 3.1 准备软件环境

在开始 SLAM 实验之前，我们需要确认 Jetson 已经安装 ROS 2、Orbbec 相机驱动、RTAB-Map 及相关开发工具。如果某个软件已经正常安装，可以直接跳过对应的安装步骤，继续检查下一个组件，避免重复安装。

**检查 ROS 2 Humble**

ROS 2 是本次实验的软件基础，Orbbec 相机驱动和 RTAB-Map 都需要在 ROS 2 环境中运行。

```bash
ls /opt/ros/
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO
ros2 --help
```

![终端中检查 ROS 2 Humble 环境可用性](./images/m5-2-ros2-humble-check.png)

如果输出 `humble`，并且 `ros2 --help` 能正常显示帮助信息，说明 ROS 2 命令行环境可用。否则，请参考[小白课程](https://github.com/Seeed-Projects/reComputer-Jetson-for-Beginners)安装 ROS。

**检查编译工具与依赖管理工具**

Orbbec ROS 2 驱动可能需要从源码编译，因此需要 Git、CMake、Colcon 和 rosdep 等工具。

```bash
git --version
cmake --version
colcon version-check
rosdep --version
```

如果命令均能正常输出版本信息，说明相关工具可用，可以继续检查 Orbbec 驱动。如果出现 `command not found`，则需要安装缺失的工具。

**检查 Orbbec Gemini 2 驱动**

确保已加载 ROS 2 环境。如果之前创建过 `~/ros2_ws` 工作空间，也需要加载其环境：

```bash
source /opt/ros/humble/setup.bash

if [ -f ~/ros2_ws/install/setup.bash ]; then
  source ~/ros2_ws/install/setup.bash
fi

ros2 pkg prefix orbbec_camera
ls /etc/udev/rules.d/99-obsensor-libusb.rules
ros2 run orbbec_camera list_devices_node
```

![终端中检查 Orbbec Gemini 2 驱动与设备识别](./images/m5-2-orbbec-driver-check.png)

如果程序能够识别 Gemini 2 并显示设备信息，说明相机连接和 SDK 访问正常。否则请执行下面的命令安装驱动。

```bash
# 下载源码
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b v2-main https://github.com/orbbec/OrbbecSDK_ROS2.git
cd ~/ros2_ws

# 安装驱动
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ros2_ws/install/setup.bash

# 配置权限
cd ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger

# 检查环境
ros2 pkg prefix orbbec_camera
ls /etc/udev/rules.d/99-obsensor-libusb.rules
```

**安装 RTAB-Map**

RTAB-Map 提供 ROS 2 Humble 的二进制软件包，可以直接使用 apt 工具安装：

```bash
sudo apt update
sudo apt install -y ros-humble-rtabmap-ros
ros2 pkg list | grep rtabmap
```

如果返回不为空，就可以继续配置可视化环境。

![终端中检查已安装的 RTAB-Map 相关软件包](./images/m5-2-rtabmap-pkg-list.png)

**检查可视化与调试工具**

```bash
command -v rviz2
command -v rtabmap-databaseViewer
ros2 pkg prefix tf2_tools
```

![终端中检查 rviz2、rtabmap-databaseViewer 与 tf2_tools](./images/m5-2-tools-check.png)

如果三个工具都能找到，则证明您的设备中已经有后续实验所需要的可视化工具，否则，请参考[小白课程](https://github.com/Seeed-Projects/reComputer-Jetson-for-Beginners)安装。

### 3.2 启动 RTAB-Map SLAM

#### 启动相机

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

#### 启动 SLAM

```bash
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
  database_path:=$HOME/slam_maps/m5_2_gemini2.db \
  rtabmap_viz:=true \
  rviz:=false
```

这些参数由 RTAB-Map 官方 ROS 2 Launch 文件提供。其中，`visual_odometry:=true` 会启动 RGB-D 里程计节点，因此不需要另外运行 `rgbd_odometry`。

几个关键参数如下：

| 参数 | 说明 |
|-|-|
| `visual_odometry:=true` | 自动启动 RGB-D 视觉里程计 |
| `frame_id:=camera_link` | 将相机本体作为运动参考 |
| `vo_frame_id:=odom` | 设置局部里程计坐标系 |
| `approx_sync:=true` | 使用近似时间同步 |
| `qos:=2` | 以 Best Effort QoS 接收传感器数据 |
| `publish_tf_odom:=true` | 发布局部里程计 TF |
| `publish_tf_map:=true` | 发布全局地图修正 TF |
| `database_path` | SLAM 数据库保存位置 |
| `rtabmap_viz:=true` | 开启 RTAB-Map 可视化界面 |

如果通过 SSH 在无桌面环境下运行，可以将 `rtabmap_viz:=false`，避免图形界面初始化失败。

首次实验应使用一个新的数据库文件名。如果已经存在同名数据库，RTAB-Map 可能加载并继续使用之前的地图。

### 3.3 观察实时 SLAM 建图

![RTAB-Map Database Viewer：3D 地图、里程计与回环检测视图](./images/m5-2-database-viewer-loop.png)

首先保持相机朝向环境中具有明显纹理的区域，例如桌子、门框或墙面上的海报，然后缓慢移动相机。正常情况下，地图会随着相机运动逐渐扩展，位姿图中也会持续增加新的节点。

**为了验证回环检测与图优化，本次实验需要让相机重新观察已经访问过的位置。**

![RTAB-Map 地图可视化：Table、Cabinet、Equipment 标签与闭合轨迹](./images/m5-2-map-labels.png)

### 3.4 使用 Database Viewer 查看地图

试验结束后，可以在终端窗口中按下 CTRL+C 结束 SLAM 程序，并等待地图保存。然后使用 RTAB-Map 专门的 Database Viewer 工具检查已保存的 SLAM 地图。

在具备图形桌面的终端执行：

```bash
rtabmap-databaseViewer ~/slam_maps/m5_2_gemini2.db
```

![RTAB-Map Database Viewer：检查已保存的 SLAM 数据库](./images/m5-2-database-viewer-saved.png)

打开数据库后，建议重点检查历史位姿节点、回环约束、优化后的轨迹和点云地图是否与采集环境一致。本节只需要完成 SLAM 数据库的保存与检查。后续 M5.5 将进一步介绍地图加载、重定位，以及导出 Nav2 所需的二维导航地图。

## 四、本节总结

通过本节课程，我们学习了完整 SLAM 系统的核心工作原理，并在 Jetson 上使用 Orbbec Gemini 2 和 RTAB-Map 完成了 RGB-D SLAM 实验。

从系统工作过程来看：

- 视觉里程计负责连续估计相机运动，提供局部位姿信息；
- 位姿图负责管理历史位姿及其约束关系；
- 回环检测负责识别曾经访问过的场景，建立新的历史位置约束；
- 全局优化根据这些约束调整相关位姿，提高运动轨迹和地图的全局一致性。

通过本次实验，我们不仅实现了实时建图，还学习了如何检查回环检测结果、分析地图质量，以及保存和查看 SLAM 数据库。不过，SLAM 系统输出的运动轨迹、位姿图和点云，并不能直接解决机器人所有的空间理解问题。当机器人需要判断障碍物占据了多大的空间、机械臂末端距离桌面还有多远，或者一条路径与障碍物之间是否存在安全距离时，就需要更加适合空间计算的三维几何表示。
