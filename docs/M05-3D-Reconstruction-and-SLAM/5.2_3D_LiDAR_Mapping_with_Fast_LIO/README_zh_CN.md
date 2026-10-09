# 5.2 与惯性里程计紧耦合的3D SLAM建图

二维栅格地图已经能支持简单环境下的导航任务，但是它只能表达某一水平切面上的障碍物轮廓，而对现实世界中的立体障碍物如桌子、楼梯等，二维栅格地图难以让机器人精确的感知障碍物，于是，3D激光雷达带来的三维SLAM技术应运而生，机器人一旦进入更复杂的三维环境，就需要由三维点云构建的度量地图。

本节课是5.1的进阶课程，将2D平面建图拓展到3D建图，并引入激光-惯性里程计的紧耦合。并在`reComputer Robotics J501` 与 Livox MID-360平台 上部署Fast-LIO2，复用 M3 里已经验证过的传感器接入方式。

## 学习目标

- 了解常见三维激光建图路线，并理解Fast-LIO2的基本原理
- 理解imu里程计的工作方式，提供哪些信息
- 能独立完成 MID-360 在 ROS 2 下的驱动编译、网络配置和点云确认
- 能在 J501 上部署并启动 Fast-LIO2，完成一次慢速室内建图
- 能保存轨迹和 PCD 等建图结果，并用几何一致性、重访误差和运行稳定性判断地图是否可用

## 前置条件

- 已完成第 5.1 课，手里有一张可复用的二维占用栅格
- 已完成 M3.1：MID-360 能通过专用以太网发布激光和 IMU 话题
- 对时间戳、坐标系、协方差有基本概念，能看懂 M3.2 / M3.4 相关内容
- 板子上已有 Ubuntu 22.04 + ROS 2 Humble；本课会从 Livox-SDK2 开始把驱动和 Fast-LIO2 装起来

---

# Part A. 从二维栅格地图到三维地图

## 5.2.1 3D SLAM VS 2D SLAM

二维 SLAM 常常可以把一圈激光近似看成“几乎同一时刻”的平面观测，。三维激光建图不是这样。

旋转式或扫描式三维激光，会在一段非零时间窗口里持续扫描周围环境。如果机器人在这段时间里发生了平移或旋转，先前打出的激光返回到雷达，后面的激光刚刚打出，而作为参照系的机器人位置姿态发生了变化，单帧点云就会变形。如果不做去畸变，扫描出的障碍物会变形，平直的墙面和地板可能会出现弯曲，影响建图效果。

所以，现代三维激光建图通常至少要具备这几件事：

1. 高频 IMU 先验
2. 逐点时间戳，或至少帧内时间信息
3. 能边运动边持续更新的状态估计器
4. 随着雷达移动不断扩展的地图结构

在 J501 + MID-360 这套组合上，这些条件已经具备：

- 点云带逐点 `offset_time`
- IMU 大约 200 Hz
- 在合理配置下，J501 能扛住 Fast-LIO2 这一类实时建图负载

![M5.2-1.png](./images/M5.2-1.png)

## 5.2.2 三维激光建图

三维激光建图经过长期发展，目前常见的工程实现主要包括激光里程计、激光-惯性里程计（LIO）以及结合后端优化的建图系统。

较早期的典型路线以 LOAM 为代表，通常从点云中提取边缘、平面等几何特征，并通过 scan-to-scan 或 scan-to-map 估计传感器运动。其基本思想是将高频运动估计与较低频的地图优化分开处理，这一架构也影响了后续许多激光建图系统。

随着 IMU 的引入，激光-惯性里程计（LIO）逐渐成为移动机器人中的重要实时建图方案。该类方法将 IMU 预测与激光观测结合起来：IMU 提供高频运动先验，并用于扫描去畸变；激光观测则进一步校正系统的姿态、位置、速度及偏置。

在此基础上，Fast-LIO 系列进一步采用紧耦合的激光-惯性状态估计框架，以较低的计算开销实现实时定位与建图。Fast-LIO2 在此基础上进一步改进了点云处理与地图匹配方式，能够直接利用原始点云进行状态估计，在保证实时性的同时具有较好的精度和鲁棒性。因此，本节将以 **Fast-LIO2** 为例，介绍三维激光-惯性建图系统的部署与基本使用方法。

本节的目标是在 J501 上完成一套可重复的三维激光建图流程。Fast-LIO2 采用紧耦合的激光-惯性里程计框架，可以直接利用 MID-360 的点云和内置 IMU，实现实时状态估计、点云去畸变和增量地图构建，并能够在 Jetson 平台上保持较低的计算负载。本节主要围绕 Fast-LIO2 的部署与基本运行展开，完整回环检测、后端图优化、视觉融合和稠密重建等内容将在后续进阶部分介绍。

在 Fast-LIO2 中，激光与 IMU 数据统一进入迭代误差状态卡尔曼滤波器（iEKF）进行状态估计。首先，IMU 以较高频率预测系统的姿态、位置、速度及偏置，并利用预测得到的短时运动状态对激光扫描进行去畸变；随后，将去畸变后的点云与当前增量地图进行匹配，构建激光观测残差，并通过迭代更新校正系统状态；完成状态估计后，将对齐后的点云加入增量地图，为下一帧点云匹配提供参考。

---

# Part B. 实践：基于 ROS 2 Humble 框架部署 Fast-LIO2

本节以 J501 和 Livox MID-360 为硬件平台，完成一套可重复的三维激光建图流程。整体部署链路如下：

**安装 Livox-SDK2 → 编译 `livox_ros_driver2` → 配置网络并验证点云 → 编译 Fast-LIO2 → 配置 `mid360.yaml` → 启动建图并保存结果。**

## 5.2.1 课程环境

本节实验基于以下软硬件环境进行：

| 项目             | 课程取值                                             |
| ---------------- | ---------------------------------------------------- |
| 系统             | Ubuntu 22.04                                         |
| ROS              | ROS 2 Humble                                         |
| 激光雷达         | Livox MID-360                                        |
| 建图算法         | Fast-LIO2                                            |
| 工作空间         | `~/ros2_ws`                                          |
| Fast-LIO 仓库    | `https://github.com/Ericsii/FAST_LIO_ROS2`           |
| 主机有线 IP 示例 | `192.168.1.50/24`                                    |
| MID-360 IP 规则  | 默认形如 `192.168.1.1xx`，后两位由雷达 SN 末两位决定 |

> **网络配置说明：** 如果主机 IP 不是 `192.168.1.50`，后续网络相关配置应替换为实际地址，并确保主机 IP、`MID360_config.json` 中的主机地址以及 MID-360 的 IP 地址保持一致。

## 5.2.2 安装 Livox-SDK2

Livox-SDK2 是连接 Livox 激光雷达的底层软件开发套件。本节首先完成 SDK 的编译与安装，为后续 `livox_ros_driver2` 的编译提供依赖。

### 1. 克隆仓库

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
```

### 2. 编译并安装

```bash
cd ~/Livox-SDK2
mkdir build && cd build
cmake .. && make -j
sudo make install
```

### 3. 验证安装

SDK 安装完成后，可以使用官方示例进行基本验证。该步骤不要求必须连接雷达；未连接设备时程序可能出现设备连接相关提示，但只要 SDK 能够正常启动，通常即可说明编译和安装过程完成。

```bash
find ~/Livox-SDK2 -name "mid360_config.json"
cd ~/Livox-SDK2/build/samples/livox_lidar_quick_start
./livox_lidar_quick_start /home/$USER/Livox-SDK2/samples/livox_lidar_quick_start/mid360_config.json
```

如果实际安装路径或用户名不同，请将命令中的路径替换为本机对应路径。

## 5.2.3 下载并编译 `livox_ros_driver2`

完成 SDK 安装后，继续在 ROS 2 工作空间中配置 Livox ROS 2 驱动。

### 1. 创建工作空间

如果当前系统尚未创建工作空间，可以执行：

```bash
mkdir -p ~/ros2_ws/src
```

### 2. 克隆驱动仓库

```bash
cd ~/ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

### 3. 准备 ROS 2 的 `package.xml`

`livox_ros_driver2` 仓库默认提供 `package_ROS1.xml` 和 `package_ROS2.xml`，需要将 ROS 2 版本复制为标准的 `package.xml`：

```bash
cd ~/ros2_ws/src/livox_ros_driver2
cp package_ROS2.xml package.xml
```

缺少该文件时，ROS 2 工作空间无法按照标准方式识别该软件包，后续编译会失败。

### 4. 使用仓库提供的编译脚本

`livox_ros_driver2` 建议使用仓库提供的 `build.sh` 完成编译，而不是直接在工作空间中执行 `colcon build`：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src/livox_ros_driver2
./build.sh humble
cd ~/ros2_ws
source install/setup.bash
```

编译过程中出现一般性的警告信息通常不会影响最终结果。重点确认编译过程最终正常完成。

## 5.2.4 配置网络并验证 MID-360 点云

Fast-LIO2 需要同时接收 MID-360 的激光点云和 IMU 数据，因此在启动建图算法之前，应首先确认主机与雷达之间的网络通信正常，并验证 ROS 2 驱动能够稳定发布点云。

### 1. 配置主机有线网卡

在系统网络设置中，将连接 MID-360 的有线网卡设置为手动 IPv4。例如：

| 项目     | 示例值          |
| -------- | --------------- |
| 地址     | `192.168.1.50`  |
| 子网掩码 | `255.255.255.0` |
| 网关     | 留空            |

### 2. 配置 `MID360_config.json`

打开驱动配置文件：

```bash
nano ~/ros2_ws/src/livox_ros_driver2/config/MID360_config.json
```

确保配置文件中的主机 IP 与实际有线网卡地址一致。例如主机使用 `192.168.1.50`，则配置文件中的对应地址也应设置为 `192.168.1.50`。

MID-360 的默认 IP 通常采用 `192.168.1.1xx` 的形式，其末两位与雷达 SN 的末两位对应。例如，若雷达 SN 末两位为 `23`，则对应 IP 通常为：

```text
192.168.1.123
```

实际使用时应以设备当前网络配置为准。

### 3. 启动驱动并验证点云

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

正常情况下，RViz 中应能够观察到稳定的 MID-360 点云。建议确认点云连续发布且不存在明显的丢帧、跳变或整体漂移。

![Screenshot from 2026-09-24 15-35-42.png](./images/Screenshot%20from%202026-09-24%2015-35-42.png)

完成该步骤后，再进入 Fast-LIO2 的部署。

## 5.2.5 安装并编译 Fast-LIO2

### 1. 安装依赖

```bash
sudo apt install -y libeigen3-dev libpcl-dev
sudo apt install -y ros-humble-pcl-conversions ros-humble-pcl-ros
```

### 2. 克隆 Fast-LIO2 ROS 2 仓库

官方 Fast-LIO 早期版本主要面向 ROS 1，本节使用 ROS 2 Humble 社区维护版本：

```text
https://github.com/Ericsii/FAST_LIO_ROS2
```

克隆时需要同时获取仓库的 submodule：

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git
```

如果克隆过程中未使用 `--recursive`，可以手动初始化 submodule：

```bash
cd ~/ros2_ws/src/FAST_LIO_ROS2
git submodule update --init --recursive
```

### 3. 编译 Fast-LIO2

仓库位于工作空间的 `src` 目录下，因此需要返回 `~/ros2_ws` 根目录执行 `colcon build`：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select fast_lio --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

为了保证后续实验结果具有可复现性，建议记录当前仓库的 Git commit：

```bash
cd ~/ros2_ws/src/FAST_LIO_ROS2
git rev-parse --short HEAD
```

## 5.2.6 配置 Fast-LIO2 参数

首先创建地图保存目录：

```bash
mkdir -p ~/maps/fast_lio
```

然后打开 MID-360 对应的 Fast-LIO2 配置文件：

```bash
nano ~/ros2_ws/src/FAST_LIO_ROS2/config/mid360.yaml
```

确认以下参数：

```yaml
extrinsic_est_en: false

map_file_path: "/home/你的用户名/maps/fast_lio/mid360_current.pcd"

pcd_save:
    pcd_save_en: true
    interval: -1
```

其中：

- `extrinsic_est_en: false`：关闭在线外参估计，首次部署时使用配置文件中的固定外参。

- `map_file_path`：指定地图文件的保存位置，建议使用绝对路径。

- `pcd_save_en: true`：启用 PCD 地图保存功能。

- `interval: -1`：按照当前配置保存最终地图。

Fast-LIO2 使用的激光和 IMU 话题通常已经在配置文件中设置为：

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/livox/imu"
```

如果 `livox_ros_driver2` 未修改默认话题名称，此处通常无需调整。

> **图片占位：** `images/mid360_extrinsic_check.png`  
> **建议内容：** 展示 `mid360.yaml` 中外参、`map_file_path` 和 `pcd_save_en` 等关键参数。

## 5.2.7 启动 Fast-LIO2 建图

Fast-LIO2 建图需要同时运行 Livox 驱动和 Fast-LIO2 节点。建议分别在两个终端中启动。

### 终端 A：启动 Livox 驱动

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 终端 B：启动 Fast-LIO2

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

如果需要同时启动 RViz，可以使用：

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=true
```

需要注意，点云验证阶段使用的是 `rviz_MID360_launch.py`，而正式建图阶段使用的是 `msg_MID360_launch.py`。

### 首次建图建议

首次运行时建议按照以下方式进行：

1. 启动后保持设备静止约 5–10 秒，使 IMU 状态得到充分初始化。

2. 初始阶段采用低速平移，并进行平缓转弯。

3. 优先选择墙面、立柱、门框等具有明显几何结构的室内环境。

4. 完成一段运动后返回具有明显结构特征的位置，观察地图重访时是否存在明显漂移。

### 检查系统状态

可以通过以下命令检查主要话题的发布状态：

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | rg -i 'livox|odom|path|cloud'
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
```

> **图片占位：** `images/j501_fastlio_rviz.jpg`  
> **建议内容：** 展示 Fast-LIO2 正常运行时的 RViz 界面，包括实时点云、增长中的地图和运动轨迹。

## 5.2.8 保存轨迹与地图

### 1. 保存 PCD 地图

如果 `mid360.yaml` 中已经启用 `pcd_save_en`，建图结束后可以调用地图保存服务：

```bash
source ~/ros2_ws/install/setup.bash
ros2 service list | rg map_save
ros2 service call /map_save std_srvs/srv/Trigger {}
ls -lh ~/maps/fast_lio/
```

### 2. 同时录制 ROS 2 Bag

为了保留原始实验数据，建议在建图过程中同步记录 ROS 2 Bag：

```bash
source ~/ros2_ws/install/setup.bash
mkdir -p ~/maps/fast_lio
ros2 bag record -o ~/maps/fast_lio/mid360_loop_bag \
  /Odometry \
  /path \
  /cloud_registered \
  /tf \
  /tf_static
```

如果当前系统的话题名称与上述示例不同，应先通过 `ros2 topic list` 确认实际名称，再进行替换。

### 3. 导出轨迹

如需进一步分析建图结果，可以从 `/path` 或 ROS 2 Bag 中导出 TUM 格式轨迹：

```text
timestamp tx ty tz qx qy qz qw
```

该格式可以用于后续轨迹可视化、误差分析以及不同算法之间的结果对比。

> **图片占位：** `images/pcd_map_and_projected_2d.png`  
> **建议内容：** 展示保存后的 PCD 三维地图，并与第 5.1 节中的二维占用栅格地图进行对照。

## 5.2.9 建图质量检查与常见问题

建图结果不能仅通过 RViz 中的视觉效果进行判断。至少需要同时关注系统运行稳定性、地图几何一致性、重访区域的漂移情况以及实时处理能力。

### 最低检查项

1. **静止稳定性：** 设备静止 5–10 秒后，轨迹应保持基本稳定。

2. **几何一致性：** 在走廊等规则环境中，墙面应保持较好的平直性。

3. **重访误差：** 返回已经经过的区域时，相同结构应能够较好地重合。

4. **实时性：** 建图过程中处理速度应能够跟随传感器数据，不应持续产生明显的数据积压。

### 常用检查命令

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
ros2 run tf2_ros tf2_echo camera_init body
top
```

如果实际系统中的世界坐标系名称不是 `camera_init`，应根据当前 TF 树中的实际名称进行替换。

### 常见问题

| 现象                               | 可能原因                            | 处理方法                                                                |
| ---------------------------------- | ----------------------------------- | ----------------------------------------------------------------------- |
| 点云出现明显拖影或“果冻”现象       | 时间戳、去畸变或 IMU 数据存在问题   | 检查点云时间信息和 IMU 发布频率，并确认建图使用 `msg_MID360_launch.py`  |
| 初始运动后地图明显倾斜             | 外参或坐标轴方向配置错误            | 检查 `mid360.yaml` 中的外参与雷达坐标系定义                             |
| 经过门口等区域时里程计出现明显跳变 | 运动速度过快或环境几何特征不足      | 降低运动速度，并选择具有更多结构特征的区域进行测试                      |
| Livox 驱动编译失败                 | 缺少 `package.xml` 或编译方式不正确 | 执行 `cp package_ROS2.xml package.xml`，并使用 `./build.sh humble` 编译 |
| Fast-LIO2 编译缺少文件             | submodule 未正确获取                | 执行 `git submodule update --init --recursive`                          |
| Fast-LIO2 编译失败                 | `colcon build` 执行目录错误         | 返回 `~/ros2_ws` 根目录执行编译                                         |
| 能够 ping 通雷达但没有点云或 IMU   | 网络配置不一致                      | 检查主机 IP、`MID360_config.json` 和雷达 IP                             |
| 节点退出后仍有残留进程             | ROS 2 节点未正常关闭                | 检查并终止残留进程                                                      |
| 重新编译后仍使用旧的编译结果       | `build` / `install` 中存在旧产物    | 删除对应软件包的编译目录后重新构建                                      |

如需进一步排查，可以使用以下命令：

```bash
# 刷新动态库缓存
sudo ldconfig

# 临时关闭防火墙，用于排查网络通信问题
sudo ufw disable

# 清理 livox_ros_driver2 的旧编译产物
rm -rf ~/ros2_ws/build/livox_ros_driver2
rm -rf ~/ros2_ws/install/livox_ros_driver2
```

> **注意：** `sudo ufw disable` 仅建议用于故障定位。完成测试后，应根据实际系统安全要求恢复防火墙配置。

## 5.2.10 实验与验收

完成上述部署后，应在实际 MID-360 平台上完成以下实验：

1. **SDK 与驱动：** 完成 Livox-SDK2 和 `livox_ros_driver2` 的编译。

2. **点云验证：** 使用 `rviz_MID360_launch.py` 确认 MID-360 点云稳定发布。

3. **建图启动：** 使用 `msg_MID360_launch.py` 和 `mapping.launch.py` 启动 Fast-LIO2。

4. **环路建图：** 在室内环境中进行低速环路运动，生成连续的三维地图。

5. **结果对照：** 将同一环境的三维建图结果与第 5.1 节中的二维占用栅格地图进行对比。

### 实验结果

建议至少保留以下实验结果：

- 包含 Git commit hash 和配置文件版本的实验记录；

- 完成室内环路后的 RViz 建图截图；

- 包含里程计和轨迹信息的 ROS 2 Bag 或轨迹文件；

- 保存后的 PCD 三维地图；

- 一份针对实际运行问题的简要排查记录。

### 验收标准

| 检查项       | 通过标准                                             |
| ------------ | ---------------------------------------------------- |
| 传感器状态   | MID-360 激光与 IMU 均能够正常发布数据                |
| 建图稳定性   | 低速环路运动过程中，Fast-LIO2 能够持续输出稳定里程计 |
| 地图结果     | 能够生成并重新读取三维地图或对应点云数据             |
| 几何一致性   | 墙面和主要环境结构清晰，无明显拖影或畸变             |
| 系统可复现性 | 坐标系、话题名称、配置文件和地图保存路径均有明确记录 |

## 5.2.11 课程小结

本节完成了从 Livox-SDK2、ROS 2 驱动配置到 Fast-LIO2 三维激光建图的完整部署流程。在前面的二维建图基础上，本节进一步引入激光与 IMU 紧耦合的三维状态估计，使系统能够在 J501 平台上实时输出运动轨迹和三维点云地图。

本节的核心目标是建立一套稳定、可重复的 **MID-360 + Fast-LIO2** 建图环境。完成部署后，应能够独立完成传感器接入、网络配置、建图启动以及地图和轨迹数据保存。

## 下一步

在 Fast-LIO2 能够稳定运行的基础上，可以进一步扩展以下方向：

- 在 LIO 的基础上引入相机，开展多传感器融合建图；

- 引入回环检测和后端优化，提高大范围环境下的全局一致性；

- 在三维地图基础上进一步进行环境重建和语义信息融合。

在进入后续内容之前，建议首先确保 MID-360 与 Fast-LIO2 的部署流程能够稳定、重复地运行。

## 参考资料

- Xu, W., Zhang, F., et al. Fast-LIO / Fast-LIO2.

- Zhang, J., Singh, S. LOAM: Lidar Odometry and Mapping in Real-time.

- Shan, T., et al. LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping.

- [Fast-LIO ROS 2](https://github.com/Ericsii/FAST_LIO_ROS2)

- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)

- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) 
