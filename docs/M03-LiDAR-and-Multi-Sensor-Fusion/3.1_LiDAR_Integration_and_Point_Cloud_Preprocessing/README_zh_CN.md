# 3.1 激光雷达接入与点云预处理

激光雷达通过测量激光回波获得环境距离。二维雷达通常输出一圈距离序列，三维雷达则输出空间点云。本课以“能看见、能检查、能清洗”为目标，为后续定位、建图和避障建立稳定的数据入口。

![激光雷达点云从原始采集到范围裁剪、机身剔除和降采样的处理流程](./images/lidar_point_cloud_preprocessing.png)

> 原始点云依次经过空间裁剪、机器人本体区域剔除和降采样，最终保留结构清晰、可供下游算法使用的环境点云。

## 学习目标

- 区分 `LaserScan` 与 `PointCloud2`；
- 在 J501 上完成以太网或 USB 雷达接入；
- 检查话题频率、带宽、时间戳、坐标系和 QoS；
- 完成范围裁剪、体素降采样和离群点剔除；
- 使用 RViz2 与 rosbag 验证处理结果。

## 3.1.1 雷达数据模型

飞行时间法可用 $d=c\Delta t/2$ 理解：光往返目标一次，因此传播距离需要除以 2。

| ROS 2 消息 | 常见设备 | 内容 | 典型用途 |
| --- | --- | --- | --- |
| `sensor_msgs/msg/LaserScan` | 2D 雷达 | 起始角、角分辨率、距离与强度数组 | 2D SLAM、平面避障 |
| `sensor_msgs/msg/PointCloud2` | 3D 雷达、深度相机 | `x/y/z` 及可选的强度、环号、逐点时间 | 3D SLAM、检测、地面分割 |

`PointCloud2` 是结构化二进制消息。不同设备的字段并不完全相同，不能默认一定存在 `intensity`、`ring` 或 `time`。二维雷达的 `LaserScan` 也可能包含 NaN、Inf 或量程外数据，下游节点必须按照消息中的 `range_min` 和 `range_max` 判断有效性。

## 3.1.2 硬件与网络准备

准备 J501、ROS 2 Humble、受 ROS 2 驱动支持的雷达、数据线和符合规格的独立电源。大功率雷达不要由普通 USB 口直接供电；上电前确认电压、峰值电流、极性和接地。

### 以太网雷达

若雷达地址为 `192.168.1.201/24`，专用网口为 `eth1`，可临时配置：

```bash
sudo ip addr flush dev eth1
sudo ip addr add 192.168.1.100/24 dev eth1
sudo ip link set eth1 up
ip -br addr show eth1
ping -c 4 192.168.1.201
```

实际参数以设备手册为准。多网口处于同一网段时，用 `ip route get 192.168.1.201` 检查数据包是否走正确网口。

### USB 或串口雷达

```bash
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
sudo dmesg | grep -Ei 'usb|ttyUSB|ttyACM' | tail -50
```

建议使用 udev 规则创建固定设备名，避免设备重插后 `/dev/ttyUSB*` 编号改变。串口权限应通过 `dialout` 用户组管理，而不是长期执行 `chmod 777`。

### 启动厂商驱动

不同品牌的驱动包和参数不同，先检查其 launch 文件，再按以下模式启动：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
ros2 launch <lidar_driver_package> <driver_launch_file> \
  <connection_parameters> frame_id:=lidar_link
```

驱动至少应发布 `/scan` 或 `/points_raw`。如果同时发布厂商专用消息与标准消息，课程后续优先使用标准 ROS 2 消息。

## 3.1.3 RPLIDAR A1 接入演示

本节使用 SLAMTEC RPLIDAR A1 演示二维 USB 雷达从安装到可视化的完整效果。它只是本课的具体示例，其他二维或三维雷达仍按前述通用方法接入。

>进行下面的操作之前，请参考[基础教程](https://github.com/Seeed-Projects/reComputer-Jetson-for-Beginners)安装ROS2.
>如果您正在使用其他型号的激光雷达，请参考激光雷达厂商提供的使用手册操作。
### 演示环境

| 项目 | 实测配置 |
| --- | --- |
| 计算设备 | reComputer Robotics J5011 |
| 操作系统 | Ubuntu 22.04.5 LTS，aarch64 |
| ROS 2 | Humble |
| 雷达 | SLAMTEC RPLIDAR A1 |
| USB 转串口 | Silicon Labs CP210x |
| ROS 驱动 | `rplidar_ros 2.1.4` |
| 驱动内置 SDK | 2.0.0 |

### 安装驱动

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y ros-humble-rplidar-ros

ros2 pkg prefix rplidar_ros
ros2 pkg executables rplidar_ros
```

如果需要修改驱动，也可以选择源码编译。APT 与源码安装二选一，避免工作区包覆盖系统包后产生版本混淆。

### 检查设备

```bash
lsusb
ls -l /dev/rplidar /dev/ttyUSB0
```

正常情况下可看到 CP210x 设备以及固定串口软链接：

```text
ID 10c4:ea60 Silicon Labs CP210x UART Bridge
/dev/rplidar -> ttyUSB0
```

驱动包提供的 udev 规则位于：

```text
/lib/udev/rules.d/60-ros-humble-rplidar-ros.rules
```

### 启动并检查健康状态

```bash
source /opt/ros/humble/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py \
  serial_port:=/dev/rplidar
```

A1 默认波特率为 `115200`。成功启动时，实测日志包含：

![启动雷达](./images/rplidar_launch.jpg)

能够读取设备信息并出现 `health status: OK`，才说明驱动与雷达主控已完成通信。

### 查看 `/scan` 效果

```bash
ros2 node list
ros2 topic info /scan --verbose
ros2 topic hz /scan
ros2 topic echo /scan --once
```

![查看话题](./images/check_topic.jpg)

参考测试结果：

| 数据项 | 实测结果 |
| --- | ---: |
| 消息类型 | `sensor_msgs/msg/LaserScan` |
| 坐标系 | `laser` |
| 单帧采样点数 | 1080 |
| 有限距离点数 | 862 |
| 有效点比例 | 79.8% |
| 实测最近距离 | 约 0.121 m |
| 实测最远距离 | 约 9.768 m |
| 消息量程配置 | 0.15–12 m |
| 扫描范围 | 接近 360° |
| ROS 消息平均频率 | 约 7.55 Hz |

驱动报告的目标频率为 10 Hz，实际 ROS 消息约为 7.55 Hz，可能受到机械转速、扫描模式和角度补偿影响。0.121 m 小于消息声明的 `range_min=0.15 m`，应用程序不应把该点当作可靠障碍物。

可以一次启动驱动和 RViz2：

```bash
ros2 launch rplidar_ros view_rplidar_a1_launch.py \
  serial_port:=/dev/rplidar
```

或者在雷达驱动已经运行时，只启动 RViz2：

```bash
rviz2 -d /opt/ros/humble/share/rplidar_ros/rviz/rplidar_ros.rviz
```

不要在普通 launch 已运行时再次执行 view launch，因为后者会再启动一个 `rplidar_node`，造成串口争用。

![查看话题](./images/rviz_rplidar.jpg)

## 3.1.4 Livox MID-360 接入演示

本节使用 Livox MID-360 演示三维以太网雷达的接入效果。与通过 USB 串口发布二维 `LaserScan` 的 RPLIDAR A1 不同，MID-360 通过有线网络传输三维点云，并同时发布高频 IMU 数据。

### 演示环境与网络拓扑

| 项目 | 实测配置 |
| --- | --- |
| 计算设备 | reComputer Robotics J5011 |
| 操作系统 | Ubuntu 22.04.5 LTS，aarch64 |
| ROS 2 | Humble |
| 雷达 | Livox MID-360 |
| ROS 驱动 | Livox ROS Driver2 1.2.7 |
| Jetson 有线 IP | `192.168.1.5`（静态） |
| MID-360 IP | `192.168.1.3` |

本次测试使用双网卡：有线网口直连雷达，Wi-Fi 继续连接外部网络。两个接口位于不同网段，默认路由仍由 Wi-Fi 承担。

```bash
ip -br link
ip -br addr
ip route
```

参考配置：

```text
enP8p1s0   192.168.1.5/24       # 雷达专用有线网口
wlP1p1s0   192.168.3.188/24     # Wi-Fi 外网
```

检查雷达链路：

```bash
ping -I enP8p1s0 -c 3 192.168.1.3
ip neigh show dev enP8p1s0
```

参考测试为 3 次发送、3 次接收、0% 丢包，平均延迟约 1.6 ms。雷达网口不需要设置默认网关，否则可能影响 Wi-Fi 联网。

### 安装 Livox-SDK2

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ~/Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

安装完成后，SDK 的共享库和头文件通常位于 `/usr/local/lib` 与 `/usr/local/include`。可检查：

```bash
ls -l /usr/local/lib/liblivox_lidar_sdk_shared.so
ls -l /usr/local/include/livox_lidar_api.h
```

### 安装并编译 Livox ROS Driver2

```bash
mkdir -p ~/ws_livox/src
cd ~/ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

cd ~/ws_livox/src/livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
```

参考测试的构建结果为：

![查看话题](./images/build_livox_sdk.jpg)

加载并检查工作区：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
ros2 pkg prefix livox_ros_driver2
ros2 pkg executables livox_ros_driver2
```

### 配置主机和雷达 IP

编辑驱动配置：

```bash
nano ~/ws_livox/src/livox_ros_driver2/config/MID360_config.json
```

参考测试的关键配置如下：

```json
{
  "MID360": {
    "host_net_info": {
      "cmd_data_ip": "192.168.1.5",
      "push_msg_ip": "192.168.1.5",
      "point_data_ip": "192.168.1.5",
      "imu_data_ip": "192.168.1.5"
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.3",
      "pcl_data_type": 1,
      "pattern_mode": 0
    }
  ]
}
```

所有 `host_*_ip` 都应填写与 MID-360 同网段的主机有线 IP，而不是 Wi-Fi IP。参考设备使用的控制、推送、点云、IMU 和日志端口分别位于 `56100–56501` 范围，不应在不了解协议时随意修改。

修改源码目录中的配置后，重新执行：

```bash
cd ~/ws_livox/src/livox_ros_driver2
./build.sh humble
source ~/ws_livox/install/setup.bash
```

### 启动自定义点云与 IMU

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

驱动成功连接时，参考日志包含：

```text
Livox Ros Driver2 Version: 1.2.7
Init lds lidar success!
successfully set lidar attitude, ip: 192.168.1.3
successfully change work mode
successfully enable Livox Lidar imu, ip: 192.168.1.3
livox/imu publish use imu format
livox/lidar publish use livox custom format
```

检查节点和话题：

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /livox/lidar --verbose
ros2 topic info /livox/imu --verbose
```

参考输出：

```text
/livox_lidar_publisher
/livox/lidar [livox_ros_driver2/msg/CustomMsg]
/livox/imu [sensor_msgs/msg/Imu]
```

`CustomMsg` 保存帧时间基准、点数、设备编号和点数组；每个点包含 `x/y/z`、反射率、标签、扫描线编号与相对时间 `offset_time`。逐点相对时间对运动去畸变非常重要。

### 启动 PointCloud2 与 RViz2

要直接演示标准 `PointCloud2` 并打开 RViz2，运行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

将 RViz2 的 `Fixed Frame` 设置为 `livox_frame`，添加驱动发布的 `PointCloud2` 话题。`msg_MID360_launch.py` 面向需要 Livox 逐点字段的算法，而 `rviz_MID360_launch.py` 更适合标准点云可视化和本课的通用点云预处理流程。

![PointCloud2 View](./images/pointcloud2_view.jpg)


### 实测数据效果

```bash
ros2 topic echo /livox/lidar --once --field header.frame_id
ros2 topic echo /livox/lidar --once --field point_num
ros2 topic echo /livox/lidar --once --field lidar_id
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic bw /livox/lidar
```

参考测试结果：

| 数据项 | 实测结果 |
| --- | ---: |
| 点云 frame | `livox_frame` |
| 单帧点数 | 19,968 |
| 点云平均频率 | 10.00 Hz |
| 点云周期范围 | 0.092–0.108 s |
| IMU 平均频率 | 199.99 Hz |
| IMU 周期范围 | 0.004–0.006 s |
| 点云带宽 | 约 3.8 MB/s |
| 单条点云消息 | 约 0.40 MB |

上述带宽说明原始点云应使用稳定的有线连接。继续检查网卡统计：

```bash
ip -s link show enP8p1s0
```

重点关注 RX/TX 的 `errors`、`dropped`、`overrun` 和 `carrier`。参考测试期间没有发现网卡错误、丢包、设备断连或驱动超时。

参考报告记录设备静止时 IMU Z 轴线加速度约为 `0.98`。由于 `sensor_msgs/msg/Imu` 的线加速度标准单位是 `m/s²`，该数量级不能单独证明 IMU 数值正确；应进一步确认驱动单位、坐标轴、是否包含重力，并与约 `9.81 m/s²` 的重力模长进行对照。

## 3.1.5 接入后的检查清单

对于二维雷达：

```bash
ros2 topic info /scan --verbose
ros2 topic hz /scan
ros2 topic bw /scan
ros2 topic echo /scan --once --field header
```

对于三维雷达：

```bash
ros2 topic info /points_raw --verbose
ros2 topic hz /points_raw
ros2 topic bw /points_raw
ros2 topic echo /points_raw --once --field header
ros2 topic echo /points_raw --once --field fields
```

需要确认：

- 频率接近设备在当前模式下的稳定实测值；
- `header.stamp` 非零并随帧递增；
- `header.frame_id` 是明确的传感器坐标系；
- `LaserScan` 的量程、角度和周期合理；
- `PointCloud2` 至少包含 `x/y/z`，所需的其他字段确实存在；
- 发布端与订阅端 QoS 兼容。高频传感器通常使用 Best Effort 的 Sensor Data QoS。


## 3.1.6 数据预处理流水线

### 二维 LaserScan

```text
/scan → 删除 NaN/Inf → 按 range_min/range_max 过滤
      → 应用量程裁剪 → 可选角度/机身区域屏蔽 → /scan_filtered
```

过滤时应保留原始 `header.stamp`、`frame_id`、角度和时间字段。无效距离通常写为正无穷，而不是写成 0，避免下游算法误认为障碍物紧贴雷达。

核心逻辑示例：

```python
import math

max_keep_range = min(8.0, msg.range_max)
filtered_ranges = [
    value
    if math.isfinite(value) and msg.range_min <= value <= max_keep_range
    else math.inf
    for value in msg.ranges
]
```

`8.0 m` 只是室内实验起始值，应根据环境和导航代价地图范围调整。机身屏蔽区必须根据实际安装位置测量，不能复制其他机器人的角度。

### 三维 PointCloud2

```text
原始点云 → 删除 NaN/Inf → 距离与机身区域裁剪
         → 体素降采样 → 离群点剔除 → 坐标变换 → 发布
```

范围裁剪只保留算法需要的区域，并排除机器人本体包围盒。体素降采样将空间划分为边长为 $l$ 的网格，每个体素只保留一个代表点：

- 室内精细场景可从 `0.03–0.05 m` 开始；
- 室外或长距离场景可从 `0.10–0.20 m` 开始；
- 体素尺寸不能大于希望保留的最小障碍物尺寸。

统计离群点滤波依据邻域平均距离删除异常点；半径离群点滤波要求给定半径内至少存在若干邻点。两者计算量较大，不应为了让画面更干净而过度滤波。

PCL 核心流程示例：

```cpp
pcl::CropBox<pcl::PointXYZI> crop;
crop.setInputCloud(input);
crop.setMin(Eigen::Vector4f(-10.0f, -10.0f, -0.3f, 1.0f));
crop.setMax(Eigen::Vector4f( 10.0f,  10.0f,  2.0f, 1.0f));
crop.filter(*cropped);

pcl::VoxelGrid<pcl::PointXYZI> voxel;
voxel.setInputCloud(cropped);
voxel.setLeafSize(0.05f, 0.05f, 0.05f);
voxel.filter(*downsampled);

pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
sor.setInputCloud(downsampled);
sor.setMeanK(20);
sor.setStddevMulThresh(1.0);
sor.filter(*output);
```

转换前后必须保留输入时间戳和 `frame_id`，否则后续同步与 TF 查询会失败。

### LaserScan 转换为点云

`laser_geometry` 可以把二维 `LaserScan` 转为 `PointCloud2`，便于统一可视化或接入点云算法：

```bash
sudo apt install -y ros-humble-laser-geometry
```

转换需要正确的 TF。机器人移动时，一圈扫描不是在同一时刻完成，还应利用 `time_increment`、IMU 或里程计补偿运动畸变，具体见 3.3。

## 3.1.7 记录与复现

二维 A1 示例：

```bash
mkdir -p ~/bags/m3_rplidar_a1
ros2 bag record -o ~/bags/m3_rplidar_a1/raw \
  /scan /scan_filtered /tf /tf_static
```

三维雷达示例：

```bash
mkdir -p ~/bags/m3_lidar
ros2 bag record -o ~/bags/m3_lidar/raw \
  /points_raw /points_filtered /tf /tf_static
```

MID-360 自定义点云与 IMU 示例：

```bash
mkdir -p ~/bags/m3_mid360
ros2 bag record -o ~/bags/m3_mid360/raw \
  /livox/lidar /livox/imu /tf /tf_static
```

回放时使用：

```bash
ros2 bag play <bag_directory>/raw/raw_0.db3 --clock
```

并将处理节点和 RViz2 的 `use_sim_time` 设为 `true`。混用系统时间和 bag 时间时，TF 常出现过去或未来外推错误。


## 常见问题

| 现象 | 可能原因 | 排查方法 |
| --- | --- | --- |
| USB 雷达未出现串口 | 数据线、端口、Hub、供电或 udev 问题 | 检查 `lsusb`、`dmesg`，更换线缆和接口 |
| 串口 operation timeout | 重复节点争用或雷达主控无响应 | 检查 `pgrep`、`fuser` 和雷达通信线 |
| 以太网雷达能 ping 但无数据 | 目标 IP/端口、防火墙或组播网卡错误 | 检查 launch 参数、路由和驱动日志 |
| RViz2 没有数据显示 | QoS、话题或 Fixed Frame 不匹配 | 检查 `topic info --verbose` 和 TF |
| 运动时点云弯曲 | 扫描期间发生位移且未去畸变 | 在 3.3 校时，并用 IMU/里程计补偿 |
| CPU 占用过高 | 点数过多或滤波顺序不合理 | 先裁剪、降采样，再做邻域滤波 |
| 近距离出现异常点 | 小于设备可靠最小量程 | 按 `range_min` 过滤 |

## 本课小结

本课建立了适用于二维和三维激光雷达的通用接入、检查、预处理与回放流程，并分别使用 RPLIDAR A1 和 Livox MID-360 演示了 USB 二维雷达与以太网三维雷达的实际效果。下一课将接入 IMU、GNSS/GPS 与 CAN-FD，为传感器融合准备姿态、全局位置和轮速信息。

## 参考资料

- [ROS 2 Humble：sensor_msgs](https://docs.ros.org/en/humble/p/sensor_msgs/)
- [ROS 2 Humble：LaserScan](https://docs.ros.org/en/humble/p/sensor_msgs/msg/LaserScan.html)
- [ROS 2 Humble：laser_geometry](https://docs.ros.org/en/humble/p/laser_geometry/)
- [SLAMTEC rplidar_ros ROS 2 分支](https://github.com/Slamtec/rplidar_ros/tree/ros2)
- [RPLIDAR A1 使用手册](https://bucket-download.slamtec.com/af084741a46129dfcf2b516110be558561d55767/LM108_SLAMTEC_rplidarkit_usermanual_A1M8_v2.2_en.pdf)
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [Livox ROS Driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- [Point Cloud Library：Filters](https://pointclouds.org/documentation/group__filters.html)
