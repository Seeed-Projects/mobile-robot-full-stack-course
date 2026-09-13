# 3.3 多传感器时间同步与时空对齐

传感器融合的前提不是“话题都在发布”，而是每条测量都能回答两个问题：它在什么时刻产生？它在机器人什么位置、沿什么方向测量？前者是时间同步，后者是空间对齐。任一项错误，都可能让静态环境在机器人运动时看起来发生位移。

![多传感器时间脉冲同步与坐标系空间对齐示意图](./images/time_and_spatial_alignment.png)

> 图 3.3：左侧表示不同传感器的数据时间戳对齐到统一时钟，右侧表示各传感器坐标系通过外参变换统一到车体坐标系。

## 学习目标

- 区分采样时间、设备时间、主机接收时间和 ROS 时间；
- 使用 chrony 或 PTP 建立主机级统一时钟；
- 正确设计 `map → odom → base_link → sensor_link` TF 树；
- 理解内参、外参和坐标系约定；
- 用可复现方法验证时间偏差与外参误差。

## 3.3.1 时间为什么重要

假设机器人以 $1\,m/s$ 行驶，雷达与轮速相差 $100\,ms$，位置对应关系就可能相差 $0.1\,m$。机器人转动时，几毫秒误差也会造成点云边缘重影。

一条传感器消息可能涉及：

- **采样时间**：物理量真正被测量的时刻；
- **设备时间**：传感器内部时钟记录的时刻；
- **接收时间**：数据到达 J501 驱动的时刻；
- **ROS 时间**：消息 `header.stamp` 所使用的时间域；
- **处理时间**：算法完成并发布结果的时刻。

驱动应尽量把 `header.stamp` 设为采样时间，而非回调开始或消息发布的时间。网络传输和批量组包延迟会让“接收时间”产生抖动。

## 3.3.2 选择同步方式

| 方法 | 适用条件 | 特点 |
| --- | --- | --- |
| 硬件触发/同步线 | 设备支持 Trigger、PPS、Sync | 精度最高，可对齐真实采样时刻 |
| PTP（IEEE 1588） | 设备与网卡支持硬件时间戳 | 适合以太网雷达、相机和多计算机 |
| GNSS PPS + 时间报文 | 有 GNSS 接收机 | PPS 提供秒边沿，报文提供绝对时间 |
| NTP/chrony | 通用网络设备 | 易部署，适合主机间校时，精度受网络影响 |
| 软件近似同步 | 已有带时间戳消息 | 只能配对邻近消息，不能修复错误时间戳 |

优先级通常是：同一硬件时钟或硬触发 > PTP/PPS > chrony > 仅靠软件配对。

## 3.3.3 主机时钟同步

### chrony

适合 J501 与开发主机位于同一局域网、但设备不支持硬件 PTP 的场景：

```bash
sudo apt install -y chrony
chronyc tracking
chronyc sources -v
```

查看 `System time`、`Last offset` 和源状态。只有主机时钟一致还不够，驱动仍需正确转换传感器设备时间。

### PTP

先确认网卡时间戳能力：

```bash
ethtool -T eth1
ls -l /dev/ptp*
```

典型从时钟端调试命令如下，实际接口、配置和主从角色以网络设计为准：

```bash
sudo apt install -y linuxptp
sudo ptp4l -i eth1 -m -s
sudo phc2sys -s eth1 -c CLOCK_REALTIME -m
```

生产部署应使用 systemd 服务和明确配置文件，避免同时让 chrony 与 `phc2sys` 互相拉扯系统时钟。记录 PTP offset 的稳定范围，并在启动融合前等待时钟锁定。

## 3.3.4 ROS 2 时间戳诊断

```bash
ros2 topic echo /points_raw --once --field header
ros2 topic echo /imu/data --once --field header
ros2 topic echo /wheel/odometry --once --field header
```

检查原则：

- 时间戳必须非零且单调递增；
- 设备重启或时间源切换不能悄悄造成时间倒退；
- 同一 rosbag 中所有消息必须使用同一时间域；
- 回放 bag 时统一使用 `/clock` 和 `use_sim_time=true`；
- 延迟、抖动和丢包应分开统计，平均延迟小不代表抖动小。

近似同步只配对时间差在阈值内的消息。阈值过小会频繁丢配对，过大则会把不同时刻的测量强行组合。应先测量时间差分布，再设置队列长度和容差。

```python
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Imu, PointCloud2

cloud_sub = Subscriber(node, PointCloud2, '/points_raw')
imu_sub = Subscriber(node, Imu, '/imu/data')
sync = ApproximateTimeSynchronizer(
    [cloud_sub, imu_sub], queue_size=30, slop=0.02)
sync.registerCallback(synchronized_callback)
```

`slop=0.02` 只表示允许最多约 20 ms 的消息时间差，它不是校时方法。雷达运动去畸变通常还需要每个点相对整帧的时间偏移。

## 3.3.5 构建统一 TF 树

推荐的基础结构：

```text
map                 全局连续性不保证，可被定位系统校正
└── odom            局部连续、不跳变，但会长期漂移
    └── base_link   机器人本体参考坐标系
        ├── lidar_link
        ├── imu_link
        └── gps_link
```

- `map → odom` 通常由全局定位或 SLAM 发布；
- `odom → base_link` 通常由里程计或本地融合器发布；
- `base_link → sensor_link` 是测量或标定得到的静态外参；
- 同一对父子坐标系只能有一个发布者。

按 REP-103，车体坐标通常为 `x` 前、`y` 左、`z` 上。传感器原始坐标若不同，应由驱动或固定变换显式转换，不能只改 `frame_id` 名称。

## 3.3.6 测量和发布外参

外参由平移 $t$ 和旋转 $R$ 构成：

$$
p_{base}=R_{base}^{sensor}p_{sensor}+t_{base}^{sensor}
$$

先用卷尺、卡尺和角度工具测得初值，再用平面、墙角或标定目标优化。记录单位、旋转顺序、父子坐标系和标定日期。

以下示例表示传感器位于 `base_link` 前方 0.20 m、左侧 0 m、上方 0.35 m，且无旋转：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.20 --y 0.00 --z 0.35 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id lidar_link
```

使用实际测量值替换示例，不要复制示例数字到真实机器人。

检查 TF：

```bash
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_tools view_frames
ros2 topic echo /tf_static --once
```

`view_frames` 可生成坐标树报告。若 TF 查找失败，检查 frame 拼写、发布方向、时间戳、是否存在孤立子树和多个发布者。

## 3.3.7 时空联合验证

### 静态验证

1. 机器人不动，将其放在平整地面与垂直墙面附近；
2. 把点云变换到 `base_link`；
3. 地面应近似水平，墙面应垂直；
4. 绕传感器移动标定板，确认前、左、上方向与 RViz2 一致。

### 动态验证

1. 机器人以低速直线经过固定墙角；
2. 随后原地慢速旋转一圈；
3. 同时记录点云、IMU、轮速、TF 与时钟状态；
4. 如果静止时对齐而运动时出现双边或弯曲，优先怀疑时间偏差；
5. 如果静止和运动时都出现固定方向偏移，优先怀疑外参。

可用“小幅修改时间偏移/偏航外参后，误差是否系统性减小”的方式定位问题，但最终参数必须有数据证据，不能只靠肉眼调图。

## 实验与验收

1. 输出 chrony 或 PTP 的锁定状态；
2. 统计雷达、IMU 和轮速消息的频率及相邻时间差；
3. 发布三个传感器静态外参，生成完整 TF 树；
4. 完成墙面静态测试、直线运动和原地旋转测试；
5. 保存 rosbag、TF 报告和最终外参参数。

验收要求：各消息处于同一时间域、时间戳不倒退；主机同步偏差满足所选算法要求；TF 树无环、无孤立、无重复发布；点云在 `base_link` 中方向正确；运动场景中没有明显由时间差引起的重影。

## 常见问题

| 现象 | 可能原因 | 处理方式 |
| --- | --- | --- |
| TF 报 extrapolation into the future | 传感器时钟领先或混用仿真时间 | 统一时钟与 `use_sim_time` |
| 静止对齐、运动重影 | 消息时间偏移或逐点时间错误 | 检查采样时间、链路延迟和去畸变 |
| 所有点固定旋转一个角度 | 外参旋转方向或 RPY 顺序错误 | 核对父子 frame 和右手定则 |
| 偶尔配对、频繁丢数据 | 同步容差太小或频率抖动 | 统计差值后调整队列和容差 |
| TF 树跳动 | 多个节点发布同一变换 | 用节点信息找到并关闭重复发布者 |

## 本课小结

本课建立了统一时钟、规范 TF 树以及静态/动态联合验证方法。下一课将在这套可信时空基准上，用 EKF 融合轮速与 IMU，并说明 GNSS 如何进入全局状态估计。

## 参考资料

- [ROS 2 Humble：message_filters](https://docs.ros.org/en/humble/p/message_filters/)
- [ROS 2 Humble：tf2](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html)
- [ROS REP-105：移动平台坐标系](https://www.ros.org/reps/rep-0105.html)
- [linuxptp 项目文档](https://www.linuxptp.org/documentation/)
