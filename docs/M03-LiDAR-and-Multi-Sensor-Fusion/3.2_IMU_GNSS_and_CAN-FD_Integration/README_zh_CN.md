# 3.2 IMU、GPS 与 CAN-FD 传感器接入

只有雷达时，机器人可以观察几何环境，却很难稳定判断自身姿态、短时运动和全局位置。本课接入三类互补信息：IMU 提供高频角速度与加速度，GNSS/GPS 提供低频绝对位置，CAN-FD 则把底盘轮速、转向和设备状态送入计算平台。

## 学习目标

- 理解 IMU、GNSS 与轮速信息的优势和局限；
- 验证 `Imu`、`NavSatFix` 与 `Odometry` 消息；
- 在 Linux SocketCAN 中启用并测试 CAN-FD；
- 从 CAN 信号生成带协方差的轮速里程计；
- 为下一课的时间同步和坐标对齐准备规范数据。

## 3.2.1 三类传感器如何互补

| 来源 | 典型频率 | 优势 | 主要误差 |
| --- | ---: | --- | --- |
| IMU | 100–1000 Hz | 响应快，可感知转动与短时加速度 | 零偏、温漂，积分会漂移 |
| GNSS/GPS | 1–20 Hz | 提供全局绝对位置，长期不累计漂移 | 遮挡、多路径、跳点、延迟 |
| 轮速/底盘 CAN | 20–200 Hz | 平面速度稳定，与控制闭环直接相关 | 打滑、轮径误差、机械间隙 |

![IMU、GNSS 与 CAN-FD 数据接入移动机器人计算平台](./images/imu_gnss_canfd_integration.png)

> IMU 提供快速姿态变化，GNSS 提供全局位置，CAN-FD 提供底盘运动状态；三类数据统一进入车载计算平台。

融合不是简单求平均，而是结合测量的不确定性，让各传感器在擅长的时间尺度上发挥作用。

## 3.2.2 IMU 接入与验证

### 坐标约定

推荐车体采用右手坐标系：`x` 向前、`y` 向左、`z` 向上。IMU 厂商常使用 NED（北、东、地）或其他轴向，驱动必须先转换为 ROS 常用的 ENU/车体约定。

标准 `sensor_msgs/msg/Imu` 包含：

- `orientation`：四元数姿态；
- `angular_velocity`：角速度，单位 `rad/s`；
- `linear_acceleration`：线加速度，单位 `m/s²`；
- 三组 `3×3` 协方差矩阵。

如果设备不输出某项估计，应按消息规范标记为不可用，不能用全零协方差伪装成“完全准确”。

启动厂商驱动后检查：

```bash
ros2 topic info /imu/data --verbose
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

静止测试应包括：

1. 静置 60 秒，角速度均值应接近零；
2. 设备水平时，加速度模长应接近重力加速度，但具体轴向和是否已去重力由驱动定义；
3. 绕 `z` 轴逆时针旋转，按右手定则 `angular_velocity.z` 应为正；
4. 查看姿态四元数范数是否接近 1；
5. 记录冷启动后零偏随温度的变化。

> 磁力计容易受电机、电源线和钢结构影响。未完成磁标定与安装环境验证前，不要盲目信任绝对航向。

### 真实案例：Livox MID-360 内置 IMU

下面使用 Livox MID-360 的内置 IMU 演示如何从“话题存在”进一步验证频率、时间戳、静态零偏、单位、姿态和协方差。

#### 1. 测试环境

| 项目 | 实测配置 |
| --- | --- |
| 计算平台 | reComputer Robotics J5011 |
| 操作系统 | Ubuntu 22.04.5 LTS，aarch64 |
| ROS 2 | Humble |
| 传感器 | Livox MID-360 内置 IMU |
| ROS 驱动 | Livox ROS Driver2 1.2.7 |
| Jetson 有线 IP | `192.168.1.5` |
| MID-360 IP | `192.168.1.3` |
| IMU 话题 | `/livox/imu` |
| 消息类型 | `sensor_msgs/msg/Imu` |
| 坐标系 | `livox_frame` |

Livox-SDK2、驱动编译和 MID-360 网络配置见 3.1.4。本节从已能连接雷达的状态开始：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

![启动 Livox Driver](./images/launch_mid360.jpg)

#### 2. 检查话题、发布者与 QoS

在第二个终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
ros2 topic list -t
ros2 topic info /livox/imu --verbose
```

参考结果：

![IMU Topic](./images/imu_topic.jpg)

这一步确认了消息类型、唯一发布者和 QoS，但仍不能证明数值单位与姿态字段正确。

#### 3. 查看一条真实消息

```bash
ros2 topic echo /livox/imu --once
```

静止状态下的一条参考消息为：

![IMU Data](./images/imu_data.jpg)

从单条消息只能看到格式，不能判断稳定性。至少应连续采集数秒并计算均值、标准差、时间间隔和延迟。

#### 4. 发布频率、时间戳与延迟

```bash
ros2 topic hz /livox/imu
```

命令行实测：

```text
average rate: 199.99 Hz
min: 0.004 s
max: 0.006 s
```

连续采集 2000 条消息的统计结果：

| 指标 | 实测结果 |
| --- | ---: |
| 消息数 | 2000 |
| 采集时间 | 9.998 s |
| 按接收时间计算的频率 | 200.030 Hz |
| 按消息时间戳计算的频率 | 199.984 Hz |
| 平均时间戳间隔 | 4.999808 ms |
| 时间戳间隔标准差 | 0.746762 ms |
| 最小 / 最大间隔 | 3.803991 / 6.085605 ms |
| 非递增时间戳 | 0 |
| 大于 7.5 ms 的间隔 | 0 |
| 平均消息延迟 | 0.700 ms |
| 最小 / 最大延迟 | 0.360 / 1.387 ms |

本次数据表明 `/livox/imu` 稳定在约 200 Hz，时间戳严格递增，未观察到明显异常间隔。正式系统仍应持续监测，而不能把一次测试结果当成所有网络和负载条件下的保证。

## 3.2.3 GNSS/GPS 接入与验证

GNSS 是卫星导航系统总称，GPS 是其中一种。接收机通常通过 UART、USB 或以太网输出 NMEA、UBX 或厂商二进制协议。驱动应转换为 `sensor_msgs/msg/NavSatFix`。

reComputer Robotics J5011 中有预留 M.2 Key B 接口，该接口可以接入 4G/5G 模组使用 GPS 定位功能，具体细节请参考：
https://wiki.seeedstudio.com/ai_robotics_recomputer_j501_robotics_getting_started/#m2-key-b-4g5g-module

建议在开阔场地静置 5–10 分钟，记录位置散点、状态、卫星数和协方差。若使用 RTK，还需分别记录 Fixed、Float 和单点定位状态，不要把一次 Fixed 当成全程固定解。

## 3.2.4 CAN-FD 接入

CAN-FD 相比经典 CAN，可在数据阶段使用更高比特率，单帧有效载荷最多 64 字节。总线两端应有匹配的终端电阻；断电测量 CAN-H 与 CAN-L 之间通常约为 60 Ω，具体以硬件设计为准。

安装工具并查看接口：

```bash
sudo apt update
sudo apt install -y can-utils
ip -details link show can0
```

以下示例使用 500 kbit/s 仲裁速率、2 Mbit/s 数据速率。参数必须与总线上所有节点一致：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
```

监听总线：

```bash
candump -tz can0
```

发送测试帧前必须确认设备允许该 ID，真实机器人处于架空轮、急停可用或其他安全状态。CAN-FD 帧示例：

```bash
cansend can0 123##1DEADBEEF
```

其中 `##` 表示 CAN-FD，后面的首个十六进制字符包含 FD 标志。不要在未知协议的量产底盘上随意发送帧。

### 使用虚拟 CAN 离线测试

没有硬件时可用 `vcan` 熟悉工具：

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up

# 终端 A
candump vcan0
# 终端 B
cansend vcan0 123#11223344
```

`vcan` 只验证软件收发逻辑，不验证比特率、终端电阻、收发器和物理层。

### 真实案例：reComputer Robotics J5011 双电机底盘调试

下面使用一台 reComputer Robotics J5011 和双 DM-H65 电机底盘进行实机验证。J5011 的 `CAN0` 接入同一条电机总线，左轮电机 ID 为 `0x01`，右轮电机 ID 为 `0x02`。

> **先区分 CAN 与 CAN-FD：** 本案例中的 DM-H65 电机实际使用 **经典 CAN，1 Mbit/s**。`ip -details link show can0` 显示接口 MTU 为 16，配置中没有 `fd on` 和 `dbitrate`。因此不要直接套用前面的 CAN-FD 配置，否则电机无法正常通信。

#### 1. 上电前检查

1. 将底盘架空并固定，确保两个车轮自由转动时不会接触人员或物体；准备好急停或底盘断电开关。
2. 确认 J5011、驱动器和电机电源地可靠共地，CAN-H 与 CAN-L 没有接反。
3. 断电后检查总线两端的终端电阻。
4. 本节的验证只读取状态，不发送运动指令。首次运动测试必须另行确认现场安全。

#### 2. 检查系统服务和 CAN0

本项目已经提供 CAN 初始化、J5011 终端电阻和底盘 WebUI 服务。先查看它们的状态：

```bash
systemctl status dm-h65-can.service \
  dm-h65-can-termination.service \
  dm-h65-webui.service --no-pager

ip -details -statistics link show can0
```

如果尚未配置服务，可以手动以经典 CAN 1 Mbit/s 启动接口：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 berr-reporting on restart-ms 100
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
```

若 `dm-h65-can.service` 已经处于 active 状态，无需重复执行手动配置。实测接口关键信息如下：

| 检查项 | 实测结果 | 判断 |
| --- | --- | --- |
| 链路状态 | `UP, LOWER_UP` | 接口已启用 |
| 帧格式 | MTU 16，经典 CAN | 与 DM-H65 匹配 |
| 仲裁速率 | `1000000 bit/s` | 与两个电机一致 |
| 控制器状态 | `ERROR-ACTIVE` | 当前可以正常参与总线通信 |
| 当前错误计数器 | `tx 0, rx 0` | 当前未进入错误告警状态 |
| 自动恢复 | `restart-ms 100` | bus-off 后 100 ms 自动恢复 |

#### 3. 只读抓包确认两个电机响应

底盘 WebUI 正在周期读取电机状态，因此可以用 `candump` 旁路观察，不需要主动发送测试帧：

```bash
timeout 4s candump -L can0
```

实测可连续看到类似以下请求和响应：

```text
can0  7FF   [4]  01 00 33 3C
can0  000   [8]  01 00 33 3C ...
can0  7FF   [4]  02 00 33 3C
can0  000   [8]  02 00 33 3C ...
```

这里 CAN 仲裁 ID `0x7FF` 是读取请求，`0x000` 是响应；负载中的第一个字节 `01` 或 `02` 才是电机 ID。驱动会轮询以下寄存器：

| 寄存器 | 含义 |
| --- | --- |
| `0x3C` | 母线电压 |
| `0x3D` | 驱动板温度 |
| `0x3E` | 电机温度 |
| `0x50` | 电机位置 |

两个电机 ID 的请求和响应都持续出现，说明 CAN0、线束以及两个节点已经建立双向通信。不要根据仲裁 ID `0x000` 误判电机 ID，也不要在不了解协议时用 `cansend` 猜测控制帧。

#### 4. 读取解码后的实时状态

浏览器访问 `http://192.168.3.201:8765` 可以打开底盘调试台。也可以在 J5011 本机只读查询 API：

```bash
curl -s http://127.0.0.1:8765/api/state | python3 -m json.tool
```

2026-09-11 的实机静止测试结果如下：

| 项目 | 左轮 `0x01` | 右轮 `0x02` |
| --- | ---: | ---: |
| 在线状态 | 在线 | 在线 |
| 转速 | 0.0 rad/s | 0.0 rad/s |
| 母线电压 | 50.23 V | 50.45 V |
| 电机温度 | 24.0 °C | 24.0 °C |
| 驱动板温度 | 49.79 °C | 50.86 °C |

接口处于实机模式，两个电机都在线且静止，电压和温度数据能够稳定解码，完成了不驱动车轮的通信闭环验证。

驱动、测试脚本和更完整的安全说明见 [`hardware/robot_drivers`](../../../hardware/robot_drivers/README.md)。

## 3.2.5 从 CAN 轮速到里程计

以差速底盘为例，左右轮线速度为 $v_l$、$v_r$，轮间距为 $b$：

$$
v = \frac{v_r+v_l}{2}, \qquad \omega = \frac{v_r-v_l}{b}
$$

离散积分可得到平面位姿：

$$
\theta_{k+1}=\theta_k+\omega\Delta t
$$

$$
x_{k+1}=x_k+v\cos\theta\Delta t, \qquad
y_{k+1}=y_k+v\sin\theta\Delta t
$$

CAN 解码节点应：

1. 根据协议校验帧 ID、长度、端序、符号位和缩放比例；
2. 使用接收时间或控制器提供的硬件时间戳；
3. 检测计数器停滞、超时和异常跳变；
4. 发布 `nav_msgs/msg/Odometry`，`frame_id=odom`、`child_frame_id=base_link`；
5. 填写合理的速度与位姿协方差；
6. 只由一个节点发布 `odom → base_link`，避免 TF 冲突。

若底盘 SDK 已经提供左右轮状态，优先复用协议解析与安全机制，只在 ROS 2 适配层统一消息格式。

## 常见问题

| 现象 | 可能原因 | 处理方式 |
| --- | --- | --- |
| IMU 静止仍持续旋转 | 零偏、温漂、单位或轴向错误 | 预热、标定并核对 `rad/s` 与坐标约定 |
| GNSS 有经纬度但轨迹乱跳 | 多路径、无有效解、协方差失真 | 到开阔区域并检查状态与协方差 |
| CAN 接口进入 BUS-OFF | 比特率、终端电阻、接线或地参考错误 | 停止发送，检查物理层和错误计数 |
| CAN 数值数量级错误 | 端序、符号位或缩放系数错误 | 用协议文档和已知工况逐字段验证 |
| 里程计转向相反 | 左右轮映射或正方向定义错误 | 架空轮低速测试并统一符号 |

## 本课小结

本课建立了 IMU、GNSS 与底盘 CAN-FD 三条数据链路，并把检查重点从“有数据”提升到“单位、坐标、时间戳和不确定性都可信”。下一课将统一这些传感器的时钟和坐标系。

## 参考资料

- [ROS 2 Humble：Imu 消息](https://docs.ros.org/en/humble/p/sensor_msgs/msg/Imu.html)
- [ROS 2 Humble：NavSatFix 消息](https://docs.ros.org/en/humble/p/sensor_msgs/msg/NavSatFix.html)
- [Linux Kernel：SocketCAN](https://docs.kernel.org/networking/can.html)
- [ROS REP-103：坐标与单位约定](https://www.ros.org/reps/rep-0103.html)
- [Livox MID-360 以太网通信协议](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md)
- [Livox ROS Driver2](https://github.com/Livox-SDK/livox_ros_driver2)
