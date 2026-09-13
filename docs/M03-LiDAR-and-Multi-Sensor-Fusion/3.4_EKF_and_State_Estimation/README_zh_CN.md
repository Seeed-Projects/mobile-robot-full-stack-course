# 3.4 传感器融合：EKF 与状态估计入门

移动机器人真正需要的不是若干互相矛盾的传感器读数，而是一条连续、带不确定性且可供控制与导航使用的状态轨迹。本课从扩展卡尔曼滤波（EKF）出发，使用 ROS 2 `robot_localization` 融合轮速里程计和 IMU，并介绍 GNSS 全局定位的标准结构。

## 学习目标

- 理解状态、预测、观测、协方差和卡尔曼增益；
- 配置二维移动机器人的 EKF；
- 正确选择被融合的消息字段，避免重复使用同源信息；
- 识别协方差、时间戳和坐标系造成的融合异常；
- 设计本地连续估计与 GNSS 全局估计的双层结构。

## 3.4.1 状态估计在解决什么问题

以二维机器人为例，可选择状态：

$$
\mathbf{x}=[x,y,\psi,v_x,v_y,\dot{\psi},a_x,a_y]^T
$$

其中 $x,y$ 是位置，$\psi$ 是偏航角，$v$ 是速度，$\dot{\psi}$ 是偏航角速度，$a$ 是加速度。

EKF 的循环包含两步：

1. **预测**：根据上一状态与运动模型推算当前状态，并增加不确定性；
2. **更新**：把传感器观测与预测比较，根据各自协方差修正状态。

简化形式为：

$$
\hat{x}_{k|k-1}=f(\hat{x}_{k-1|k-1},u_k)
$$

$$
K_k=P_{k|k-1}H_k^T(H_kP_{k|k-1}H_k^T+R_k)^{-1}
$$

$$
\hat{x}_{k|k}=\hat{x}_{k|k-1}+K_k(z_k-h(\hat{x}_{k|k-1}))
$$

$P$ 表示状态估计协方差，$R$ 表示测量协方差，$K$ 是卡尔曼增益。测量协方差越小，滤波器越信任该测量；因此错误地填入零或极小协方差，会让一个有噪声的传感器支配整个结果。

## 3.4.2 本课融合架构

先完成不依赖 GNSS 的本地融合：

```text
/wheel/odometry ─┐
                  ├─ EKF local ─→ /odometry/filtered ─→ odom → base_link
/imu/data ────────┘
```

![轮速、IMU 与 GNSS 通过 EKF 形成连续状态轨迹](./images/ekf_sensor_fusion.png)

> 图 3.4：轮速、IMU 和 GNSS 的测量精度与漂移特性不同；滤波器综合预测与观测，输出带有不确定性范围的连续轨迹。

选择原则：

- 轮速提供机器人前向速度 `vx`；
- IMU 提供偏航角速度 `vyaw`；
- 对差速底盘启用 `two_d_mode`，约束 `z/roll/pitch`；
- 初学阶段不融合未经验证的磁力计绝对航向；
- 不把由同一轮编码器积分得到的位置、速度和航向全部重复当成独立测量。

## 3.4.3 安装与输入检查

```bash
sudo apt update
sudo apt install -y ros-humble-robot-localization

ros2 topic info /wheel/odometry --verbose
ros2 topic info /imu/data --verbose
ros2 topic hz /wheel/odometry
ros2 topic hz /imu/data
```

启动 EKF 前必须完成：

- 两条消息时间戳有效且处于同一时间域；
- `base_link → imu_link` 静态变换存在；
- 轮速和 IMU 都遵守 `x` 前、`y` 左、`z` 上；
- 速度、角速度单位分别为 `m/s`、`rad/s`；
- 协方差非零且与静态数据方差大致相符；
- 没有其他节点同时发布 `odom → base_link`。

## 3.4.4 最小可用 EKF 配置

创建 `config/ekf_local.yaml`：

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.2
    two_d_mode: true
    publish_tf: true
    print_diagnostics: true

    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    odom0: /wheel/odometry
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  false, false,
                   false, false, false,
                   false, false, false]
    odom0_queue_size: 10
    odom0_differential: false
    odom0_relative: false

    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,
                  false, false, false]
    imu0_queue_size: 50
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: false

    process_noise_covariance: [
      0.05, 0.0,  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0,  0.05, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0,  0.0,  0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0,  0.0,  0.0,  0.03,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0,  0.0,  0.0,  0.0, 0.03,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0,  0.0,  0.0,  0.0, 0.0, 0.06,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0,  0.0,  0.0,  0.0, 0.0, 0.0, 0.025,0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0,  0.0,  0.0,  0.0, 0.0, 0.0, 0.0, 0.025,0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0,  0.0,  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.04,0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0,  0.0,  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01,0.0, 0.0, 0.0, 0.0, 0.0,
      0.0,  0.0,  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01,0.0, 0.0, 0.0, 0.0,
      0.0,  0.0,  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02,0.0, 0.0, 0.0,
      0.0,  0.0,  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01,0.0, 0.0,
      0.0,  0.0,  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01,0.0,
      0.0,  0.0,  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.015]
```

15 个布尔值依次对应：`x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az`。上例仅融合轮速 `vx` 与 IMU `vyaw`，先建立容易诊断的基线。

`process_noise_covariance` 只是起始示例，不是所有机器人的通用答案。正式参数应结合运动模型、控制周期和实测数据调整。

启动滤波器：

```bash
ros2 run robot_localization ekf_node --ros-args \
  --params-file ~/robot_ws/src/<your_package>/config/ekf_local.yaml
```

## 3.4.5 验证输出

```bash
ros2 topic hz /odometry/filtered
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo /diagnostics
```

按以下顺序测试：

1. **静止**：位置不应快速游走，速度接近零；
2. **直线**：`x` 增长方向正确，横向漂移有限；
3. **原地旋转**：位置变化小，偏航角连续；
4. **矩形路线**：终点误差应优于或至少不差于原始轮速积分；
5. **短时断开单个输入**：滤波器应短时预测并给出诊断，恢复后不应剧烈跳变。

在 RViz2 同时显示原始轮速路径与融合路径。不要只看轨迹“更平滑”，还要比较终点误差、航向误差、延迟和协方差是否合理增长。

## 3.4.6 如何调整协方差

推荐从实测开始：

1. 机器人静止记录 5–10 分钟 IMU；
2. 计算各轴均值、方差和随时间漂移；
3. 多次重复直线和原地旋转，估计轮速误差；
4. 将测量方差写入消息协方差，而不是只在 EKF 中“调感觉”；
5. 检查创新是否长期偏向同一方向，若是，应先排查系统性误差。

常见误区：

- 把协方差设为零，导致错误测量被无限信任；
- 为了压制噪声把协方差设得极大，等于没有融合该数据；
- 同时融合同一编码器推导出的位姿、速度和航向，却把它们当作相互独立；
- 试图用 EKF 修复错误单位、轴向、时间戳或外参。

## 3.4.7 加入 GNSS 的全局估计

GNSS 经纬度不能直接作为局部笛卡尔坐标输入本地 EKF。`navsat_transform_node` 通常接收：

- `/gps/fix`：`NavSatFix`；
- `/imu/data`：具有地球参考航向的 IMU；
- `/odometry/filtered`：本地 EKF 输出。

它把 GNSS 转换为与机器人世界坐标一致的里程计，再送入全局滤波器。推荐结构：

```text
轮速 + IMU ─→ EKF local (world=odom) ─→ odom → base_link
      │                    │
GNSS ─┴─→ navsat_transform ─→ /odometry/gps
                              │
轮速 + IMU + odometry/gps ─→ EKF global (world=map) ─→ map → odom
```

这样 `odom → base_link` 保持局部连续，GNSS 修正体现在 `map → odom`，避免控制器因 GPS 跳点而突然改变局部坐标。

使用前必须确认：

- IMU 航向参考是 ENU 还是磁北/真北，并正确设置磁偏角与 `yaw_offset`；
- GNSS 天线外参 `base_link → gps_link` 正确；
- `/odometry/gps` 作为绝对测量输入时不要设置为 differential；
- 室内或定位无效时，不把零经纬度或旧数据继续送入全局滤波器；
- 首次测试应在开阔场地低速进行。

## 实验与验收

1. 先只启动轮速，保存基线轨迹；
2. 加入 IMU 角速度，重复同一路线；
3. 比较静止漂移、转向响应和矩形闭合误差；
4. 人为停止 IMU 数据，观察超时诊断和恢复行为；
5. 有可靠 GNSS 和绝对航向时，再搭建全局 EKF；
6. 保存参数、rosbag、TF 树和结果截图。

验收要求：`/odometry/filtered` 频率稳定；TF 只有一个 `odom → base_link` 发布者；直线和转向方向正确；融合结果没有持续振荡或无故跳变；输入失效时有诊断；参数中的每个启用字段都能说明数据来源和原因。

## 常见问题

| 现象 | 可能原因 | 处理方式 |
| --- | --- | --- |
| 输出剧烈抖动 | 协方差过小、时间不同步或两源互相冲突 | 检查时间与坐标，再重新估计协方差 |
| 静止时航向持续漂移 | 陀螺零偏或没有绝对航向约束 | 标定 IMU；需要时加入经验证的绝对航向 |
| 原地转动时位置画圆 | IMU 外参平移/旋转错误或轮速模型错误 | 核对 TF、轮距和左右轮符号 |
| TF 出现重复或闪烁 | 原始里程计和 EKF 同时发布同一变换 | 关闭其中一个 TF 发布者 |
| 加 GPS 后轨迹跳跃 | 多路径、航向偏置、天线外参或协方差错误 | 先检查 GNSS 状态，再检查转换参数 |
| 日志提示 queue/old measurement | 时间戳倒退、延迟过大或队列过小 | 修复时间源并检查链路延迟 |

## 本课小结

本课完成了 M3 的最小状态估计闭环：轮速描述底盘平移，IMU 描述快速转动，EKF 根据不确定性生成连续的 `odom → base_link`。在此基础上，可通过 `navsat_transform_node` 与全局 EKF 引入 GNSS，形成兼顾局部连续性和全局准确性的定位结构。

## 参考资料

- [robot_localization 文档](https://docs.ros.org/en/rolling/p/robot_localization/)
- [navsat_transform_node 参数说明](https://docs.ros.org/en/lunar/api/robot_localization/html/navsat_transform_node.html)
- [ROS REP-105：移动平台坐标系](https://www.ros.org/reps/rep-0105.html)
- [ROS 2 Humble：Odometry 消息](https://docs.ros.org/en/humble/p/nav_msgs/msg/Odometry.html)
