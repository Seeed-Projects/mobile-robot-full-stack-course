# M03 感知层：雷达与多传感器融合 / Perception: LiDAR and Multi-Sensor Fusion

本模块面向 reComputer Robotics J501、Ubuntu 22.04 与 ROS 2 Humble，用 4 个课时完成从传感器数据接入到机器人状态估计的基础闭环。

## English

1. [3.1 LiDAR Integration and Point Cloud Preprocessing](./3.1_LiDAR_Integration_and_Point_Cloud_Preprocessing/README_en_US.md)
2. [3.2 Integrating IMU, GPS, and CAN-FD Sensors](./3.2_IMU_GNSS_and_CAN-FD_Integration/README_en_US.md)
3. [3.3 Multi-Sensor Time Synchronization and Spatial Alignment](./3.3_Multi-Sensor_Time_Synchronization_and_Spatial_Alignment/README_en_US.md)
4. [3.4 Sensor Fusion: Introduction to EKF and State Estimation](./3.4_EKF_and_State_Estimation/README_en_US.md)

## 中文

1. [3.1 激光雷达接入与点云预处理](./3.1_LiDAR_Integration_and_Point_Cloud_Preprocessing/README_zh_CN.md)
2. [3.2 IMU、GPS 与 CAN-FD 传感器接入](./3.2_IMU_GNSS_and_CAN-FD_Integration/README_zh_CN.md)
3. [3.3 多传感器时间同步与时空对齐](./3.3_Multi-Sensor_Time_Synchronization_and_Spatial_Alignment/README_zh_CN.md)
4. [3.4 传感器融合：EKF 与状态估计入门](./3.4_EKF_and_State_Estimation/README_zh_CN.md)

## 模块学习成果

完成本模块后，学习者应能够：

- 在 ROS 2 中识别并验证 `LaserScan`、`PointCloud2`、`Imu`、`NavSatFix` 与 `Odometry` 数据；
- 完成点云裁剪、降采样、离群点剔除和坐标变换；
- 接入 IMU、GNSS/GPS 与 CAN-FD，并判断数据质量是否满足融合要求；
- 建立统一的时间基准与 TF 坐标树，识别时间戳和外参错误；
- 使用 `robot_localization` 配置二维移动机器人的 EKF，融合轮速里程计与 IMU；
- 理解 GNSS 参与全局定位时 `map`、`odom`、`base_link` 三个坐标系的职责。

> 本模块先建立稳定、可诊断的传感器数据链路。激光/视觉 SLAM 和建图算法将在 M5 中展开。
