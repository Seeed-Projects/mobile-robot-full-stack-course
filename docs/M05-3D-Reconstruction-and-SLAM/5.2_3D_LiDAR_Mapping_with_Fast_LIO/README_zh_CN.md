# 5.2 三维激光建图与 J501 上的 Fast-LIO 部署

三维激光建图是移动机器人从平面导航走向完整空间感知的关键一步。二维栅格地图可以支撑 Nav2 规划，但无法表达悬空物、斜坡及其他垂直几何，也就是只能在空间中的一个切平面上显示障碍物，这无疑不能满足在复杂三维空间中导航的需求。当这些结构开始影响任务时，机器人需要由三维点云构建的度量地图。

本课仍属于 M5 的激光路线，将从平面建图推进到紧耦合激光-惯性里程计。算法重点是 Fast-LIO / Fast-LIO2。部署平台为 `reComputer Robotics J501` 与 Livox MID-360，并复用 M3 中已验证的传感器接入流程。第 5.1 课得到的二维地图应予保留：二维继续作为默认规划接口，三维则提供更丰富的几何，服务于后续投影、重建与多传感器工作。

## 学习目标

- 说明三维激光建图为何需要运动去畸变与 IMU 先验；
- 在部署前了解 Fast-LIO2 的紧耦合估计流程，以及 MID-360 场景下外参与偏置的处理方式；
- 比较移动机器人中常见的主要三维激光建图路线；
- 从 Livox-SDK2 与 `livox_ros_driver2` 开始，完成 MID-360 驱动编译、网络配置与点云确认；
- 按已验证流程编译并启动 Fast-LIO2，完成慢速建图并保存轨迹 / PCD 产物；
- 用对后续导航与重建真正有意义的检查项评估地图质量。

## 前置条件

- 已完成第 5.1 课：明确 SLAM 任务，并得到可复用二维占用栅格；
- M3.1：MID-360 已通过专用以太网链路发布激光与 IMU 话题；
- M3.2 / M3.4：能够在实践中处理时间戳、坐标系与协方差；
- 具备 Ubuntu 22.04 + ROS 2 Humble 基本环境；本课会从 Livox-SDK2 开始完整安装驱动与 Fast-LIO2。

---

# Part A. 从平面地图到三维激光建图

## 5.2.1 三维 SLAM 的特性

二维 SLAM 常常可以把一圈激光近似视为瞬时平面观测。三维激光建图面对的是另一类观测过程。

旋转式或扫描式三维激光会在一段非零时间窗口内累积点。如果机器人在该窗口内发生平移或旋转，单帧点云本身就已经变形。若不去畸变，墙面会弯曲、地面会起皱，scan-to-map 匹配也会变得不可靠。

因此，现代三维激光建图通常包含：

1. 高频 IMU 先验；
2. 逐点或帧内时间戳；
3. 可持续更新的状态估计器，系统边运动边校正；
4. 能够随新点高效扩展的地图结构。

在 J501 与 MID-360 组合上，这些条件已经具备：

- 带逐点 `offset_time` 的点云；
- 约 200 Hz 的 IMU；
- 在合理配置下可支撑 Fast-LIO2 类负载的嵌入式算力平台。

![M5.2-1.png](./images/M5.2-1.png)

## 5.2.2 三维激光建图的算法版图

三维激光建图发展到今天，已经形成几条常见工程路线。理解这些路线，比记住具体功能包名称更重要。

较早的一条路线来自 LOAM 风格的激光里程计。它通常先从点云中提取边、面一类几何特征，再通过 scan-to-scan 或 scan-to-map 匹配估计运动。这类方法把问题拆成了两个层次：高频运动估计，以及较低频的地图精化。后来很多三维激光系统，都还保留着这一思路。

在此基础上，紧耦合激光-惯性里程计（LIO）逐渐成为移动机器人中更常见的实时方案。它把 IMU 预测与激光残差更新放进同一个估计器：IMU 提供高频运动先验，并支撑扫描内去畸变；激光再校正姿态、位置、速度与偏置。Fast-LIO / Fast-LIO2 就是这条路线的代表。

如果任务更强调长距离一致性，系统往往会再增加一层图优化。LIO-SAM 一类方法通常保留局部 LIO，同时引入位姿图与回环约束，用显式后端去修正累积漂移。相对地，面向重建的系统更关注稠密表面或 surfel 模型；而 Fast-LIVO、R3LIVE 一类多模态方案，则是在稳定 LIO 之上继续加入相机残差。

因此，可以把当前主流路线粗略看成：

| 路线               | 代表性系统            | 核心做法                       | 更适合回答的问题                           |
| ------------------ | --------------------- | ------------------------------ | ------------------------------------------ |
| 特征匹配激光里程计 | LOAM 风格流程         | 提取边/面特征并随时间匹配      | 三维激光里程计如何起步                     |
| 紧耦合 LIO         | Fast-LIO / Fast-LIO2  | IMU 预测与激光更新共享同一状态 | 如何在机载平台上实时得到稳定轨迹与局部地图 |
| 图优化三维 SLAM    | LIO-SAM 等            | 局部 LIO + 位姿图 / 回环       | 如何进一步压低长距离漂移                   |
| 稠密 / surfel 建图 | 面向重建的系统        | 维护更稠密的表面模型           | 如何服务后续重建                           |
| 多模态固态 LIO     | Fast-LIVO / R3LIVE 等 | 在 LIO 上增加相机残差          | 如何把视觉信息接进来                       |

对本课来说，目标是先在 J501 上得到一套可重复的三维激光建图流程。Fast-LIO2 正好落在这条需求上：它属于紧耦合 LIO，能直接消费 MID-360 的点云与内置 IMU，维护增量地图，并在 Jetson 级平台上以合理负载输出里程计与对齐点云。完整回环与更重的图优化，可以留到后续进阶阅读；相机融合与稠密重建，也分别对应后面的多传感器与重建课程。本课先把 Fast-LIO2 这条主线部署清楚。

---

# Part B. 实践：Ubuntu 22.04 + ROS 2 Humble 上部署 MID-360 与 Fast-LIO2

本部分流程：安装 Livox-SDK2 → 编译 `livox_ros_driver2` → 配置网络并看到点云 → 编译 Fast-LIO2 → 修改 `mid360.yaml` → 启动建图。课程基线如下：

| 项目             | 课程取值                                                          |
| ---------------- | ----------------------------------------------------------------- |
| 系统             | Ubuntu 22.04                                                      |
| ROS              | ROS 2 Humble                                                      |
| 激光雷达         | Livox MID-360                                                     |
| 建图算法         | Fast-LIO2                                                         |
| 工作空间         | `~/ros2_ws`                                                       |
| Fast-LIO 仓库    | [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) |
| 主机有线 IP 示例 | `192.168.1.50/24`                                                 |
| MID-360 IP 规则  | 默认形如 `192.168.1.1xx`，后两位由雷达 SN 末两位决定              |

> 如果你的主机 IP 不是 `192.168.1.50`，后面所有网络相关配置都改成你的实际地址，并保持“主机 IP、`MID360_config.json`、雷达 IP”三者一致。

## 5.2.3 部署前先了解 Fast-LIO2

动手前，先把 Fast-LIO2 看成一条完整的紧耦合估计链。

激光与 IMU 进入同一个迭代误差状态卡尔曼滤波（iEKF）。IMU 先以高频率预测姿态、位置、速度与偏置，并用这段短时轨迹完成扫描内去畸变；去畸变后的点再与当前地图匹配，形成激光残差，回过来校正同一套状态；对齐后的点写入增量地图，供下一次匹配使用。

对 MID-360，本课这样处理关键量：

| 类别           | 本课处理方式                                                              |
| -------------- | ------------------------------------------------------------------------- |
| LiDAR-IMU 外参 | 先用仓库 `mid360.yaml` 中的默认几何，并把 `extrinsic_est_en` 设为 `false` |
| IMU 偏置       | 启动后静止数秒，由 Fast-LIO2 在线估计                                     |
| 时间同步       | 先通过驱动健康检查确认 `/livox/lidar` 与 `/livox/imu`                     |
| 建图输入       | 使用 `msg_MID360_launch.py`，不要用只做可视化的 `rviz_MID360_launch.py`   |

## 5.2.4 安装 Livox-SDK2

先安装 SDK，再编译 ROS 驱动。

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

### 3. 可选验证

这一步可以先不接雷达。若未接雷达，程序可能报连接错误，但只要日志里已经出现启动成功信息，通常说明 SDK 已经编译安装完成。

```bash
find ~/Livox-SDK2 -name "mid360_config.json"
cd ~/Livox-SDK2/build/samples/livox_lidar_quick_start
./livox_lidar_quick_start /home/$USER/Livox-SDK2/samples/livox_lidar_quick_start/mid360_config.json
```

把命令里的路径换成你本机的实际用户名路径。

## 5.2.5 下载并编译 livox_ros_driver2

下面默认工作空间名为 `~/ros2_ws`。如果还没有，先创建：

```bash
mkdir -p ~/ros2_ws/src
```

### 1. 克隆驱动

```bash
cd ~/ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

### 2. 补齐 `package.xml`

克隆下来的仓库默认只有 `package_ROS1.xml` 和 `package_ROS2.xml`，没有直接可用的 `package.xml`。ROS 2 下必须先复制一份：

```bash
cd ~/ros2_ws/src/livox_ros_driver2
cp package_ROS2.xml package.xml
```

没有这一步，后续编译会失败。

### 3. 用官方脚本编译

`livox_ros_driver2` 不要直接在工作空间根目录用 `colcon build` 硬编，按下面顺序执行：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src/livox_ros_driver2
./build.sh humble
cd ~/ros2_ws
source install/setup.bash
```

编译过程中可能出现黄色警告，只要最终编译成功即可。

## 5.2.6 配置网络并确认能看到 MID-360 点云

### 1. 设置主机有线网卡

在系统网络设置里，把连接 MID-360 的有线网卡改为手动 IPv4，例如：

| 项目     | 示例值          |
| -------- | --------------- |
| 地址     | `192.168.1.50`  |
| 子网掩码 | `255.255.255.0` |
| 网关     | 留空            |

### 2. 修改 `MID360_config.json`

打开：

```bash
nano ~/ros2_ws/src/livox_ros_driver2/config/MID360_config.json
```

把主机相关 IP 改成与网卡一致，例如都写成 `192.168.1.50`。雷达 IP 一般出厂形如 `192.168.1.1xx`：查看雷达盒上的 SN，把末两位补到 `192.168.1.1` 后面。例如 SN 以 `23` 结尾，则雷达 IP 常为 `192.168.1.123`。

### 3. 点云

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

![Screenshot from 2026-09-24 15-35-42.png](./images/Screenshot%20from%202026-09-24%2015-35-42.png)

## 5.2.7 安装并编译 Fast-LIO2

### 1. 安装依赖

```bash
sudo apt install -y libeigen3-dev libpcl-dev
sudo apt install -y ros-humble-pcl-conversions ros-humble-pcl-ros
```

### 2. 克隆仓库

官方 Fast-LIO 早期以 ROS 1 为主；本课直接使用面向 ROS 2 的社区维护版 [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)。克隆时必须加 `--recursive`，否则会缺 submodule 文件。

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git
```

如果克隆时漏了 submodule，补执行：

```bash
cd ~/ros2_ws/src/FAST_LIO_ROS2
git submodule update --init --recursive
```

### 3. 回到工作空间根目录编译

注意：clone 在 `src` 下，但 `colcon build` 必须在 `~/ros2_ws` 根目录执行。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select fast_lio --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

记录当前提交，便于复现：

```bash
cd ~/ros2_ws/src/FAST_LIO_ROS2
git rev-parse --short HEAD
```

## 5.2.8 修改 Fast-LIO2 参数

打开：

```bash
nano ~/ros2_ws/src/FAST_LIO_ROS2/config/mid360.yaml
```

先准备一个地图保存目录：

```bash
mkdir -p ~/maps/fast_lio
```

然后确认或修改这些项：

```yaml
extrinsic_est_en: false

map_file_path: "/home/你的用户名/maps/fast_lio/mid360_current.pcd"

pcd_save:
    pcd_save_en: true
    interval: -1
```

说明：

- `extrinsic_est_en: false`：先按配置文件中的固定外参跑，启动更稳；
- `map_file_path`：必须写成你板子上的绝对路径；
- 若要保存 PCD，把 `pcd_save_en` 设为 `true`。

仓库默认话题通常已经是：

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/livox/imu"
```

如果没有改过驱动话题名，这里一般不用动。

> **图片占位：** `images/mid360_extrinsic_check.png`
> **需要补充：** `mid360.yaml` 中外参、`map_file_path` 与 `pcd_save_en` 的关键截图。

## 5.2.9 启动 Fast-LIO2 建图

准备两个终端。注意：建图时运行的是 `msg_MID360_launch.py`，不是前面只看点云的 `rviz_MID360_launch.py`。

### 终端 A：Livox 驱动

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 终端 B：Fast-LIO2

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

如需一起打开 RViz：

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=true
```

### 首次运行建议

1. 启动后先静止 5–10 秒，等待偏置收敛；
2. 先慢速平移，再做平缓转弯；
3. 选择墙面、立柱、门框等结构清晰的室内环境；
4. 重访一个有辨识度的墙角，目视检查漂移。

另开一个终端可做健康检查：

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | rg -i 'livox|odom|path|cloud'
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
```

> **图片占位：** `images/j501_fastlio_rviz.jpg`
> **需要补充：** Fast-LIO2 运行中的 RViz 截图，显示增长中的点云地图与轨迹。

## 5.2.10 保存轨迹与地图产物

### 保存 PCD

若 `mid360.yaml` 中已打开 `pcd_save_en`，建图结束后调用：

```bash
source ~/ros2_ws/install/setup.bash
ros2 service list | rg map_save
ros2 service call /map_save std_srvs/srv/Trigger {}
ls -lh ~/maps/fast_lio/
```

### 同步录包

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

话题名如有差异，先用 `ros2 topic list` 替换。

### 可选：导出简单轨迹文本

可从 `/path` 或 bag 中导出 TUM 风格文本，便于和第 5.1 课二维地图对照：

```text
timestamp tx ty tz qx qy qz qw
```

> **图片占位：** `images/pcd_map_and_projected_2d.png`
> **需要补充：** 保存下来的 PCD，以及与第 5.1 课二维地图的对照图。

## 5.2.11 质量检查与常见故障

一张好看的截图不足以判定成功。需要同时检查初始化、几何一致性、重访误差与运行健康度。

### 最低检查项

1. **静止预热：** 机器人静止 5–10 s，路径应保持稳定；
2. **走廊平直性：** 完整通过后，长墙应大致保持平直；
3. **返回起点的目视误差：** 绕行一圈后，重访墙角应接近原始结构；
4. **IMU-激光一致性：** 在受控失效演示中，故意破坏 IMU 时序应明显损伤去畸变；
5. **运行健康度：** 可持续实时建图，处理队列保持稳定。

### 实用命令

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
ros2 run tf2_ros tf2_echo camera_init body
top
```

若 TF 中的世界系名称不是 `camera_init`，按实际名称替换。

### 常见问题

| 现象                        | 可能原因                                               | 处理                                                                              |
| --------------------------- | ------------------------------------------------------ | --------------------------------------------------------------------------------- |
| 点云像果冻一样增长          | 去畸变 / 时间戳问题                                    | 检查逐点时间与 IMU 频率；确认建图用的是 `msg_MID360_launch.py`                    |
| 一开始运动地图就倾斜        | 外参或轴向约定问题                                     | 复查 `mid360.yaml` 与安装方向                                                     |
| 过门口时里程计跳变          | 运动过猛或结构过弱                                     | 减速、停顿，并以更好重叠重访                                                      |
| 驱动编译失败                | 缺少 `package.xml` 或用错编译方式                      | 先 `cp package_ROS2.xml package.xml`，再用 `./build.sh humble`                    |
| Fast-LIO 缺文件 / 编不过    | 克隆时未加 `--recursive`，或在 `src` 下执行了 `colcon` | 补 submodule，并回到 `~/ros2_ws` 根目录重编                                       |
| 能 ping 通但无点云 / 无 IMU | `MID360_config.json` 与网卡 IP 不一致                  | 统一主机 IP、配置文件与雷达 SN 对应 IP                                            |
| 节点关不干净                | 旧进程残留                                             | `killall -9 livox_ros_driver2_node`                                               |
| 重新编译仍沿用旧错误产物    | `build` / `install` 残留                               | 删除对应包后再编，例如 `rm -rf build/livox_ros_driver2 install/livox_ros_driver2` |

补充排查命令：

```bash
# 刷新动态库缓存
sudo ldconfig

# 若怀疑防火墙拦截雷达通信，可临时关闭排查
sudo ufw disable

# 清理某个包的旧编译产物后再重编
rm -rf ~/ros2_ws/build/livox_ros_driver2
rm -rf ~/ros2_ws/install/livox_ros_driver2
```

## 5.2.12 实验与验收

请在真实的 MID-360 平台上完成以下内容。

1. **SDK 与驱动：** 完成 Livox-SDK2 与 `livox_ros_driver2` 编译；
2. **点云复验：** 用 `rviz_MID360_launch.py` 确认点云稳定；
3. **建图拉起：** 用 `msg_MID360_launch.py` + `mapping.launch.py` 启动 Fast-LIO2；
4. **慢速环路建图：** 行驶一圈室内环路，产出对齐点云 / PCD 产物；
5. **与 5.1 交叉核对：** 将同一房间的三维结构与先前二维占用栅格做目视比较。

### 交付物

- 含 commit hash 与配置文件名的启动记录；
- 完成室内环路后的 RViz 截图；
- 含里程计/路径的轨迹文件或 bag；
- PCD 或等价三维地图产物；
- 一份覆盖真实故障的简短诊断说明。

### 验收标准

| 检查项     | 通过标准                                         |
| ---------- | ------------------------------------------------ |
| 传感器契约 | MID-360 激光与 IMU 在当前网络配置下均健康        |
| 估计器锁定 | 慢速环路行驶时，Fast-LIO2 可持续输出里程计       |
| 地图产物   | 存在可重新打开的三维地图文件或对齐点云 bag       |
| 几何合理性 | 墙面与结构可识别，无明显果冻畸变                 |
| 课程交接   | 坐标系名、话题与保存路径已记录，可供后续模块使用 |

## 课程小结

本课将课程从面向导航的二维地图推进到 J501 上的三维激光建图。课中比较了主要三维算法路线，在部署前压缩介绍了 Fast-LIO2 的紧耦合估计链，并给出了一条从 Livox-SDK2、驱动编译、网络配置到 Fast-LIO2 启动的可重复部署流程。成功标准是能够稳定看到点云、跑起建图，并留下轨迹与地图产物。

## 下一步

后续课程可从这里走向两个方向：

- 在稳定 LIO 之上增加相机的多传感器建图；
- 将几何进一步转化为重建与语义层，供 M6/M7 使用。

先把 MID-360 + Fast-LIO2 的安装与拉起流程做到稳定可重复，再进入下一步。

## 参考

- Xu, Zhang, et al.: Fast-LIO / Fast-LIO2 论文与开源仓库
- Zhang, Singh: LOAM
- Shan et al.: LIO-SAM
- 课程前置：[M3.1 激光雷达接入与点云预处理](../../M03-LiDAR-and-Multi-Sensor-Fusion/3.1_LiDAR_Integration_and_Point_Cloud_Preprocessing/README_zh_CN.md)
- 上一课：[5.1 认识SLAM，构建你的第一张栅格地图](../5.1_What_SLAM_Is_and_2D_LiDAR_Mapping/README_zh_CN.md)
