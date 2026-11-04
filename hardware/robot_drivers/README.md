# Simple AMR

[简体中文](README.md) | [English](README_EN.md)

Simple AMR 是基于 Jetson、SocketCAN 和 DM-H65 轮毂电机的两轮差速底盘软件，包含 Python SDK、浏览器控制台，以及 ROS 2 Humble `/cmd_vel` 桥接。WebUI 后端是唯一打开 CAN 的进程；网页与 ROS 2 速度控制互斥。

## 已验证的实机配置

| 项目 | 配置 |
| --- | --- |
| CAN | `can1`，Classic CAN，1 Mbps |
| 左轮 / 右轮 | ID `0x01` / `0x02`，安装方向 `-1` / `+1` |
| 轮半径 / 轮距 | `0.0825 m` / `0.42 m` |
| Jetson 项目目录 | `/home/seeed/seeed_amr_sdk` |
| WebUI | `http://<Jetson-IP>:8765/` |

这些参数来自当前底盘；在其他机器人上使用前必须重新核对。不要同时运行第二个访问 `can1` 的电机控制进程。

## 在已配置的 Jetson 上管理底盘 SDK

重启后从项目目录运行。**不带参数默认网页控制**，启动过程没有交互菜单：

```bash
cd /home/seeed/seeed_amr_sdk
python3 scripts/start_simple_amr.py start         # 默认：网页控制
python3 scripts/start_simple_amr.py start ros2    # ROS 2 /cmd_vel 控制
python3 scripts/start_simple_amr.py status        # 查看运行状态
python3 scripts/start_simple_amr.py stop          # 安全停车并关闭整个 SDK
```

原有的不带参数、`webui`、`ros2` 写法仍可使用。脚本会启动手动 CAN1 服务，复用或启动 WebUI，并切换控制方式。选择 `ros2` 时还会启动 ROS 2 桥接并等待零速度待命；选择 `webui` 时停止桥接。两种模式下均可访问 WebUI 查看状态。`stop` 会先急停并确认电机输出已禁用，再停止桥接、优雅退出 WebUI，最后关闭 CAN1；若无法确认安全停车，会报错并保留 CAN1。首次使用前需安装下述 CAN1 与 ROS 2 服务；旧版 CAN0/J501 服务模板不适用于这台 CAN1 设备。

在当前 Jetson 上可选装更短的命令（仅需一次，不设置开机自启）：

```bash
chmod +x scripts/start_simple_amr.py
sudo ln -s /home/seeed/seeed_amr_sdk/scripts/start_simple_amr.py /usr/local/bin/simple-amr
simple-amr start
simple-amr start ros2
simple-amr status
simple-amr stop
```

确认状态：

```bash
ip -details -statistics link show can1
curl http://127.0.0.1:8765/api/state
```

CAN 应为 `UP`、`ERROR-ACTIVE`、`bitrate 1000000`。浏览器打开 `http://<Jetson-IP>:8765/`，确认页面显示 `can1 · 实机模式`、两轮在线且没有急停锁定。

## 网页控制

在页面选择“网页控制”，用摇杆、W/A/S/D、方向键或原地旋转按钮操作；松开输入后平滑减速至零。无需取得控制权或按住启用。急停会失能电机；确认现场安全后才能复位。页面上的目标转速是下发目标值，**不是**电机实测速度。

驾驶台的“加减速时间”可在停车时设置为 `0.3–1.5 秒`，默认 `0.8 秒`，并保存在 Jetson 的 `~/.config/simple-amr/drive.json`。网页控制的加速和松手停车都按此时间平滑变化；数值越大越柔和，但停车距离越长。它不改变 ROS 2 控制的速度响应，也不延迟急停、控制方式切换或通信超时保护。

## ROS 2 控制

运行 `python3 scripts/start_simple_amr.py ros2`，或在桥接在线时于 WebUI 选择“ROS 2 控制”。桥接订阅 `geometry_msgs/msg/Twist` 的 `/cmd_vel`：`linear.x` 正值为前进，`angular.z` 正值为左转，其余四个分量必须为零。网页摇杆在 ROS 2 模式下暂停。

```bash
source /opt/ros/humble/setup.bash
ros2 topic info /cmd_vel
sudo apt install ros-humble-teleop-twist-keyboard  # 仅首次安装
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.4
```

`ros2 topic info` 应看到 1 个订阅者。在运行键盘节点的终端按 `i` 前进、`,` 后退、`j` 左转、`l` 右转、`k` 停车，`Ctrl+C` 退出。终端必须获得键盘焦点。速度消息中断超过 750 ms 会自动停车，单次按键可能只产生短暂运动。详细安全约束见 [ROS 2 控制指南](docs/ROS2.md)。

## 首次安装或开发环境

在 Jetson 上准备 Python 环境；仓库内已包含构建好的 WebUI 静态文件，运行时无需 Node.js：

```bash
sudo apt update
sudo apt install -y can-utils gpiod python3-venv
cd /home/seeed/seeed_amr_sdk
python3 -m venv .venv
.venv/bin/python -m pip install -e .
.venv/bin/dm-h65-webui --help
```

新设备必须先按载板手册核对终端电阻、GPIO 映射和 CAN 接线。当前 J401/CAN1 实机的手动服务模板位于 `deploy/systemd/dm-h65-can1-manual.service`，其中使用 `gpiochip2` line 4；**不要在其他载板上直接套用**。核对硬件后安装服务（不设开机自启）：

```bash
sudo cp deploy/systemd/dm-h65-can1-manual.service deploy/systemd/dm-h65-ros2-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
```

**不要直接运行旧版 `install_jetson.sh`**：它是早期 CAN0/J501 配置。`sudo bash scripts/setup_can.sh can1 1000000` 只能临时配置 CAN1，不会安装 systemd 服务。ROS 2 桥接说明见 [ROS 2 控制指南](docs/ROS2.md)。

只读电机诊断也应在 WebUI 底盘服务停止后执行，避免多个进程争用 CAN 响应帧：

```bash
.venv/bin/dm-h65 --interface can1 scan --first 0 --last 15
.venv/bin/dm-h65 --interface can1 status 0x01
.venv/bin/dm-h65 --interface can1 status 0x02
```

本地测试与前端构建：

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
cd webui/frontend
npm install
npm run build
npm run test:sites
cd ../..
python3 scripts/sync_webui_assets.py
```

`sync_webui_assets.py` 会用新构建资源**替换** `src/dm_h65/webui_static/`，仅在需要更新前端时运行。Python SDK 位于 `src/dm_h65/`，ROS 2 桥接位于 `scripts/ros2_cmd_vel_bridge.py`，前端位于 `webui/frontend/`，测试位于 `tests/`。

## 安全说明

- 服务端限制线速度绝对值不超过 `0.6 m/s`、角速度绝对值不超过 `2.0 rad/s`；两轮必须在线才允许运动。
- 切换控制方式时先停车；ROS 2 桥接中断会停车。急停会失能电机并切回网页模式。
- 软件急停和超时停车**不能代替独立的硬件断电急停**。首次运动测试前，架空并固定车轮，清空周围人员和障碍物。
- WebUI 没有公网身份认证。不要把 8765 端口直接暴露到互联网；远程访问请使用可信局域网、VPN 或 SSH 隧道。
