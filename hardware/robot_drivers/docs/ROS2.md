# Simple AMR 的 ROS 2 控制

Jetson 上使用 ROS 2 Humble。`/cmd_vel`（`geometry_msgs/msg/Twist`）由独立桥接进程订阅，并通过 `127.0.0.1:8765` 转交给 WebUI 服务。只有 WebUI 服务打开 `can1`，不要另起直接访问 CAN 的 ROS 驱动。

现在建议从项目目录使用统一入口选择启动模式（不设开机自启）：

```bash
cd /home/seeed/seeed_amr_sdk
python3 scripts/start_simple_amr.py webui  # 网页控制
python3 scripts/start_simple_amr.py ros2   # ROS 2 /cmd_vel 控制
```

可指定 `webui` 或 `ros2` 参数；不带参数时默认启动网页控制，不会出现交互菜单。该入口会启动 CAN1、复用或启动 WebUI；选择 ROS 2 时启动桥接并等待零速度待命，选择网页时切回网页并停止桥接。两种模式都保留 WebUI 状态页，只有 WebUI 服务打开 CAN。

如果不使用统一入口，也可在 WebUI 和 CAN 已启动后，单独手动启动桥接服务：

```bash
sudo systemctl start dm-h65-ros2-bridge.service
systemctl status dm-h65-ros2-bridge.service
```

桥接服务不设开机自启。首次安装服务文件：

```bash
sudo cp deploy/systemd/dm-h65-ros2-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
```

在 WebUI 中确认“ROS 2 桥接在线”；若未使用统一入口，再点击“ROS 2 控制”。切换时先停车，并等待桥接节点发送零速度后解锁 ROS 2 指令。网页摇杆和方向键在 ROS 2 模式下不可用。`linear.x` 正值为前进，`angular.z` 正值为左转；其他四个 Twist 分量必须为零。上限为 ±0.6 m/s、±2.0 rad/s，超出上限的指令会被拒绝。速度消息中断超过 750 ms 自动停车；桥接心跳中断超过 3 s 停车并需重新收到零速度。急停会失能电机并切回网页模式。

仅在车轮已架空、现场确认安全后测试非零速度。零速度联通测试：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

切回网页控制请在 WebUI 点击“网页控制”；切换会停车。
