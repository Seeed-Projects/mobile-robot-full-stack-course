# WebUI 使用指南

## 启动方式

安装 SDK 后：

```bash
source .venv/bin/activate
dm-h65-webui --host 0.0.0.0 --port 8765 --interface can0
```

模拟模式只用于界面开发，不会访问 CAN：

```bash
dm-h65-webui --simulate --host 127.0.0.1 --port 8765
```

不要把模拟模式的速度数字当成真实电机反馈。页面底部必须显示“`can0 · 实机模式`”才能控制真实底盘。

## 浏览器连接

局域网：

```text
http://<Jetson-IP>:8765/
```

SSH 隧道：

```bash
ssh -N -L 127.0.0.1:8765:127.0.0.1:8765 -p 8084 seeed@18.136.89.187
```

如果本机 8765 已被占用，可改用 8766：

```bash
ssh -N -L 127.0.0.1:8766:127.0.0.1:8765 -p 8084 seeed@18.136.89.187
```

然后访问 `http://127.0.0.1:8766/`。

## 操作流程

1. 确认服务、CAN0、左轮 0x01 和右轮 0x02 均在线。
2. 点击“取得控制权”。控制权没有时间租约，会保持到主动释放、急停或服务重启。
3. 按住并拖动摇杆。按下时后端使能两台电机，拖动时每 100 ms 发送速度命令。
4. 松开摇杆后立即发送停止命令并失能两台电机。
5. 完成调试后点击控制权按钮主动释放。

键盘模式使用空格保持使能，W/A/S/D 或方向键控制移动。

## 状态含义

- “线速度/角速度”：后端最近接受的底盘命令。
- “目标转速”：由底盘命令换算并下发的左右轮目标角速度，不是编码器测得的实际速度。
- “在线”：最近一次寄存器遥测成功。
- “急停”：停止、失能并释放当前控制权。

## 安全机制

- 同一时间只有一个客户端持有控制权。
- 750 ms 内没有新的速度命令时，看门狗停止并失能电机。
- 线速度绝对值上限 `0.6 m/s`。
- 角速度绝对值上限 `2.0 rad/s`。
- 两台电机必须同时在线才允许启用。
- 急停复位后电机仍保持禁用，需要重新取得控制权和使能。

## systemd 管理

```bash
sudo systemctl start dm-h65-webui.service
sudo systemctl restart dm-h65-webui.service
sudo systemctl stop dm-h65-webui.service
systemctl status dm-h65-webui.service
journalctl -u dm-h65-webui.service -f
```

API 健康检查：

```bash
curl http://127.0.0.1:8765/api/state
curl http://127.0.0.1:8765/api/events
```

WebUI 是底盘调试工具，不包含公网身份认证。不要直接把 8765 端口暴露到互联网；使用 SSH 隧道、VPN 或带 TLS 和认证的反向代理。

