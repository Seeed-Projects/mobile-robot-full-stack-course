# 故障排查

## 上电后 `ResponseTimeout`

依次检查：

```bash
systemctl status dm-h65-can-termination.service
systemctl status dm-h65-can.service
ip -details -statistics link show can0
dm-h65 --interface can0 scan --first 0 --last 15
```

硬件检查：电机动力电源、Jetson 与驱动器共地、CAN_H/CAN_L 未接反、两端终端电阻正确、波特率为 1 Mbps。上电后如果驱动器 RAM 参数复位，也要重新确认 CAN ID 和控制模式。

## WebUI 有速度数值但轮子不转

先看页面底部：

- “模拟模式”：没有连接真实 CAN，数值只用于预览。
- “`can0 · 实机模式`”：连接 Jetson 实机服务。

确认本机端口实际由 SSH 隧道占用，而不是本地模拟服务：

```bash
curl http://127.0.0.1:8765/api/state
```

返回值必须包含：

```json
{"simulation": false, "can_state": "up"}
```

Jetson 上检查事件：

```bash
curl http://127.0.0.1:8765/api/events
journalctl -u dm-h65-webui.service -f
```

“目标转速”只是下发值，不代表编码器已经确认运动。需要用 `read_telemetry()` 的位置变化或有界旋转测试确认真实运动。

## 控制权取得失败

控制权会持续保持。若另一个浏览器仍持有控制权，应在原页面主动释放。无法访问原客户端时，可以在确认底盘安全后重启服务：

```bash
sudo systemctl restart dm-h65-webui.service
```

服务重启会停止、失能并清除控制权。

## 指令超时后自动失能

这是 750 ms 命令看门狗的预期行为。检查浏览器网络、SSH 隧道和服务日志。不要通过删除看门狗来掩盖网络不稳定。

## CAN 错误或 bus-off

```bash
ip -details -statistics link show can0
sudo ip link set can0 down
sudo ip link set can0 up
```

如果 `bus-errors` 持续增加，重点检查终端电阻、线缆、共地和波特率。`restart-ms 100` 只能帮助总线恢复，不能修复物理层问题。

## 多个程序同时运行

WebUI 和 CLI 状态命令都会读取 CAN 响应。不要同时运行 `dual_motor_status.py`、WebUI 和另一个控制循环，否则一个进程可能消费另一个进程等待的响应帧。调试 CLI 前先停止 WebUI：

```bash
sudo systemctl stop dm-h65-webui.service
```

测试完成后再启动：

```bash
sudo systemctl start dm-h65-webui.service
```

