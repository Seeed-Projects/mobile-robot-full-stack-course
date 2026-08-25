# Jetson 部署指南

## 依赖和安装

```bash
sudo apt update
sudo apt install -y can-utils gpiod python3-venv

cd /path/to/dm_h65_sdk
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e .
```

## 终端电阻

CAN 总线只应在线缆物理两端各有一个 120 Ω 终端电阻。断电测量 CAN_H 与 CAN_L，正常通常约为 60 Ω。

当前验证过的 reComputer Robotics J501 通过 `gpiochip2` line 9 的低电平接入板载 CAN0 终端电阻：

```bash
gpioinfo gpiochip2
sudo gpioset --mode=signal --background gpiochip2 9=0
```

不同 Jetson 载板或系统镜像的 GPIO 映射可能不同。必须先用 `gpioinfo` 核对；使用外置终端电阻时不要盲目启用板载终端。

## 手动启动 CAN

```bash
sudo bash scripts/setup_can.sh can0 1000000
ip -details -statistics link show can0
```

## 安装 systemd 服务

对于当前 J501 配置：

```bash
sudo bash scripts/install_jetson.sh
```

脚本会：

1. 安装 `can-utils`、`gpiod` 和 `python3-venv`。
2. 创建 `.venv` 并以 editable 模式安装 SDK。
3. 安装终端电阻、CAN0 和 WebUI 三个服务。
4. 设置开机启动并立即启动服务。

验证：

```bash
systemctl status dm-h65-can-termination.service
systemctl status dm-h65-can.service
systemctl status dm-h65-webui.service
curl http://127.0.0.1:8765/api/state
```

如果项目移动到新目录，应重新运行安装脚本，让 WebUI unit 中的绝对路径更新。

## 服务启动顺序

```text
dm-h65-can-termination.service
              ↓
dm-h65-can.service
              ↓
dm-h65-webui.service
```

WebUI 服务异常退出会自动重启；服务停止时后端会尝试停止并失能两台电机。

