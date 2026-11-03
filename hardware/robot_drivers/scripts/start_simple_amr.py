#!/usr/bin/env python3
"""Start the manual Simple AMR stack in WebUI or ROS 2 control mode."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
from urllib import error, request


PROJECT = Path(__file__).resolve().parents[1]
API = "http://127.0.0.1:8765/api"
CAN_UNIT = "dm-h65-can1-manual.service"
BRIDGE_UNIT = "dm-h65-ros2-bridge.service"


def api(path: str, payload: dict | None = None) -> dict:
    data = None if payload is None else json.dumps(payload).encode()
    req = request.Request(API + path, data=data,
                          headers={"Content-Type": "application/json"})
    try:
        with request.urlopen(req, timeout=3) as response:
            return json.load(response)
    except error.HTTPError as exc:
        try:
            detail = json.load(exc).get("error", exc.reason)
        except (json.JSONDecodeError, ValueError):
            detail = exc.reason
        raise RuntimeError(f"{path}: {detail}") from exc


def state() -> dict | None:
    try:
        return api("/state")
    except (error.URLError, TimeoutError):
        return None


def wait_for(predicate, description: str, seconds: int = 15) -> dict:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        current = state()
        if current and predicate(current):
            return current
        time.sleep(0.3)
    raise RuntimeError(f"等待{description}超时，请检查 WebUI 和桥接服务日志")


def systemctl(verb: str, unit: str) -> None:
    subprocess.run(["sudo", "systemctl", verb, unit], check=True)


def choose_mode(args: list[str] | None = None) -> str:
    parser = argparse.ArgumentParser(description="启动 Simple AMR，选择网页或 ROS 2 控制")
    parser.add_argument("mode", nargs="?", default="webui", choices=("webui", "ros2"),
                        help="webui 或 ros2；省略时默认 webui")
    return parser.parse_args(args).mode


def main(args: list[str] | None = None) -> None:
    mode = choose_mode(args)
    systemctl("start", CAN_UNIT)
    current = state()
    if current is None:
        executable = PROJECT / ".venv/bin/dm-h65-webui"
        if not executable.is_file():
            raise RuntimeError(f"找不到 WebUI 程序：{executable}")
        log = PROJECT / "webui.log"
        with log.open("ab") as output:
            subprocess.Popen([str(executable), "--host", "0.0.0.0", "--port", "8765",
                              "--interface", "can1"], cwd=PROJECT,
                             stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                             start_new_session=True)
        current = wait_for(lambda s: s.get("service_online"), "WebUI 启动")
    if current.get("can_interface") != "can1" or current.get("simulation") or current.get("can_state") != "up":
        raise RuntimeError("WebUI 不是可用的 can1 实机服务；不会切换控制方式")
    if "control_mode" not in current:
        raise RuntimeError("当前 WebUI 版本不支持启动模式选择，请先更新后端")

    if mode == "webui":
        api("/control-mode", {"mode": "webui"})
        systemctl("stop", BRIDGE_UNIT)
    else:
        systemctl("start", BRIDGE_UNIT)
        wait_for(lambda s: s.get("ros2", {}).get("online"), "ROS 2 桥接在线")
        api("/control-mode", {"mode": "ros2"})
        wait_for(lambda s: s.get("control_mode") == "ros2" and s.get("ros2", {}).get("armed"),
                 "ROS 2 零速度待命")

    final = state()
    print(f"Simple AMR 已启动：{'网页控制' if mode == 'webui' else 'ROS 2 /cmd_vel 控制'}")
    print(f"WebUI: http://<Jetson-IP>:8765/  |  CAN: {final['can_interface']}  |  "
          f"速度: {final['drive']['linear_m_s']:.2f} m/s, "
          f"{final['drive']['angular_rad_s']:.2f} rad/s")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError, error.URLError) as exc:
        print(f"启动失败：{exc}", file=sys.stderr)
        raise SystemExit(1) from exc
