#!/usr/bin/env python3
"""Manage the manual Simple AMR stack in WebUI or ROS 2 control mode."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
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
    raise RuntimeError(f"Timed out waiting for {description}; check the WebUI and bridge logs")


def systemctl(verb: str, unit: str) -> None:
    subprocess.run(["sudo", "systemctl", verb, unit], check=True)


def choose_mode(args: list[str] | None = None) -> str:
    parser = argparse.ArgumentParser(description="Start Simple AMR in WebUI or ROS 2 control mode")
    parser.add_argument("mode", nargs="?", default="webui", choices=("webui", "ros2"),
                        help="webui or ros2; defaults to webui")
    return parser.parse_args(args).mode


def parse_action(args: list[str] | None = None) -> tuple[str, str]:
    parser = argparse.ArgumentParser(description="Manage the Simple AMR chassis SDK")
    parser.add_argument("action", nargs="?", default="start",
                        choices=("start", "stop", "status", "webui", "ros2"),
                        help="start [webui|ros2], stop, or status; defaults to WebUI control")
    parser.add_argument("mode", nargs="?", choices=("webui", "ros2"))
    parsed = parser.parse_args(args)
    if parsed.action in ("webui", "ros2"):
        if parsed.mode:
            parser.error("the legacy mode form does not accept another argument")
        return "start", parsed.action
    if parsed.action != "start" and parsed.mode:
        parser.error("stop and status do not accept a control mode")
    return parsed.action, parsed.mode or "webui"


def webui_pids() -> list[int]:
    """Only select this project's WebUI, never an unrelated process on the port."""
    executable = os.fsencode(PROJECT / ".venv/bin/dm-h65-webui")
    found = []
    for proc in Path("/proc").glob("[0-9]*"):
        try:
            argv = (proc / "cmdline").read_bytes().split(b"\0")
        except (OSError, PermissionError):
            continue
        if executable in argv:
            found.append(int(proc.name))
    return found


def stop_stack() -> None:
    current = state()
    pids = webui_pids()
    if current is not None:
        if not pids:
            raise RuntimeError("Port 8765 responds, but it is not this project's WebUI; CAN1 remains up for safety")
        api("/drive/emergency-stop", {})
        wait_for(lambda s: not s.get("drive", {}).get("enabled", True)
                 and s.get("drive", {}).get("linear_m_s") == 0
                 and s.get("drive", {}).get("angular_rad_s") == 0,
                 "motor output to be disabled", seconds=5)
    elif pids:
        raise RuntimeError("The WebUI process exists but its API is unavailable; motor stop cannot be verified, so CAN1 remains up")

    systemctl("stop", BRIDGE_UNIT)
    for pid in pids:
        os.kill(pid, signal.SIGINT)  # WebUI handles Ctrl+C and calls service.close().
    deadline = time.monotonic() + 10
    while webui_pids() and time.monotonic() < deadline:
        time.sleep(0.2)
    if webui_pids() or state() is not None:
        raise RuntimeError("WebUI did not exit cleanly; CAN1 remains up for safety. Check webui.log")
    systemctl("stop", CAN_UNIT)
    print("Simple AMR stopped: motors disabled, ROS 2 bridge and WebUI exited, CAN1 down")


def show_status() -> None:
    for label, unit in (("CAN1", CAN_UNIT), ("ROS 2 bridge", BRIDGE_UNIT)):
        result = subprocess.run(["systemctl", "is-active", unit], capture_output=True,
                                text=True, check=False)
        print(f"{label}: {result.stdout.strip() or 'unknown'}")
    current = state()
    if current is None:
        print("WebUI: offline")
        return
    drive = current.get("drive", {})
    motors = current.get("motors", {})
    print(f"WebUI: online | mode: {current.get('control_mode')} | "
          f"motor output: {'enabled' if drive.get('enabled') else 'disabled'} | "
          f"E-stop: {'active' if drive.get('emergency_stop') else 'clear'}")
    print(f"Left wheel: {'online' if motors.get('left', {}).get('online') else 'offline'} | "
          f"right wheel: {'online' if motors.get('right', {}).get('online') else 'offline'} | "
          f"speed: {drive.get('linear_m_s', 0):.2f} m/s, "
          f"{drive.get('angular_rad_s', 0):.2f} rad/s")


def start_stack(mode: str) -> None:
    systemctl("start", CAN_UNIT)
    current = state()
    if current is None:
        executable = PROJECT / ".venv/bin/dm-h65-webui"
        if not executable.is_file():
            raise RuntimeError(f"WebUI executable not found: {executable}")
        log = PROJECT / "webui.log"
        with log.open("ab") as output:
            subprocess.Popen([str(executable), "--host", "0.0.0.0", "--port", "8765",
                              "--interface", "can1"], cwd=PROJECT,
                             stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                             start_new_session=True)
        current = wait_for(lambda s: s.get("service_online"), "WebUI startup")
    if current.get("can_interface") != "can1" or current.get("simulation") or current.get("can_state") != "up":
        raise RuntimeError("WebUI is not a usable real-hardware can1 service; control mode was not changed")
    if "control_mode" not in current:
        raise RuntimeError("This WebUI version does not support control mode selection; update the backend")

    if mode == "webui":
        api("/control-mode", {"mode": "webui"})
        systemctl("stop", BRIDGE_UNIT)
    else:
        systemctl("start", BRIDGE_UNIT)
        wait_for(lambda s: s.get("ros2", {}).get("online"), "the ROS 2 bridge to come online")
        api("/control-mode", {"mode": "ros2"})
        wait_for(lambda s: s.get("control_mode") == "ros2" and s.get("ros2", {}).get("armed"),
                 "ROS 2 zero-velocity readiness")

    final = state()
    print(f"Simple AMR started: {'WebUI control' if mode == 'webui' else 'ROS 2 /cmd_vel control'}")
    print(f"WebUI: http://<Jetson-IP>:8765/  |  CAN: {final['can_interface']}  |  "
          f"speed: {final['drive']['linear_m_s']:.2f} m/s, "
          f"{final['drive']['angular_rad_s']:.2f} rad/s")


def main(args: list[str] | None = None) -> None:
    action, mode = parse_action(args)
    if action == "stop":
        stop_stack()
    elif action == "status":
        show_status()
    else:
        start_stack(mode)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError, error.URLError) as exc:
        print(f"Simple AMR operation failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
