"""Safe HTTP service for the DM-H65 two-wheel chassis WebUI."""
from __future__ import annotations
import argparse, json, math, secrets, threading, time
from collections import deque
from datetime import datetime
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

HERE = Path(__file__).resolve().parent
from . import DMH65Motor, DifferentialDrive
from .transport import open_socketcan

class ServiceError(RuntimeError):
    def __init__(self, message: str, status: int = HTTPStatus.BAD_REQUEST):
        super().__init__(message); self.status = status

class ChassisService:
    """Serializes all CAN access and enforces input mode, watchdog and e-stop."""
    RAMP_MIN_S = 0.3
    RAMP_MAX_S = 1.5
    RAMP_DEFAULT_S = 0.8

    def __init__(self, interface="can1", simulate=False, wheel_radius_m=0.0825, track_width_m=0.42,
                 settings_path=None):
        self.interface, self.simulate = interface, simulate
        self.wheel_radius_m, self.track_width_m = wheel_radius_m, track_width_m
        self._lock = threading.RLock(); self._control_lock = threading.RLock(); self._events = deque(maxlen=100)
        self._token = self._owner = None
        self._enabled = self._emergency = False; self._linear = self._angular = 0.0; self._last_command = 0.0
        self._target_linear = self._target_angular = 0.0
        self._settings_path = Path(settings_path) if settings_path else Path.home() / ".config/simple-amr/drive.json"
        self._ramp_time_s = self._load_ramp_time()
        self._control_mode = "webui"; self._ros2_last_heartbeat = 0.0; self._ros2_armed = False
        self._transport = self._drive = None; self._closed = threading.Event()
        self._motors = {"left": self._blank_motor(1), "right": self._blank_motor(2)}
        self._connect()
        threading.Thread(target=self._motion_loop, name="chassis-motion", daemon=True).start()
        threading.Thread(target=self._watchdog_loop, name="chassis-watchdog", daemon=True).start()
        threading.Thread(target=self._telemetry_loop, name="chassis-telemetry", daemon=True).start()

    @staticmethod
    def _blank_motor(motor_id):
        return {"id": motor_id, "online": False, "velocity_rad_s": 0.0, "bus_voltage_v": 0.0, "motor_temperature_c": 0.0, "pcb_temperature_c": 0.0, "position_rad": 0.0}

    def _event(self, message, level="info"):
        self._events.appendleft({"time": datetime.now().strftime("%H:%M:%S"), "message": message, "level": level})

    @classmethod
    def _valid_ramp_time(cls, seconds):
        if isinstance(seconds, bool) or not isinstance(seconds, (int, float)):
            return False
        return math.isfinite(seconds) and cls.RAMP_MIN_S <= seconds <= cls.RAMP_MAX_S

    def _load_ramp_time(self):
        try:
            seconds = json.loads(self._settings_path.read_text(encoding="utf-8"))["ramp_time_s"]
            if self._valid_ramp_time(seconds):
                return float(seconds)
        except (OSError, ValueError, KeyError, TypeError):
            pass
        return self.RAMP_DEFAULT_S

    def set_ramp_time(self, seconds):
        if not self._valid_ramp_time(seconds):
            raise ServiceError(f"加减速时间必须在 {self.RAMP_MIN_S:.1f}～{self.RAMP_MAX_S:.1f} 秒之间")
        with self._lock:
            if any(abs(v) > 1e-6 for v in (self._linear, self._angular,
                                          self._target_linear, self._target_angular)):
                raise ServiceError("请先停车，再调整加减速时间", HTTPStatus.CONFLICT)
            try:
                self._settings_path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self._settings_path.with_suffix(".tmp")
                temporary.write_text(json.dumps({"ramp_time_s": float(seconds)}) + "\n", encoding="utf-8")
                temporary.replace(self._settings_path)
            except OSError as exc:
                raise ServiceError(f"无法保存加减速设置：{exc}", HTTPStatus.INTERNAL_SERVER_ERROR) from exc
            self._ramp_time_s = float(seconds)
            self._event(f"加减速时间已设为 {self._ramp_time_s:.1f} 秒")
            return self._ramp_time_s

    @staticmethod
    def _approach(current, target, maximum_delta):
        return current + max(-maximum_delta, min(maximum_delta, target - current))

    def _motion_step(self, duration_s):
        with self._lock:
            if not self._enabled or self._control_mode != "webui":
                return
            duration_s = max(0.0, min(duration_s, 0.1))
            linear = self._approach(self._linear, self._target_linear,
                                    .6 * duration_s / self._ramp_time_s)
            angular = self._approach(self._angular, self._target_angular,
                                     2.0 * duration_s / self._ramp_time_s)
            if linear == self._linear and angular == self._angular:
                return
            speeds = self._drive.set_twist(-linear, angular) if self._drive else None
            self._linear, self._angular = linear, angular
            if speeds:
                self._motors["left"]["velocity_rad_s"] = speeds.left_rad_s
                self._motors["right"]["velocity_rad_s"] = speeds.right_rad_s

    def _motion_loop(self):
        last = time.monotonic()
        while not self._closed.wait(.05):
            now = time.monotonic()
            try:
                self._motion_step(now - last)
            except Exception as exc:
                with self._lock:
                    self._safe_disable(f"速度斜坡下发失败：{exc}", "error")
            last = now

    def _connect(self):
        if self.simulate:
            self._motors["left"].update(online=True, bus_voltage_v=47.8, motor_temperature_c=31.4)
            self._motors["right"].update(online=True, bus_voltage_v=47.9, motor_temperature_c=30.9)
            self._event("模拟底盘服务已启动"); return
        try:
            self._transport = open_socketcan(self.interface)
            left = DMH65Motor(self._transport, 0x01, direction=-1, max_speed_rad_s=12.0)
            right = DMH65Motor(self._transport, 0x02, direction=1, max_speed_rad_s=12.0)
            self._drive = DifferentialDrive(left, right, wheel_radius_m=self.wheel_radius_m, track_width_m=self.track_width_m, max_wheel_speed_rad_s=8.0, max_wheel_acceleration_rad_s2=3.0)
            self._event(f"已打开 {self.interface}，等待电机响应")
        except Exception as exc: self._event(f"CAN 初始化失败：{exc}", "error")

    def _control_active(self): return bool(self._token)
    def _require_token(self, token):
        return

    def acquire(self, owner):
        owner = owner.strip()[:64]
        if not owner: raise ServiceError("控制端名称不能为空")
        with self._control_lock:
            if self._control_active() and owner != self._owner: raise ServiceError(f"控制权正由 {self._owner} 持有", HTTPStatus.CONFLICT)
            if not self._control_active():
                self._token, self._owner = secrets.token_urlsafe(24), owner; self._event(f"{owner} 已取得控制权")
            return {"token": self._token, "expires_in_s": None}

    def heartbeat(self, token):
        self._require_token(token)
    def release(self, token):
        self._require_token(token)
        with self._lock: self._safe_disable("控制权已释放")
        with self._control_lock:
            if self._token and secrets.compare_digest(token, self._token): self._token = self._owner = None
    def enable(self, token):
        with self._lock:
            self._require_token(token)
            if self._emergency: raise ServiceError("急停已锁定，请确认安全后复位", HTTPStatus.CONFLICT)
            if not all(m["online"] for m in self._motors.values()): raise ServiceError("两个电机必须同时在线才可启用", HTTPStatus.SERVICE_UNAVAILABLE)
            if self._enabled: return
            if self._drive: self._drive.enable(timeout_s=0.25)
            elif not self.simulate: raise ServiceError("CAN 设备尚未就绪", HTTPStatus.SERVICE_UNAVAILABLE)
            self._enabled = True; self._last_command = time.monotonic(); self._event("电机输出已启用", "warning")
    def ros2_heartbeat(self):
        with self._lock:
            self._ros2_last_heartbeat = time.monotonic()
            return {"mode": self._control_mode, "armed": self._ros2_armed}
    def set_control_mode(self, mode):
        if mode not in ("webui", "ros2"):
            raise ServiceError("控制方式必须是 webui 或 ros2")
        with self._lock:
            if mode == self._control_mode:
                return mode
            if mode == "ros2" and self._emergency:
                raise ServiceError("急停已锁定，请先确认安全并复位", HTTPStatus.CONFLICT)
            if mode == "ros2" and time.monotonic() - self._ros2_last_heartbeat > 3.0:
                raise ServiceError("ROS 2 桥接节点离线，请先启动桥接服务", HTTPStatus.SERVICE_UNAVAILABLE)
            self.stop(require_token=False)
            self._control_mode = mode
            self._ros2_armed = False
            self._event(f"控制方式已切换为 {'ROS 2' if mode == 'ros2' else '网页'}；底盘已停车")
            return mode
    def command(self, token, linear, angular, source="webui"):
        if not math.isfinite(linear) or not math.isfinite(angular): raise ServiceError("速度必须是有限数值")
        if abs(linear) > .6 or abs(angular) > 2: raise ServiceError("速度超过服务端安全上限")
        with self._lock:
            self._require_token(token)
            if source != self._control_mode: raise ServiceError("当前控制方式不接受此速度指令", HTTPStatus.CONFLICT)
            if self._emergency: raise ServiceError("急停已锁定", HTTPStatus.CONFLICT)
            if source == "ros2" and time.monotonic() - self._ros2_last_heartbeat > 3.0:
                raise ServiceError("ROS 2 桥接节点心跳超时", HTTPStatus.SERVICE_UNAVAILABLE)
            if source == "ros2" and not self._ros2_armed:
                if linear or angular: raise ServiceError("ROS 2 模式需先收到零速度指令", HTTPStatus.CONFLICT)
                self._ros2_armed = True
            if not self._enabled and not (linear or angular):
                self._last_command = time.monotonic()
                return {"linear_m_s": 0.0, "angular_rad_s": 0.0, "left_rad_s": 0.0, "right_rad_s": 0.0}
            if not self._enabled: self.enable(token)
            self._last_command = time.monotonic()
            if source == "webui":
                self._target_linear, self._target_angular = linear, angular
                return {"linear_m_s": linear, "angular_rad_s": angular,
                        "left_rad_s": self._motors["left"]["velocity_rad_s"],
                        "right_rad_s": self._motors["right"]["velocity_rad_s"]}
            # This installation moves backward for positive motor-frame linear speed.
            # Keep the API in robot coordinates: +linear forward, +angular left.
            speeds = self._drive.set_twist(-linear, angular) if self._drive else None
            self._linear, self._angular = linear, angular
            if speeds:
                self._motors["left"]["velocity_rad_s"] = speeds.left_rad_s
                self._motors["right"]["velocity_rad_s"] = speeds.right_rad_s
            return {"linear_m_s": linear, "angular_rad_s": angular,
                    "left_rad_s": speeds.left_rad_s if speeds else 0.0,
                    "right_rad_s": speeds.right_rad_s if speeds else 0.0}
    def stop(self, token=None, require_token=True, source=None, graceful=False):
        with self._lock:
            if require_token: self._require_token(token)
            if source is not None and source != self._control_mode: return False
            self._target_linear = self._target_angular = 0.0
            if graceful and self._control_mode == "webui":
                self._last_command = time.monotonic()
                return True
            if self._drive: self._drive.stop()
            self._linear = self._angular = 0.0
            self._motors["left"]["velocity_rad_s"] = self._motors["right"]["velocity_rad_s"] = 0.0
            return True
    def disable(self, token):
        with self._lock: self._require_token(token); self._safe_disable("电机输出已禁用")
    def emergency_stop(self):
        with self._lock:
            self._safe_disable("急停已触发", "error")
            self._emergency = True
            self._control_mode = "webui"
            self._ros2_armed = False
        with self._control_lock: self._token = self._owner = None
    def reset_emergency(self, token):
        with self._lock: self._require_token(token); self._emergency = False; self._event("急停已复位，电机仍保持禁用", "warning")
    def _safe_disable(self, event, level="info"):
        if self._drive:
            try: self._drive.disable(restore_modes=False)
            except Exception as exc: self._event(f"停止电机时发生错误：{exc}", "error")
        self._enabled = False; self._linear = self._angular = 0.0
        self._target_linear = self._target_angular = 0.0
        self._motors["left"]["velocity_rad_s"] = self._motors["right"]["velocity_rad_s"] = 0.0
        self._event(event, level)
    def _watchdog_loop(self):
        while not self._closed.wait(.05):
            with self._lock:
                now = time.monotonic()
                moving_without_fresh_input = (
                    (self._control_mode == "webui" and (self._target_linear or self._target_angular))
                    or (self._control_mode == "ros2" and (self._linear or self._angular))
                )
                if self._enabled and moving_without_fresh_input and now - self._last_command > .75:
                    self.stop(require_token=False)
                    self._event("速度指令超时，看门狗已停车", "warning")
                if self._control_mode == "ros2" and self._ros2_armed and now - self._ros2_last_heartbeat > 3.0:
                    self.stop(require_token=False)
                    self._ros2_armed = False
                    self._event("ROS 2 桥接节点心跳中断，已停车并等待零速度指令", "warning")
    def _telemetry_loop(self):
        phase = 0.0
        while not self._closed.wait(.5):
            with self._lock:
                if self.simulate:
                    phase += .16; half = self.track_width_m / 2
                    left = (self._linear - self._angular * half) / self.wheel_radius_m; right = (self._linear + self._angular * half) / self.wheel_radius_m
                    self._motors["left"].update(online=True, velocity_rad_s=left, bus_voltage_v=47.8 + math.sin(phase)*.1, motor_temperature_c=31.4+abs(left)*.08)
                    self._motors["right"].update(online=True, velocity_rad_s=right, bus_voltage_v=47.9 + math.cos(phase)*.1, motor_temperature_c=30.9+abs(right)*.08)
                elif self._drive and not self._enabled:
                    for name, motor in (("left", self._drive.left), ("right", self._drive.right)):
                        try:
                            telemetry = motor.read_telemetry(timeout_s=.25)
                            self._motors[name].update(online=True, bus_voltage_v=telemetry.bus_voltage_v, pcb_temperature_c=telemetry.pcb_temperature_c, motor_temperature_c=telemetry.motor_temperature_c, position_rad=telemetry.motor_position_rad)
                        except Exception:
                            try:
                                self._motors[name]["online"] = motor.is_online(timeout_s=.25)
                            except Exception:
                                self._motors[name]["online"] = False
    def snapshot(self, presented_token=None):
        with self._control_lock:
            active = self._control_active(); owner = self._owner if active else None
            owned = active and bool(presented_token) and secrets.compare_digest(presented_token or "", self._token or "")
        with self._lock:
            return {"service_online": True, "simulation": self.simulate, "can_interface": self.interface, "can_state": "up" if (self.simulate or self._drive) else "down", "control": {"owned": owned, "owner": owner, "expires_in_s": None}, "control_mode": self._control_mode, "ros2": {"online": time.monotonic() - self._ros2_last_heartbeat <= 3.0, "armed": self._ros2_armed}, "drive": {"enabled": self._enabled, "emergency_stop": self._emergency, "linear_m_s": self._linear, "angular_rad_s": self._angular, "target_linear_m_s": self._target_linear, "target_angular_rad_s": self._target_angular, "ramp_time_s": self._ramp_time_s}, "motors": {k: dict(v) for k,v in self._motors.items()}, "updated_at": datetime.now().strftime("%H:%M:%S")}
    def events(self):
        with self._lock: return list(self._events)
    def close(self):
        self._closed.set()
        with self._lock:
            self._safe_disable("底盘服务已停止")
            if self._transport: self._transport.shutdown()

def make_handler(service, static_dir):
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs): super().__init__(*args, directory=str(static_dir), **kwargs)
        def _json(self, payload, status=HTTPStatus.OK):
            body = json.dumps(payload, ensure_ascii=False).encode(); self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8"); self.send_header("Content-Length", str(len(body))); self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(body)
        def _body(self):
            length = int(self.headers.get("Content-Length", 0))
            if length > 16384: raise ServiceError("请求体过大", HTTPStatus.REQUEST_ENTITY_TOO_LARGE)
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
                if not isinstance(body, dict): raise ServiceError("请求体必须是 JSON 对象")
                return body
            except json.JSONDecodeError as exc: raise ServiceError("JSON 格式无效") from exc
        def _local_only(self):
            if self.client_address[0] not in ("127.0.0.1", "::1"):
                raise ServiceError("ROS 2 桥接接口只允许 Jetson 本机访问", HTTPStatus.FORBIDDEN)
        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/api/state": return self._json(service.snapshot(self.headers.get("X-Control-Token")))
            if path == "/api/events": return self._json({"events": service.events()})
            if path.startswith("/api/"): return self._json({"error":"接口不存在"}, HTTPStatus.NOT_FOUND)
            if path != "/" and not (static_dir / path.lstrip("/")).exists(): self.path = "/"
            return super().do_GET()
        def do_POST(self):
            try:
                path, body = urlparse(self.path).path, self._body(); token = str(body.get("token", ""))
                if path == "/api/drive/command": result={"ok":True, **service.command(token,float(body.get("linear",0)),float(body.get("angular",0)),source="webui")}
                elif path == "/api/drive/stop": result={"ok":True,"stopped":service.stop(token,source="webui",graceful=True)}
                elif path == "/api/drive/ramp-time": result={"ok":True,"ramp_time_s":service.set_ramp_time(body.get("seconds"))}
                elif path == "/api/control-mode": result={"ok":True,"mode":service.set_control_mode(str(body.get("mode", "")))}
                elif path == "/api/ros2/heartbeat": self._local_only(); result={"ok":True,**service.ros2_heartbeat()}
                elif path == "/api/ros2/command":
                    self._local_only()
                    result={"ok":True,**service.command("",float(body.get("linear",0)),float(body.get("angular",0)),source="ros2")}
                elif path == "/api/ros2/stop": self._local_only(); result={"ok":True,"stopped":service.stop(require_token=False,source="ros2")}
                elif path == "/api/drive/emergency-stop": service.emergency_stop(); result={"ok":True}
                elif path == "/api/drive/reset-emergency": service.reset_emergency(token); result={"ok":True}
                else: raise ServiceError("接口不存在", HTTPStatus.NOT_FOUND)
                self._json(result)
            except ServiceError as exc: self._json({"error":str(exc)}, exc.status)
            except Exception as exc: service._event(f"请求处理失败：{exc}","error"); self._json({"error":str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)
    return Handler

def main():
    parser=argparse.ArgumentParser(description="DM-H65 chassis WebUI service")
    parser.add_argument("--host",default="0.0.0.0"); parser.add_argument("--port",type=int,default=8765); parser.add_argument("--interface",default="can1")
    parser.add_argument("--simulate",action="store_true"); parser.add_argument("--wheel-radius",type=float,default=.0825); parser.add_argument("--track-width",type=float,default=.42)
    parser.add_argument("--static-dir", type=Path, default=HERE/"webui_static"); args=parser.parse_args()
    static_dir=args.static_dir.resolve()
    if not (static_dir / "index.html").exists(): raise SystemExit(f"WebUI static files not found: {static_dir}")
    service=ChassisService(args.interface,args.simulate,args.wheel_radius,args.track_width); server=ThreadingHTTPServer((args.host,args.port),make_handler(service,static_dir))
    print(f"DM-H65 WebUI: http://{args.host}:{args.port} ({'simulation' if args.simulate else args.interface})")
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close(); service.close()
if __name__ == "__main__": main()
