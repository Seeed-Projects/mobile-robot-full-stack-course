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
    """Serializes CAN access and enforces ownership, watchdog and e-stop."""
    def __init__(self, interface="can0", simulate=False, wheel_radius_m=0.0825, track_width_m=0.42):
        self.interface, self.simulate = interface, simulate
        self.wheel_radius_m, self.track_width_m = wheel_radius_m, track_width_m
        self._lock = threading.RLock(); self._control_lock = threading.RLock(); self._events = deque(maxlen=100)
        self._token = self._owner = None
        self._enabled = self._emergency = False; self._linear = self._angular = 0.0; self._last_command = 0.0
        self._transport = self._drive = None; self._closed = threading.Event()
        self._motors = {"left": self._blank_motor(1), "right": self._blank_motor(2)}
        self._connect()
        threading.Thread(target=self._watchdog_loop, name="chassis-watchdog", daemon=True).start()
        threading.Thread(target=self._telemetry_loop, name="chassis-telemetry", daemon=True).start()

    @staticmethod
    def _blank_motor(motor_id):
        return {"id": motor_id, "online": False, "velocity_rad_s": 0.0, "bus_voltage_v": 0.0, "motor_temperature_c": 0.0, "pcb_temperature_c": 0.0, "position_rad": 0.0}

    def _event(self, message, level="info"):
        self._events.appendleft({"time": datetime.now().strftime("%H:%M:%S"), "message": message, "level": level})

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
        with self._control_lock:
            if not self._control_active() or not token or not secrets.compare_digest(token, self._token or ""):
                raise ServiceError("控制权无效，请重新取得控制权", HTTPStatus.CONFLICT)

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
    def command(self, token, linear, angular):
        if not math.isfinite(linear) or not math.isfinite(angular): raise ServiceError("速度必须是有限数值")
        if abs(linear) > .6 or abs(angular) > 2: raise ServiceError("速度超过服务端安全上限")
        with self._lock:
            self._require_token(token)
            if self._emergency or not self._enabled: raise ServiceError("电机未启用或急停已锁定", HTTPStatus.CONFLICT)
            speeds = self._drive.set_twist(linear, angular) if self._drive else None
            self._linear, self._angular, self._last_command = linear, angular, time.monotonic()
            if speeds:
                self._motors["left"]["velocity_rad_s"] = speeds.left_rad_s
                self._motors["right"]["velocity_rad_s"] = speeds.right_rad_s
            return {"linear_m_s": linear, "angular_rad_s": angular,
                    "left_rad_s": speeds.left_rad_s if speeds else 0.0,
                    "right_rad_s": speeds.right_rad_s if speeds else 0.0}
    def stop(self, token=None, require_token=True):
        with self._lock:
            if require_token: self._require_token(token)
            if self._drive: self._drive.stop()
            self._linear = self._angular = 0.0
            self._motors["left"]["velocity_rad_s"] = self._motors["right"]["velocity_rad_s"] = 0.0
    def disable(self, token):
        with self._lock: self._require_token(token); self._safe_disable("电机输出已禁用")
    def emergency_stop(self):
        with self._lock: self._safe_disable("急停已触发", "error"); self._emergency = True
        with self._control_lock: self._token = self._owner = None
    def reset_emergency(self, token):
        with self._lock: self._require_token(token); self._emergency = False; self._event("急停已复位，电机仍保持禁用", "warning")
    def _safe_disable(self, event, level="info"):
        if self._drive:
            try: self._drive.disable(restore_modes=False)
            except Exception as exc: self._event(f"停止电机时发生错误：{exc}", "error")
        self._enabled = False; self._linear = self._angular = 0.0
        self._motors["left"]["velocity_rad_s"] = self._motors["right"]["velocity_rad_s"] = 0.0
        self._event(event, level)
    def _watchdog_loop(self):
        while not self._closed.wait(.05):
            with self._lock:
                now = time.monotonic()
                if self._enabled and now - self._last_command > .75: self._safe_disable("速度指令超时，看门狗已停止并禁用电机", "error")
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
                    try:
                        chassis = self._drive.read_state(timeout_s=.08)
                        for name, telemetry in (("left", chassis.left), ("right", chassis.right)):
                            self._motors[name].update(online=True, bus_voltage_v=telemetry.bus_voltage_v, pcb_temperature_c=telemetry.pcb_temperature_c, motor_temperature_c=telemetry.motor_temperature_c, position_rad=telemetry.motor_position_rad)
                    except Exception:
                        self._motors["left"]["online"] = self._motors["right"]["online"] = False
    def snapshot(self, presented_token=None):
        with self._control_lock:
            active = self._control_active(); owner = self._owner if active else None
            owned = active and bool(presented_token) and secrets.compare_digest(presented_token or "", self._token or "")
        with self._lock:
            return {"service_online": True, "simulation": self.simulate, "can_interface": self.interface, "can_state": "up" if (self.simulate or self._drive) else "down", "control": {"owned": owned, "owner": owner, "expires_in_s": None}, "drive": {"enabled": self._enabled, "emergency_stop": self._emergency, "linear_m_s": self._linear, "angular_rad_s": self._angular}, "motors": {k: dict(v) for k,v in self._motors.items()}, "updated_at": datetime.now().strftime("%H:%M:%S")}
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
            try: return json.loads(self.rfile.read(length) or b"{}")
            except json.JSONDecodeError as exc: raise ServiceError("JSON 格式无效") from exc
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
                if path == "/api/control/acquire": result = service.acquire(str(body.get("owner", "webui")))
                elif path == "/api/control/heartbeat": service.heartbeat(token); result={"ok":True}
                elif path == "/api/control/release": service.release(token); result={"ok":True}
                elif path == "/api/drive/enable": service.enable(token); result={"ok":True}
                elif path == "/api/drive/disable": service.disable(token); result={"ok":True}
                elif path == "/api/drive/command": result={"ok":True, **service.command(token,float(body.get("linear",0)),float(body.get("angular",0)))}
                elif path == "/api/drive/stop": service.stop(token); result={"ok":True}
                elif path == "/api/drive/emergency-stop": service.emergency_stop(); result={"ok":True}
                elif path == "/api/drive/reset-emergency": service.reset_emergency(token); result={"ok":True}
                else: raise ServiceError("接口不存在", HTTPStatus.NOT_FOUND)
                self._json(result)
            except ServiceError as exc: self._json({"error":str(exc)}, exc.status)
            except Exception as exc: service._event(f"请求处理失败：{exc}","error"); self._json({"error":str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)
    return Handler

def main():
    parser=argparse.ArgumentParser(description="DM-H65 chassis WebUI service")
    parser.add_argument("--host",default="0.0.0.0"); parser.add_argument("--port",type=int,default=8765); parser.add_argument("--interface",default="can0")
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


