"""Command-line diagnostics for the DM-H65 Python SDK."""

from __future__ import annotations

import argparse
import signal
import sys
import time

from .motor import DMH65Motor
from .protocol import ControlMode, Register
from .transport import open_socketcan


def _integer(text: str) -> int:
    return int(text, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="dm-h65")
    parser.add_argument("--interface", default="can0", help="SocketCAN interface")
    subparsers = parser.add_subparsers(dest="command", required=True)

    scan = subparsers.add_parser("scan", help="read-only scan for motor IDs")
    scan.add_argument("--first", type=_integer, default=0)
    scan.add_argument("--last", type=_integer, default=15)

    status = subparsers.add_parser("status", help="read identity and telemetry")
    status.add_argument("motor_id", type=_integer)

    monitor = subparsers.add_parser("monitor", help="listen for normal feedback frames")
    monitor.add_argument("motor_id", type=_integer)
    monitor.add_argument("--seconds", type=float, default=10.0)

    speed = subparsers.add_parser("set-speed", help="explicit single-motor motion test")
    speed.add_argument("motor_id", type=_integer)
    speed.add_argument("velocity", type=float, help="rad/s")
    speed.add_argument("seconds", type=float)
    speed.add_argument("--max-speed", type=float, default=12.0)
    speed.add_argument("--confirm-motion", action="store_true", required=True)
    return parser


def _print_status(motor: DMH65Motor) -> None:
    identity = motor.read_identity()
    telemetry = motor.read_telemetry()
    print(f"motor_id: 0x{identity.esc_id:X}")
    print(f"master_id: 0x{identity.master_id:X}")
    print(f"control_mode: {identity.control_mode}")
    print(f"software_version_raw: 0x{identity.software_version:X}")
    print(f"boot_version_raw: 0x{identity.boot_version:X}")
    print(f"can_bitrate_code: {identity.can_bitrate_code}")
    print(f"bus_voltage_v: {telemetry.bus_voltage_v:.3f}")
    print(f"pcb_temperature_c: {telemetry.pcb_temperature_c:.3f}")
    print(f"motor_temperature_c: {telemetry.motor_temperature_c:.3f}")
    print(f"motor_position_rad: {telemetry.motor_position_rad:.3f}")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        with open_socketcan(args.interface) as transport:
            if args.command == "scan":
                motors = DMH65Motor.scan(transport, first_id=args.first, last_id=args.last)
                for motor_id in motors:
                    print(f"found motor 0x{motor_id:X}")
                return 0 if motors else 1

            motor = DMH65Motor(
                transport,
                args.motor_id,
                max_speed_rad_s=getattr(args, "max_speed", 12.0),
            )
            if args.command == "status":
                _print_status(motor)
                return 0
            if args.command == "monitor":
                deadline = time.monotonic() + args.seconds
                while time.monotonic() < deadline:
                    state = motor.receive_feedback(timeout_s=0.25)
                    if state:
                        print(state)
                return 0
            if args.command == "set-speed":
                if args.seconds < 0:
                    raise ValueError("seconds must be non-negative")
                mode = int(motor.read_register(Register.CONTROL_MODE))
                if mode != ControlMode.VELOCITY:
                    raise RuntimeError(
                        f"motor reports control mode {mode}, not velocity mode 3; refusing motion"
                    )
                interrupted = False

                def request_stop(signum: int, frame: object) -> None:
                    nonlocal interrupted
                    interrupted = True

                signal.signal(signal.SIGINT, request_stop)
                signal.signal(signal.SIGTERM, request_stop)
                deadline = time.monotonic() + args.seconds
                motor.stop()
                motor.enable()
                try:
                    while not interrupted and time.monotonic() < deadline:
                        motor.set_velocity(args.velocity)
                        time.sleep(0.02)
                finally:
                    motor.stop_and_disable()
                return 0
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 2


if __name__ == "__main__":
    raise SystemExit(main())

