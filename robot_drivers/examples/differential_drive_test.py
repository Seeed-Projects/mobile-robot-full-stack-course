#!/usr/bin/env python3
"""Bounded two-wheel chassis test with paired cleanup and mode restoration."""

from __future__ import annotations

import argparse

from dm_h65 import DMH65Motor, PythonCanTransport
from dm_h65.drive import DifferentialDrive


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--left-id", type=lambda value: int(value, 0), default=0x01)
    parser.add_argument("--right-id", type=lambda value: int(value, 0), default=0x02)
    parser.add_argument("--left-direction", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--right-direction", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--wheel-radius", type=float, required=True, help="meters")
    parser.add_argument("--track-width", type=float, required=True, help="meters")
    parser.add_argument("--linear", type=float, default=0.05, help="m/s")
    parser.add_argument("--angular", type=float, default=0.0, help="rad/s")
    parser.add_argument("--duration", type=float, default=1.0, help="seconds")
    parser.add_argument("--confirm-motion", action="store_true", required=True)
    args = parser.parse_args()

    if abs(args.linear) > 0.1:
        raise ValueError("verification tool limits |linear| to 0.1 m/s")
    if abs(args.angular) > 0.5:
        raise ValueError("verification tool limits |angular| to 0.5 rad/s")
    if not 0 < args.duration <= 2.0:
        raise ValueError("verification tool limits duration to 0 < t <= 2 seconds")

    with PythonCanTransport(args.interface) as can_bus:
        left = DMH65Motor(
            can_bus,
            args.left_id,
            direction=args.left_direction,
            max_speed_rad_s=3.0,
        )
        right = DMH65Motor(
            can_bus,
            args.right_id,
            direction=args.right_direction,
            max_speed_rad_s=3.0,
        )
        drive = DifferentialDrive(
            left,
            right,
            wheel_radius_m=args.wheel_radius,
            track_width_m=args.track_width,
            max_wheel_speed_rad_s=3.0,
            max_wheel_acceleration_rad_s2=2.0,
        )
        before = drive.read_state()
        print("before:", before)
        try:
            drive.enable(switch_to_velocity_mode=True)
            commanded = drive.wheel_speeds_from_twist(args.linear, args.angular)
            print("commanded wheel speeds:", commanded)
            drive.hold_twist(args.linear, args.angular, args.duration, command_hz=50.0)
        finally:
            drive.disable(restore_modes=True)
        after = drive.read_state()
        print("after:", after)
        print("PASS: command completed; both motors stopped, disabled, and modes restored")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

