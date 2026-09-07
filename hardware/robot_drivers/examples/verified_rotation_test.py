#!/usr/bin/env python3
"""Short, bounded DM-H65 rotation test with cleanup and mode restoration."""

from __future__ import annotations

import argparse
import math
import time

from dm_h65 import ControlMode, DMH65Motor, PythonCanTransport, Register


def wrapped_delta(after: float, before: float) -> float:
    return (after - before + math.pi) % (2.0 * math.pi) - math.pi


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--motor-id", type=lambda value: int(value, 0), default=1)
    parser.add_argument("--velocity", type=float, default=0.5)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--confirm-motion", action="store_true", required=True)
    args = parser.parse_args()

    if not 0 < abs(args.velocity) <= 1.0:
        raise ValueError("this verification tool limits velocity to 0 < |v| <= 1 rad/s")
    if not 0 < args.duration <= 2.0:
        raise ValueError("this verification tool limits duration to 0 < t <= 2 seconds")

    with PythonCanTransport(args.interface) as transport:
        motor = DMH65Motor(transport, args.motor_id, max_speed_rad_s=1.0)
        identity = motor.read_identity()
        before = motor.read_telemetry()
        if not 20.0 <= before.bus_voltage_v <= 65.0:
            raise RuntimeError(f"unsafe bus voltage: {before.bus_voltage_v:.3f} V")
        if before.pcb_temperature_c >= 90.0 or before.motor_temperature_c >= 80.0:
            raise RuntimeError(
                "temperature too high: "
                f"PCB={before.pcb_temperature_c:.1f} C, motor={before.motor_temperature_c:.1f} C"
            )

        original_mode = identity.control_mode
        mode_changed = original_mode != int(ControlMode.VELOCITY)
        print(
            f"preflight: id=0x{identity.esc_id:X} mode={original_mode} "
            f"voltage={before.bus_voltage_v:.3f}V "
            f"pcb={before.pcb_temperature_c:.1f}C motor={before.motor_temperature_c:.1f}C "
            f"position={before.motor_position_rad:.6f}rad"
        )

        feedback_samples = []
        motion_started = False
        try:
            if mode_changed:
                motor.select_velocity_mode(timeout_s=0.2)
                confirmed_mode = int(motor.read_register(Register.CONTROL_MODE, timeout_s=0.2))
                if confirmed_mode != int(ControlMode.VELOCITY):
                    raise RuntimeError(f"velocity mode switch failed, read back {confirmed_mode}")
                print("temporary RAM mode switch: velocity mode 3 confirmed")

            motor.stop()
            time.sleep(0.05)
            motor.enable()
            motion_started = True
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                cycle_started = time.monotonic()
                motor.set_velocity(args.velocity)
                state = motor.receive_feedback(timeout_s=0.01)
                if state is not None:
                    feedback_samples.append(state)
                    if state.error not in (0, 1):
                        raise RuntimeError(f"motor feedback error code: 0x{state.error:X}")
                time.sleep(max(0.0, 0.02 - (time.monotonic() - cycle_started)))
        finally:
            if motion_started:
                try:
                    motor.stop()
                    time.sleep(0.15)
                finally:
                    motor.disable()
            if mode_changed:
                motor.write_register(Register.CONTROL_MODE, original_mode, timeout_s=0.2)
                restored = int(motor.read_register(Register.CONTROL_MODE, timeout_s=0.2))
                if restored != original_mode:
                    raise RuntimeError(f"failed to restore original control mode {original_mode}")
                print(f"original RAM mode restored: {restored}")

        after = motor.read_telemetry()
        delta = wrapped_delta(after.motor_position_rad, before.motor_position_rad)
        expected_direction = math.copysign(1.0, args.velocity)
        rotation_confirmed = abs(delta) >= 0.05
        direction_matches = rotation_confirmed and math.copysign(1.0, delta) == expected_direction
        print(
            f"result: position_before={before.motor_position_rad:.6f}rad "
            f"position_after={after.motor_position_rad:.6f}rad delta={delta:.6f}rad "
            f"feedback_samples={len(feedback_samples)}"
        )
        print(
            f"postflight: voltage={after.bus_voltage_v:.3f}V "
            f"pcb={after.pcb_temperature_c:.1f}C motor={after.motor_temperature_c:.1f}C"
        )
        if not rotation_confirmed:
            raise RuntimeError("position feedback did not confirm rotation")
        if not direction_matches:
            print("NOTE: encoder direction is opposite the command; configure this motor with direction=-1")
        print("PASS: rotation confirmed; motor stopped, disabled, and original mode restored")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())

