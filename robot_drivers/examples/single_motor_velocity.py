#!/usr/bin/env python3
"""Explicit low-speed example. Secure and lift the wheel before running."""

import time

from dm_h65 import ControlMode, DMH65Motor, PythonCanTransport, Register


with PythonCanTransport("can0") as can_bus:
    motor = DMH65Motor(can_bus, 0x01, max_speed_rad_s=3.0)
    mode = int(motor.read_register(Register.CONTROL_MODE))
    if mode != ControlMode.VELOCITY:
        raise RuntimeError(f"expected velocity mode 3, motor reports {mode}")
    motor.stop()
    motor.enable()
    try:
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            motor.set_velocity(1.0)
            time.sleep(0.02)
    finally:
        motor.stop_and_disable()

