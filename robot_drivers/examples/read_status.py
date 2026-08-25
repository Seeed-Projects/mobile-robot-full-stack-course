#!/usr/bin/env python3
"""Discover motors and read identity/telemetry without causing motion."""

from dm_h65 import DMH65Motor, PythonCanTransport


with PythonCanTransport("can0") as can_bus:
    found = DMH65Motor.scan(can_bus, first_id=0, last_id=15)
    print("found:", [f"0x{motor_id:02X}" for motor_id in found])
    for motor_id in found:
        motor = DMH65Motor(can_bus, motor_id)
        print(motor.read_identity())
        print(motor.read_telemetry())

