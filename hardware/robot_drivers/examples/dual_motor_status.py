#!/usr/bin/env python3
"""Read-only status check for the configured left and right wheel motors."""

from dm_h65 import DMH65Motor, PythonCanTransport


with PythonCanTransport("can0") as can_bus:
    left = DMH65Motor(can_bus, 0x01, direction=-1)
    right = DMH65Motor(can_bus, 0x02, direction=1)
    for name, motor in (("left", left), ("right", right)):
        print(name, motor.read_identity())
        print(name, motor.read_telemetry())

