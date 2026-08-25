import collections
import struct
import unittest

from dm_h65 import DMH65Motor, Register
from dm_h65.transport import CanFrame


def response(motor_id: int, register: Register, value: int | float) -> CanFrame:
    encoded = struct.pack("<I", value) if isinstance(value, int) else struct.pack("<f", value)
    return CanFrame(0, struct.pack("<HBB", motor_id, 0x33, register) + encoded)


class FakeTransport:
    def __init__(self, online_ids: set[int] | None = None) -> None:
        self.sent: list[CanFrame] = []
        self.incoming: collections.deque[CanFrame] = collections.deque()
        self.online_ids = online_ids or set()

    def send(self, frame: CanFrame, timeout: float | None = None) -> None:
        self.sent.append(frame)
        if (
            frame.arbitration_id == 0x7FF
            and len(frame.data) == 4
            and frame.data[2:] == bytes([0x33, Register.ESC_ID])
        ):
            motor_id = frame.data[0] | (frame.data[1] << 8)
            if motor_id in self.online_ids:
                self.incoming.append(response(motor_id, Register.ESC_ID, motor_id))

    def recv(self, timeout: float | None = None) -> CanFrame | None:
        return self.incoming.popleft() if self.incoming else None

    def shutdown(self) -> None:
        pass


class MotorTests(unittest.TestCase):
    def test_register_read(self) -> None:
        bus = FakeTransport()
        bus.incoming.append(response(1, Register.BUS_VOLTAGE, 48.5))
        motor = DMH65Motor(bus, 1)
        self.assertAlmostEqual(motor.read_register(Register.BUS_VOLTAGE), 48.5)

    def test_scan(self) -> None:
        bus = FakeTransport({1, 3})
        self.assertEqual(DMH65Motor.scan(bus, first_id=0, last_id=4), [1, 3])

    def test_velocity_direction_and_limit(self) -> None:
        bus = FakeTransport()
        motor = DMH65Motor(bus, 2, direction=-1, max_speed_rad_s=8.0)
        motor.set_velocity(5.0)
        self.assertEqual(bus.sent[-1].arbitration_id, 0x202)
        self.assertEqual(bus.sent[-1].data, struct.pack("<f", -5.0) + bytes(4))
        with self.assertRaises(ValueError):
            motor.set_velocity(9.0)

    def test_special_commands(self) -> None:
        bus = FakeTransport()
        motor = DMH65Motor(bus, 1)
        motor.enable()
        motor.stop()
        motor.disable()
        self.assertEqual([frame.arbitration_id for frame in bus.sent], [1, 0x201, 1])
        self.assertEqual(bus.sent[0].data[-1], 0xFC)
        self.assertEqual(bus.sent[-1].data[-1], 0xFD)


if __name__ == "__main__":
    unittest.main()

