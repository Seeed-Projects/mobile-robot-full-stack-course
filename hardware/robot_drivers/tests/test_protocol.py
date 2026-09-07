import math
import struct
import unittest

from dm_h65.protocol import (
    ControlMode,
    Register,
    control_frame_id,
    decode_feedback,
    decode_register_response,
    make_enable,
    make_parameter_read,
    make_velocity_command,
    rpm_to_rad_s,
)
from dm_h65.transport import CanFrame


class ProtocolTests(unittest.TestCase):
    def test_parameter_read(self) -> None:
        frame = make_parameter_read(1, Register.ESC_ID)
        self.assertEqual(frame.arbitration_id, 0x7FF)
        self.assertEqual(frame.data, bytes.fromhex("01003308"))

    def test_velocity_frame(self) -> None:
        frame = make_velocity_command(2, -5.0)
        self.assertEqual(frame.arbitration_id, control_frame_id(2, ControlMode.VELOCITY))
        self.assertEqual(frame.data, struct.pack("<f", -5.0) + bytes(4))

    def test_enable(self) -> None:
        self.assertEqual(make_enable(1).data, bytes([0xFF] * 7 + [0xFC]))

    def test_register_decode(self) -> None:
        frame = CanFrame(0, bytes.fromhex("0100330801000000"))
        response = decode_register_response(frame, uint32=True)
        self.assertEqual(response.motor_id, 1)
        self.assertEqual(response.register, Register.ESC_ID)
        self.assertEqual(response.value, 1)

    def test_feedback_decode(self) -> None:
        state = decode_feedback(CanFrame(0, bytes([1, 0x80, 0, 0x80, 0x08, 0, 53, 26])))
        self.assertEqual(state.motor_id, 1)
        self.assertEqual(state.error, 0)
        self.assertAlmostEqual(state.position_rad, 0.0, delta=0.01)
        self.assertAlmostEqual(state.velocity_rad_s, 0.0, delta=0.1)
        self.assertEqual(state.pcb_temperature_c, 53.0)

    def test_rpm_conversion(self) -> None:
        self.assertAlmostEqual(rpm_to_rad_s(60), 2 * math.pi)


if __name__ == "__main__":
    unittest.main()

