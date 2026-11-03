import threading
import unittest

from dm_h65.drive import WheelSpeeds
from dm_h65.webui import ChassisService


class FakeDrive:
    def __init__(self):
        self.commands = []

    def set_twist(self, linear, angular):
        self.commands.append((linear, angular))
        return WheelSpeeds(linear, angular)


class WebUIDirectionTests(unittest.TestCase):
    def setUp(self):
        self.service = ChassisService.__new__(ChassisService)
        self.service._lock = threading.RLock()
        self.service._enabled = True
        self.service._emergency = False
        self.service._drive = FakeDrive()
        self.service._motors = {"left": {}, "right": {}}
        self.service._linear = self.service._angular = 0.0
        self.service._last_command = 0.0
        self.service._control_mode = "webui"
        self.service._ros2_last_heartbeat = 0.0
        self.service._ros2_armed = False

    def test_forward_input_reverses_installed_motor_frame(self):
        result = self.service.command("", 0.2, 0.0)
        self.assertEqual(self.service._drive.commands[-1], (-0.2, 0.0))
        self.assertEqual(result["linear_m_s"], 0.2)

    def test_positive_left_and_negative_right_yaw(self):
        self.service.command("", 0.0, 0.8)
        self.service.command("", 0.0, -0.8)
        self.assertEqual(self.service._drive.commands, [(-0.0, 0.8), (-0.0, -0.8)])


if __name__ == "__main__":
    unittest.main()
