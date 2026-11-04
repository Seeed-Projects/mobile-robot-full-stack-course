from collections import deque
import threading
import tempfile
import unittest
from pathlib import Path

from dm_h65.drive import WheelSpeeds
from dm_h65.webui import ChassisService, ServiceError


class FakeDrive:
    def __init__(self):
        self.commands = []
        self.disabled = False

    def set_twist(self, linear, angular):
        self.commands.append((linear, angular))
        return WheelSpeeds(linear, angular)

    def stop(self):
        self.commands.append((0.0, 0.0))

    def disable(self, **kwargs):
        self.disabled = True


class WebUIDirectionTests(unittest.TestCase):
    def setUp(self):
        self.service = ChassisService.__new__(ChassisService)
        self.service._lock = threading.RLock()
        self.service._control_lock = threading.RLock()
        self.service._token = self.service._owner = None
        self.service._events = deque(maxlen=100)
        self.service._enabled = True
        self.service._emergency = False
        self.service._drive = FakeDrive()
        self.service._motors = {"left": {"velocity_rad_s": 0.0},
                                "right": {"velocity_rad_s": 0.0}}
        self.service._linear = self.service._angular = 0.0
        self.service._target_linear = self.service._target_angular = 0.0
        self.service._ramp_time_s = 0.8
        self.service._last_command = 0.0
        self.service._control_mode = "webui"
        self.service._ros2_last_heartbeat = 0.0
        self.service._ros2_armed = False

    def test_forward_input_reverses_installed_motor_frame(self):
        result = self.service.command("", 0.2, 0.0)
        self.assertEqual(result["linear_m_s"], 0.2)
        self.service._motion_step(0.05)
        self.assertLess(self.service._drive.commands[-1][0], 0)
        self.assertLess(abs(self.service._drive.commands[-1][0]), 0.2)

    def test_positive_left_and_negative_right_yaw(self):
        self.service.command("", 0.0, 0.8)
        self.service._motion_step(0.05)
        self.service.command("", 0.0, -0.8)
        for _ in range(10):
            self.service._motion_step(0.05)
        self.assertGreater(self.service._drive.commands[0][1], 0)
        self.assertLess(self.service._drive.commands[-1][1], 0)

    def test_release_ramps_down_instead_of_hard_stop(self):
        self.service.command("", 0.6, 0.0)
        for _ in range(8):
            self.service._motion_step(0.05)
        moving = self.service._linear
        self.assertGreater(moving, 0)
        self.service.stop(graceful=True, source="webui")
        self.assertEqual(self.service._target_linear, 0)
        self.assertEqual(self.service._linear, moving)
        self.assertEqual(len(self.service._drive.commands), 8)
        self.service._motion_step(0.05)
        self.assertGreater(self.service._linear, 0)
        self.assertLess(self.service._linear, moving)
        for _ in range(20):
            self.service._motion_step(0.05)
        self.assertEqual(self.service._linear, 0)
        self.assertEqual(self.service._drive.commands[-1][0], 0)

    def test_emergency_stop_bypasses_ramp(self):
        self.service.command("", 0.6, 0.0)
        self.service._motion_step(0.05)
        self.assertGreater(self.service._linear, 0)
        self.service.emergency_stop()
        self.assertEqual(self.service._linear, 0)
        self.assertEqual(self.service._target_linear, 0)
        self.assertTrue(self.service._drive.disabled)

    def test_ramp_setting_is_validated_and_persisted(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "drive.json"
            self.service._settings_path = path
            self.assertEqual(self.service.set_ramp_time(1.2), 1.2)
            self.assertEqual(path.read_text().strip(), '{"ramp_time_s": 1.2}')
            with self.assertRaises(ServiceError):
                self.service.set_ramp_time(3.0)
            self.assertEqual(self.service._ramp_time_s, 1.2)
            self.service.command("", 0.2, 0)
            with self.assertRaises(ServiceError):
                self.service.set_ramp_time(0.5)


if __name__ == "__main__":
    unittest.main()
