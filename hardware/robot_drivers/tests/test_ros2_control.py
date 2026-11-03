import importlib.util
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from dm_h65.webui import ChassisService, ServiceError


BRIDGE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "ros2_cmd_vel_bridge.py"
spec = importlib.util.spec_from_file_location("ros2_cmd_vel_bridge", BRIDGE_PATH)
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


class ROS2ControlTests(unittest.TestCase):
    def setUp(self):
        self.service = ChassisService(simulate=True)

    def tearDown(self):
        self.service.close()

    def test_mode_switch_requires_bridge_and_zero_arming(self):
        with self.assertRaises(ServiceError):
            self.service.set_control_mode("ros2")
        self.service.ros2_heartbeat()
        self.service.set_control_mode("ros2")
        with self.assertRaises(ServiceError):
            self.service.command("", 0.1, 0.0)
        with self.assertRaises(ServiceError):
            self.service.command("", 0.1, 0.0, source="ros2")
        self.service.command("", 0.0, 0.0, source="ros2")
        self.assertFalse(self.service.snapshot()["drive"]["enabled"])
        self.service.command("", 0.1, 0.0, source="ros2")
        self.assertEqual(self.service.snapshot()["drive"]["linear_m_s"], 0.1)
        self.service.set_control_mode("webui")
        self.assertEqual(self.service.snapshot()["drive"]["linear_m_s"], 0.0)
        with self.assertRaises(ServiceError):
            self.service.command("", 0.1, 0.0, source="ros2")

    def test_watchdog_and_emergency(self):
        self.service.ros2_heartbeat()
        self.service.set_control_mode("ros2")
        self.service.command("", 0, 0, source="ros2")
        self.service.command("", 0.1, 0, source="ros2")
        self.service._last_command = time.monotonic() - 1
        time.sleep(0.12)
        self.assertEqual(self.service.snapshot()["drive"]["linear_m_s"], 0)
        self.service.emergency_stop()
        state = self.service.snapshot()
        self.assertEqual(state["control_mode"], "webui")
        self.assertFalse(state["ros2"]["armed"])

    def test_twist_validation_and_local_url(self):
        vec = lambda x=0, y=0, z=0: SimpleNamespace(x=x, y=y, z=z)
        msg = SimpleNamespace(linear=vec(0.2), angular=vec(z=0.5))
        self.assertEqual(bridge.twist_to_command(msg), {"linear": 0.2, "angular": 0.5})
        msg.linear.y = 0.1
        with self.assertRaises(ValueError):
            bridge.twist_to_command(msg)
        with self.assertRaises(ValueError):
            bridge.WebUIClient("http://192.168.3.168:8765")


if __name__ == "__main__":
    unittest.main()
