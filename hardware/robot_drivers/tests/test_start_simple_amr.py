import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "start_simple_amr.py"
spec = importlib.util.spec_from_file_location("start_simple_amr", SCRIPT)
startup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(startup)


class StartupTests(unittest.TestCase):
    def test_explicit_modes(self):
        self.assertEqual(startup.choose_mode(["webui"]), "webui")
        self.assertEqual(startup.choose_mode(["ros2"]), "ros2")

    def test_mode_defaults_to_webui(self):
        self.assertEqual(startup.choose_mode([]), "webui")

    def test_webui_mode_reuses_service_and_stops_bridge(self):
        current = {"service_online": True, "can_interface": "can1", "can_state": "up",
                   "simulation": False, "control_mode": "ros2", "drive": {"linear_m_s": 0.0, "angular_rad_s": 0.0}}
        with patch.object(startup, "systemctl") as ctl, patch.object(startup, "state", return_value=current), \
             patch.object(startup, "api") as api:
            startup.main(["webui"])
        api.assert_called_once_with("/control-mode", {"mode": "webui"})
        self.assertEqual(ctl.call_args_list[-1].args, ("stop", startup.BRIDGE_UNIT))

    def test_ros2_mode_waits_for_bridge_and_zero_arming(self):
        current = {"service_online": True, "can_interface": "can1", "can_state": "up",
                   "simulation": False, "control_mode": "webui", "drive": {"linear_m_s": 0.0, "angular_rad_s": 0.0}}
        ready = {**current, "control_mode": "ros2", "ros2": {"online": True, "armed": True}}
        with patch.object(startup, "systemctl") as ctl, patch.object(startup, "state", side_effect=[current, ready]), \
             patch.object(startup, "wait_for", return_value=ready) as wait, patch.object(startup, "api") as api:
            startup.main(["ros2"])
        self.assertEqual(ctl.call_args_list[-1].args, ("start", startup.BRIDGE_UNIT))
        api.assert_called_once_with("/control-mode", {"mode": "ros2"})
        self.assertEqual(wait.call_count, 2)


if __name__ == "__main__":
    unittest.main()
