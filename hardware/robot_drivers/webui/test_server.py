import time
import unittest

from server import ChassisService, ServiceError


class ChassisServiceTests(unittest.TestCase):
    def setUp(self):
        self.service = ChassisService(
            simulate=True,
            max_linear_accel_m_s2=10.0,
            max_angular_accel_rad_s2=20.0,
            control_hz=100.0,
        )
        self.token = self.service.acquire("test-client")["token"]

    def tearDown(self):
        self.service.close()

    def _wait_until(self, predicate, timeout_s=1.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        self.fail("condition not met before timeout")

    def test_control_enable_command_stop(self):
        self.service.enable(self.token)
        self.service.command(self.token, 0.2, 0.4)
        self._wait_until(lambda: abs(self.service.snapshot(self.token)["drive"]["linear_m_s"] - 0.2) < 1e-3)
        state = self.service.snapshot(self.token)
        self.assertTrue(state["drive"]["enabled"])
        self.assertAlmostEqual(state["drive"]["linear_m_s"], 0.2, places=3)
        self.assertAlmostEqual(state["drive"]["target_linear_m_s"], 0.2, places=3)
        self.service.stop(self.token)
        self._wait_until(lambda: abs(self.service.snapshot(self.token)["drive"]["linear_m_s"]) < 1e-3)
        self.assertEqual(self.service.snapshot(self.token)["drive"]["linear_m_s"], 0)

    def test_command_is_slewed(self):
        slow = ChassisService(
            simulate=True,
            max_linear_accel_m_s2=0.4,
            max_angular_accel_rad_s2=1.0,
            control_hz=50.0,
            command_timeout_s=5.0,
        )
        token = slow.acquire("slew-client")["token"]
        try:
            slow.enable(token)
            slow.command(token, 0.6, 0.0)
            time.sleep(0.25)
            mid = slow.snapshot(token)["drive"]["linear_m_s"]
            self.assertGreater(mid, 0.05)
            self.assertLess(mid, 0.55)

            def reached_target():
                slow.command(token, 0.6, 0.0)
                return abs(slow.snapshot(token)["drive"]["linear_m_s"] - 0.6) < 0.02

            self._wait_until(reached_target, timeout_s=3.0)
        finally:
            slow.close()

    def test_disable_soft_stops_before_cutting_enable(self):
        self.service.enable(self.token)
        self.service.command(self.token, 0.3, 0.0)
        self._wait_until(lambda: self.service.snapshot(self.token)["drive"]["linear_m_s"] > 0.2)
        self.service.disable(self.token)
        state = self.service.snapshot(self.token)
        self.assertTrue(state["drive"]["enabled"] or state["drive"]["soft_stopping"])
        self.assertAlmostEqual(state["drive"]["target_linear_m_s"], 0.0, places=3)
        self._wait_until(lambda: not self.service.snapshot(self.token)["drive"]["enabled"], timeout_s=2.0)
        final = self.service.snapshot(self.token)
        self.assertFalse(final["drive"]["enabled"])
        self.assertFalse(final["drive"]["soft_stopping"])
        self.assertEqual(final["drive"]["linear_m_s"], 0)

    def test_emergency_stop_latches(self):
        self.service.enable(self.token)
        self.service.emergency_stop()
        self.assertTrue(self.service.snapshot(self.token)["drive"]["emergency_stop"])
        with self.assertRaises(ServiceError):
            self.service.enable(self.token)

    def test_watchdog_disables_output(self):
        self.service.enable(self.token)
        self.service.command(self.token, 0.2, 0.0)
        self._wait_until(lambda: not self.service.snapshot(self.token)["drive"]["enabled"], timeout_s=2.0)
        self.assertFalse(self.service.snapshot(self.token)["drive"]["enabled"])

    def test_control_is_persistent_and_exclusive(self):
        self.assertIsNone(self.service.snapshot(self.token)["control"]["expires_in_s"])
        self.assertEqual(self.service.acquire("test-client")["token"], self.token)
        with self.assertRaises(ServiceError):
            self.service.acquire("another-client")
        self.service.heartbeat(self.token)
        self.assertTrue(self.service.snapshot(self.token)["control"]["owned"])


if __name__ == "__main__":
    unittest.main()
