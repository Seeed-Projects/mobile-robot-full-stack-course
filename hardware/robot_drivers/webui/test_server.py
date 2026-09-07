import time
import unittest
from server import ChassisService, ServiceError
class ChassisServiceTests(unittest.TestCase):
    def setUp(self): self.service=ChassisService(simulate=True); self.token=self.service.acquire("test-client")["token"]
    def tearDown(self): self.service.close()
    def test_control_enable_command_stop(self):
        self.service.enable(self.token); self.service.command(self.token,.2,.4); state=self.service.snapshot(self.token)
        self.assertTrue(state["drive"]["enabled"]); self.assertAlmostEqual(state["drive"]["linear_m_s"],.2); self.service.stop(self.token); self.assertEqual(self.service.snapshot(self.token)["drive"]["linear_m_s"],0)
    def test_emergency_stop_latches(self):
        self.service.enable(self.token); self.service.emergency_stop(); self.assertTrue(self.service.snapshot(self.token)["drive"]["emergency_stop"])
        with self.assertRaises(ServiceError): self.service.enable(self.token)
    def test_watchdog_disables_output(self):
        self.service.enable(self.token); time.sleep(.9); self.assertFalse(self.service.snapshot(self.token)["drive"]["enabled"])
    def test_control_is_persistent_and_exclusive(self):
        self.assertIsNone(self.service.snapshot(self.token)["control"]["expires_in_s"])
        self.assertEqual(self.service.acquire("test-client")["token"], self.token)
        with self.assertRaises(ServiceError): self.service.acquire("another-client")
        self.service.heartbeat(self.token)
        self.assertTrue(self.service.snapshot(self.token)["control"]["owned"])
if __name__ == "__main__": unittest.main()
