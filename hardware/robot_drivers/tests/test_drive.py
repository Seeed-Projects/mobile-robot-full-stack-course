import unittest

from dm_h65.drive import DifferentialDrive, WheelSpeeds
from dm_h65.motor import MotorTelemetry


class FakeMotor:
    def __init__(self, motor_id: int) -> None:
        self.motor_id = motor_id
        self.max_speed_rad_s = 10.0
        self.commands: list[float] = []
        self.enabled = False
        self.mode = 3

    def is_online(self, *, timeout_s: float = 0.1) -> bool:
        return True

    def read_register(self, register, *, timeout_s: float = 0.1):
        return self.mode

    def select_velocity_mode(self, *, timeout_s: float = 0.1) -> None:
        self.mode = 3

    def write_register(self, register, value, *, timeout_s: float = 0.1):
        self.mode = int(value)
        return self.mode

    def set_velocity(self, value: float) -> None:
        self.commands.append(value)

    def stop(self) -> None:
        self.commands.append(0.0)

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False

    def read_telemetry(self, *, timeout_s: float = 0.1) -> MotorTelemetry:
        return MotorTelemetry(48.0, 30.0, 25.0, 0.0)


class DifferentialDriveTests(unittest.TestCase):
    def make_drive(self, *, max_speed: float = 10.0) -> DifferentialDrive:
        return DifferentialDrive(
            FakeMotor(1),
            FakeMotor(2),
            wheel_radius_m=0.1,
            track_width_m=0.4,
            max_wheel_speed_rad_s=max_speed,
            max_wheel_acceleration_rad_s2=10.0,
        )

    def test_forward_kinematics(self) -> None:
        drive = self.make_drive()
        self.assertEqual(drive.wheel_speeds_from_twist(0.5, 0.0), WheelSpeeds(5.0, 5.0))

    def test_turn_kinematics(self) -> None:
        drive = self.make_drive()
        speeds = drive.wheel_speeds_from_twist(0.0, 1.0)
        self.assertAlmostEqual(speeds.left_rad_s, -2.0)
        self.assertAlmostEqual(speeds.right_rad_s, 2.0)

    def test_speed_saturation_preserves_ratio(self) -> None:
        drive = self.make_drive(max_speed=5.0)
        speeds = drive.wheel_speeds_from_twist(1.0, 1.0)
        self.assertAlmostEqual(max(abs(speeds.left_rad_s), abs(speeds.right_rad_s)), 5.0)
        self.assertAlmostEqual(speeds.left_rad_s / speeds.right_rad_s, 2.0 / 3.0)

    def test_enable_command_disable(self) -> None:
        drive = self.make_drive()
        drive.enable()
        self.assertTrue(drive.enabled)
        drive.set_twist(0.1, 0.0)
        self.assertAlmostEqual(drive.left.commands[-1], 1.0)
        self.assertAlmostEqual(drive.right.commands[-1], 1.0)
        drive.disable()
        self.assertFalse(drive.enabled)
        self.assertFalse(drive.left.enabled)
        self.assertFalse(drive.right.enabled)

    def test_rejects_duplicate_motor_id(self) -> None:
        with self.assertRaises(ValueError):
            DifferentialDrive(
                FakeMotor(1),
                FakeMotor(1),
                wheel_radius_m=0.1,
                track_width_m=0.4,
            )


if __name__ == "__main__":
    unittest.main()

