"""Safe synchronous differential-drive control for two DM-H65 motors."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .motor import DMH65Motor, MotorTelemetry
from .protocol import ControlMode, Register


@dataclass(frozen=True, slots=True)
class WheelSpeeds:
    left_rad_s: float
    right_rad_s: float


@dataclass(frozen=True, slots=True)
class ChassisState:
    left: MotorTelemetry
    right: MotorTelemetry


class DifferentialDrive:
    """Two-wheel differential drive with paired safety behavior.

    Motor direction is configured on each ``DMH65Motor``. For the currently
    tested mirrored installation, use left ``direction=-1`` and right
    ``direction=+1`` so positive chassis velocity means forward.
    """

    def __init__(
        self,
        left: DMH65Motor,
        right: DMH65Motor,
        *,
        wheel_radius_m: float,
        track_width_m: float,
        max_wheel_speed_rad_s: float | None = None,
        max_wheel_acceleration_rad_s2: float = 4.0,
    ) -> None:
        if left.motor_id == right.motor_id:
            raise ValueError("left and right motors must use different CAN IDs")
        if not math.isfinite(wheel_radius_m) or wheel_radius_m <= 0:
            raise ValueError("wheel_radius_m must be finite and positive")
        if not math.isfinite(track_width_m) or track_width_m <= 0:
            raise ValueError("track_width_m must be finite and positive")
        if max_wheel_speed_rad_s is None:
            max_wheel_speed_rad_s = min(left.max_speed_rad_s, right.max_speed_rad_s)
        if not math.isfinite(max_wheel_speed_rad_s) or max_wheel_speed_rad_s <= 0:
            raise ValueError("max_wheel_speed_rad_s must be finite and positive")
        if not math.isfinite(max_wheel_acceleration_rad_s2) or max_wheel_acceleration_rad_s2 <= 0:
            raise ValueError("max_wheel_acceleration_rad_s2 must be finite and positive")

        self.left = left
        self.right = right
        self.wheel_radius_m = float(wheel_radius_m)
        self.track_width_m = float(track_width_m)
        self.max_wheel_speed_rad_s = float(max_wheel_speed_rad_s)
        self.max_wheel_acceleration_rad_s2 = float(max_wheel_acceleration_rad_s2)
        self._enabled = False
        self._original_modes: dict[int, int] = {}
        self._last_command = WheelSpeeds(0.0, 0.0)

    @property
    def enabled(self) -> bool:
        return self._enabled

    def wheel_speeds_from_twist(self, linear_m_s: float, angular_rad_s: float) -> WheelSpeeds:
        if not math.isfinite(linear_m_s) or not math.isfinite(angular_rad_s):
            raise ValueError("linear and angular velocity must be finite")
        half_track = self.track_width_m / 2.0
        left = (linear_m_s - angular_rad_s * half_track) / self.wheel_radius_m
        right = (linear_m_s + angular_rad_s * half_track) / self.wheel_radius_m
        peak = max(abs(left), abs(right))
        if peak > self.max_wheel_speed_rad_s:
            scale = self.max_wheel_speed_rad_s / peak
            left *= scale
            right *= scale
        return WheelSpeeds(left, right)

    def enable(self, *, switch_to_velocity_mode: bool = True, timeout_s: float = 0.2) -> None:
        if self._enabled:
            return
        motors = (self.left, self.right)
        self._original_modes = {}
        try:
            for motor in motors:
                if not motor.is_online(timeout_s=timeout_s):
                    raise RuntimeError(f"motor 0x{motor.motor_id:02X} is offline")
                mode = int(motor.read_register(Register.CONTROL_MODE, timeout_s=timeout_s))
                self._original_modes[motor.motor_id] = mode
                if mode != int(ControlMode.VELOCITY):
                    if not switch_to_velocity_mode:
                        raise RuntimeError(
                            f"motor 0x{motor.motor_id:02X} is in mode {mode}, not velocity mode 3"
                        )
                    motor.select_velocity_mode(timeout_s=timeout_s)
                    confirmed = int(
                        motor.read_register(Register.CONTROL_MODE, timeout_s=timeout_s)
                    )
                    if confirmed != int(ControlMode.VELOCITY):
                        raise RuntimeError(
                            f"motor 0x{motor.motor_id:02X} failed to enter velocity mode"
                        )

            self._send_pair(WheelSpeeds(0.0, 0.0), require_enabled=False)
            time.sleep(0.05)
            self.left.enable()
            try:
                self.right.enable()
            except Exception:
                self.left.disable()
                raise
            self._enabled = True
            self._last_command = WheelSpeeds(0.0, 0.0)
        except Exception:
            self._best_effort_stop_and_disable()
            self._restore_modes(timeout_s=timeout_s)
            raise

    def _send_pair(self, speeds: WheelSpeeds, *, require_enabled: bool = True) -> None:
        if require_enabled and not self._enabled:
            raise RuntimeError("differential drive is not enabled")
        self.left.set_velocity(speeds.left_rad_s)
        try:
            self.right.set_velocity(speeds.right_rad_s)
        except Exception:
            self._best_effort_stop_and_disable()
            self._enabled = False
            raise
        self._last_command = speeds

    def set_wheel_speeds(self, left_rad_s: float, right_rad_s: float) -> WheelSpeeds:
        requested = WheelSpeeds(float(left_rad_s), float(right_rad_s))
        if not math.isfinite(requested.left_rad_s) or not math.isfinite(requested.right_rad_s):
            raise ValueError("wheel speeds must be finite")
        peak = max(abs(requested.left_rad_s), abs(requested.right_rad_s))
        if peak > self.max_wheel_speed_rad_s:
            scale = self.max_wheel_speed_rad_s / peak
            requested = WheelSpeeds(
                requested.left_rad_s * scale,
                requested.right_rad_s * scale,
            )
        self._send_pair(requested)
        return requested

    def set_twist(self, linear_m_s: float, angular_rad_s: float) -> WheelSpeeds:
        speeds = self.wheel_speeds_from_twist(linear_m_s, angular_rad_s)
        self._send_pair(speeds)
        return speeds

    @staticmethod
    def _approach(current: float, target: float, maximum_delta: float) -> float:
        delta = max(-maximum_delta, min(maximum_delta, target - current))
        return current + delta

    def hold_twist(
        self,
        linear_m_s: float,
        angular_rad_s: float,
        duration_s: float,
        *,
        command_hz: float = 50.0,
    ) -> None:
        if duration_s < 0 or command_hz <= 0:
            raise ValueError("duration_s must be >= 0 and command_hz must be > 0")
        if not self._enabled:
            raise RuntimeError("differential drive is not enabled")
        target = self.wheel_speeds_from_twist(linear_m_s, angular_rad_s)
        period = 1.0 / command_hz
        maximum_delta = self.max_wheel_acceleration_rad_s2 * period
        deadline = time.monotonic() + duration_s
        current = self._last_command
        try:
            while time.monotonic() < deadline:
                started = time.monotonic()
                current = WheelSpeeds(
                    self._approach(current.left_rad_s, target.left_rad_s, maximum_delta),
                    self._approach(current.right_rad_s, target.right_rad_s, maximum_delta),
                )
                self._send_pair(current)
                time.sleep(max(0.0, period - (time.monotonic() - started)))
        except BaseException:
            self._best_effort_stop_and_disable()
            self._enabled = False
            raise
        self.ramp_to_stop(command_hz=command_hz)

    def ramp_to_stop(self, *, command_hz: float = 50.0) -> None:
        if command_hz <= 0:
            raise ValueError("command_hz must be positive")
        period = 1.0 / command_hz
        maximum_delta = self.max_wheel_acceleration_rad_s2 * period
        current = self._last_command
        while abs(current.left_rad_s) > 1e-6 or abs(current.right_rad_s) > 1e-6:
            started = time.monotonic()
            current = WheelSpeeds(
                self._approach(current.left_rad_s, 0.0, maximum_delta),
                self._approach(current.right_rad_s, 0.0, maximum_delta),
            )
            self._send_pair(current)
            time.sleep(max(0.0, period - (time.monotonic() - started)))
        self._send_pair(WheelSpeeds(0.0, 0.0))

    def stop(self) -> None:
        self._send_pair(WheelSpeeds(0.0, 0.0), require_enabled=False)

    def _best_effort_stop_and_disable(self) -> None:
        for action in (
            self.left.stop,
            self.right.stop,
            self.left.disable,
            self.right.disable,
        ):
            try:
                action()
            except Exception:
                pass

    def _restore_modes(self, *, timeout_s: float) -> None:
        for motor in (self.left, self.right):
            original = self._original_modes.get(motor.motor_id)
            if original is None or original == int(ControlMode.VELOCITY):
                continue
            try:
                motor.write_register(Register.CONTROL_MODE, original, timeout_s=timeout_s)
            except Exception:
                pass

    def disable(self, *, restore_modes: bool = True, timeout_s: float = 0.2) -> None:
        errors: list[Exception] = []
        for action in (
            self.left.stop,
            self.right.stop,
            self.left.disable,
            self.right.disable,
        ):
            try:
                action()
            except Exception as exc:
                errors.append(exc)
        self._enabled = False
        self._last_command = WheelSpeeds(0.0, 0.0)
        if restore_modes:
            for motor in (self.left, self.right):
                original = self._original_modes.get(motor.motor_id)
                if original is None or original == int(ControlMode.VELOCITY):
                    continue
                try:
                    motor.write_register(Register.CONTROL_MODE, original, timeout_s=timeout_s)
                except Exception as exc:
                    errors.append(exc)
        if errors:
            raise errors[0]

    def read_state(self, *, timeout_s: float = 0.1) -> ChassisState:
        return ChassisState(
            left=self.left.read_telemetry(timeout_s=timeout_s),
            right=self.right.read_telemetry(timeout_s=timeout_s),
        )

    def __enter__(self) -> "DifferentialDrive":
        self.enable()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.disable()

