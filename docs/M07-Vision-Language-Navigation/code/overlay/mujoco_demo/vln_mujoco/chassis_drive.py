"""DM-H65 command contract. No SocketCAN import, discovery or connection here.

Simulation uses DifferentialGeometry only. SDKDriveAdapter requires an explicitly
supplied SDK drive/transport; constructing the adapter cannot open a CAN bus.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


@dataclass(frozen=True)
class WheelSpeeds:
    left_rad_s: float
    right_rad_s: float


@dataclass(frozen=True)
class DifferentialGeometry:
    wheel_radius_m: float
    track_width_m: float
    max_wheel_speed_rad_s: float = 8.0

    def __post_init__(self) -> None:
        for value in (self.wheel_radius_m, self.track_width_m, self.max_wheel_speed_rad_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Geometry and wheel-speed limit must be finite and positive')

    @classmethod
    def from_config(cls, path) -> DifferentialGeometry:
        """Read the same versioned geometry as the SDK, with no SDK/CAN import."""
        import json
        from pathlib import Path
        data = json.loads(Path(path).read_text())
        if type(data.get('schema')) is not int or data['schema'] != 1:
            raise ValueError('Chassis configuration requires schema=1')
        geometry = data['geometry']
        values = [geometry[k] for k in ('wheel_radius_m', 'track_width_m', 'max_wheel_speed_rad_s')]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in values):
            raise ValueError('Geometry values must be numbers')
        return cls(*values)

    def limit(self, left: float, right: float) -> WheelSpeeds:
        if not math.isfinite(left) or not math.isfinite(right):
            raise ValueError('Wheel speeds must be finite')
        peak = max(abs(left), abs(right))
        scale = min(1.0, self.max_wheel_speed_rad_s / peak) if peak else 1.0
        return WheelSpeeds(left * scale, right * scale)

    def from_twist(self, linear: float, angular: float) -> WheelSpeeds:
        if not math.isfinite(linear) or not math.isfinite(angular):
            raise ValueError('Twist must be finite')
        half = self.track_width_m / 2
        return self.limit((linear - angular * half) / self.wheel_radius_m,
                          (linear + angular * half) / self.wheel_radius_m)

    def encoder_twist(self, left: float, right: float) -> tuple[float, float]:
        if not math.isfinite(left) or not math.isfinite(right):
            raise ValueError('Encoder wheel speeds must be finite')
        return (self.wheel_radius_m * (left + right) / 2,
                self.wheel_radius_m * (right - left) / self.track_width_m)


class SDKDriveAdapter:
    """Explicitly injected SDK drive; never chooses or creates a CAN interface.

    The SDK applies installation direction to outgoing motor commands exactly
    once. Wheel targets returned here are normalized forward-positive values.
    SDK read_state() exposes positions, voltage and temperatures, not actual
    wheel velocity. A command must never be reported as a measurement.
    """

    def __init__(self, drive: Any, *, left_direction: int = -1, right_direction: int = 1):
        if left_direction not in (-1, 1) or right_direction not in (-1, 1):
            raise ValueError('Installation directions must be -1 or +1')
        if (drive.left.direction, drive.right.direction) != (left_direction, right_direction):
            raise ValueError('SDK motor directions do not match adapter configuration')
        self.drive = drive
        self.directions = (left_direction, right_direction)
        self.enabled = False
        self.generation = 0
        self.last_command = WheelSpeeds(0.0, 0.0)

    @classmethod
    def from_transport(cls, transport: Any, geometry: DifferentialGeometry,
                       *, left_id: int = 1, right_id: int = 2) -> SDKDriveAdapter:
        # Importing the SDK is safe; the caller alone supplies transport ownership.
        from dm_h65 import DMH65Motor, DifferentialDrive
        left = DMH65Motor(transport, left_id, direction=-1,
                         max_speed_rad_s=geometry.max_wheel_speed_rad_s)
        right = DMH65Motor(transport, right_id, direction=1,
                          max_speed_rad_s=geometry.max_wheel_speed_rad_s)
        drive = DifferentialDrive(left, right, wheel_radius_m=geometry.wheel_radius_m,
                                  track_width_m=geometry.track_width_m,
                                  max_wheel_speed_rad_s=geometry.max_wheel_speed_rad_s)
        return cls(drive)

    @classmethod
    def from_profile(cls, transport: Any, path) -> SDKDriveAdapter:
        """Use an SDK profile with caller-owned (in tests, in-memory) transport."""
        from dm_h65.chassis_config import ChassisProfile
        profile = ChassisProfile.load(path)
        return cls(profile.make_drive(transport), left_direction=profile.left_direction,
                   right_direction=profile.right_direction)

    def enable(self) -> None:
        try:
            self.drive.enable()
            self.enabled = True
        except Exception:
            self._failed_command()
            raise

    def _failed_command(self) -> None:
        self.enabled = False
        self.generation += 1
        self.last_command = WheelSpeeds(0.0, 0.0)
        try:
            self.drive.disable()
        except Exception:
            # Preserve the original failure; both-wheel stopping is best effort.
            pass

    def set_twist(self, linear: float, angular: float, *, generation: int | None = None) -> WheelSpeeds:
        if not self.enabled:
            raise RuntimeError('Drive is disabled')
        if generation is not None and generation != self.generation:
            raise RuntimeError('Cancelled/stale command generation')
        try:
            speeds = self.drive.set_twist(linear, angular)
        except Exception:
            self._failed_command()
            raise
        self.last_command = WheelSpeeds(speeds.left_rad_s, speeds.right_rad_s)
        return self.last_command

    def set_wheel_speeds(self, left: float, right: float) -> WheelSpeeds:
        if not self.enabled:
            raise RuntimeError('Drive is disabled')
        try:
            speeds = self.drive.set_wheel_speeds(left, right)
        except Exception:
            self._failed_command()
            raise
        self.last_command = WheelSpeeds(speeds.left_rad_s, speeds.right_rad_s)
        return self.last_command

    def stop(self) -> None:
        self.generation += 1
        self.last_command = WheelSpeeds(0.0, 0.0)
        try:
            self.drive.stop()
        except Exception:
            self._failed_command()
            raise

    def disable(self) -> None:
        self.generation += 1
        self.enabled = False
        self.last_command = WheelSpeeds(0.0, 0.0)
        self.drive.disable()

    def read_state(self, *, timeout_s: float = 0.1) -> dict:
        state = self.drive.read_state(timeout_s=timeout_s)
        return {'positions_rad': [state.left.motor_position_rad * self.directions[0],
                                  state.right.motor_position_rad * self.directions[1]],
                'measured_wheel_velocity_rad_s': None,
                'wheel_command_rad_s': [self.last_command.left_rad_s, self.last_command.right_rad_s],
                'raw_sdk_telemetry': {'left': vars(state.left) if hasattr(state.left, '__dict__') else {
                    field: getattr(state.left, field) for field in state.left.__dataclass_fields__},
                    'right': vars(state.right) if hasattr(state.right, '__dict__') else {
                    field: getattr(state.right, field) for field in state.right.__dataclass_fields__}}}

    @staticmethod
    def normalize_feedback(feedback: Any, direction: int) -> dict:
        if direction not in (-1, 1):
            raise ValueError('Direction must be -1 or +1')
        values = (feedback.position_rad, feedback.velocity_rad_s, feedback.torque_nm)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Non-finite feedback')
        return {'position_rad': values[0] * direction, 'velocity_rad_s': values[1] * direction,
                'torque_nm': values[2] * direction, 'error': feedback.error,
                'source': 'SDK feedback frame; decoder ranges require hardware verification'}
