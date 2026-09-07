"""Synchronous high-level API for one DM-H65 motor."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .protocol import (
    ControlMode,
    FeedbackState,
    Register,
    RegisterResponse,
    decode_feedback,
    decode_register_response,
    make_clear_error,
    make_disable,
    make_enable,
    make_parameter_read,
    make_parameter_write,
    make_velocity_command,
    register_is_uint32,
    rpm_to_rad_s,
    validate_motor_id,
)
from .transport import CanFrame, CanTransport


class ProtocolError(RuntimeError):
    pass


class ResponseTimeout(ProtocolError):
    pass


class CommunicationError(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class MotorIdentity:
    esc_id: int
    master_id: int
    control_mode: int
    software_version: int
    boot_version: int
    can_bitrate_code: int


@dataclass(frozen=True, slots=True)
class MotorTelemetry:
    bus_voltage_v: float
    pcb_temperature_c: float
    motor_temperature_c: float
    motor_position_rad: float


class DMH65Motor:
    def __init__(
        self,
        transport: CanTransport,
        motor_id: int,
        *,
        direction: int = 1,
        max_speed_rad_s: float = 12.0,
        send_timeout_s: float = 0.05,
    ) -> None:
        validate_motor_id(motor_id)
        if direction not in (-1, 1):
            raise ValueError("direction must be +1 or -1")
        if not math.isfinite(max_speed_rad_s) or max_speed_rad_s <= 0:
            raise ValueError("max_speed_rad_s must be finite and positive")
        if send_timeout_s < 0:
            raise ValueError("send_timeout_s must be non-negative")
        self.transport = transport
        self.motor_id = motor_id
        self.direction = direction
        self.max_speed_rad_s = float(max_speed_rad_s)
        self.send_timeout_s = float(send_timeout_s)

    def _send(self, frame: CanFrame) -> None:
        try:
            self.transport.send(frame, timeout=self.send_timeout_s)
        except Exception as exc:
            raise CommunicationError(
                f"CAN transmit failed for motor 0x{self.motor_id:02X}; check power, "
                "CAN_H/CAN_L, ground, termination, bitrate, and bus-off counters"
            ) from exc

    def _transact(
        self,
        request: CanFrame,
        register: Register,
        operation: int,
        timeout_s: float,
    ) -> RegisterResponse:
        if timeout_s < 0:
            raise ValueError("timeout_s must be non-negative")
        self._send(request)
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ResponseTimeout(
                    f"timeout waiting for motor 0x{self.motor_id:02X} register {register.name}"
                )
            frame = self.transport.recv(timeout=remaining)
            if frame is None or frame.is_extended_id or len(frame.data) != 8:
                continue
            data = frame.data
            response_id = data[0] | (data[1] << 8)
            if (
                response_id != self.motor_id
                or data[2] != operation
                or data[3] != register
            ):
                continue
            return decode_register_response(frame, uint32=register_is_uint32(register))

    def read_register(self, register: Register, *, timeout_s: float = 0.1) -> int | float:
        return self._transact(
            make_parameter_read(self.motor_id, register), register, 0x33, timeout_s
        ).value

    def write_register(
        self, register: Register, value: int | float, *, timeout_s: float = 0.1
    ) -> int | float:
        """Write RAM only. This SDK never sends the flash-store command."""
        return self._transact(
            make_parameter_write(self.motor_id, register, value),
            register,
            0x55,
            timeout_s,
        ).value

    def is_online(self, *, timeout_s: float = 0.1) -> bool:
        try:
            return self.read_register(Register.ESC_ID, timeout_s=timeout_s) == self.motor_id
        except ResponseTimeout:
            return False

    def read_identity(self, *, timeout_s: float = 0.1) -> MotorIdentity:
        return MotorIdentity(
            esc_id=int(self.read_register(Register.ESC_ID, timeout_s=timeout_s)),
            master_id=int(self.read_register(Register.MASTER_ID, timeout_s=timeout_s)),
            control_mode=int(self.read_register(Register.CONTROL_MODE, timeout_s=timeout_s)),
            software_version=int(
                self.read_register(Register.SOFTWARE_VERSION, timeout_s=timeout_s)
            ),
            boot_version=int(self.read_register(Register.BOOT_VERSION, timeout_s=timeout_s)),
            can_bitrate_code=int(self.read_register(Register.CAN_BITRATE, timeout_s=timeout_s)),
        )

    def read_telemetry(self, *, timeout_s: float = 0.1) -> MotorTelemetry:
        return MotorTelemetry(
            bus_voltage_v=float(self.read_register(Register.BUS_VOLTAGE, timeout_s=timeout_s)),
            pcb_temperature_c=float(
                self.read_register(Register.PCB_TEMPERATURE, timeout_s=timeout_s)
            ),
            motor_temperature_c=float(
                self.read_register(Register.MOTOR_TEMPERATURE, timeout_s=timeout_s)
            ),
            motor_position_rad=float(
                self.read_register(Register.MOTOR_POSITION, timeout_s=timeout_s)
            ),
        )

    def clear_error(self) -> None:
        self._send(make_clear_error(self.motor_id))

    def enable(self) -> None:
        self._send(make_enable(self.motor_id))

    def disable(self) -> None:
        self._send(make_disable(self.motor_id))

    def stop(self) -> None:
        self.set_velocity(0.0)

    def set_velocity(self, velocity_rad_s: float) -> None:
        requested = float(velocity_rad_s)
        if not math.isfinite(requested):
            raise ValueError("velocity must be finite")
        if abs(requested) > self.max_speed_rad_s:
            raise ValueError(
                f"requested speed {requested:.3f} rad/s exceeds software limit "
                f"{self.max_speed_rad_s:.3f} rad/s"
            )
        self._send(make_velocity_command(self.motor_id, requested * self.direction))

    def set_speed_rpm(self, rpm: float) -> None:
        self.set_velocity(rpm_to_rad_s(rpm))

    def select_velocity_mode(self, *, timeout_s: float = 0.1) -> None:
        """Switch the RAM mode only; the change is deliberately not persisted."""
        self.write_register(Register.CONTROL_MODE, int(ControlMode.VELOCITY), timeout_s=timeout_s)

    def receive_feedback(
        self,
        *,
        timeout_s: float = 0.1,
        position_max_rad: float = 12.566,
        velocity_max_rad_s: float = 100.0,
        torque_max_nm: float = 40.0,
    ) -> FeedbackState | None:
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            frame = self.transport.recv(timeout=remaining)
            if frame is None or frame.is_extended_id or len(frame.data) != 8:
                continue
            if frame.data[2] in (0x33, 0x55):
                continue
            state = decode_feedback(
                frame,
                position_max_rad=position_max_rad,
                velocity_max_rad_s=velocity_max_rad_s,
                torque_max_nm=torque_max_nm,
            )
            if state.motor_id == (self.motor_id & 0x0F):
                return state

    def stop_and_disable(self) -> None:
        errors: list[Exception] = []
        for action in (self.stop, self.disable):
            try:
                action()
            except Exception as exc:
                errors.append(exc)
        if len(errors) == 2:
            raise errors[0]

    @classmethod
    def scan(
        cls,
        transport: CanTransport,
        *,
        first_id: int = 0,
        last_id: int = 15,
        timeout_per_id_s: float = 0.03,
    ) -> list[int]:
        validate_motor_id(first_id)
        validate_motor_id(last_id)
        if first_id > last_id:
            raise ValueError("first_id must not exceed last_id")
        found: list[int] = []
        for motor_id in range(first_id, last_id + 1):
            if cls(transport, motor_id).is_online(timeout_s=timeout_per_id_s):
                found.append(motor_id)
        return found

