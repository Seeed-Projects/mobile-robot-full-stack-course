"""Pure DM6540-1EC protocol encoding and decoding."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from enum import IntEnum

from .transport import CanFrame


PARAMETER_REQUEST_ID = 0x7FF


class ControlMode(IntEnum):
    MIT = 1
    POSITION_VELOCITY = 2
    VELOCITY = 3
    POSITION_VELOCITY_TORQUE = 4


class Register(IntEnum):
    MAX_SPEED = 0x06
    MASTER_ID = 0x07
    ESC_ID = 0x08
    TIMEOUT = 0x09
    CONTROL_MODE = 0x0A
    SOFTWARE_VERSION = 0x0E
    POLE_PAIRS = 0x10
    CAN_BITRATE = 0x23
    BOOT_VERSION = 0x25
    MAX_CURRENT = 0x3B
    BUS_VOLTAGE = 0x3C
    PCB_TEMPERATURE = 0x3D
    MOTOR_TEMPERATURE = 0x3E
    MOTOR_POSITION = 0x50


UINT32_REGISTERS = frozenset(
    {
        Register.MASTER_ID,
        Register.ESC_ID,
        Register.TIMEOUT,
        Register.CONTROL_MODE,
        Register.SOFTWARE_VERSION,
        Register.POLE_PAIRS,
        Register.CAN_BITRATE,
        Register.BOOT_VERSION,
    }
)


@dataclass(frozen=True, slots=True)
class RegisterResponse:
    motor_id: int
    operation: int
    register: Register
    value: int | float


@dataclass(frozen=True, slots=True)
class FeedbackState:
    motor_id: int
    error: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    pcb_temperature_c: float
    motor_temperature_c: float

    @property
    def enabled(self) -> bool:
        return self.error == 1


def validate_motor_id(motor_id: int) -> None:
    if not 0 <= motor_id <= 0xFF:
        raise ValueError("motor_id must be in 0x00..0xFF")


def control_frame_id(motor_id: int, mode: ControlMode) -> int:
    validate_motor_id(motor_id)
    offsets = {
        ControlMode.MIT: 0x000,
        ControlMode.POSITION_VELOCITY: 0x100,
        ControlMode.VELOCITY: 0x200,
        ControlMode.POSITION_VELOCITY_TORQUE: 0x300,
    }
    return offsets[mode] + motor_id


def register_is_uint32(register: Register) -> bool:
    return register in UINT32_REGISTERS


def make_parameter_read(motor_id: int, register: Register) -> CanFrame:
    validate_motor_id(motor_id)
    return CanFrame(PARAMETER_REQUEST_ID, struct.pack("<HBB", motor_id, 0x33, register))


def make_parameter_write(motor_id: int, register: Register, value: int | float) -> CanFrame:
    validate_motor_id(motor_id)
    if register_is_uint32(register):
        if not isinstance(value, int):
            raise TypeError(f"{register.name} requires an integer value")
        encoded = struct.pack("<I", value)
    else:
        if not isinstance(value, (int, float)):
            raise TypeError(f"{register.name} requires a floating-point value")
        encoded = struct.pack("<f", float(value))
    return CanFrame(PARAMETER_REQUEST_ID, struct.pack("<HBB", motor_id, 0x55, register) + encoded)


def _special_command(motor_id: int, final_byte: int) -> CanFrame:
    validate_motor_id(motor_id)
    return CanFrame(motor_id, bytes([0xFF] * 7 + [final_byte]))


def make_enable(motor_id: int) -> CanFrame:
    return _special_command(motor_id, 0xFC)


def make_disable(motor_id: int) -> CanFrame:
    return _special_command(motor_id, 0xFD)


def make_clear_error(motor_id: int) -> CanFrame:
    return _special_command(motor_id, 0xFB)


def make_velocity_command(motor_id: int, velocity_rad_s: float) -> CanFrame:
    if not math.isfinite(velocity_rad_s):
        raise ValueError("velocity must be finite")
    return CanFrame(
        control_frame_id(motor_id, ControlMode.VELOCITY),
        struct.pack("<f", float(velocity_rad_s)) + bytes(4),
    )


def decode_register_response(frame: CanFrame, *, uint32: bool) -> RegisterResponse:
    if frame.is_extended_id or len(frame.data) != 8:
        raise ValueError("register response must be an 8-byte standard CAN frame")
    motor_id, operation, raw_register = struct.unpack_from("<HBB", frame.data)
    if operation not in (0x33, 0x55):
        raise ValueError("frame is not a register response")
    register = Register(raw_register)
    value = struct.unpack_from("<I" if uint32 else "<f", frame.data, 4)[0]
    return RegisterResponse(motor_id, operation, register, value)


def _uint_to_float(value: int, minimum: float, maximum: float, bits: int) -> float:
    return value * (maximum - minimum) / ((1 << bits) - 1) + minimum


def decode_feedback(
    frame: CanFrame,
    *,
    position_max_rad: float = 12.566,
    velocity_max_rad_s: float = 100.0,
    torque_max_nm: float = 40.0,
) -> FeedbackState:
    if frame.is_extended_id or len(frame.data) != 8:
        raise ValueError("feedback must be an 8-byte standard CAN frame")
    if min(position_max_rad, velocity_max_rad_s, torque_max_nm) <= 0:
        raise ValueError("feedback mapping maxima must be positive")
    data = frame.data
    position_raw = (data[1] << 8) | data[2]
    velocity_raw = (data[3] << 4) | (data[4] >> 4)
    torque_raw = ((data[4] & 0x0F) << 8) | data[5]
    return FeedbackState(
        motor_id=data[0] & 0x0F,
        error=(data[0] >> 4) & 0x0F,
        position_rad=_uint_to_float(position_raw, -position_max_rad, position_max_rad, 16),
        velocity_rad_s=_uint_to_float(velocity_raw, -velocity_max_rad_s, velocity_max_rad_s, 12),
        torque_nm=_uint_to_float(torque_raw, -torque_max_nm, torque_max_nm, 12),
        pcb_temperature_c=float(data[6]),
        motor_temperature_c=float(data[7]),
    )


def rpm_to_rad_s(rpm: float) -> float:
    return float(rpm) * 2.0 * math.pi / 60.0


def rad_s_to_rpm(rad_s: float) -> float:
    return float(rad_s) * 60.0 / (2.0 * math.pi)

