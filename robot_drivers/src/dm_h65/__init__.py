"""Python SDK for the DM-H65 wheel motor."""

__version__ = "0.2.0"

from .drive import ChassisState, DifferentialDrive, WheelSpeeds
from .motor import (
    CommunicationError,
    DMH65Motor,
    MotorIdentity,
    MotorTelemetry,
    ProtocolError,
    ResponseTimeout,
)
from .protocol import ControlMode, FeedbackState, Register
from .transport import CanFrame, PythonCanTransport

__all__ = [
    "CanFrame",
    "ChassisState",
    "CommunicationError",
    "ControlMode",
    "DifferentialDrive",
    "DMH65Motor",
    "FeedbackState",
    "MotorIdentity",
    "MotorTelemetry",
    "ProtocolError",
    "PythonCanTransport",
    "Register",
    "ResponseTimeout",
    "WheelSpeeds",
    "__version__",
]

