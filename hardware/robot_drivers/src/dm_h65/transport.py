"""CAN transport abstractions and a python-can SocketCAN implementation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable


@dataclass(frozen=True, slots=True)
class CanFrame:
    arbitration_id: int
    data: bytes = b""
    is_extended_id: bool = False

    def __post_init__(self) -> None:
        if not 0 <= self.arbitration_id <= 0x1FFFFFFF:
            raise ValueError("arbitration_id is outside the CAN identifier range")
        if not self.is_extended_id and self.arbitration_id > 0x7FF:
            raise ValueError("standard CAN identifier must be <= 0x7FF")
        if len(self.data) > 8:
            raise ValueError("DM-H65 uses classic CAN frames with at most 8 bytes")


@runtime_checkable
class CanTransport(Protocol):
    def send(self, frame: CanFrame, timeout: float | None = None) -> None: ...

    def recv(self, timeout: float | None = None) -> CanFrame | None: ...

    def shutdown(self) -> None: ...


class PythonCanTransport:
    """Adapter around a python-can bus.

    Importing the SDK does not require python-can. It is imported lazily only
    when this real transport is constructed, keeping pure protocol tests light.
    """

    def __init__(self, channel: str = "can0", *, interface: str = "socketcan", **kwargs: object) -> None:
        try:
            import can
        except ImportError as exc:  # pragma: no cover - depends on host setup
            raise RuntimeError("python-can is required for real CAN communication") from exc
        self._can = can
        self._bus = can.Bus(interface=interface, channel=channel, **kwargs)

    def send(self, frame: CanFrame, timeout: float | None = None) -> None:
        message = self._can.Message(
            arbitration_id=frame.arbitration_id,
            data=frame.data,
            is_extended_id=frame.is_extended_id,
        )
        self._bus.send(message, timeout=timeout)

    def recv(self, timeout: float | None = None) -> CanFrame | None:
        message = self._bus.recv(timeout=timeout)
        if message is None:
            return None
        if getattr(message, "is_error_frame", False) or getattr(message, "is_remote_frame", False):
            return None
        return CanFrame(
            arbitration_id=message.arbitration_id,
            data=bytes(message.data),
            is_extended_id=message.is_extended_id,
        )

    def shutdown(self) -> None:
        self._bus.shutdown()

    def __enter__(self) -> "PythonCanTransport":
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.shutdown()


def open_socketcan(channel: str = "can0", **kwargs: object) -> PythonCanTransport:
    return PythonCanTransport(channel=channel, interface="socketcan", **kwargs)

