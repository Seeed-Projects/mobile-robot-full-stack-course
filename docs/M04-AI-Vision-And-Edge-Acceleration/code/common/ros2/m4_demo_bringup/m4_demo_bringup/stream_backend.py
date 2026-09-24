"""StreamBackend abstraction for the M4 web demo.

The M4 web preview server consumes ROS Image messages and exposes a
single aiortc.VideoStreamTrack for browser consumption. Two backends
are implemented:

  * VP8AiortcBackend  - software-encoded VP8 via aiortc's built-in path.
                        Simple, browser-native, but uses CPU.
                        PROTOTYPE / FALLBACK ONLY.
  * H264GstBackend    - hardware H.264 via nvv4l2h264enc on Jetson.
                        Production candidate (low CPU, low latency).

Both backends implement StreamBackend and are swappable at startup
time via the `backend` ROS parameter. The benchmark in
scripts/regression/benchmark_stream_backends.py measures both and the
result is recorded in docs/M4_WEB_PREVIEW.md.

DO NOT hardcode either backend as the default in code. The default is
whatever the benchmark proves is faster AND meets the 10 fps regression
threshold; that is a project policy decision, not a code one.
"""
from __future__ import annotations

import abc
import logging
import threading
from typing import Optional

from aiortc import VideoStreamTrack  # type: ignore

from .frame_slot import LockFreeLatestFrameSlot


_log = logging.getLogger(__name__)


class StreamBackend(abc.ABC):
    """Abstract base for stream backends.

    All backends MUST:
      * Take frames from a LockFreeLatestFrameSlot
      * Implement `track` property that returns an aiortc.VideoStreamTrack
      * Implement `start()` and `stop()` lifecycle
      * Hold NO unbounded queue
      * Be safe to call from the asyncio main thread only after start()
    """

    name: str = "abstract"

    def __init__(self, slot: LockFreeLatestFrameSlot) -> None:
        self._slot = slot
        self._track: Optional[VideoStreamTrack] = None
        self._lock = threading.Lock()
        self._started = False

    @property
    def track(self) -> VideoStreamTrack:
        with self._lock:
            if self._track is None:
                raise RuntimeError(f"{self.name}: track requested before start()")
            return self._track

    @abc.abstractmethod
    def start(self) -> None:
        ...

    @abc.abstractmethod
    def stop(self) -> None:
        ...

    def is_started(self) -> bool:
        with self._lock:
            return self._started
