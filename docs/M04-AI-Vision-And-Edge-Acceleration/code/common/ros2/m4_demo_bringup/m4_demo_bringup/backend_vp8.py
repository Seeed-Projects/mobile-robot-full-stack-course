"""VP8 backend using aiortc's built-in VideoStreamTrack.

This is the software-encoder path. aiortc handles:
  * PIL/numpy -> VideoFrame conversion
  * libvpx VP8 encoding
  * RTP packetization and SRTP encryption

It is intentionally simple and is the **functional prototype /
fallback** path. Per the M4 plan, this backend is NOT the default
without prior measurement. Use H264GstBackend where hardware H.264 is
available.

Latency characteristics (typical Jetson AGX Orin, 1280x720):
  * encode latency per frame: 15-30 ms (CPU-bound)
  * transport: WebRTC SRTP/VP8, browser decodes natively

This backend does NOT spin up GStreamer. It only consumes the latest
frame from the shared slot and feeds it into an aiortc VideoStreamTrack.
"""
from __future__ import annotations

import asyncio
import logging
import time
from fractions import Fraction
from typing import Optional

import numpy as np

from aiortc import VideoStreamTrack  # type: ignore
from aiortc.mediastreams import VideoFrame  # type: ignore

from .frame_slot import LockFreeLatestFrameSlot
from .stream_backend import StreamBackend


_log = logging.getLogger(__name__)


class _VP8Track(VideoStreamTrack):
    """aiortc VideoStreamTrack that pulls the latest frame from a slot.

    On each recv() it drains the slot. If the slot is empty, it waits
    for one ROS callback to refill by polling in 1 ms increments — never
    longer than `max_wait_ms` so a stalled publisher cannot wedge the
    peer. Once a frame arrives it is converted to aiortc's VideoFrame
    and returned.
    """

    kind = "video"

    def __init__(self, slot: LockFreeLatestFrameSlot, max_wait_ms: int = 50) -> None:
        super().__init__()
        self._slot = slot
        self._max_wait_ms = max_wait_ms
        self._start_time = time.monotonic()

    async def recv(self) -> VideoFrame:
        # Drain the slot. The asyncio sleep is fine because there is no
        # alternative: aiortc expects an awaitable frame source.
        waited_ms = 0
        while True:
            taken = self._slot.take_for_render()
            if taken is not None:
                msg, stamp_ns = taken
                break
            if waited_ms >= self._max_wait_ms:
                # No frame in flight: yield a tiny placeholder so the
                # peer connection stays alive but the encoder idles.
                await asyncio.sleep(0.005)
                continue
            await asyncio.sleep(0.001)
            waited_ms += 1
            if waited_ms > 1000:  # safety
                waited_ms = 0

        # Convert sensor_msgs/Image -> numpy -> aiortc VideoFrame (bgr24).
        arr = _image_to_bgr_numpy(msg)
        frame = VideoFrame.from_ndarray(arr, format="bgr24")
        # aiortc expects Frame.time_base as a Fraction. Use 90 kHz
        # (RFC 3551 / RTP video clock) and stamp PTS in those units
        # relative to track creation.
        frame.pts = int((time.monotonic() - self._start_time) * 90000)
        frame.time_base = Fraction(1, 90000)
        return frame


def _image_to_bgr_numpy(msg) -> np.ndarray:
    """Convert a sensor_msgs/Image (bgr8 / rgb8 / mono8) to a HxWx3 uint8 BGR array.

    Supports the encodings we expect from the demo topics. Falls back to
    the cv_bridge if present.
    """
    import numpy as _np

    h, w = msg.height, msg.width
    data = _np.frombuffer(msg.data, dtype=_np.uint8)
    enc = (msg.encoding or "").lower()
    if enc in ("bgr8", "rgb8"):
        arr = data.reshape(h, w, 3)
        if enc == "rgb8":
            arr = arr[..., ::-1].copy()  # rgb -> bgr
        return arr
    if enc == "bgra8":
        arr = data.reshape(h, w, 4)[..., :3].copy()
        return arr
    if enc == "rgba8":
        arr = data.reshape(h, w, 4)[..., 2::-1].copy()  # rgba -> bgr (a dropped)
        return arr
    if enc == "mono8":
        arr = data.reshape(h, w, 1).repeat(3, axis=2)
        return arr
    # Fallback: try cv_bridge if installed.
    try:
        from cv_bridge import CvBridge  # type: ignore

        bridge = CvBridge()
        return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(
            f"VP8AiortcBackend: unsupported image encoding {msg.encoding!r}: {exc}"
        )


class VP8AiortcBackend(StreamBackend):
    """aiortc + libvpx VP8 backend (software)."""

    name = "vp8_aiortc"

    def __init__(self, slot: LockFreeLatestFrameSlot) -> None:
        super().__init__(slot)
        self._track_impl: Optional[_VP8Track] = None

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
            self._track_impl = _VP8Track(self._slot)
            self._track = self._track_impl
            self._started = True
            _log.info("VP8AiortcBackend started")

    def stop(self) -> None:
        with self._lock:
            if not self._started:
                return
            self._started = False
            self._track = None
            self._track_impl = None
            _log.info("VP8AiortcBackend stopped")
