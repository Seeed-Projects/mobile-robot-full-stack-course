"""MJPEG-over-HTTP stream backend — the "crisp preview" path.

Why this backend exists
-----------------------
The WebRTC path is low latency, but aiortc's software encoders are hard
clamped (VP8 <= 1.5 Mbps, H.264 <= 3 Mbps in aiortc/codecs/*.py) and the
only thing that raises them at runtime is RTCP REMB, which current Chrome
does not send. Measured on this deployment: the web server log contained
**zero** "receiver estimated maximum bitrate" lines, so the encoder stayed at
its 500 kbps default — about 2 KB per 1920x1080 frame — which is exactly the
"blurry / blocky" picture users reported.

MJPEG has no bitrate negotiation and no inter-frame dependency at all:

  ROS Image -> BGR ndarray -> optional resize -> cv2.imencode('.jpg') -> HTTP

Consequences, all of them desirable for a teaching/demo preview:
  * Quality is a directly configurable number (JPEG quality), not a codec
    rate-control outcome.
  * Bandwidth is honest and predictable (~100-200 KB/frame at 1280x720 q85).
  * Every frame is independently decodable, so switching modules, opening a
    second tab, or a packet loss never produces smearing or a frozen picture
    waiting for a keyframe.
  * It works in any browser with no WebRTC/codec negotiation, and it is
    trivially testable headlessly (just fetch one JPEG).
  * No aiortc track at all, so no encoder CPU on the Jetson.

Trade-off: latency is one JPEG encode+transfer (~150-300 ms on the LAN)
instead of WebRTC's tens of ms, and steady-state bandwidth is higher. That is
why this is a selectable transport rather than the only one — `--backend auto`
prefers hardware H.264 and falls back here.

The frame source is the SAME LockFreeLatestFrameSlot the WebRTC backends use,
so latest-frame-only semantics (never queue, drop stale) are preserved.
"""
from __future__ import annotations

import logging
import time
from typing import Optional

import cv2
import numpy as np

from .frame_slot import LockFreeLatestFrameSlot
from .stream_backend import StreamBackend

_log = logging.getLogger(__name__)

# Multipart boundary used by the /stream route and the browser's <img>.
BOUNDARY = "m4frame"


class MjpegBackend(StreamBackend):
    """Pull the latest frame from the slot and serve it as a JPEG.

    There is deliberately NO aiortc track: `track` is left as None so a
    mis-wired `pc.addTrack(backend.track)` fails loudly instead of silently
    negotiating a second, unused video stream.
    """

    name = "mjpeg"

    def __init__(
        self,
        slot: LockFreeLatestFrameSlot,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        quality: int = 85,
    ) -> None:
        super().__init__(slot)
        self._width = int(width)
        self._height = int(height)
        self._fps = max(1, min(int(fps), 60))
        self._quality = max(30, min(int(quality), 95))
        self._encode_params = [cv2.IMWRITE_JPEG_QUALITY, self._quality]
        self._clients = 0
        self._encoded_total = 0

    # ---- lifecycle -------------------------------------------------------

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
            self._started = True
        _log.info(
            "MjpegBackend started (%dx%d target, %d fps, jpeg quality %d)",
            self._width, self._height, self._fps, self._quality,
        )

    def stop(self) -> None:
        with self._lock:
            if not self._started:
                return
            self._started = False
        _log.info("MjpegBackend stopped")

    # ---- frame preparation ----------------------------------------------

    def _decode_bgr(self, msg) -> Optional[np.ndarray]:
        """sensor_msgs/Image -> HxWx3 BGR ndarray, resized to the target box."""
        from .backend_vp8 import _image_to_bgr_numpy
        try:
            arr = _image_to_bgr_numpy(msg)
        except Exception as exc:
            _log.warning("mjpeg: image decode failed: %s", exc)
            return None
        if arr.ndim != 3 or arr.shape[2] != 3:
            return None

        h, w = arr.shape[:2]
        tw, th = self._width, self._height
        if tw <= 0 or th <= 0:
            return arr
        # Fit inside the target box preserving aspect ratio; never upscale,
        # because upscaling only costs bandwidth without adding detail.
        scale = min(tw / float(w), th / float(h), 1.0)
        if scale >= 1.0:
            return np.ascontiguousarray(arr)
        nw, nh = max(2, int(round(w * scale))), max(2, int(round(h * scale)))
        return cv2.resize(arr, (nw, nh), interpolation=cv2.INTER_AREA)

    def encode_frame(self, msg) -> Optional[bytes]:
        """Encode one ROS Image as JPEG bytes (None on failure)."""
        arr = self._decode_bgr(msg)
        if arr is None:
            return None
        ok, buf = cv2.imencode(".jpg", arr, self._encode_params)
        if not ok:
            return None
        self._encoded_total += 1
        return buf.tobytes()

    # ---- streaming -------------------------------------------------------

    async def _generate(self, slot):
        """Yield JPEG frames from `slot`, paced to `fps`.

        Backpressure is intentional: the consumer (aiohttp) awaits each write,
        so a slow client stops us from draining the slot. The slot keeps only
        the newest frame, so a slow client degrades to a lower frame rate
        rather than to a growing queue or increasingly stale video.
        """
        import asyncio

        interval = 1.0 / float(self._fps)
        next_at = time.monotonic()
        while self._started:
            taken = slot.take_for_render()
            if taken is None:
                # Nothing new: do not advance the schedule.
                await asyncio.sleep(min(0.005, interval))
                continue
            jpeg = self.encode_frame(taken[0])
            if jpeg is None:
                continue
            now = time.monotonic()
            delay = next_at - now
            yield jpeg
            if delay > 0:
                await asyncio.sleep(delay)
                next_at += interval
            else:
                # We are behind schedule; resynchronise instead of trying to
                # catch up in a burst.
                next_at = time.monotonic() + interval

    def frames(self):
        """Async generator over the backend's own slot."""
        return self._generate(self._slot)

    def frames_for(self, slot):
        """Async generator over a SPECIFIC slot.

        Used by `GET /stream?module=<key>` in hub mode so one request can serve
        any module's slot without mutating the shared active slot that the
        WebRTC backend reads.
        """
        return self._generate(slot)

    # ---- diagnostics -----------------------------------------------------

    def client_opened(self) -> None:
        self._clients += 1

    def client_closed(self) -> None:
        self._clients = max(0, self._clients - 1)

    @property
    def clients(self) -> int:
        return self._clients

    @property
    def encoded_total(self) -> int:
        return self._encoded_total