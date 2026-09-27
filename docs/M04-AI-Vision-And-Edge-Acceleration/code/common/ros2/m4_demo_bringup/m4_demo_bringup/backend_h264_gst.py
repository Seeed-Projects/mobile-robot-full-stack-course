"""Hardware H.264 backend using nvv4l2h264enc on Jetson.

Data path (this is the REAL hardware path; the previous implementation was a
stub — see "History" below):

  ROS Image (bgr8) on the ROS executor thread
        |
        v
  LockFreeLatestFrameSlot            (single slot, drops older)
        |
        v  H264GstBackend.run()  — asyncio coroutine, paced to `fps`
  appsrc (is-live, max-bytes=1 frame, no queue growth; I420)
        |
        v
  nvvidconv                          (I420 system memory -> NVMM; scales to
                                      the --encode-* box in hardware)
        |
        v
  nvv4l2h264enc                      (Jetson hardware encoder,
                                      baseline profile, configurable bitrate)
        |
        v
  h264parse config-interval=-1
        |
        v
  video/x-h264,stream-format=byte-stream,alignment=au   (Annex-B)
        |
        v
  appsink (max-buffers=1, drop=true)
        |
        v  _H264GstTrack.recv() returns an av.Packet
  aiortc RTCRtpSender._next_encoded_frame()
        |
        v
  aiortc H264Encoder.pack(packet)    (aiortc/codecs/h264.py — packetisation only)
        |
        v  RTP -> SRTP -> browser <video>

WHY THIS MATTERS — aiortc clamps software encoder bitrate

`RTCRtpSender` only changes `encoder.target_bitrate` when it receives an RTCP
REMB packet. Chrome no longer sends REMB, so the software path stayed pinned at
the codec default (VP8 500 kbps, H.264 1 Mbps) and, even when nudged, aiortc's
setters hard-clamp (VP8 <= 1.5 Mbps, H.264 <= 3 Mbps). At 1920x1080 that is a
couple of KB per frame: the blurry picture users reported.

Handing aiortc a **pre-encoded** `av.Packet` instead of a raw VideoFrame
bypasses its encoder completely — `_run_rtp` takes the `pack()` branch for any
non-Frame object — so the bitrate is whatever `nvv4l2h264enc` was told to use
(default 6 Mbps here). CPU cost on the Jetson is near zero.

History: the original version of this file built the GStreamer pipeline but
never called `push_frame()` (nothing did), and `recv()` returned a raw BGR
VideoFrame, so the pipeline idled while aiortc software-encoded anyway. That is
why the module previously claimed "production candidate" while measuring like
VP8.

If nvv4l2h264enc is unavailable (non-Jetson) `probe()` returns False and the
server falls back to MJPEG or VP8.
"""
from __future__ import annotations

import asyncio
import logging
import time
from fractions import Fraction

import cv2
import numpy as np

from aiortc import VideoStreamTrack  # type: ignore
from av import Packet  # type: ignore

from .frame_slot import LockFreeLatestFrameSlot
from .stream_backend import StreamBackend

_log = logging.getLogger(__name__)

# Lazy import of gi (GStreamer Python binding). Some sandboxed dev envs do not
# have GStreamer; we guard and report cleanly.
try:
    import gi  # type: ignore

    gi.require_version("Gst", "1.0")
    gi.require_version("GstApp", "1.0")
    from gi.repository import Gst, GstApp, GLib  # noqa: F401

    _GST_OK = True
except Exception as _exc:  # pragma: no cover
    _GST_OK = False
    _GST_IMPORT_ERROR = _exc

# RTP video clock (RFC 3551).
_CLOCK_RATE = 90000


def probe() -> bool:
    """True iff nvv4l2h264enc is registered with the running GStreamer."""
    if not _GST_OK:
        return False
    try:
        Gst.init(None)
        return Gst.ElementFactory.make("nvv4l2h264enc", "probe") is not None
    except Exception:
        return False


class H264GstBackend(StreamBackend):
    """Jetson hardware H.264 encoder feeding aiortc as pre-encoded packets."""

    name = "h264_gst"

    def __init__(
        self,
        slot: LockFreeLatestFrameSlot,
        width: int = 1920,
        height: int = 1080,
        fps: int = 30,
        bitrate: int = 8_000_000,
    ) -> None:
        super().__init__(slot)
        if not _GST_OK:
            raise RuntimeError(
                f"H264GstBackend: gi/Gst unavailable: {_GST_IMPORT_ERROR}"
            )
        self._width = int(width)
        self._height = int(height)
        self._fps = max(1, min(int(fps), 60))
        self._bitrate = int(bitrate)
        self._pipeline = None
        self._appsrc = None
        self._appsink = None
        self._track_impl = None
        self._pump: asyncio.Task | None = None
        # Size of the frames the appsrc caps currently accept. Starts unset so
        # the caps are taken from the FIRST real frame rather than from CLI
        # defaults (the old code pinned them to --width/--height while the slot
        # carried whatever the topic published, which would have made
        # push-buffer fail caps negotiation).
        self._caps_size: tuple[int, int] | None = None
        self._pushed = 0
        self._pull_warned = False

    # ---- pipeline --------------------------------------------------------

    def _pipeline_str(self) -> str:
        """Build the pipeline.

        There is deliberately NO `videoconvert` here. `nvvidconv` accepts
        I420 from system memory directly, and pushing I420 (converted in one
        `cv2.cvtColor`) is both cheaper and removes a pipeline element.
        Measured per-frame cost to feed the encoder at 1080p (1920x1080 in,
        1280x720 out, 60-frame runs):

            BGR 1080p  + videoconvert            4.1 ms   241 fps
            BGR 720p via cv2.resize + videoconvert  6.2 ms   160 fps
            I420 1080p, no videoconvert          3.3 ms   299 fps

        So the naive "downscale first" idea is actually WORSE: an INTER_AREA
        resize costs more than the conversion it was meant to save. Letting
        `nvvidconv` scale in hardware while we pay one cheap BGR->I420 step
        wins. The encoder still emits the configured
        `--encode-width/--encode-height` because the NVMM caps below say so.

        appsrc caps are set in `_ensure_caps()` from the real frame size.
        """
        return (
            "appsrc name=m4src is-live=true format=time block=false "
            "! queue max-size-buffers=1 leaky=downstream "
            "! nvvidconv "
            f"! video/x-raw(memory:NVMM),width={self._width},height={self._height} "
            f"! nvv4l2h264enc maxperf-enable=1 bitrate={self._bitrate} "
            f"vbv-size={max(self._bitrate // 2, 1_000_000)} "
            f"preset-level=1 iframeinterval={self._fps} idrinterval={self._fps} "
            "control-rate=1 ratecontrol-enable=1 insert-vui=1 "
            # Baseline (profile=0) + SPS/PPS in-band: the combination every
            # browser accepts. config-interval=-1 repeats them per IDR.
            "insert-sps-pps=1 profile=0 "
            "! h264parse config-interval=-1 "
            # Annex-B byte-stream is REQUIRED: aiortc's H264Encoder.pack()
            # calls _split_bitstream(), which looks for start codes. Without
            # this caps filter h264parse emits length-prefixed AVCC instead.
            "! video/x-h264,stream-format=byte-stream,alignment=au "
            "! appsink name=m4sink max-buffers=1 drop=true sync=false"
        )

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
            Gst.init(None)
            desc = self._pipeline_str()
            _log.info("H264GstBackend pipeline: %s", desc)
            self._pipeline = Gst.parse_launch(desc)
            self._appsrc = self._pipeline.get_by_name("m4src")
            self._appsink = self._pipeline.get_by_name("m4sink")
            if self._appsrc is None or self._appsink is None:
                raise RuntimeError("H264GstBackend: pipeline is missing appsrc/appsink")
            ret = self._pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("H264GstBackend: failed to set pipeline to PLAYING")
            self._track_impl = _H264GstTrack(self._appsink)
            self._track = self._track_impl
            self._started = True
        _log.info(
            "H264GstBackend started (nvv4l2h264enc, %d bps, %dx%d target, %d fps)",
            self._bitrate, self._width, self._height, self._fps,
        )

    def stop(self) -> None:
        with self._lock:
            if not self._started:
                return
            self._started = False
            pump, self._pump = self._pump, None
            pipeline, self._pipeline = self._pipeline, None
            self._appsrc = None
            self._appsink = None
            self._track = None
            self._track_impl = None
        if pump is not None:
            pump.cancel()
        try:
            if pipeline is not None:
                pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass
        _log.info("H264GstBackend stopped (pushed %d frames)", self._pushed)

    # ---- appsrc caps -----------------------------------------------------

    def _ensure_caps(self, h: int, w: int) -> bool:
        """Pin appsrc caps to the ACTUAL frame size, renegotiating on change.

        I420 is required (see _pipeline_str): nvvidconv takes it straight from
        system memory, so no videoconvert is needed.
        """
        if self._caps_size == (w, h):
            return True
        caps = Gst.Caps.from_string(
            f"video/x-raw,format=I420,width={w},height={h},framerate={self._fps}/1"
        )
        self._appsrc.set_property("caps", caps)
        # I420 is 1.5 bytes/px; w*h*3 is a safe upper bound for the queue.
        self._appsrc.set_property("max-bytes", w * h * 3)
        self._caps_size = (w, h)
        _log.info("H264GstBackend appsrc caps -> %dx%d I420 @%d fps", w, h, self._fps)
        return True

    def push_frame(self, bgr: np.ndarray, pts_ns: int) -> None:
        """Push one BGR frame into the pipeline (asyncio loop only)."""
        if self._appsrc is None:
            return
        # `nvvidconv` will obey a fixed output caps box by stretching a source
        # with a different aspect ratio.  M4.3 deliberately publishes a
        # side-by-side 32:9 teaching view, so fit it into the 16:9 WebRTC box
        # ourselves and use the lab's dark background as letterbox padding.
        # A normal M4.1/M4.2 1920x1080 frame takes this fast path unchanged.
        bgr = self._fit_to_encode_box(bgr)
        h, w = bgr.shape[:2]
        # I420 needs even dimensions; a one-pixel crop is invisible here and
        # far cheaper than a resize.
        if w % 2 or h % 2:
            bgr = bgr[: h - (h % 2), : w - (w % 2)]
            h, w = bgr.shape[:2]
        if h == 0 or w == 0:
            return
        if not self._ensure_caps(h, w):
            return
        # One cheap CPU colour conversion, then the hardware does the scaling.
        i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)
        buf = Gst.Buffer.new_wrapped(i420.tobytes())
        buf.pts = int(pts_ns)
        buf.duration = int(Gst.SECOND // self._fps)
        ret = self._appsrc.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK and not self._pull_warned:
            self._pull_warned = True
            _log.warning("H264GstBackend: push-buffer returned %s", ret)
        self._pushed += 1

    def _fit_to_encode_box(self, bgr: np.ndarray) -> np.ndarray:
        h, w = bgr.shape[:2]
        if (w, h) == (self._width, self._height):
            return bgr
        scale = min(self._width / float(w), self._height / float(h))
        resized_w = max(2, int(round(w * scale)) // 2 * 2)
        resized_h = max(2, int(round(h * scale)) // 2 * 2)
        resized = cv2.resize(bgr, (resized_w, resized_h),
                             interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR)
        canvas = np.full((self._height, self._width, 3), (24, 28, 34), dtype=np.uint8)
        x = (self._width - resized_w) // 2
        y = (self._height - resized_h) // 2
        canvas[y:y + resized_h, x:x + resized_w] = resized
        return canvas

    # ---- pump: slot -> pipeline -----------------------------------------

    async def run(self) -> None:
        """Drain the slot into the encoder, paced to `fps`.

        Started by the server once the asyncio loop is running. Without this
        coroutine `push_frame()` is never called and the hardware encoder
        idles — which is exactly the bug this rewrite fixes.
        """
        from .backend_vp8 import _image_to_bgr_numpy

        interval = 1.0 / float(self._fps)
        next_at = time.monotonic()
        start = next_at
        while self._started:
            taken = self._slot.take_for_render()
            if taken is None:
                await asyncio.sleep(min(0.004, interval))
                continue
            msg, stamp_ns = taken
            try:
                arr = _image_to_bgr_numpy(msg)
            except Exception as exc:
                _log.warning("H264GstBackend: image decode failed: %s", exc)
                continue
            self.push_frame(arr, int((time.monotonic() - start) * 1e9))
            now = time.monotonic()
            delay = next_at - now
            if delay > 0:
                await asyncio.sleep(delay)
                next_at += interval
            else:
                next_at = time.monotonic() + interval

    async def start_pump(self) -> None:
        if self._pump is None or self._pump.done():
            self._pump = asyncio.create_task(self.run())


class _H264GstTrack(VideoStreamTrack):
    """aiortc track that returns pre-encoded H.264 access units.

    `recv()` returns an `av.Packet`, NOT an `av.VideoFrame`. aiortc's sender
    branches on that: a Frame goes through the encoder, anything else goes
    through `encoder.pack()`. That is what keeps the bitrate under
    nvv4l2h264enc's control instead of aiortc's clamped software encoder.
    """

    kind = "video"

    def __init__(self, appsink, max_wait_ms: int = 200) -> None:
        super().__init__()
        self._appsink = appsink
        self._start_time = time.monotonic()
        self._max_wait_ms = max_wait_ms

    async def recv(self) -> Packet:
        loop = asyncio.get_event_loop()
        # Pull with a short blocking timeout on the executor: GstApp is not
        # asyncio-aware, and a 0-timeout busy loop would burn a core.
        deadline = time.monotonic() + self._max_wait_ms / 1000.0
        sample = None
        while sample is None:
            sample = await loop.run_in_executor(
                None, self._appsink.try_pull_sample, Gst.MSECOND * 20,
            )
            if sample is None:
                if time.monotonic() > deadline:
                    # Keep the connection alive while the source is idle; the
                    # sender will simply have nothing to packetise.
                    deadline = time.monotonic() + self._max_wait_ms / 1000.0
                await asyncio.sleep(0.001)

        buf = sample.get_buffer()
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            raise RuntimeError("H264GstBackend: could not map encoded buffer")
        try:
            # Copy out: the Gst buffer is recycled as soon as we unmap.
            data = bytes(info.data)
        finally:
            buf.unmap(info)

        packet = Packet(data)
        packet.pts = int((time.monotonic() - self._start_time) * _CLOCK_RATE)
        packet.time_base = Fraction(1, _CLOCK_RATE)
        return packet
