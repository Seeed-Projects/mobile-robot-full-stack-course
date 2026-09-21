"""Low-latency WebRTC browser preview server for the M4 demos.

Replaces the legacy rqt_image_view / cv_viewer local viewer with a
browser-native preview served from the Jetson. End-to-end data path:

  /perception/demo/m4_X (sensor_msgs/Image, bgr8)
        |
        v  (best-effort, depth=1, ROS executor thread)
  LockFreeLatestFrameSlot   (single-slot, drops older; thread-safe)
        |
        v  (drained by asyncio track.recv)
  StreamBackend             (H264GstBackend or VP8AiortcBackend)
        |
        v  (encoded frames over WebRTC)
  aiortc.RTCPeerConnection  (offer/answer over /signaling websocket)
        |
        v
  Browser <video autoplay muted>

Two operating modes:

  single-demo (default, unchanged)
      `--demo m4_1 --video-topic /perception/demo/m4_1`
      One server process per demo, one topic, page at /m4/1.
      This is what scripts/m4/run_m4_1_demo.sh (etc.) start, and what
      the existing regression cycle exercises.

  hub (`--demo hub --topics m4_1=<t1>,m4_2=<t2>,m4_3=<t3>`)
      ONE server process serves all three modules on ONE port with a
      tabbed page, so 4.1/4.2/4.3 can be previewed without three
      competing servers. Every topic gets its own latest-frame slot; a
      SwitchableFrameSlot proxies the active one to the video backend,
      and the browser switches modules at runtime over /signaling with
      `{"type":"select","demo":"m4_2"}`. No backend restart on switch.

Threading model:
  * ROS rclpy executor runs on its OWN thread (single-threaded executor).
    Its callback only does `slot.try_replace(msg, stamp_ns)`. No queue,
    no spin_once in the asyncio loop, no interleaving.
  * The asyncio main loop hosts aiohttp, aiortc, and the StreamBackend.
  * Communication between the threads is exclusively through the
    LockFreeLatestFrameSlot / SwitchableFrameSlot.

Readiness:
  /healthz returns three booleans plus per-topic detail. peer_ready only
  transitions true when an actual WebRTC client has signalled and the
  connection state is "connected". server_ready and ros_frame_ready are
  set on this process alone. We never lie about peer readiness before a
  browser connects. In hub mode `topics` reports which modules have
  produced at least one frame, which is how the page can show a module
  whose engine is missing (e.g. M4.3 before its engine is built) as
  not-ready instead of a black rectangle.

Process lifecycle:
  Owned by the bash supervisor (scripts/m4/run_m4_*_demo.sh, or
  scripts/m4/run_m4_web_hub.sh for hub mode). Started via
  `setsid ros2 run m4_demo_bringup m4_web_demo_server ...` and killed by
  `m4_cleanup`. PID/PGID recorded in /tmp/m4_demo/<demo>.webmeta.json
  (env M4_WEB_META_PATH) so `cleanup_demo_residual.sh` can recognise
  orphans on next run.

DO NOT add algorithm logic here. This is the integration layer.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import signal
import threading
import time
import traceback
from functools import partial
from typing import Callable, Dict, Optional, Set

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.node import Node
from sensor_msgs.msg import Image

from aiohttp import web, WSMsgType
from aiortc import (
    RTCConfiguration,
    RTCPeerConnection,
    RTCSessionDescription,
)

from .frame_slot import LockFreeLatestFrameSlot, SwitchableFrameSlot


def _first_video_codec(sdp: str) -> str:
    """Codec name of the first a=rtpmap in the sdp's m=video section.

    In an SDP ANSWER this is the codec the answerer will SEND with, which is
    what decides whether a pre-encoded track can be used at all. aiortc answers
    with VP8 first, so "H264 appears somewhere in the answer" is NOT a valid
    check: an answer always lists every codec it supports.
    """
    in_video = False
    for line in (sdp or "").splitlines():
        line = line.strip()
        if line.startswith("m="):
            in_video = line.startswith("m=video")
            continue
        if in_video and line.startswith("a=rtpmap:"):
            parts = line.split(None, 1)
            if len(parts) == 2 and "/" in parts[1]:
                return parts[1].split("/", 1)[0]
    return ""


from .stream_backend import StreamBackend
from .backend_vp8 import VP8AiortcBackend
from .backend_h264_gst import H264GstBackend, probe as h264_probe
from .backend_mjpeg import BOUNDARY, MjpegBackend

_log = logging.getLogger("m4_web_demo_server")

# Set once in main_async so the /static route can resolve the directory.
_STATIC_DIR_REF = [os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "web", "static"
)]


# ---- Title / copy per demo (UI text) -----------------------------------
DEMO_TITLES = {
    "m4_1": "M4.1 — YOLO TensorRT Detection",
    "m4_2": "M4.2 — ByteTrack Multi-Object Tracking",
    "m4_3": "M4.3 — Semantic Segmentation & Drivable Area",
    "m4_4": "M4.4 — FoundationPose 6D Pose",
}

# Short tab labels for the hub page.
HUB_TITLES = {
    "m4_1": "4.1 检测",
    "m4_2": "4.2 跟踪",
    "m4_3": "4.3 分割",
}

HUB_TITLE = "M4 感知演示 — 统一预览"


def parse_topics(spec: str) -> Dict[str, str]:
    """Parse 'm4_1=<topic>,m4_2=<topic>' into an ordered dict.

    Raises SystemExit with a clear message on a malformed spec, because a
    typo here would otherwise silently produce a hub page with no data.
    """
    mapping: Dict[str, str] = {}
    for chunk in (spec or "").split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if "=" not in chunk:
            raise SystemExit(f"--topics entry {chunk!r} is not in <key>=<topic> form")
        key, topic = chunk.split("=", 1)
        key, topic = key.strip(), topic.strip()
        if not key or not topic:
            raise SystemExit(f"--topics entry {chunk!r} has an empty key or topic")
        mapping[key] = topic
    if not mapping:
        raise SystemExit("--topics must contain at least one <key>=<topic> entry")
    return mapping


# ---- Health state -------------------------------------------------------
class HealthState:
    """Booleans exposed at /healthz. All transitions are sticky.

    In hub mode `_topics` additionally tracks per-module readiness so the
    page can show which tabs actually have frames.
    """

    __slots__ = (
        "server_ready",
        "ros_frame_ready",
        "peer_ready",
        "_peer_count",
        "_topics",
    )

    def __init__(self, topic_keys=()) -> None:
        self.server_ready: bool = False
        self.ros_frame_ready: bool = False
        self.peer_ready: bool = False
        self._peer_count: int = 0
        self._topics: Dict[str, bool] = {k: False for k in topic_keys}

    def set_server_ready(self) -> None:
        self.server_ready = True

    def set_ros_frame_ready(self) -> None:
        if not self.ros_frame_ready:
            _log.info("health: ros_frame_ready=true (first frame received)")
        self.ros_frame_ready = True

    def set_topic_ready(self, key: str) -> None:
        if key and not self._topics.get(key, False):
            _log.info("health: topic %s ready (first frame)", key)
        if key:
            self._topics[key] = True
        self.set_ros_frame_ready()

    def topic_ready(self, key: str) -> bool:
        return bool(self._topics.get(key, False))

    def peer_connected(self) -> None:
        self._peer_count += 1
        if not self.peer_ready:
            _log.info("health: peer_ready=true (peer count=%d)", self._peer_count)
        self.peer_ready = True

    def peer_disconnected(self) -> None:
        if self._peer_count > 0:
            self._peer_count -= 1
        if self._peer_count == 0:
            if self.peer_ready:
                _log.info("health: peer_ready=false (no peers)")
            self.peer_ready = False

    def snapshot(
        self, active: Optional[str] = None, transport: Optional[str] = None,
    ) -> dict:
        snap = {
            "server_ready": self.server_ready,
            "ros_frame_ready": self.ros_frame_ready,
            "peer_ready": self.peer_ready,
            "peer_count": self._peer_count,
        }
        if self._topics:
            snap["topics"] = dict(self._topics)
        if active is not None:
            snap["active"] = active
        if transport:
            snap["transport"] = transport
        return snap


# ---- ROS executor thread -----------------------------------------------
class RosImageSubscriber(Node):
    """Owns the ROS subscription(s) and writes into the shared slot(s).

    Lifecycle:
      * The Node is created in the asyncio main thread.
      * A SingleThreadedExecutor runs it on a dedicated thread.
      * Each callback ONLY calls `slot.try_replace(msg, stamp_ns)`.
      * On shutdown, `executor.shutdown()` is called and the thread joins.

    Single-demo mode uses `attach(topic)`; hub mode uses
    `attach_many({key: topic})` with one slot per key.
    """

    def __init__(self, slot, health: HealthState, node_name: str = "m4_web_demo_server") -> None:
        super().__init__(node_name)
        self._slot = slot
        self._health = health
        self._executor: Optional[SingleThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._subs = []
        # Per-module throughput telemetry. Written on the ROS executor thread
        # and read on the aiohttp thread; every update is a single dict-slot
        # assignment, so a reader can never see a half-built number.
        self._rate_count: Dict[str, int] = {}
        self._rate_start: Dict[str, float] = {}
        self._rate_value: Dict[str, float] = {}
        self._age_ms: Dict[str, float] = {}

    def _qos(self) -> QoSProfile:
        return QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

    def attach(self, topic: str) -> None:
        self._subs.append(self.create_subscription(Image, topic, self._on_image, self._qos()))
        _log.info("subscribed to %s with depth=1 best_effort", topic)

    def attach_many(self, mapping: Dict[str, str]) -> None:
        """Hub mode: one subscription + one slot per module key."""
        for key, topic in mapping.items():
            slot = self._slot.slot_for(key)
            if slot is None:
                _log.warning("no slot for key %s (topic %s) — skipped", key, topic)
                continue
            self._subs.append(
                self.create_subscription(
                    Image,
                    topic,
                    partial(self._on_image_keyed, key=key, slot=slot),
                    self._qos(),
                )
            )
            _log.info("subscribed %s -> %s (depth=1 best_effort)", key, topic)

    def _on_image(self, msg: Image) -> None:
        # Runs on the ROS executor thread ONLY.
        self._slot.try_replace(msg, time.monotonic_ns())
        self._health.set_ros_frame_ready()

    def _on_image_keyed(self, msg: Image, *, key: str, slot) -> None:
        slot.try_replace(msg, time.monotonic_ns())
        self._health.set_topic_ready(key)
        self._note_rate(key, msg)

    def _note_rate(self, key: str, msg: Image) -> None:
        """Count frames per module, and how old the newest one is.

        The count is rolled over on a ~2 s window so the number the page shows
        is a current rate rather than a since-boot average. The age is measured
        against the image header stamped by the camera, i.e. the
        camera-to-server hop — it is NOT an end-to-end latency and the page
        must not label it as one.
        """
        now = time.monotonic()
        start = self._rate_start.get(key)
        if start is None:
            self._rate_start[key] = now
            start = now
        self._rate_count[key] = self._rate_count.get(key, 0) + 1
        elapsed = now - start
        if elapsed >= 2.0:
            self._rate_value[key] = self._rate_count[key] / elapsed
            self._rate_count[key] = 0
            self._rate_start[key] = now

        try:
            stamp = msg.header.stamp
            ros_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            if ros_s > 0.0:
                self._age_ms[key] = max(0.0, (time.time() - ros_s) * 1000.0)
        except Exception:
            # A node without a usable clock simply reports no age.
            pass

    def fps_snapshots(self) -> Dict[str, Dict[str, float]]:
        """Per-module telemetry for /api/demos. Measures, never invents."""
        out: Dict[str, Dict[str, float]] = {}
        for key, fps in self._rate_value.items():
            entry: Dict[str, float] = {"fps": round(fps, 1)}
            age = self._age_ms.get(key)
            if age is not None:
                entry["age_ms"] = round(age, 1)
            out[key] = entry
        return out

    def start(self) -> None:
        if self._executor is not None:
            return
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._thread = threading.Thread(
            target=self._executor.spin,
            name="m4_web_ros_thread",
            daemon=True,
        )
        self._thread.start()
        _log.info("ROS executor thread started")

    def stop(self) -> None:
        try:
            if self._executor is not None:
                self._executor.shutdown()
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        try:
            self.destroy_node()
        except Exception:
            pass


# ---- WebRTC peer management --------------------------------------------
class PeerHub:
    """Tracks active RTCPeerConnections and handles signaling messages.

    On the offer message from the browser we:
      1. create the RTCPeerConnection,
      2. add the backend's track,
      3. set remote description (offer),
      4. create answer, set local description,
      5. send the answer back as JSON over the websocket.

    Hub mode adds a `select` message that switches the active module. The
    backend's track is added once and reused, so a switch does NOT tear
    down or restart the WebRTC connection.
    """

    def __init__(
        self,
        backend: StreamBackend,
        health: HealthState,
        pcs: Set[RTCPeerConnection],
        switchable: Optional[SwitchableFrameSlot] = None,
    ) -> None:
        self._backend = backend
        self._health = health
        self._pcs = pcs
        self._switchable = switchable
        # Backend name drives the H.264 negotiation check below.
        self._backend_name = getattr(backend, "name", "")

    async def handle_offer(self, ws: web.WebSocketResponse, payload: dict) -> None:
        offer = RTCSessionDescription(sdp=payload["sdp"], type=payload["type"])
        pc = RTCPeerConnection(configuration=RTCConfiguration())
        self._pcs.add(pc)

        @pc.on("connectionstatechange")
        async def _on_state_change():
            if pc.connectionState == "connected":
                self._health.peer_connected()
            elif pc.connectionState in ("closed", "failed", "disconnected"):
                self._health.peer_disconnected()
                try:
                    await pc.close()
                except Exception:
                    pass
                self._pcs.discard(pc)

        # Add the backend's track for video.
        pc.addTrack(self._backend.track)

        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        await ws.send_json(
            {
                "type": "answer",
                "sdp": pc.localDescription.sdp,
                "kind": pc.localDescription.type,
            }
        )
        _log.info("WebRTC answer sent (state=%s)", pc.connectionState)

        # The SENDER must end up using H.264, not merely have it available.
        # aiortc answers with its own codec preference (VP8 first), and a
        # presence check is useless because an answer lists every codec it
        # supports. The codec the answerer actually sends with is the FIRST
        # rtpmap of its m=video section, so test that. Otherwise the sender
        # hands H.264 NALs to Vp8Encoder.pack() and the browser decodes
        # nothing — observed: the page reported "streaming" while
        # video.videoWidth stayed 0 and framesDecoded was 0.
        if self._backend_name == "h264_gst":
            chosen = _first_video_codec(pc.localDescription.sdp or "")
            if chosen and "H264" not in chosen.upper():
                _log.warning(
                    "hardware H.264 backend, but the negotiated send codec is "
                    "%s; directing this peer to the MJPEG transport", chosen,
                )
                await ws.send_json(
                    {"type": "transport", "transport": "mjpeg"}
                )

    async def handle_select(self, ws: web.WebSocketResponse, payload: dict) -> None:
        """Hub mode: switch the live module without restarting the stream."""
        key = str(payload.get("demo") or "").strip()
        if self._switchable is None:
            await ws.send_json({"type": "error", "message": "not in hub mode"})
            return
        if not self._switchable.set_active(key):
            _log.warning("select rejected: unknown module %r", key)
            await ws.send_json({"type": "error", "message": f"unknown module {key!r}"})
            return
        # Force a keyframe on every peer. Without this the encoder keeps
        # predicting from the PREVIOUS module's picture, so a switch shows
        # smearing (or nothing at all, because aiortc's VP8 gop_size is 3000)
        # until the next natural IDR.
        for pc in list(self._pcs):
            for sender in pc.getSenders():
                try:
                    sender._send_keyframe()
                except Exception:
                    pass
        _log.info("active module switched to %s", key)
        await ws.send_json(
            {"type": "selected", "demo": key, "ready": self._health.topic_ready(key)}
        )


# ---- aiohttp app factory -----------------------------------------------
def build_app(
    *,
    demo: str,
    backend: StreamBackend,
    static_dir: str,
    health: HealthState,
    switchable: Optional[SwitchableFrameSlot] = None,
    topics: Optional[Dict[str, str]] = None,
    mjpeg: Optional[MjpegBackend] = None,
    transport: str = "",
    get_fps: Optional[Callable[[], Dict[str, Dict[str, float]]]] = None,
) -> web.Application:
    pcs: Set[RTCPeerConnection] = set()
    hub = PeerHub(backend, health, pcs, switchable=switchable)
    topics = topics or {}
    is_hub = demo == "hub"

    async def healthz(_req: web.Request) -> web.Response:
        active = switchable.active if switchable is not None else None
        return web.json_response(
            health.snapshot(active=active, transport=transport)
        )

    async def demos(_req: web.Request) -> web.Response:
        """Module inventory for the hub page (declaration order preserved)."""
        fps = get_fps() if get_fps is not None else {}
        payload = []
        for key, topic in topics.items():
            entry = {
                "key": key,
                "title": HUB_TITLES.get(key, DEMO_TITLES.get(key, key)),
                "topic": topic,
                "ready": health.topic_ready(key),
            }
            # Throughput telemetry only. This never adds, drops or reorders a
            # module, so the hub regression's key/order assertion still holds.
            entry.update(fps.get(key, {}))
            payload.append(entry)
        return web.json_response(
            {
                "mode": demo,
                "modules": payload,
                # Primary transport for the WebRTC-capable path...
                "transport": transport,
                # MJPEG is intentionally opt-in for the hardware H.264 path:
                # keeping a second encoder alive doubled the preview cost on
                # the Jetson even when nobody used the crisp fallback.
                "mjpeg": mjpeg is not None,
                "stream": "/stream" if mjpeg is not None else None,
            }
        )

    async def index(req: web.Request) -> web.Response:
        # /m4/1, /m4/2, /m4/3 -> index.html template with the right title.
        demo_key = req.match_info.get("demo", demo)
        # Never answer a non-page path with HTML: returning the page for
        # "/m4/app.js" is what produced "Unexpected token '<'" in the browser.
        if demo_key not in VALID_DEMO_KEYS:
            _log.warning("page route rejected unknown path segment %r", demo_key)
            return web.Response(status=404, text="unknown demo route")
        if is_hub:
            title = HUB_TITLE
        else:
            title = DEMO_TITLES.get(demo_key, f"M4 Demo ({demo_key})")
        path = os.path.join(static_dir, "index.html")
        if not os.path.isfile(path):
            return web.Response(text="static/index.html missing", status=500)
        with open(path, "r", encoding="utf-8") as f:
            html = f.read()
        # data-demo/data-hub are what app.js reads. They were previously never
        # emitted, so every page behaved as m4_1 regardless of the URL.
        html = html.replace("{{TITLE}}", title)
        html = html.replace("{{DEMO_KEY}}", "hub" if is_hub else demo_key)
        html = html.replace("{{HUB_ATTR}}", 'data-hub="1"' if is_hub else 'data-hub="0"')
        return web.Response(text=html, content_type="text/html")

    async def stream(req: web.Request) -> web.StreamResponse:
        """MJPEG endpoint: multipart/x-mixed-replace, one JPEG per frame.

        Used by the `mjpeg` transport, where the page renders
        <img src="/stream?module=m4_X"> instead of negotiating WebRTC. Each
        frame is independently decodable, so switching modules or reconnecting
        never waits for a keyframe.
        """
        if mjpeg is None:
            return web.Response(status=404, text="mjpeg transport not enabled")
        key = (req.query.get("module") or "").strip()
        target = None
        if switchable is not None:
            target = switchable.slot_for(key) if key else None
            if target is None:
                target = switchable.slot_for(switchable.active)
        if target is None:
            target = getattr(backend, "_slot", None)
        if target is None:
            return web.Response(status=409, text="no frame source")

        resp = web.StreamResponse(
            status=200,
            headers={
                "Content-Type": f"multipart/x-mixed-replace; boundary={BOUNDARY}",
                # No caching, and do not let a proxy buffer the stream.
                "Cache-Control": "no-store, no-cache, must-revalidate",
                "Pragma": "no-cache",
                "X-Accel-Buffering": "no",
            },
        )
        await resp.prepare(req)
        # Serve the requested module by pointing the backend at it for this
        # request via a per-request slot proxy.
        frames = mjpeg.frames_for(target)
        mjpeg.client_opened()
        try:
            async for jpeg in frames:
                await resp.write(
                    b"--" + BOUNDARY.encode() + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                    + jpeg + b"\r\n"
                )
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        except Exception as exc:  # pragma: no cover - client went away
            _log.info("mjpeg stream ended: %s", exc)
        finally:
            mjpeg.client_closed()
            try:
                await resp.write_eof()
            except Exception:
                pass
        return resp

    async def signaling(req: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(req)
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    payload = json.loads(msg.data)
                except Exception:
                    continue
                kind = payload.get("type")
                if kind == "offer":
                    await hub.handle_offer(ws, payload)
                elif kind == "select":
                    await hub.handle_select(ws, payload)
                elif kind == "bye":
                    break
                else:
                    _log.warning("unknown signaling type: %r", kind)
        finally:
            await ws.close()
        return ws

    app = web.Application()
    app.router.add_get("/healthz", healthz)
    # MJPEG transport (multipart/x-mixed-replace).
    app.router.add_get("/stream", stream)
    app.router.add_get("/api/demos", demos)
    app.router.add_get("/signaling", signaling)
    # Assets are referenced ABSOLUTELY (/static/...) from index.html.
    #
    # The original page used href="style.css" RELATIVE to "/m4/1", which has no
    # trailing slash, so the browser resolved it against "/m4/" and requested
    # "/m4/style.css". That matched the catch-all page route "/m4/{demo}" with
    # demo="style.css", which returned index.html with HTTP 200 — and Chrome
    # then tried to parse HTML as JavaScript ("Unexpected token '<'"). See the
    # access log: 'GET /m4/app.js' 200 with the size of index.html.
    app.router.add_get("/static/{name}", static_asset)
    # Kept so a page cached from an older build still loads its assets.
    app.router.add_get("/m4/{demo}/app.js", lambda r: _static(r, "app.js", static_dir))
    app.router.add_get("/m4/{demo}/style.css", lambda r: _static(r, "style.css", static_dir))
    app.router.add_get("/favicon.ico", lambda r: web.Response(status=204))
    # Index for the three demos (and the hub).
    app.router.add_get("/m4/{demo}", index)
    app.router.add_get("/", lambda r: web.HTTPFound("/m4/1"))

    async def _on_cleanup(_app):
        # Close any open peer connections on shutdown.
        for pc in list(pcs):
            try:
                await pc.close()
            except Exception:
                pass
        pcs.clear()

    app.on_cleanup.append(_on_cleanup)
    return app


# Only these files may ever be served as static assets, so a crafted name
# (e.g. "../../etc/passwd") can never escape the static directory.
STATIC_ALLOWLIST = ("app.js", "style.css")

# Path segments accepted by the page route.
VALID_DEMO_KEYS = ("1", "2", "3", "4", "hub", "m4_1", "m4_2", "m4_3", "m4_4")


async def static_asset(req: web.Request) -> web.Response:
    """Serve /static/<name> with an allowlisted name."""
    return await _static(req, req.match_info.get("name", ""), _STATIC_DIR_REF[0])


async def _static(req: web.Request, name: str, static_dir: str) -> web.Response:
    if name not in STATIC_ALLOWLIST:
        return web.Response(status=404, text="unknown asset")
    path = os.path.join(static_dir, name)
    if not os.path.isfile(path):
        return web.Response(status=404)
    # NOTE: web.FileResponse() does NOT accept a `content_type` keyword, so
    # the previous implementation raised TypeError and EVERY asset request
    # returned HTTP 500. index.html loads style.css/app.js as RELATIVE paths
    # (/m4/1/style.css, /m4/1/app.js), so the page had no JS at all: the
    # WebRTC client never ran and the preview stayed black. Reading the file
    # and building a Response keeps MIME types explicit and version-safe.
    with open(path, "rb") as f:
        body = f.read()
    if name.endswith(".js"):
        return web.Response(
            body=body, content_type="application/javascript", charset="utf-8"
        )
    return web.Response(body=body, content_type="text/css", charset="utf-8")


# ---- main_async ---------------------------------------------------------
async def main_async(args: argparse.Namespace) -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    if not rclpy.ok():
        rclpy.init()

    topics: Dict[str, str] = {}
    if args.demo == "hub":
        topics = parse_topics(args.topics)
        health = HealthState(topic_keys=topics.keys())
        # One latest-frame slot per module; the switchable proxy is what the
        # video backend reads, so a module switch never restarts the backend.
        slots = {key: LockFreeLatestFrameSlot(max_age_ms=1000) for key in topics}
        switchable: Optional[SwitchableFrameSlot] = SwitchableFrameSlot(
            slots, active=args.active or next(iter(topics))
        )
        slot_for_backend = switchable
    else:
        health = HealthState()
        switchable = None
        slot_for_backend = LockFreeLatestFrameSlot(max_age_ms=1000)

    # 1. Resolve backend.
    #
    # `auto` prefers real hardware H.264 (near-zero CPU, and the GStreamer
    # encoder's bitrate is NOT clamped by aiortc), then MJPEG (crisp, no
    # negotiation, no inter-frame dependency), then software VP8 as a last
    # resort. Each step is tried for real: a probe returning True is not
    # enough, because pipeline construction or state transition can still fail.
    requested = args.backend
    order = {
        "auto": ["h264", "mjpeg", "vp8"],
        "h264": ["h264", "mjpeg", "vp8"],
        "mjpeg": ["mjpeg", "vp8"],
        "vp8": ["vp8"],
    }[requested]

    backend: Optional[StreamBackend] = None
    mjpeg: Optional[MjpegBackend] = None
    backend_name = ""
    for candidate in order:
        try:
            if candidate == "h264":
                if not h264_probe():
                    _log.info("auto: nvv4l2h264enc not registered, trying next")
                    continue
                cand: StreamBackend = H264GstBackend(
                    slot_for_backend,
                    width=args.encode_width,
                    height=args.encode_height,
                    fps=args.encode_fps,
                    bitrate=args.h264_bitrate,
                )
                cand.start()
            elif candidate == "mjpeg":
                cand = MjpegBackend(
                    slot_for_backend,
                    width=args.encode_width,
                    height=args.encode_height,
                    fps=args.encode_fps,
                    quality=args.jpeg_quality,
                )
                cand.start()
                mjpeg = cand  # type: ignore[assignment]
            else:
                cand = VP8AiortcBackend(slot_for_backend)
                cand.start()
        except Exception as exc:
            _log.warning("backend %s failed to start (%s); trying next", candidate, exc)
            continue
        backend = cand
        backend_name = cand.name
        break

    if backend is None:
        raise SystemExit("no stream backend could be started")
    _log.info("backend=%s started (requested=%s)", backend.name, requested)
    # The hardware path only encodes if something actually feeds appsrc; this
    # coroutine is what makes the pipeline non-idle, and it needs a running
    # event loop, so it starts here rather than inside start().
    start_pump = getattr(backend, "start_pump", None)
    if start_pump is not None:
        await start_pump()
        _log.info("h264 pump coroutine started")

    # MJPEG remains available for software backends and explicit opt-in. The
    # default Jetson H.264 path avoids a second encoder unless requested.
    if mjpeg is None and (backend_name != "h264_gst" or args.mjpeg_side_channel):
        try:
            mjpeg = MjpegBackend(
                slot_for_backend,
                width=args.encode_width,
                height=args.encode_height,
                fps=args.encode_fps,
                quality=args.jpeg_quality,
            )
            mjpeg.start()
            _log.info("mjpeg crisp view ready at /stream")
        except Exception as exc:
            _log.warning("mjpeg side-channel unavailable: %s", exc)
            mjpeg = None

    # 2. Start the ROS executor thread.
    node_name = "m4_web_demo_server" if args.demo != "hub" else "m4_web_hub_server"
    ros_node = RosImageSubscriber(slot_for_backend, health, node_name=node_name)
    if args.demo == "hub":
        ros_node.attach_many(topics)
    else:
        ros_node.attach(args.video_topic)
    ros_node.start()

    # 3. Build the aiohttp app and bind the port.
    static_dir = args.static_dir
    _STATIC_DIR_REF[0] = static_dir
    if not os.path.isdir(static_dir):
        _log.warning("static dir %s missing; UI will fail", static_dir)
    app = build_app(
        demo=args.demo,
        backend=backend,
        static_dir=static_dir,
        health=health,
        switchable=switchable,
        topics=topics,
        mjpeg=mjpeg,
        transport=backend_name,
        get_fps=ros_node.fps_snapshots,
    )

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, args.host, args.port)
    try:
        await site.start()
    except OSError as exc:
        _log.error("bind %s:%d failed: %s", args.host, args.port, exc)
        ros_node.stop()
        backend.stop()
        return 2
    health.set_server_ready()
    _log.info("server_ready http://%s:%d", args.host, args.port)

    # 4. Write ownership metadata so the bash supervisor knows our PGID.
    #    The bash side passes M4_WEB_META_PATH; nothing is written when unset.
    meta_path = os.environ.get("M4_WEB_META_PATH")
    if meta_path:
        try:
            os.makedirs(os.path.dirname(meta_path), exist_ok=True)
            with open(meta_path, "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "demo": args.demo,
                        "pid": os.getpid(),
                        "pgid": os.getpgid(0),
                        "sid": os.getsid(0),
                        "started_at_iso": time.strftime("%Y-%m-%dT%H:%M:%S"),
                        "started_at_tick": _proc_start_tick(os.getpid()),
                        "port": args.port,
                        "host": args.host,
                        "backend": backend.name,
                        "video_topic": args.video_topic,
                        "topics": topics,
                    },
                    f,
                )
            _log.info("webmeta: %s", meta_path)
        except Exception as exc:
            _log.warning("failed to write %s: %s", meta_path, exc)

    # 5. Wait until SIGINT/SIGTERM.
    stop_event = asyncio.Event()

    def _stop(*_a):
        stop_event.set()

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _stop)
        except NotImplementedError:
            pass

    try:
        await stop_event.wait()
    finally:
        _log.info("shutting down")
        health.peer_disconnected()
        await runner.cleanup()
        backend.stop()
        ros_node.stop()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
    return 0


def _proc_start_tick(pid: int) -> int:
    try:
        with open(f"/proc/{pid}/stat", "r") as f:
            return int(f.read().split()[21])
    except Exception:
        return 0


def main() -> int:
    p = argparse.ArgumentParser(description="M4 low-latency web demo server")
    p.add_argument(
        "--demo",
        required=True,
        choices=["m4_1", "m4_2", "m4_3", "m4_4", "hub"],
        help="'hub' serves all modules on one port with a tabbed page",
    )
    # Single-demo mode requires a topic; hub mode uses --topics instead.
    p.add_argument("--video-topic", default="")
    p.add_argument(
        "--topics",
        default="",
        help="hub mode only: 'm4_1=<topic>,m4_2=<topic>,m4_3=<topic>'",
    )
    p.add_argument(
        "--active",
        default="",
        help="hub mode only: initially selected module key (default: first)",
    )
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument(
        "--backend",
        default="h264",
        choices=["auto", "h264", "mjpeg", "vp8"],
        help="h264 = Jetson hardware path; auto falls back to MJPEG/VP8",
    )
    p.add_argument(
        "--encode-width", type=int, default=1280,
        help="encode/JPEG width (frames are fitted inside this box)",
    )
    p.add_argument(
        "--encode-height", type=int, default=720,
        help="encode/JPEG height (frames are fitted inside this box)",
    )
    p.add_argument(
        "--encode-fps", type=int, default=15,
        help="maximum encoded frame rate",
    )
    p.add_argument(
        "--h264-bitrate", type=int, default=3_500_000,
        help="hardware H.264 bitrate in bits/s (aiortc's 1.5/3 Mbps clamps do "
             "NOT apply on this path)",
    )
    p.add_argument(
        "--jpeg-quality", type=int, default=85,
        help="MJPEG JPEG quality (30-95)",
    )
    p.add_argument(
        "--mjpeg-side-channel", action="store_true",
        help="also run the MJPEG fallback beside hardware H.264",
    )
    p.add_argument(
        "--width", type=int, default=1280,
        help="DEPRECATED alias kept for compatibility; use --encode-width",
    )
    p.add_argument(
        "--height", type=int, default=720,
        help="DEPRECATED alias kept for compatibility; use --encode-height",
    )
    p.add_argument(
        "--fps", type=int, default=30,
        help="DEPRECATED alias kept for compatibility; use --encode-fps",
    )
    p.add_argument("--static-dir", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "web", "static",
    ))
    args = p.parse_args()

    if args.demo != "hub" and not args.video_topic:
        raise SystemExit("--video-topic is required unless --demo hub")
    if args.demo == "hub" and not args.topics:
        raise SystemExit("--demo hub requires --topics")

    try:
        return asyncio.run(main_async(args))
    except KeyboardInterrupt:
        return 0
    except Exception:
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
