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
import shutil
import signal
import subprocess
import sys
import threading
import time
import traceback
import uuid
from functools import partial
from pathlib import Path
from typing import Callable, Dict, Optional, Set

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, UInt64
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters, SetParametersAtomically

from .control_settings import DEFAULTS, LIMITS, VERSION, load_settings, validate_patch, write_settings
from .video_file_publisher import MAX_VIDEO_BYTES, probe_video_file

from aiohttp import web, WSMsgType
from aiortc import (
    RTCConfiguration,
    RTCPeerConnection,
    RTCSessionDescription,
)
from aiortc.contrib.media import MediaRelay

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
    "m4_4": "4.4 位姿",
}

# Per-module startup grace (seconds) before topic_status flips a
# never-seen topic from "starting" to "offline". M4.4 launches the Isaac ROS
# FoundationPose container pipeline: engines load in ~30-60 s, and a first
# run that has to build the score engine takes ~213 s.
_MODULE_START_GRACE = {
    "m4_4": 240.0,
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


def parse_unavailable_modules(spec: str) -> Dict[str, str]:
    """Parse an optional '<key>=<reason>[,<key>=<reason>]' inventory."""
    if not spec:
        return {}
    return parse_topics(spec)


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
        "_topic_last_seen",
        "_started_at",
        "_slow_start",
    )

    def __init__(self, topic_keys=(), slow_start: Optional[Dict[str, float]] = None) -> None:
        self.server_ready: bool = False
        self.ros_frame_ready: bool = False
        self.peer_ready: bool = False
        self._peer_count: int = 0
        self._topics: Dict[str, bool] = {k: False for k in topic_keys}
        self._topic_last_seen: Dict[str, float] = {}
        self._started_at = time.monotonic()
        self._slow_start: Dict[str, float] = dict(slow_start or {})

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
            self._topic_last_seen[key] = time.monotonic()
        self.set_ros_frame_ready()

    def topic_ready(self, key: str) -> bool:
        return self.topic_status(key)["status"] == "ready"

    def reset_topics(self) -> None:
        """Discard readiness from the stopped module before a new one starts."""
        self._topics = {key: False for key in self._topics}
        self._topic_last_seen.clear()
        self._started_at = time.monotonic()

    def topic_status(self, key: str) -> dict:
        """Live module state, separate from the sticky health evidence."""
        now = time.monotonic()
        grace = self._slow_start.get(key, 15.0)
        seen = self._topic_last_seen.get(key)
        if seen is None:
            elapsed = now - self._started_at
            if elapsed <= grace:
                return {"status": "starting", "error": None, "last_frame_age_ms": None}
            return {
                "status": "offline",
                "error": f"no frames received from {key} within {grace:.0f} seconds",
                "last_frame_age_ms": None,
            }
        age_ms = max(0.0, (now - seen) * 1000.0)
        if age_ms > 3000.0:
            return {
                "status": "offline",
                "error": f"frame stream stalled for {age_ms / 1000.0:.1f} seconds",
                "last_frame_age_ms": round(age_ms, 1),
            }
        return {"status": "ready", "error": None, "last_frame_age_ms": round(age_ms, 1)}

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

    def set_peer_count(self, count: int) -> None:
        """Synchronize health with the peer set owned by PeerHub."""
        count = max(0, int(count))
        changed = count != self._peer_count
        self._peer_count = count
        self.peer_ready = count > 0
        if changed:
            _log.info("health: peer_count=%d", count)

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
        self._executor_lock = threading.Lock()
        self._subscription_renew_lock = threading.Lock()
        self._executor_stopping = False
        self._subs = []
        # Per-module throughput telemetry. Written on the ROS executor thread
        # and read on the aiohttp thread; every update is a single dict-slot
        # assignment, so a reader can never see a half-built number.
        self._rate_count: Dict[str, int] = {}
        self._rate_start: Dict[str, float] = {}
        self._rate_value: Dict[str, float] = {}
        self._age_ms: Dict[str, float] = {}
        self._input_state_lock = threading.Lock()
        self._input_source = "camera"
        self._input_last_seen: Dict[str, float] = {}
        self._input_publisher = None
        self._input_subscriptions = []
        self._video_subscription = None
        self._camera_topic = "/perception/inputs/camera"
        self._video_topic = "/perception/inputs/video"
        self._output_topic = "/perception/cameras/front/image"
        self._loop_topic = "/perception/inputs/video_loop"
        self._input_received = {"camera": 0, "video": 0}
        self._input_forwarded = {"camera": 0, "video": 0}
        self._video_loop_count = 0
        self._segmentation_stats: Optional[dict] = None
        self._segmentation_stats_seen = 0.0
        self._pose_stats: Optional[dict] = None
        self._pose_stats_seen = 0.0
        self._tracker_reset_client = self.create_client(
            Trigger, "/tracking_node/reset_tracker")

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

    def attach_pose_stats(self, topic: str = "/perception/demo/m4_4/stats") -> None:
        """Subscribe to the M4.4 pose telemetry (JSON String).

        Attached unconditionally in hub mode — also without
        --managed-modules — so the regression server can assert the
        /api/visualization/m4_4 endpoint with a synthetic publisher.
        """
        self._subs.append(
            self.create_subscription(String, topic, self._on_pose_stats, self._qos()))
        _log.info("subscribed to pose stats %s", topic)

    def _on_pose_stats(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                self._pose_stats = payload
                self._pose_stats_seen = time.monotonic()
        except Exception:
            _log.warning("ignored malformed m4_4 stats message")

    def attach_input_mux(
        self,
        camera_topic: str = "/perception/inputs/camera",
        video_topic: str = "/perception/inputs/video",
        output_topic: str = "/perception/cameras/front/image",
        loop_topic: str = "/perception/inputs/video_loop",
    ) -> None:
        """Forward exactly one selected source to the stable perception topic."""
        self._camera_topic = camera_topic
        self._video_topic = video_topic
        self._output_topic = output_topic
        self._loop_topic = loop_topic
        qos = self._qos()
        self._input_publisher = self.create_publisher(Image, output_topic, qos)
        for source, topic in (("camera", camera_topic), ("video", video_topic)):
            subscription = self.create_subscription(
                Image, topic, partial(self._on_input, source=source), qos)
            self._input_subscriptions.append(subscription)
            if source == "video":
                self._video_subscription = subscription
        self._input_subscriptions.append(
            self.create_subscription(UInt64, loop_topic, self._on_video_loop, qos))
        self._input_subscriptions.append(
            self.create_subscription(
                String, "/perception/demo/m4_3/stats",
                self._on_segmentation_stats, qos))
        _log.info(
            "input mux: camera=%s video=%s -> %s", camera_topic, video_topic, output_topic)

    def _on_input(self, msg: Image, *, source: str) -> None:
        with self._input_state_lock:
            self._input_last_seen[source] = time.monotonic()
            self._input_received[source] += 1
            selected = source == self._input_source
        if selected and self._input_publisher is not None:
            self._input_publisher.publish(msg)
            with self._input_state_lock:
                self._input_forwarded[source] += 1

    def renew_video_subscription(self, timeout: float = 3.0) -> None:
        """Recreate the video reader on the ROS executor thread, not aiohttp's thread.

        A fresh publisher can be visible to another ROS subscriber while the
        Hub's long-lived endpoint receives nothing after repeated uploads.
        Recreating this endpoint before each new publisher avoids retaining a
        stale DDS match without disrupting the camera or WebRTC subscriptions.
        """
        with self._subscription_renew_lock:
            with self._executor_lock:
                executor = self._executor
                stopping = self._executor_stopping
                thread = getattr(self, "_thread", None)
            if executor is None:
                return  # Unit tests and the pre-start initialization path.
            if stopping or thread is None or not thread.is_alive():
                raise RuntimeError("ROS executor is not running; cannot renew video subscription")

            done = threading.Event()
            failure = []

            def replace() -> None:
                fresh = None
                try:
                    old = self._video_subscription
                    fresh = self.create_subscription(
                        Image, self._video_topic,
                        partial(self._on_input, source="video"), self._qos())
                    if old is not None:
                        self.destroy_subscription(old)
                        if old in self._input_subscriptions:
                            self._input_subscriptions.remove(old)
                    self._video_subscription = fresh
                    self._input_subscriptions.append(fresh)
                    _log.info("renewed Hub video subscription before publisher start")
                except Exception as exc:
                    if fresh is not None and fresh is not self._video_subscription:
                        try:
                            self.destroy_subscription(fresh)
                        except Exception:
                            pass
                    failure.append(exc)
                finally:
                    done.set()

            task = executor.create_task(replace)
            if not done.wait(timeout):
                task.cancel()
                raise RuntimeError("Hub video subscription renewal timed out")
            if failure:
                raise RuntimeError(f"Hub video subscription renewal failed: {failure[0]}")

    def _on_video_loop(self, msg: UInt64) -> None:
        with self._input_state_lock:
            self._video_loop_count = int(msg.data)
            selected = self._input_source == "video"
        if selected:
            self.reset_tracker()

    def _on_segmentation_stats(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                self._segmentation_stats = payload
                self._segmentation_stats_seen = time.monotonic()
        except Exception:
            _log.warning("ignored malformed m4_3 stats message")

    def begin_input_session(self, source: str) -> None:
        """Forget telemetry from a previous publisher session."""
        if source not in ("camera", "video"):
            raise ValueError("input source must be camera or video")
        with self._input_state_lock:
            self._input_last_seen.pop(source, None)
            self._input_received[source] = 0
            self._input_forwarded[source] = 0
            if source == "video":
                self._video_loop_count = 0

    def set_input_source(self, source: str) -> None:
        if source not in ("camera", "video"):
            raise ValueError("input source must be camera or video")
        with self._input_state_lock:
            if source != "video":
                self._video_loop_count = 0
            self._input_source = source

    def input_snapshot(self) -> dict:
        now = time.monotonic()
        with self._input_state_lock:
            source = self._input_source
            last_seen = dict(self._input_last_seen)
            loop_count = self._video_loop_count
            received = dict(self._input_received)
            forwarded = dict(self._input_forwarded)
        return {
            "source": source,
            "last_frame_age_ms": {
                key: round(max(0.0, (now - seen) * 1000.0), 1)
                for key, seen in last_seen.items()
            },
            "loop_count": loop_count,
            "video_topic": self._video_topic,
            "received": received,
            "forwarded": forwarded,
        }

    def segmentation_stats(self) -> Optional[dict]:
        if self._segmentation_stats is None:
            return None
        payload = dict(self._segmentation_stats)
        payload["age_ms"] = round(
            max(0.0, (time.monotonic() - self._segmentation_stats_seen) * 1000.0), 1)
        return payload

    def pose_stats(self) -> Optional[dict]:
        if self._pose_stats is None:
            return None
        payload = dict(self._pose_stats)
        payload["age_ms"] = round(
            max(0.0, (time.monotonic() - self._pose_stats_seen) * 1000.0), 1)
        return payload

    def reset_tracker(self) -> bool:
        if not self._tracker_reset_client.service_is_ready():
            return False
        self._tracker_reset_client.call_async(Trigger.Request())
        return True

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

    def reset_metrics(self) -> None:
        """Forget samples belonging to the module that was just stopped."""
        self._rate_count.clear()
        self._rate_start.clear()
        self._rate_value.clear()
        self._age_ms.clear()

    def start(self) -> None:
        with self._subscription_renew_lock:
            with self._executor_lock:
                if self._executor is not None:
                    return
                self._executor_stopping = False
                executor = SingleThreadedExecutor()
                executor.add_node(self)
                thread = threading.Thread(
                    target=executor.spin,
                    name="m4_web_ros_thread",
                    daemon=True,
                )
                self._executor = executor
                self._thread = thread
            thread.start()
        _log.info("ROS executor thread started")

    def stop(self) -> None:
        with self._subscription_renew_lock:
            with self._executor_lock:
                executor = self._executor
                thread = self._thread
                self._executor_stopping = True
            try:
                if executor is not None:
                    executor.shutdown(timeout_sec=2.0)
            except Exception:
                pass
            if thread is not None:
                thread.join(timeout=2.0)
            try:
                self.destroy_node()
            except Exception:
                pass
            with self._executor_lock:
                self._executor = None
                self._thread = None


# ---- Runtime controls ----------------------------------------------------
class RuntimeControls:
    """Bridge authenticated-by-origin HTTP controls to ROS parameter services.

    The web server and rclpy executor intentionally live on different threads.
    rclpy futures are therefore polled asynchronously instead of ever blocking
    aiohttp's event loop or performing shell ``ros2 param`` calls.
    """

    _NODE_FOR_KEY = {
        "undistort_enabled": "/csi_camera_publisher",
        "confidence_threshold": "/yolo_trt_node",
        "nms_threshold": "/yolo_trt_node",
        "track_activation_threshold": "/tracking_node",
        "minimum_matching_threshold": "/tracking_node",
        "lost_track_buffer": "/tracking_node",
        "minimum_consecutive_frames": "/tracking_node",
    }

    def __init__(self, node: Node) -> None:
        self._node = node
        controlled_nodes = set(self._NODE_FOR_KEY.values()) | {"/segmentation_visualizer"}
        self._clients = {
            name: {
                "get": node.create_client(GetParameters, name + "/get_parameters"),
                "set": node.create_client(
                    SetParametersAtomically, name + "/set_parameters_atomically"),
            }
            for name in sorted(controlled_nodes)
        }
        self.values, self.warning = load_settings()
        self.undistort_available = False
        self.undistort_error = "camera parameter service unavailable"
        self._lock = asyncio.Lock()

    @staticmethod
    async def _wait(future, timeout: float = 2.0):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            await asyncio.sleep(0.01)
        if not future.done():
            raise TimeoutError("ROS parameter service timed out")
        return future.result()

    async def _set(self, node_name: str, patch: dict) -> tuple[bool, str]:
        try:
            client = self._clients.get(node_name)
            if client is None:
                return False, f"{node_name}: parameter client is not registered"
            request = SetParametersAtomically.Request()
            request.parameters = [
                Parameter(key, value=value).to_parameter_msg()
                for key, value in patch.items()
            ]
            response = await self._wait(client["set"].call_async(request))
            if response.result.successful:
                return True, ""
            return False, response.result.reason or f"{node_name} rejected the update"
        except Exception as exc:
            return False, f"{node_name}: {exc}"

    async def _refresh_camera_state(self) -> None:
        try:
            request = GetParameters.Request()
            request.names = ["undistort_enabled", "undistort_available", "undistort_error"]
            response = await self._wait(
                self._clients["/csi_camera_publisher"]["get"].call_async(request), timeout=0.75)
            values = response.values
            if len(values) == 3:
                self.values["undistort_enabled"] = bool(values[0].bool_value)
                self.undistort_available = bool(values[1].bool_value)
                self.undistort_error = str(values[2].string_value or "")
                return
        except Exception as exc:
            self.undistort_error = f"camera parameter service unavailable: {exc}"
        self.undistort_available = False

    async def initialize(self) -> None:
        await self._refresh_camera_state()
        # A persisted true value is never allowed to masquerade as active when
        # the camera rejected it because calibration is missing or invalid.
        if self.values["undistort_enabled"] and not self.undistort_available:
            self.values["undistort_enabled"] = False
            try:
                write_settings(self.values)
            except Exception as exc:
                self.warning = f"{self.warning or ''} could not repair settings: {exc}".strip()

    async def snapshot(self) -> dict:
        await self._refresh_camera_state()
        return {
            "version": VERSION,
            "values": dict(self.values),
            "defaults": dict(DEFAULTS),
            "limits": {
                key: {"min": low, "max": high, "step": step}
                for key, (low, high, step) in LIMITS.items()
            },
            "undistort_available": self.undistort_available,
            "undistort_error": self.undistort_error,
            "warning": self.warning,
        }

    async def apply(self, patch: dict) -> dict:
        checked, error = validate_patch(patch)
        if error or checked is None or not checked:
            return {"ok": False, "error": error or "empty settings patch"}
        async with self._lock:
            candidate = dict(self.values)
            candidate.update(checked)
            changed = {key: value for key, value in checked.items()
                       if self.values.get(key) != value}
            if not changed:
                return {"ok": True, "changed": [], "tracker_reset": False,
                        "values": dict(self.values)}

            grouped: Dict[str, dict] = {}
            for key, value in changed.items():
                grouped.setdefault(self._NODE_FOR_KEY[key], {})[key] = value
            completed: list[tuple[str, dict]] = []
            for node_name, group in grouped.items():
                ok, reason = await self._set(node_name, group)
                if not ok:
                    # Each node update is atomic.  Roll back earlier groups so
                    # the user never gets a silently mixed configuration.
                    for completed_node, completed_group in reversed(completed):
                        await self._set(completed_node, {
                            key: self.values[key] for key in completed_group})
                    await self._refresh_camera_state()
                    return {"ok": False, "error": reason, "values": dict(self.values)}
                completed.append((node_name, group))

            self.values = candidate
            await self._refresh_camera_state()
            if checked.get("undistort_enabled") and not self.values["undistort_enabled"]:
                return {"ok": False, "error": self.undistort_error or "undistortion was rejected",
                        "values": dict(self.values)}
            try:
                write_settings(self.values)
            except Exception as exc:
                # Runtime values remain active but the UI must make the restart
                # persistence failure visible rather than claiming success.
                return {"ok": False, "error": f"applied but could not save: {exc}",
                        "values": dict(self.values)}
            return {
                "ok": True,
                "changed": sorted(changed),
                "tracker_reset": any(key in changed for key in (
                    "track_activation_threshold", "minimum_matching_threshold",
                    "lost_track_buffer", "minimum_consecutive_frames")),
                "values": dict(self.values),
            }

    async def reset(self) -> dict:
        return await self.apply(dict(DEFAULTS))

    async def set_node_parameters(self, node_name: str, patch: dict) -> tuple[bool, str]:
        return await self._set(node_name, patch)


# ---- Hub single-module runtime ownership --------------------------------
class ModuleRuntimeManager:
    """Own exactly one course pipeline below the shared camera at a time.

    The Hub intentionally keeps its ROS image subscriptions, encoder and
    WebRTC peers alive.  Only the selected ROS launch process changes.  This
    gives the Jetson all of its inference budget to the selected lesson while
    preserving the browser connection during a chapter switch.
    """

    _FLAGS = {
        "m4_1": ("true", "false", "false"),
        # Tracking needs YOLO detections, but this is the 4.2 dependency, not
        # a separately selected 4.1 lesson.
        "m4_2": ("true", "true", "false"),
        "m4_3": ("false", "false", "true"),
    }

    # Docker teardown (docker exec client dies -> quickstart deferred TERM
    # trap -> container-side cleanup) needs longer than the 5 s default; a
    # premature SIGKILL would leak the container launch + looping bag.
    _STOP_GRACE = {
        "m4_4": 20.0,
    }

    # M4.4 runs through the Isaac ROS FoundationPose container via a wrapper
    # script (scripts/m4/m4_4_hub_module.sh) instead of a host ros2 launch.
    _HUB_MODULE_SCRIPT_ENV = "M4_4_HUB_MODULE_SCRIPT"

    def _module_command(self, key: str) -> Optional[list]:
        """Command line that starts a module, or None if it cannot run here."""
        if key in self._FLAGS:
            detection, tracking, segmentation = self._FLAGS[key]
            return [
                "ros2", "launch", "m4_demo_bringup", "m4_all_demo.launch.py",
                "camera_topic:=/perception/cameras/front/image",
                "detections_topic:=/perception/detections",
                "tracks_topic:=/perception/tracks",
                f"enable_detection:={detection}",
                f"enable_tracking:={tracking}",
                f"enable_segmentation:={segmentation}",
                f"segmentation_max_fps:={self._segmentation_fps}",
                f"segmentation_view_mode:={self._segmentation_view}",
            ]
        if key == "m4_4":
            script = os.environ.get(self._HUB_MODULE_SCRIPT_ENV, "")
            if script and os.path.isfile(script):
                return ["bash", script]
        return None

    def __init__(
        self, health: HealthState, available: Dict[str, str], segmentation_fps: float,
        reset_telemetry: Optional[Callable[[], None]] = None,
    ) -> None:
        self._health = health
        self._available = dict(available)
        self._segmentation_fps = segmentation_fps
        self._reset_telemetry = reset_telemetry
        self._active: Optional[str] = None
        self._proc: Optional[subprocess.Popen] = None
        self._log_file = None
        self._lock = asyncio.Lock()
        self._segmentation_view = "semantic"

    @property
    def active(self) -> Optional[str]:
        return self._active

    def is_active(self, key: str) -> bool:
        return key == self._active

    @property
    def segmentation_view(self) -> str:
        return self._segmentation_view

    def remember_segmentation_view(self, view: str) -> None:
        if view not in ("original", "semantic", "drivable"):
            raise ValueError("invalid M4.3 view")
        self._segmentation_view = view

    async def _terminate_locked(self) -> None:
        proc = self._proc
        if proc is None:
            return
        grace = self._STOP_GRACE.get(self._active, 5.0)
        if proc.poll() is None:
            try:
                # This server is itself started as a detached background job.
                # SIGINT may therefore be inherited as ignored by descendants;
                # SIGTERM is the supervisor's real shutdown contract.
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                await asyncio.wait_for(asyncio.to_thread(proc.wait), timeout=grace)
            except asyncio.TimeoutError:
                _log.warning(
                    "module process group %s did not stop within %.0fs; sending SIGKILL",
                    proc.pid, grace)
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                await asyncio.to_thread(proc.wait)
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None
        self._proc = None
        self._active = None

    async def stop(self) -> None:
        async with self._lock:
            await self._terminate_locked()

    async def activate(self, key: str) -> tuple[bool, str]:
        async with self._lock:
            command = self._module_command(key) if key in self._available else None
            if command is None:
                return False, f"module {key!r} is unavailable"
            if self._active == key and self._proc is not None and self._proc.poll() is None:
                return True, ""

            await self._terminate_locked()
            self._health.reset_topics()
            if self._reset_telemetry is not None:
                self._reset_telemetry()
            log_path = os.environ.get("M4_MODULE_RUNTIME_LOG", "/tmp/m4_module_runtime.log")
            try:
                self._log_file = open(log_path, "a", encoding="utf-8")
                self._log_file.write(
                    f"\n=== {time.strftime('%Y-%m-%d %H:%M:%S')} activate {key} ===\n"
                )
                self._log_file.flush()
                self._proc = subprocess.Popen(
                    command,
                    stdin=subprocess.DEVNULL,
                    stdout=self._log_file,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    env=os.environ.copy(),
                )
            except Exception as exc:
                if self._log_file is not None:
                    self._log_file.close()
                    self._log_file = None
                self._proc = None
                return False, f"could not start {key}: {exc}"

            self._active = key
            # Surface a launch syntax/missing-artifact failure immediately;
            # longer startup remains a normal `starting` UI state.
            await asyncio.sleep(0.25)
            if self._proc.poll() is not None:
                code = self._proc.returncode
                await self._terminate_locked()
                return False, f"{key} exited during startup (code {code}; see {log_path})"
            _log.info("single-module runtime active=%s pid=%s", key, self._proc.pid)
            return True, ""


class VideoInputManager:
    """Own the single persistent upload and the optional looping publisher."""

    def __init__(
        self,
        ros_node: RosImageSubscriber,
        storage_dir: str,
        state_path: str,
    ) -> None:
        self._ros_node = ros_node
        self._storage_dir = Path(storage_dir)
        self._state_path = Path(state_path)
        self._source = "camera"
        self._video: Optional[dict] = None
        self._proc: Optional[subprocess.Popen] = None
        self._log_file = None
        self._started_at = 0.0
        self._error: Optional[str] = None
        self._lock = asyncio.Lock()
        self._load_state()

    def _load_state(self) -> None:
        try:
            payload = json.loads(self._state_path.read_text(encoding="utf-8"))
            video = payload.get("video")
            source = payload.get("source", "camera")
            if isinstance(video, dict):
                path = str(video.get("path") or "")
                probed = probe_video_file(path)
                self._video = dict(probed)
                self._video.update({
                    "path": path,
                    "filename": os.path.basename(str(video.get("filename") or Path(path).name)),
                })
            self._source = source if source in ("camera", "video") else "camera"
            if self._source == "video" and self._video is None:
                self._source = "camera"
        except FileNotFoundError:
            return
        except Exception as exc:
            self._source = "camera"
            self._video = None
            self._error = f"saved video state is invalid: {exc}"

    def _save_state(self) -> None:
        self._state_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self._state_path.with_name(self._state_path.name + ".tmp")
        payload = {"version": 1, "source": self._source, "video": self._video}
        tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(tmp, self._state_path)

    async def initialize(self) -> None:
        async with self._lock:
            if self._source == "video" and self._video is not None:
                try:
                    await self._start_video_locked()
                    self._ros_node.set_input_source("video")
                except Exception as exc:
                    self._error = f"could not restore video input: {exc}"
                    self._source = "camera"
                    self._ros_node.set_input_source("camera")
                    self._save_state()
            else:
                self._ros_node.set_input_source("camera")

    async def _start_video_locked(self) -> None:
        if self._video is None:
            raise RuntimeError("no uploaded video is available")
        if self._proc is not None and self._proc.poll() is None:
            return
        await self._stop_video_locked()
        await asyncio.to_thread(self._ros_node.renew_video_subscription)
        self._ros_node.begin_input_session("video")
        log_path = os.environ.get("M4_VIDEO_INPUT_LOG", "/tmp/m4_video_input.log")
        self._log_file = open(log_path, "a", encoding="utf-8")
        self._log_file.write(
            f"\n=== {time.strftime('%Y-%m-%d %H:%M:%S')} play "
            f"{self._video['filename']} ===\n")
        self._log_file.flush()
        command = self._publisher_command()
        try:
            self._proc = subprocess.Popen(
                command, stdin=subprocess.DEVNULL, stdout=self._log_file,
                stderr=subprocess.STDOUT, start_new_session=True, env=os.environ.copy())
            _log.info("video publisher pid=%s cmd=%s", self._proc.pid, " ".join(command))
            self._started_at = time.monotonic()
            await asyncio.sleep(0.2)
            if self._proc.poll() is not None:
                code = self._proc.returncode
                raise RuntimeError(f"video publisher exited during startup (code {code})")
        except Exception:
            await self._stop_video_locked()
            raise

    def _publisher_command(self) -> list:
        """Start the installed console script, not `ros2 run`."""
        args = [
            "--ros-args",
            "-p", f"video_path:={self._video['path']}",
            "-p", "output_topic:=/perception/inputs/video",
            "-p", "loop_topic:=/perception/inputs/video_loop",
            "-p", "target_width:=1920", "-p", "target_height:=1080",
            "-p", "max_fps:=30.0",
        ]
        ws_root = os.environ.get("M4_WS_ROOT", "")
        if not ws_root:
            raise RuntimeError("M4_WS_ROOT must point to the built ROS workspace")
        entry = os.path.join(
            ws_root, "install", "m4_demo_bringup", "lib", "m4_demo_bringup",
            "video_file_publisher")
        if os.path.isfile(entry):
            return [sys.executable, entry, *args]
        raise RuntimeError(
            f"installed video_file_publisher is missing at {entry}; "
            "rebuild m4_demo_bringup instead of falling back to ros2 run")

    async def _stop_video_locked(self) -> None:
        proc = self._proc
        self._proc = None
        if proc is not None and proc.poll() is None:
            try:
                # Match the supervisor shutdown contract. A detached child can
                # inherit SIGINT as ignored and otherwise survive a clean Hub
                # shutdown as an orphaned publisher.
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                await asyncio.wait_for(asyncio.to_thread(proc.wait), timeout=3.0)
            except asyncio.TimeoutError:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                await asyncio.to_thread(proc.wait)
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None

    async def _fallback_to_camera_locked(self, reason: str) -> None:
        """Return to the live camera and preserve the failure for the UI."""
        self._error = reason
        self._source = "camera"
        self._ros_node.set_input_source("camera")
        await self._stop_video_locked()
        self._save_state()
        _log.warning("video input reverted to camera: %s", reason)

    async def select(self, source: str) -> dict:
        if source not in ("camera", "video"):
            raise ValueError("source must be camera or video")
        async with self._lock:
            if source == "video":
                if self._video is None:
                    raise ValueError("upload a valid video before selecting video input")
                await self._start_video_locked()
            self._ros_node.set_input_source(source)
            self._ros_node.reset_tracker()
            self._source = source
            self._error = None
            if source == "camera":
                await self._stop_video_locked()
            self._save_state()
            return await self._snapshot_locked()

    async def snapshot(self) -> dict:
        async with self._lock:
            return await self._snapshot_locked()

    async def _snapshot_locked(self) -> dict:
        mux = self._ros_node.input_snapshot()
        status = "ready" if self._video is not None else "unavailable"
        error = self._error
        if self._source == "video":
            if self._proc is None or self._proc.poll() is not None:
                error = "video playback stopped unexpectedly; reverted to camera"
                await self._fallback_to_camera_locked(error)
                status = "error"
            else:
                age = mux["last_frame_age_ms"].get("video")
                if age is None:
                    status = "starting"
                    if time.monotonic() - self._started_at > 20.0:
                        received = mux["received"].get("video", 0)
                        forwarded = mux["forwarded"].get("video", 0)
                        if self._proc.poll() is not None:
                            error = "video publisher exited before producing frames; reverted to camera"
                        elif received == 0:
                            error = "Hub received no frames from the video publisher; reverted to camera"
                        elif forwarded == 0:
                            error = "Hub received video frames but forwarded none; reverted to camera"
                        else:
                            error = "video input produced no usable frames; reverted to camera"
                        _log.error(
                            "video first-frame timeout: publisher_pid=%s received=%s forwarded=%s topic=%s",
                            self._proc.pid, mux["received"], mux["forwarded"], mux["video_topic"])
                        await self._fallback_to_camera_locked(error)
                        status = "error"
                elif age > 3000.0:
                    status = "error"
                    error = f"video stream stalled for {age / 1000.0:.1f} seconds"
                    await self._fallback_to_camera_locked(error)
                else:
                    status = "playing"
        public_video = None
        if self._video is not None:
            public_video = {key: value for key, value in self._video.items() if key != "path"}
            public_video.update({
                "status": status,
                "error": error,
                "loop_count": mux["loop_count"],
            })
        return {
            "source": self._source,
            "available_sources": ["camera"] + (["video"] if self._video else []),
            "video": public_video,
            "error": error,
        }

    async def install_upload(self, field, original_filename: str) -> dict:
        filename = os.path.basename(original_filename or "video")
        extension = Path(filename).suffix.lower()
        if extension not in (".mp4", ".mov", ".mkv"):
            raise ValueError("only MP4, MOV and MKV files are supported")
        self._storage_dir.mkdir(parents=True, exist_ok=True)
        free = shutil.disk_usage(self._storage_dir).free
        if free < 2 * 1024 * 1024 * 1024:
            raise ValueError("Jetson has less than 2 GB of free storage")
        temp_path = self._storage_dir / f".{uuid.uuid4().hex}.upload{extension}"
        total = 0
        try:
            with open(temp_path, "xb") as out:
                while True:
                    chunk = await field.read_chunk(size=1024 * 1024)
                    if not chunk:
                        break
                    total += len(chunk)
                    if total > MAX_VIDEO_BYTES:
                        raise ValueError("video exceeds the 4 GB limit")
                    if total + 2 * 1024 * 1024 * 1024 > free:
                        raise ValueError("not enough free storage for this upload")
                    out.write(chunk)
                out.flush()
                os.fsync(out.fileno())
            metadata = await asyncio.to_thread(probe_video_file, str(temp_path))
            async with self._lock:
                old_video = dict(self._video) if self._video else None
                old_source = self._source
                old_path = Path(old_video["path"]) if old_video else None
                backup_path = None
                await self._stop_video_locked()
                final_path = self._storage_dir / ("current" + extension)
                if old_path is not None and old_path.exists():
                    backup_path = self._storage_dir / (".previous" + old_path.suffix)
                    os.replace(old_path, backup_path)
                try:
                    os.replace(temp_path, final_path)
                    self._video = dict(metadata)
                    self._video.update({"path": str(final_path), "filename": filename})
                    self._source = "video"
                    await self._start_video_locked()
                    self._ros_node.set_input_source("video")
                    self._ros_node.reset_tracker()
                    self._error = None
                    self._save_state()
                except Exception:
                    await self._stop_video_locked()
                    try:
                        final_path.unlink(missing_ok=True)
                    except Exception:
                        pass
                    if backup_path is not None and backup_path.exists() and old_path is not None:
                        os.replace(backup_path, old_path)
                    self._video = old_video
                    self._source = old_source if old_video or old_source == "camera" else "camera"
                    if self._source == "video" and self._video is not None:
                        await self._start_video_locked()
                    self._ros_node.set_input_source(self._source)
                    self._save_state()
                    raise
                if backup_path is not None:
                    backup_path.unlink(missing_ok=True)
                for stale in self._storage_dir.glob("current.*"):
                    if stale != final_path:
                        stale.unlink(missing_ok=True)
                return await self._snapshot_locked()
        finally:
            temp_path.unlink(missing_ok=True)

    async def delete(self) -> dict:
        async with self._lock:
            self._ros_node.set_input_source("camera")
            self._ros_node.reset_tracker()
            await self._stop_video_locked()
            if self._video is not None:
                Path(self._video["path"]).unlink(missing_ok=True)
            self._video = None
            self._source = "camera"
            self._error = None
            self._save_state()
            return await self._snapshot_locked()

    async def close(self) -> None:
        async with self._lock:
            await self._stop_video_locked()


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
        mjpeg_available: bool = False,
        module_manager: Optional["ModuleRuntimeManager"] = None,
    ) -> None:
        self._backend = backend
        self._health = health
        self._pcs = pcs
        self._switchable = switchable
        self._mjpeg_available = mjpeg_available
        self._module_manager = module_manager
        self._relay = MediaRelay()
        self._connected: Set[RTCPeerConnection] = set()
        # Backend name drives the H.264 negotiation check below.
        self._backend_name = getattr(backend, "name", "")

    async def handle_offer(
        self, ws: web.WebSocketResponse, payload: dict,
    ) -> RTCPeerConnection:
        offer = RTCSessionDescription(sdp=payload["sdp"], type=payload["type"])
        pc = RTCPeerConnection(configuration=RTCConfiguration())
        self._pcs.add(pc)

        @pc.on("connectionstatechange")
        async def _on_state_change():
            if pc.connectionState == "connected":
                self._connected.add(pc)
                self._health.set_peer_count(len(self._connected))
            elif pc.connectionState in ("closed", "failed", "disconnected"):
                await self.close_peer(pc)

        # One backend track feeds all peers. The unbuffered relay broadcasts
        # the newest encoded packet without letting slow tabs build queues or
        # compete for alternating samples from appsink.
        pc.addTrack(self._relay.subscribe(self._backend.track, buffered=False))

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
                if self._mjpeg_available:
                    _log.warning(
                        "hardware H.264 backend, but the negotiated send codec is "
                        "%s; directing this peer to the MJPEG transport", chosen,
                    )
                    await ws.send_json(
                        {"type": "transport", "transport": "mjpeg"}
                    )
                else:
                    # Do not point the client at /stream when the low-CPU
                    # default deliberately did not start a second encoder.
                    # A clear error is preferable to a permanent black video
                    # element and an endless reconnect loop.
                    _log.error(
                        "hardware H.264 backend negotiated %s, but MJPEG "
                        "fallback is disabled", chosen,
                    )
                    await ws.send_json({
                        "type": "error",
                        "code": "codec_unavailable",
                        "message": (
                            "浏览器未协商 H.264；请使用支持 H.264 的浏览器，"
                            "或以 WEB_MJPEG_SIDE_CHANNEL=1 重启服务"
                        ),
                    })
                    await self.close_peer(pc)
        return pc

    async def close_peer(self, pc: Optional[RTCPeerConnection]) -> None:
        if pc is None:
            return
        self._connected.discard(pc)
        self._pcs.discard(pc)
        self._health.set_peer_count(len(self._connected))
        if pc.connectionState != "closed":
            try:
                await pc.close()
            except Exception:
                pass

    async def close_all(self) -> None:
        for pc in list(self._pcs):
            await self.close_peer(pc)

    async def handle_select(self, ws: web.WebSocketResponse, payload: dict) -> None:
        """Hub mode: switch the live module without restarting the stream."""
        key = str(payload.get("demo") or "").strip()
        if self._switchable is None:
            await ws.send_json({"type": "error", "message": "not in hub mode"})
            return
        if self._module_manager is not None:
            ok, reason = await self._module_manager.activate(key)
            if not ok:
                _log.warning("select rejected for %s: %s", key, reason)
                await ws.send_json({"type": "error", "message": reason})
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
    unavailable_modules: Optional[Dict[str, str]] = None,
    mjpeg: Optional[MjpegBackend] = None,
    transport: str = "",
    get_fps: Optional[Callable[[], Dict[str, Dict[str, float]]]] = None,
    stream_info: Optional[dict] = None,
    controls: Optional[RuntimeControls] = None,
    module_manager: Optional["ModuleRuntimeManager"] = None,
    input_manager: Optional[VideoInputManager] = None,
    ros_node: Optional[RosImageSubscriber] = None,
) -> web.Application:
    pcs: Set[RTCPeerConnection] = set()
    hub = PeerHub(
        backend,
        health,
        pcs,
        switchable=switchable,
        mjpeg_available=mjpeg is not None,
        module_manager=module_manager,
    )
    topics = topics or {}
    unavailable_modules = unavailable_modules or {}
    stream_info = dict(stream_info or {})
    is_hub = demo == "hub"

    async def healthz(_req: web.Request) -> web.Response:
        active = switchable.active if switchable is not None else None
        payload = health.snapshot(active=active, transport=transport)
        payload["video"] = stream_info
        return web.json_response(payload)

    async def demos(_req: web.Request) -> web.Response:
        """Module inventory for the hub page (declaration order preserved)."""
        fps = get_fps() if get_fps is not None else {}
        payload = []
        for key, topic in topics.items():
            if module_manager is not None and not module_manager.is_active(key):
                status = {"status": "idle", "error": None, "last_frame_age_ms": None}
            else:
                status = health.topic_status(key)
            entry = {
                "key": key,
                "title": HUB_TITLES.get(key, DEMO_TITLES.get(key, key)),
                "topic": topic,
                "ready": status["status"] == "ready",
            }
            entry.update(status)
            # Throughput telemetry only. This never adds, drops or reorders a
            # module, so the hub regression's key/order assertion still holds.
            entry.update(fps.get(key, {}))
            payload.append(entry)
        for key, reason in unavailable_modules.items():
            payload.append({
                "key": key,
                "title": HUB_TITLES.get(key, DEMO_TITLES.get(key, key)),
                "topic": "",
                "ready": False,
                "status": "unavailable",
                "error": reason,
                "last_frame_age_ms": None,
            })
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
                "video": stream_info,
            }
        )

    async def settings(_req: web.Request) -> web.Response:
        if controls is None:
            return web.json_response({"error": "runtime controls unavailable"}, status=503)
        return web.json_response(await controls.snapshot())

    async def patch_settings(req: web.Request) -> web.Response:
        if controls is None:
            return web.json_response({"ok": False, "error": "runtime controls unavailable"}, status=503)
        try:
            payload = await req.json()
        except Exception:
            return web.json_response({"ok": False, "error": "JSON object required"}, status=400)
        patch = payload.get("values", payload) if isinstance(payload, dict) else None
        outcome = await controls.apply(patch)
        return web.json_response(outcome, status=200 if outcome.get("ok") else 409)

    async def reset_settings(_req: web.Request) -> web.Response:
        if controls is None:
            return web.json_response({"ok": False, "error": "runtime controls unavailable"}, status=503)
        outcome = await controls.reset()
        return web.json_response(outcome, status=200 if outcome.get("ok") else 409)

    async def input_state(_req: web.Request) -> web.Response:
        if input_manager is None:
            return web.json_response({"error": "input control unavailable"}, status=503)
        return web.json_response(await input_manager.snapshot())

    async def patch_input(req: web.Request) -> web.Response:
        if input_manager is None:
            return web.json_response({"ok": False, "error": "input control unavailable"}, status=503)
        try:
            payload = await req.json()
            if not isinstance(payload, dict) or set(payload) != {"source"}:
                raise ValueError("expected exactly one field: source")
            state = await input_manager.select(str(payload["source"]))
            return web.json_response({"ok": True, **state})
        except ValueError as exc:
            return web.json_response({"ok": False, "error": str(exc)}, status=400)
        except Exception as exc:
            _log.exception("input switch failed")
            return web.json_response({"ok": False, "error": str(exc)}, status=409)

    async def upload_video(req: web.Request) -> web.Response:
        if input_manager is None:
            return web.json_response({"ok": False, "error": "input control unavailable"}, status=503)
        if req.content_length is not None and req.content_length > MAX_VIDEO_BYTES + 1024 * 1024:
            return web.json_response({"ok": False, "error": "video exceeds the 4 GB limit"}, status=413)
        try:
            reader = await req.multipart()
            field = await reader.next()
            if field is None or field.name != "file" or not field.filename:
                raise ValueError("multipart field 'file' is required")
            state = await input_manager.install_upload(field, field.filename)
            return web.json_response({"ok": True, **state}, status=201)
        except ValueError as exc:
            return web.json_response({"ok": False, "error": str(exc)}, status=400)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            _log.exception("video upload failed")
            return web.json_response({"ok": False, "error": str(exc)}, status=409)

    async def delete_video(_req: web.Request) -> web.Response:
        if input_manager is None:
            return web.json_response({"ok": False, "error": "input control unavailable"}, status=503)
        try:
            state = await input_manager.delete()
            return web.json_response({"ok": True, **state})
        except Exception as exc:
            return web.json_response({"ok": False, "error": str(exc)}, status=409)

    async def visualization_state(_req: web.Request) -> web.Response:
        view = module_manager.segmentation_view if module_manager is not None else "semantic"
        stats = ros_node.segmentation_stats() if ros_node is not None else None
        if module_manager is not None and not module_manager.is_active("m4_3"):
            stats = None
        return web.json_response({
            "view": view,
            "options": ["original", "semantic", "drivable"],
            "results": stats,
        })

    async def pose_visualization_state(_req: web.Request) -> web.Response:
        stats = ros_node.pose_stats() if ros_node is not None else None
        if module_manager is not None and not module_manager.is_active("m4_4"):
            stats = None
        return web.json_response({"results": stats})

    async def patch_visualization(req: web.Request) -> web.Response:
        if controls is None or module_manager is None:
            return web.json_response({"ok": False, "error": "visualization control unavailable"}, status=503)
        try:
            payload = await req.json()
            if not isinstance(payload, dict) or set(payload) != {"view"}:
                raise ValueError("expected exactly one field: view")
            view = str(payload["view"])
            if view not in ("original", "semantic", "drivable"):
                raise ValueError("view must be original, semantic or drivable")
            if module_manager.is_active("m4_3"):
                ok, reason = await controls.set_node_parameters(
                    "/segmentation_visualizer", {"view_mode": view})
                if not ok:
                    return web.json_response({"ok": False, "error": reason}, status=409)
            module_manager.remember_segmentation_view(view)
            return web.json_response({"ok": True, "view": view})
        except ValueError as exc:
            return web.json_response({"ok": False, "error": str(exc)}, status=400)
        except Exception as exc:
            return web.json_response({"ok": False, "error": str(exc)}, status=409)

    async def index(req: web.Request) -> web.Response:
        # /m4/1, /m4/2, /m4/3 -> index.html template with the right title.
        route_demo = req.match_info.get("demo", demo)
        # Never answer a non-page path with HTML: returning the page for
        # "/m4/app.js" is what produced "Unexpected token '<'" in the browser.
        if route_demo not in VALID_DEMO_KEYS:
            _log.warning("page route rejected unknown path segment %r", route_demo)
            return web.Response(status=404, text="unknown demo route")
        demo_key = ROUTE_TO_DEMO_KEY.get(route_demo, route_demo)
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
        # In hub mode preserve the requested route (for example /m4/3) so the
        # browser can select that chapter after its one shared WebRTC session
        # is ready.  The hub flag still controls multi-module behaviour.
        html = html.replace("{{DEMO_KEY}}", demo_key)
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
        peer: Optional[RTCPeerConnection] = None
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
                    if peer is not None:
                        await hub.close_peer(peer)
                    peer = await hub.handle_offer(ws, payload)
                elif kind == "select":
                    await hub.handle_select(ws, payload)
                elif kind == "bye":
                    break
                else:
                    _log.warning("unknown signaling type: %r", kind)
        finally:
            await hub.close_peer(peer)
            await ws.close()
        return ws

    app = web.Application(client_max_size=MAX_VIDEO_BYTES + 1024 * 1024)
    app.router.add_get("/healthz", healthz)
    # MJPEG transport (multipart/x-mixed-replace).
    app.router.add_get("/stream", stream)
    app.router.add_get("/api/demos", demos)
    app.router.add_get("/api/settings", settings)
    app.router.add_patch("/api/settings", patch_settings)
    app.router.add_post("/api/settings/reset", reset_settings)
    app.router.add_get("/api/input", input_state)
    app.router.add_patch("/api/input", patch_input)
    app.router.add_post("/api/input/video", upload_video)
    app.router.add_delete("/api/input/video", delete_video)
    app.router.add_get("/api/visualization/m4_3", visualization_state)
    app.router.add_patch("/api/visualization/m4_3", patch_visualization)
    app.router.add_get("/api/visualization/m4_4", pose_visualization_state)
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
        await hub.close_all()
        if module_manager is not None:
            await module_manager.stop()
        if input_manager is not None:
            await input_manager.close()

    app.on_cleanup.append(_on_cleanup)
    return app


# Only these files may ever be served as static assets, so a crafted name
# (e.g. "../../etc/passwd") can never escape the static directory.
STATIC_ALLOWLIST = ("app.js", "style.css")

# Path segments accepted by the page route.
VALID_DEMO_KEYS = ("1", "2", "3", "4", "hub", "m4_1", "m4_2", "m4_3", "m4_4")
ROUTE_TO_DEMO_KEY = {"1": "m4_1", "2": "m4_2", "3": "m4_3", "4": "m4_4"}


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
    unavailable_modules: Dict[str, str] = {}
    if args.demo == "hub":
        topics = parse_topics(args.topics)
        unavailable_modules = parse_unavailable_modules(args.unavailable_modules)
        health = HealthState(topic_keys=topics.keys(), slow_start=_MODULE_START_GRACE)
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
        # Pose telemetry is hub-mode-unconditional (not tied to
        # --managed-modules) so the regression server can drive it too.
        ros_node.attach_pose_stats()
        if args.managed_modules:
            ros_node.attach_input_mux()
    else:
        ros_node.attach(args.video_topic)
    ros_node.start()
    controls = RuntimeControls(ros_node)
    await controls.initialize()

    input_manager: Optional[VideoInputManager] = None
    if args.demo == "hub" and args.managed_modules:
        input_manager = VideoInputManager(
            ros_node,
            os.environ.get(
                "M4_VIDEO_STORAGE_DIR", str(Path.home() / ".local/share/m4-perception/input")),
            os.environ.get(
                "M4_VIDEO_STATE_PATH", str(Path.home() / ".config/m4-perception/video_input.json")),
        )
        await input_manager.initialize()

    module_manager: Optional[ModuleRuntimeManager] = None
    if args.managed_modules:
        if args.demo != "hub":
            raise SystemExit("--managed-modules is valid only with --demo hub")
        if not 1.0 <= args.segmentation_max_fps <= 30.0:
            raise SystemExit("--segmentation-max-fps must be within 1..30")
        module_manager = ModuleRuntimeManager(
            health, topics, args.segmentation_max_fps, ros_node.reset_metrics,
        )
        initial = switchable.active if switchable is not None else ""
        ok, reason = await module_manager.activate(initial)
        if not ok:
            ros_node.stop()
            backend.stop()
            raise SystemExit(f"initial module launch failed: {reason}")

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
        unavailable_modules=unavailable_modules,
        mjpeg=mjpeg,
        transport=backend_name,
        get_fps=ros_node.fps_snapshots,
        stream_info={
            "width": args.encode_width,
            "height": args.encode_height,
            "fps": args.encode_fps,
            "bitrate": args.h264_bitrate if backend_name == "h264_gst" else None,
        },
        controls=controls,
        module_manager=module_manager,
        input_manager=input_manager,
        ros_node=ros_node,
    )

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, args.host, args.port)
    try:
        await site.start()
    except OSError as exc:
        _log.error("bind %s:%d failed: %s", args.host, args.port, exc)
        if module_manager is not None:
            await module_manager.stop()
        if input_manager is not None:
            await input_manager.close()
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
    p.add_argument(
        "--unavailable-modules",
        default="",
        help="hub mode: '<key>=<reason>' entries retained in /api/demos",
    )
    p.add_argument(
        "--managed-modules", action="store_true",
        help="hub mode: keep one selected ROS inference pipeline active at a time",
    )
    p.add_argument(
        "--segmentation-max-fps", type=float, default=10.0,
        help="M4.3 cap used by --managed-modules (1..30)",
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
        "--encode-width", type=int, default=1920,
        help="encode/JPEG width (frames are fitted inside this box)",
    )
    p.add_argument(
        "--encode-height", type=int, default=1080,
        help="encode/JPEG height (frames are fitted inside this box)",
    )
    p.add_argument(
        "--encode-fps", type=int, default=30,
        help="maximum encoded frame rate",
    )
    p.add_argument(
        "--h264-bitrate", type=int, default=8_000_000,
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
        "--width", type=int, default=1920,
        help="DEPRECATED alias kept for compatibility; use --encode-width",
    )
    p.add_argument(
        "--height", type=int, default=1080,
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
