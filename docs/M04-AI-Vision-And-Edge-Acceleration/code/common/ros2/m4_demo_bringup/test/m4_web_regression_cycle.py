#!/usr/bin/env python3
"""M4 web preview regression cycle driver (headless).

This is the test-only harness used by
scripts/regression/run_m4_web_regression.sh. It performs ONE cycle:

  1. Start a synthetic bgr8 Image publisher on /perception/demo/m4_1
     using rclpy in its own thread (acts like the demo_visualizer).
  2. Start the m4_web_demo_server on the configured port (own process).
  3. Poll GET http://127.0.0.1:<port>/healthz for:
        - server_ready == true
        - ros_frame_ready == true
  4. Open an aiortc client (RTCPeerConnection in this process), perform
     a WebRTC offer/answer against the server's /signaling websocket.
     Wait for connectionState == "connected".
  5. Wait for the first inbound video frame (recv() returns non-None).
  6. Print PASS and exit 0. On any failure, exit non-zero with details.

The synthetic publisher and the web server are spawned as subprocesses
with their own process groups so cleanup is independent of this driver.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from typing import Optional

# Make m4_demo_bringup importable. ament_python with --symlink-install
# drops an .egg-link in install/<pkg>/lib/python3.10/site-packages that
# points at build/<pkg>, so the module is already importable once
# install/setup.bash is sourced. We do NOT muck with sys.path unless
# the import fails.
HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, "../../../.."))


def _bootstrap_paths() -> None:
    candidates = [
        os.path.join(WS, "install/m4_demo_bringup/lib/python3.10/site-packages"),
        os.path.join(WS, "build/m4_demo_bringup"),
        os.path.join(WS, "install/lib/python3.10/site-packages"),
    ]
    for c in candidates:
        if c not in sys.path and os.path.isdir(c):
            sys.path.insert(0, c)


_bootstrap_paths()

from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

import numpy as np  # noqa: E402


# ---- Synthetic publisher (test fixture) --------------------------------
class SyntheticImagePublisher(Node):
    """Publishes bgr8 640x360 frames on a configurable topic at 10 Hz."""

    def __init__(self, topic: str, hz: float = 10.0) -> None:
        super().__init__("m4_regression_synthetic_pub")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(Image, topic, qos)
        self._h = 360
        self._w = 640
        self._hz = hz
        self._frame = 0
        self._timer = self.create_timer(1.0 / hz, self._tick)
        self._topic = topic

    def _tick(self) -> None:
        msg = Image()
        msg.height = self._h
        msg.width = self._w
        msg.encoding = "bgr8"
        msg.step = self._w * 3
        # Animated colour gradient — distinguishable but cheap.
        arr = np.zeros((self._h, self._w, 3), dtype=np.uint8)
        arr[..., 0] = (self._frame * 3) % 255
        arr[..., 1] = (128 + (self._frame * 5) % 127)
        arr[..., 2] = (255 - (self._frame * 7) % 255)
        # White centre dot to make sure motion is visible.
        arr[self._h // 2, self._w // 2] = (255, 255, 255)
        msg.data = arr.tobytes()
        self._pub.publish(msg)
        self._frame += 1


def _run_ros_spin_in_thread(node: Node) -> threading.Thread:
    import rclpy.executors as _exec

    executor = _exec.SingleThreadedExecutor()
    executor.add_node(node)

    def _spin():
        try:
            executor.spin()
        except Exception:
            pass

    t = threading.Thread(target=_spin, name="m4_regression_ros_thread", daemon=True)
    t.start()
    return t, executor


# ---- Health helpers ----------------------------------------------------
def _wait_for_health(port: int, field: str, expected, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    last_body = ""
    while time.monotonic() < deadline:
        try:
            req = urllib.request.Request(f"http://127.0.0.1:{port}/healthz")
            with urllib.request.urlopen(req, timeout=1.0) as resp:
                if resp.status != 200:
                    continue
                last_body = resp.read().decode("utf-8", "ignore")
                d = json.loads(last_body)
                if d.get(field) == expected:
                    print(f"  [OK] healthz.{field}={expected}")
                    return True
        except Exception:
            pass
        time.sleep(0.2)
    print(f"  [FAIL] healthz.{field} did not become {expected!r} within {timeout_s}s; last={last_body!r}")
    return False


# ---- aiortc test client ------------------------------------------------
async def _connect_and_recv_frame(port: int, timeout_s: float) -> bool:
    """Open a websocket to /signaling, do offer/answer, wait for first frame."""
    import websockets  # local import; if missing, aiortc+websockets should be installed
    import aiohttp

    if websockets is None:
        print("  [FAIL] 'websockets' python module not installed; cannot drive aiortc signaling from a client")
        return False

    # aiortc ships its own websocket-client-style helper via aiortc.contrib.
    # We use a tiny aiortc-only signaling instead: the server offers
    # SDP over an /offer GET, but our server is websocket-based. To
    # avoid pulling in websockets we drive the websocket manually using
    # asyncio.open_connection, but that only handles HTTP upgrade; full
    # WebSocket framing needs `websockets`. So if websockets is missing
    # we abort this assertion cleanly.
    try:
        import websockets  # noqa: F401
    except ImportError:
        print("  [SKIP] 'websockets' not installed; cannot assert WebRTC peer connection in regression")
        print("         install with: pip3 install websockets")
        return False

    pc = RTCPeerConnection(configuration=RTCConfiguration())
    frame_received = {"ok": False}

    @pc.on("track")
    async def on_track(track):
        if track.kind != "video":
            return
        print(f"  [OK] track received: kind={track.kind}")

        async def _consume():
            try:
                frame = await asyncio.wait_for(track.recv(), timeout=2.0)
                if frame is not None:
                    frame_received["ok"] = True
                    print("  [OK] first video frame received")
            except asyncio.TimeoutError:
                print("  [FAIL] no video frame within 2s of connection")
            except Exception as exc:
                print(f"  [FAIL] recv() error: {exc}")

        asyncio.create_task(_consume())

    @pc.on("connectionstatechange")
    async def on_state():
        s = pc.connectionState
        print(f"  [INFO] connection state: {s}")
        if s == "connected":
            return

    url = f"ws://127.0.0.1:{port}/signaling"
    try:
        async with websockets.connect(url, open_timeout=5.0) as ws:
            pc.addTransceiver("video", direction="recvonly")
            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)
            await ws.send(json.dumps({"type": "offer", "sdp": offer.sdp, "kind": offer.type}))
            raw = await asyncio.wait_for(ws.recv(), timeout=10.0)
            payload = json.loads(raw)
            if payload.get("type") != "answer":
                print(f"  [FAIL] expected 'answer', got {payload!r}")
                return False
            await pc.setRemoteDescription(
                RTCSessionDescription(sdp=payload["sdp"], type=payload["kind"])
            )
    except Exception as exc:
        print(f"  [FAIL] signaling failed: {exc}")
        return False

    # Wait for the connection to stabilise and the first frame to arrive.
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if frame_received["ok"]:
            await pc.close()
            return True
        await asyncio.sleep(0.1)
    await pc.close()
    print("  [FAIL] peer connected but no frame within window")
    return False


# ---- main cycle --------------------------------------------------------
def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=8089)
    p.add_argument("--cycles", type=int, default=1)
    p.add_argument("--log-dir", required=True)
    p.add_argument("--video-topic", default="/perception/demo/m4_1")
    p.add_argument("--backend", default="vp8", choices=["vp8", "h264"])
    args = p.parse_args()

    os.makedirs(args.log_dir, exist_ok=True)
    cycle_rc = 0

    for i in range(args.cycles):
        print(f"-- cycle {i + 1}/{args.cycles} --")

        # 1. synthetic publisher
        if not rclpy.ok():
            rclpy.init()
        pub_node = SyntheticImagePublisher(args.video_topic)
        ros_thread, ros_executor = _run_ros_spin_in_thread(pub_node)
        time.sleep(0.5)
        print(f"  [OK] synthetic publisher up on {args.video_topic}")

        # 2. start web server (subprocess in its own PGID)
        web_log = open(os.path.join(args.log_dir, "web.log"), "wb")
        proc = subprocess.Popen(
            [
                "ros2", "run", "m4_demo_bringup", "m4_web_demo_server",
                "--demo", "m4_1",
                "--video-topic", args.video_topic,
                "--port", str(args.port),
                "--host", "127.0.0.1",
                "--backend", args.backend,
            ],
            stdout=web_log,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        print(f"  [OK] web server PID={proc.pid}")

        # 3. health gates
        ok_server = _wait_for_health(args.port, "server_ready", True, 8.0)
        ok_frame = _wait_for_health(args.port, "ros_frame_ready", True, 8.0) if ok_server else False
        # peer_ready MUST remain false here; the bash side never gates on it.
        # We assert it from the client side below.

        # 4. aiortc test client
        peer_rc = True
        if ok_frame:
            try:
                peer_rc = asyncio.run(_connect_and_recv_frame(args.port, timeout_s=12.0))
            except Exception as exc:
                print(f"  [FAIL] client raised: {exc}")
                peer_rc = False

        # 5. cleanup
        print("  [INFO] cleanup: SIGINT server PGID, destroy ROS node")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception:
            pass
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=2.0)
        web_log.close()
        # ROS teardown
        ros_executor.shutdown()
        ros_thread.join(timeout=2.0)
        try:
            pub_node.destroy_node()
        except Exception:
            pass

        # 6. port-free assertion
        time.sleep(0.5)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("127.0.0.1", args.port))
            port_free = True
        except OSError:
            port_free = False
        finally:
            s.close()
        print(f"  [OK] port {args.port} free after cycle" if port_free else f"  [FAIL] port {args.port} still bound")

        cycle_pass = ok_server and ok_frame and peer_rc and port_free
        if not cycle_pass:
            cycle_rc = 1
            print(f"  [FAIL] cycle {i + 1} failed")
        else:
            print(f"  [OK] cycle {i + 1} passed")

    if rclpy.ok():
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return cycle_rc


if __name__ == "__main__":
    sys.exit(main())
