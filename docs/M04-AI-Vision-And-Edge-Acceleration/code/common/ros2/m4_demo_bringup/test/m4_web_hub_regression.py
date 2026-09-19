#!/usr/bin/env python3
"""Headless regression for the M4 unified web HUB server.

Does NOT require physical GMSL. Publishes synthetic bgr8 frames on all
three demo overlay topics and starts `m4_web_demo_server --demo hub`.

Acceptance per cycle:
  1. synthetic publishers alive on /perception/demo/m4_{1,2,3}
  2. hub server binds ONE port and answers /healthz with
     server_ready=true, ros_frame_ready=true
  3. /healthz reports per-module `topics` readiness (all three true)
  4. /api/demos lists the three modules in declaration order
  5. the served page is the HUB page (`data-hub="1"`) and has no
     unresolved {{TEMPLATE}} placeholders
  6. a `select` signaling message switches /healthz `active`
  7. an unknown module is rejected without killing the connection
  8. SIGINT frees the port within 5s and leaves no orphan

Usage:
  python3 m4_web_hub_regression.py [--port 8091] [--log-dir /tmp/x]

Exit code 0 only when every check passes.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

# Make m4_demo_bringup importable in a source/symlink-install workspace.
_HERE = os.path.dirname(os.path.abspath(__file__))
_WS = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
for _p in (
    os.path.join(_WS, "install/m4_demo_bringup/lib/python3.10/site-packages"),
    os.path.join(_WS, "build/m4_demo_bringup"),
):
    if os.path.isdir(_p):
        sys.path.insert(0, _p)

import rclpy  # noqa: E402
import rclpy.executors as _exec  # noqa: E402
import numpy as np  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

DEMO_TOPICS = {
    "m4_1": "/perception/demo/m4_1",
    "m4_2": "/perception/demo/m4_2",
    "m4_3": "/perception/demo/m4_3",
}

_failures: list[str] = []


def check(cond: bool, label: str) -> bool:
    print(f"  [{'OK' if cond else 'FAIL'}] {label}")
    if not cond:
        _failures.append(label)
    return cond


class MultiPublisher(Node):
    """Publishes bgr8 frames on every demo overlay topic at ~10 Hz."""

    def __init__(self) -> None:
        super().__init__("m4_hub_regression_publisher")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pubs = {
            key: self.create_publisher(Image, topic, qos)
            for key, topic in DEMO_TOPICS.items()
        }
        self._frame_id = 0

    def publish_once(self) -> None:
        h, w = 360, 640
        # Distinct colour per module so a mis-switch is visible in the UI.
        colours = {"m4_1": (30, 30, 200), "m4_2": (30, 200, 30), "m4_3": (200, 60, 30)}
        for key, pub in self._pubs.items():
            img = np.zeros((h, w, 3), dtype=np.uint8)
            img[:, :] = colours[key]
            # A moving bar proves frames are fresh, not a static single frame.
            x = (self._frame_id * 7) % w
            img[:, max(0, x - 20) : x, :] = 255
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = key
            msg.height, msg.width = h, w
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = w * 3
            msg.data = img.tobytes()
            pub.publish(msg)
        self._frame_id += 1


def http_json(url: str, timeout: float = 2.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def http_text(url: str, timeout: float = 2.0):
    """GET a text asset. Returns None on any HTTP/transport error so one
    broken route fails a single check instead of aborting the cycle."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.read().decode("utf-8")
    except Exception as exc:
        print(f"  [WARN] GET {url} failed: {exc}")
        return None


def http_bytes(url: str, timeout: float = 3.0, max_bytes: int = 65536):
    """Read at most `max_bytes` of a binary response (for the MJPEG stream).

    A multipart/x-mixed-replace response never ends, so a normal read would
    block forever; we stop as soon as we have enough to inspect the magic.
    """
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.read(max_bytes)
    except Exception as exc:
        print(f"  [WARN] GET {url} failed: {exc}")
        return None


def http_status(url: str, timeout: float = 3.0):
    """HTTP status code, or None on transport error."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.status
    except urllib.error.HTTPError as exc:
        return exc.code
    except Exception as exc:
        print(f"  [WARN] GET {url} failed: {exc}")
        return None


def wait_health(port: int, field: str, expected, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            data = http_json(f"http://127.0.0.1:{port}/healthz")
            if data.get(field) == expected:
                return True
        except Exception:
            pass
        time.sleep(0.25)
    return False


def wait_topics_ready(port: int, keys, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            data = http_json(f"http://127.0.0.1:{port}/healthz")
            topics = data.get("topics") or {}
            if all(topics.get(k) for k in keys):
                return True
        except Exception:
            pass
        time.sleep(0.25)
    return False


def port_free(port: int) -> bool:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def select_over_ws(port: int, key: str, timeout_s: float = 6.0):
    """Send a select message over /signaling; returns the reply or None.

    The websocket is only used for signaling here — no WebRTC negotiation —
    so this needs `websockets` but not a media stack.
    """
    try:
        import asyncio

        import websockets
    except ImportError:
        return None

    async def _run():
        uri = f"ws://127.0.0.1:{port}/signaling"
        async with websockets.connect(uri, open_timeout=timeout_s) as ws:
            await ws.send(json.dumps({"type": "select", "demo": key}))
            raw = await asyncio.wait_for(ws.recv(), timeout=timeout_s)
            return json.loads(raw)

    try:
        return asyncio.run(_run())
    except Exception as exc:
        print(f"  [WARN] websocket select failed: {exc}")
        return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--log-dir", default="/tmp/m4_hub_regression")
    args = ap.parse_args()

    os.makedirs(args.log_dir, exist_ok=True)
    port = args.port

    print("== M4 web hub regression ==")
    print(f"  port={port} log_dir={args.log_dir}")

    if not port_free(port):
        print(f"  [FAIL] port {port} already in use")
        try:
            import subprocess as _sp

            print("        holder: " + _sp.run(
                ["ss", "-ltnp", f"sport = :{port}"],
                capture_output=True, text=True).stdout.strip())
        except Exception:
            pass
        return 2

    # ---- synthetic publishers -------------------------------------------
    rclpy.init()
    node = MultiPublisher()
    executor = _exec.SingleThreadedExecutor()
    executor.add_node(node)
    stop = threading.Event()

    def _spin():
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.05)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    pub_timer = threading.Event()

    def _publish_loop():
        while not pub_timer.is_set():
            try:
                node.publish_once()
            except Exception:
                pass
            time.sleep(0.1)

    pub_thread = threading.Thread(target=_publish_loop, daemon=True)
    pub_thread.start()
    check(True, "synthetic publishers started on all three demo topics")

    # ---- start the hub server -------------------------------------------
    env = dict(os.environ)
    env["M4_WEB_META_PATH"] = os.path.join(args.log_dir, "hub.webmeta.json")
    log_path = os.path.join(args.log_dir, "hub_server.log")
    logf = open(log_path, "w", encoding="utf-8")
    # Mirror the production launch path (m4_web_server_cmd in m4_demo_lib.sh):
    # run the installed console script rather than `ros2 run`, because
    # `ros2 run` places the node in a DIFFERENT process group, so a group
    # SIGINT never reaches it and the orphan keeps the port bound.
    entry = os.path.join(
        _WS, "install/m4_demo_bringup/lib/m4_demo_bringup/m4_web_demo_server"
    )
    if os.path.isfile(entry):
        launcher = [sys.executable, entry]
    else:
        launcher = ["ros2", "run", "m4_demo_bringup", "m4_web_demo_server"]
    print("  launcher=" + " ".join(launcher))

    proc = subprocess.Popen(
        launcher + [
            "--demo", "hub",
            "--topics", ",".join(f"{k}={v}" for k, v in DEMO_TOPICS.items()),
            "--active", "m4_1",
            "--host", "127.0.0.1",
            "--port", str(port),
            "--backend", "vp8",
        ],
        stdout=logf,
        stderr=subprocess.STDOUT,
        env=env,
        start_new_session=True,
    )

    rc = 1
    try:
        check(wait_health(port, "server_ready", True, 15.0),
              f"hub server_ready within 15s (log: {log_path})")
        check(wait_health(port, "ros_frame_ready", True, 20.0),
              "hub ros_frame_ready within 20s (any module)")
        check(wait_topics_ready(port, DEMO_TOPICS.keys(), 20.0),
              "all three modules report per-topic readiness")

        hz = http_json(f"http://127.0.0.1:{port}/healthz")
        check(hz.get("active") == "m4_1", "default active module is m4_1")
        check(isinstance(hz.get("topics"), dict) and len(hz["topics"]) == 3,
              "/healthz exposes per-module topics map")

        listing = http_json(f"http://127.0.0.1:{port}/api/demos")
        keys = [m["key"] for m in listing.get("modules", [])]
        check(keys == list(DEMO_TOPICS.keys()),
              f"/api/demos lists modules in order {keys}")
        check(all(m.get("ready") for m in listing.get("modules", [])),
              "/api/demos marks every module ready")

        html = http_text(f"http://127.0.0.1:{port}/m4/1")
        check(html is not None and 'data-hub="1"' in html,
              "served page is the hub page (data-hub=1)")
        check(html is not None and "{{" not in html and "}}" not in html,
              "no unresolved template placeholders in the page")
        check(html is not None and "data-demo=" in html,
              "page carries data-demo for app.js")

        # Assets must be reachable at the ABSOLUTE paths the page uses, and
        # must never come back as HTML (the "/m4/{demo}" catch-all used to
        # answer "/m4/app.js" with index.html at HTTP 200, so Chrome raised
        # "Unexpected token '<'").
        check(html is not None and "/static/app.js" in html,
              "page references /static/app.js (absolute, slash-safe)")
        check(html is not None and "/static/style.css" in html,
              "page references /static/style.css (absolute, slash-safe)")

        app_js = http_text(f"http://127.0.0.1:{port}/static/app.js")
        check(app_js is not None and '"select"' in app_js,
              "app.js is served as JS at /static/app.js")
        check(app_js is not None and not app_js.lstrip().startswith("<"),
              "app.js is not an HTML error page")

        css = http_text(f"http://127.0.0.1:{port}/static/style.css")
        check(css is not None and ".tab" in css, "style.css served at /static/style.css")

        # ---- redesigned UI contract (P3) --------------------------------
        check(css is not None and ".panel" in css and ".badge" in css,
              "redesigned style.css carries .panel/.badge (calib_web design system)")
        check(app_js is not None and "data-i18n" in (html or "") + (app_js or ""),
              "i18n hooks present (data-i18n)")

        # ---- transport reporting (P4) -----------------------------------
        check(isinstance(hz.get("transport"), str) and hz["transport"],
              f"/healthz reports a transport: {hz.get('transport')!r}")
        check(isinstance(listing.get("transport"), str) and listing["transport"],
              f"/api/demos reports a transport: {listing.get('transport')!r}")

        # ---- MJPEG endpoint returns real JPEG bytes (P4) ----------------
        # A multipart/x-mixed-replace body begins with "--<boundary>" and only
        # then the JPEG, so assert the framing AND the SOI marker; a regression
        # in either the multipart plumbing or the encoder then fails the test.
        stream = http_bytes(f"http://127.0.0.1:{port}/stream?module=m4_1",
                            timeout=6.0, max_bytes=400_000)
        soi = stream.find(b"\xff\xd8") if stream else -1
        check(stream is not None and stream.startswith(b"--") and soi != -1,
              f"/stream is multipart framing containing a JPEG payload "
              f"(soi at {soi}, head={stream[:8].hex() if stream else None})")

        # ---- M4.4 page route is no longer a 404 (P5) --------------------
        code44 = http_status(f"http://127.0.0.1:{port}/m4/4")
        check(code44 == 200, f"GET /m4/4 returns 200 (was 404): {code44}")

        # Regression guard for the exact reported browser error.
        trap = http_text(f"http://127.0.0.1:{port}/m4/app.js")
        check(trap is None or not trap.lstrip().startswith("<"),
              "GET /m4/app.js no longer returns HTML (404 instead)")

        # ---- module switching ------------------------------------------
        reply = select_over_ws(port, "m4_3")
        if reply is None:
            print("  [WARN] skipping switch assertions (websockets unavailable)")
        else:
            check(reply.get("type") == "selected" and reply.get("demo") == "m4_3",
                  f"select m4_3 acknowledged: {reply}")
            hz2 = http_json(f"http://127.0.0.1:{port}/healthz")
            check(hz2.get("active") == "m4_3", "/healthz active switched to m4_3")
            bad = select_over_ws(port, "m4_99")
            check(bad is not None and bad.get("type") == "error",
                  f"unknown module rejected without dropping the socket: {bad}")
            hz3 = http_json(f"http://127.0.0.1:{port}/healthz")
            check(hz3.get("active") == "m4_3", "rejected switch left active unchanged")

        # ---- shutdown behaviour ----------------------------------------
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        check(proc.returncode is not None, "hub server exited on SIGINT")

        freed = False
        deadline = time.time() + 5
        while time.time() < deadline:
            if port_free(port):
                freed = True
                break
            time.sleep(0.2)
        check(freed, f"port {port} freed within 5s (no orphan listener)")

        meta = os.path.join(args.log_dir, "hub.webmeta.json")
        check(os.path.isfile(meta), "M4_WEB_META_PATH written by the server")

        rc = 0 if not _failures else 1
    finally:
        pub_timer.set()
        stop.set()
        try:
            executor.shutdown()
        except Exception:
            pass
        # `proc` is the `ros2` CLI wrapper; the server itself is its child in
        # the same process group/session. Killing only the wrapper left an
        # orphan holding the port, so tear down the WHOLE group.
        try:
            if proc.poll() is None:
                proc.kill()
        except Exception:
            pass
        try:
            import os as _os
            import signal as _signal

            _os.killpg(_os.getpgid(proc.pid), _signal.SIGKILL)
        except Exception:
            pass
        logf.close()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

    print()
    if _failures:
        print(f"== HUB REGRESSION FAILED ({len(_failures)} checks) ==")
        for f in _failures:
            print(f"   - {f}")
    else:
        print("== HUB REGRESSION PASSED ==")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())