#!/usr/bin/env python3
"""Subscribe to a ROS Image topic and show it as an OpenCV window in real time.

Replaces the PNG-screenshot workflow with a live cv2.imshow window, so the yolo
debug image (with bounding boxes / class labels / confidence) is visible while
the script runs.

Window lives on $DISPLAY (defaults to :10 for VNC). Ctrl-C (SIGINT) closes the
window cleanly. Pressing 'q' while the window is focused also quits.

CLI:
  cv_viewer.py [--topic /perception/debug/detection_image]
               [--display :10] [--width 960]
               [--window "YOLO live"] [--timeout 0]
"""
import argparse
import os
import signal
import sys
import time

# Set DISPLAY BEFORE importing cv2 (cv2 captures the X11 connection at import)
# Use the env var if set, otherwise default to :10 (VNC on this Jetson).
if "DISPLAY" not in os.environ or not os.environ["DISPLAY"]:
    os.environ["DISPLAY"] = ":10"

# Silence the Qt font-database warnings that OpenCV spits out on startup when
# it picks the QT backend for window decoration. They don't affect rendering
# of the image itself (yolo already drew all text). We filter at the file-descriptor
# level by replacing fd 2 with one that drops matching lines.
class _StderrFilter:
    """File-like wrapper that drops known-noisy Qt font warnings."""
    _PATTERNS = (
        b"QFontDatabase: Cannot find font directory",
        b"Note that Qt no longer ships fonts",
        b"qt.qpa.plugin: Could not find the Qt platform plugin",
    )
    def __init__(self, real):
        self._real = real
    def write(self, s):
        if isinstance(s, str):
            sb = s.encode("utf-8", "replace")
        else:
            sb = s
        for p in self._PATTERNS:
            if p in sb:
                return len(s) if isinstance(s, str) else len(sb)
        self._real.write(s)
        return len(s) if isinstance(s, str) else len(sb)
    def flush(self):
        try:
            self._real.flush()
        except Exception:
            pass

sys.stderr = _StderrFilter(sys.stderr)

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    QoSProfile, ReliabilityPolicy, HistoryPolicy,
)
from sensor_msgs.msg import Image  # noqa: E402


_BGR_ENCODINGS = {"bgr8", "rgb8"}
_BGRA_ENCODINGS = {"bgra8", "rgba8"}
_MONO_ENCODINGS = {"mono8", "8UC1"}


def image_msg_to_numpy(msg):
    """Convert sensor_msgs/Image to a HxWx3 uint8 BGR numpy array.

    Mirrors the implementation in screenshot_saver.py so we don't pull in
    cv_bridge and so encoding edge-cases (mono16, bayer, 32FC1) behave the same.
    """
    h, w = msg.height, msg.width
    enc = msg.encoding or "bgr8"
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in _BGR_ENCODINGS:
        if enc == "rgb8":
            arr = raw.reshape(h, w, 3)[:, :, ::-1].copy()
        else:
            arr = raw.reshape(h, w, 3).copy()
    elif enc in _BGRA_ENCODINGS:
        bgra = raw.reshape(h, w, 4).copy()
        arr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
    elif enc in _MONO_ENCODINGS:
        gray = raw.reshape(h, w).copy()
        arr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    elif enc == "16UC1" or enc == "mono16":
        arr16 = raw.reshape(h, w).copy()
        arr8 = cv2.convertScaleAbs(arr16, alpha=(255.0 / 65535.0))
        arr = cv2.cvtColor(arr8, cv2.COLOR_GRAY2BGR)
    elif enc == "32FC1":
        arr32 = raw.reshape(h, w).copy()
        arr8 = cv2.normalize(arr32, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        arr = cv2.cvtColor(arr8, cv2.COLOR_GRAY2BGR)
    elif enc == "bayer_rggb8":
        bayer = raw.reshape(h, w).copy()
        arr = cv2.cvtColor(bayer, cv2.COLOR_BayerRGGB2BGR)
    else:
        raise ValueError(f"unsupported encoding {enc!r}")
    return arr


class CvViewer(Node):
    def __init__(self, topic, window_name, max_width, timeout_s):
        super().__init__("cv_viewer")
        self.window_name = window_name
        self.max_width = max_width
        self.timeout_s = timeout_s
        self.frames_shown = 0
        self.start = time.monotonic()
        self.last_log = self.start
        self.quit_requested = False
        self.got_first_frame = False

        # Install our own SIGINT/SIGTERM handlers so we can close the window
        # BEFORE rclpy tears the node down (otherwise the window may linger
        # until X11 garbage-collects it).
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.sub = self.create_subscription(Image, topic, self.on_msg, qos)
        self.get_logger().info(
            f"Subscribing to {topic} (BEST_EFFORT), DISPLAY={os.environ.get('DISPLAY','?')}, "
            f"window='{window_name}', max_width={max_width}"
        )

    def _on_signal(self, signum, frame):
        self.quit_requested = True

    def on_msg(self, msg):
        try:
            arr = image_msg_to_numpy(msg)
        except Exception as e:
            self.get_logger().warn(f"image parse failed: {e}")
            return

        # Optional down-scale so the window fits comfortably in the VNC viewer.
        h, w = arr.shape[:2]
        if self.max_width and w > self.max_width:
            scale = self.max_width / float(w)
            new_w = self.max_width
            new_h = max(1, int(round(h * scale)))
            arr_show = cv2.resize(arr, (new_w, new_h), interpolation=cv2.INTER_AREA)
        else:
            arr_show = arr

        cv2.imshow(self.window_name, arr_show)
        # waitKey(1) is REQUIRED for HighGUI to actually pump events on Linux/X11.
        # Also gives us a chance to catch the 'q' key (focus must be on the window).
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("'q' pressed, quitting")
            self.quit_requested = True

        self.frames_shown += 1
        self.got_first_frame = True
        now = time.monotonic()
        if now - self.last_log > 2.0:
            elapsed = now - self.start
            fps = self.frames_shown / elapsed if elapsed > 0 else 0.0
            self.get_logger().info(
                f"window: {self.frames_shown} frames shown "
                f"({fps:.1f} fps), size={arr_show.shape[1]}x{arr_show.shape[0]}"
            )
            self.last_log = now

    def run(self):
        # We do not call rclpy.spin() because it blocks the waitKey() event loop
        # (the OpenCV window would freeze). Instead, pump rclpy manually each
        # loop iteration. KeyboardInterrupt is caught by the outer try/except in
        # main() and turns into a clean exit.
        try:
            while rclpy.ok() and not self.quit_requested:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.timeout_s > 0 and (time.monotonic() - self.start) > self.timeout_s:
                    self.get_logger().info(
                        f"timeout {self.timeout_s}s reached, quitting"
                    )
                    break
        except KeyboardInterrupt:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--topic",
        default="/perception/debug/detection_image",
        help="ROS Image topic to subscribe to (default: %(default)s)",
    )
    parser.add_argument(
        "--display",
        default=os.environ.get("DISPLAY", ":10"),
        help="X11 DISPLAY target (default: $DISPLAY or :10)",
    )
    parser.add_argument(
        "--window",
        default="YOLO live",
        help="OpenCV window title (default: %(default)s)",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=960,
        help="Max window width in px (down-scale if larger, 0=keep original). "
             "Default: %(default)s",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.0,
        help="Auto-quit after N seconds (0=infinite). Default: %(default)s",
    )
    args = parser.parse_args()

    os.environ["DISPLAY"] = args.display

    rclpy.init()
    node = CvViewer(
        topic=args.topic,
        window_name=args.window,
        max_width=args.width,
        timeout_s=args.timeout,
    )
    try:
        node.run()
    finally:
        try:
            cv2.destroyWindow(node.window_name)
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        cv2.waitKey(1)
        node.get_logger().info(
            f"DONE: {node.frames_shown} frames shown over "
            f"{time.monotonic() - node.start:.1f}s"
        )
        try:
            node.destroy_node()
        except Exception:
            pass
        # ROS2's signal handler may have already shut rclpy down (when invoked
        # by 'kill -INT $pid'); calling rclpy.shutdown() again raises RCLError.
        # Only call it if the context is still initialized.
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
