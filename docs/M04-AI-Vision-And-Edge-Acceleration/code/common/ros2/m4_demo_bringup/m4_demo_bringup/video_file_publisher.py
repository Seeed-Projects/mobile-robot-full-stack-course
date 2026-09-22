"""Loop a validated local video into the shared M4 perception pipeline."""
from __future__ import annotations

import argparse
import array
import json
import os
import subprocess
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt64


SUPPORTED_EXTENSIONS = {".mp4", ".mov", ".mkv"}
SUPPORTED_CODECS = {"h264": "h264", "avc1": "h264", "hevc": "h265", "h265": "h265"}
MAX_VIDEO_BYTES = 4 * 1024 * 1024 * 1024


def probe_video_file(path: str) -> dict:
    """Return normalized ffprobe metadata or raise ValueError."""
    video_path = Path(path)
    if not video_path.is_file():
        raise ValueError("video file does not exist")
    if video_path.suffix.lower() not in SUPPORTED_EXTENSIONS:
        raise ValueError("only MP4, MOV and MKV containers are supported")
    size = video_path.stat().st_size
    if size <= 0:
        raise ValueError("video file is empty")
    if size > MAX_VIDEO_BYTES:
        raise ValueError("video exceeds the 4 GB limit")

    command = [
        "ffprobe", "-v", "error", "-select_streams", "v:0",
        "-show_entries",
        "stream=codec_name,width,height,avg_frame_rate,duration:format=format_name,duration",
        "-of", "json", str(video_path),
    ]
    try:
        result = subprocess.run(command, check=True, capture_output=True, text=True, timeout=20)
        payload = json.loads(result.stdout or "{}")
    except (OSError, subprocess.SubprocessError, json.JSONDecodeError) as exc:
        raise ValueError(f"could not inspect video: {exc}") from exc

    streams = payload.get("streams") or []
    if not streams:
        raise ValueError("file has no decodable video stream")
    stream = streams[0]
    codec_raw = str(stream.get("codec_name") or "").lower()
    codec = SUPPORTED_CODECS.get(codec_raw)
    if not codec:
        raise ValueError(f"unsupported video codec: {codec_raw or 'unknown'}")
    width = int(stream.get("width") or 0)
    height = int(stream.get("height") or 0)
    if width <= 0 or height <= 0:
        raise ValueError("video dimensions are invalid")

    rate = str(stream.get("avg_frame_rate") or "0/1")
    try:
        numerator, denominator = rate.split("/", 1)
        fps = float(numerator) / max(float(denominator), 1.0)
    except (TypeError, ValueError, ZeroDivisionError):
        fps = 0.0
    duration_raw = stream.get("duration") or (payload.get("format") or {}).get("duration") or 0
    try:
        duration = float(duration_raw)
    except (TypeError, ValueError):
        duration = 0.0
    if duration <= 0.0:
        raise ValueError("video duration is invalid")

    return {
        "size_bytes": size,
        "duration_ms": int(round(duration * 1000.0)),
        "width": width,
        "height": height,
        "fps": round(fps if fps > 0.0 else 30.0, 3),
        "codec": codec,
        "container": str((payload.get("format") or {}).get("format_name") or video_path.suffix[1:]),
    }


def fit_letterbox(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """Fit BGR frame into a dark target canvas without changing geometry."""
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("expected an HxWx3 BGR frame")
    src_h, src_w = frame.shape[:2]
    if src_w <= 0 or src_h <= 0 or width <= 0 or height <= 0:
        raise ValueError("invalid source or target dimensions")
    scale = min(width / float(src_w), height / float(src_h))
    out_w = max(1, min(width, int(round(src_w * scale))))
    out_h = max(1, min(height, int(round(src_h * scale))))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(frame, (out_w, out_h), interpolation=interpolation)
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    x = (width - out_w) // 2
    y = (height - out_h) // 2
    canvas[y:y + out_h, x:x + out_w] = resized
    return canvas


class VideoFilePublisher(Node):
    def __init__(self) -> None:
        super().__init__("m4_video_file_publisher")
        self.declare_parameter("video_path", "")
        self.declare_parameter("output_topic", "/perception/inputs/video")
        self.declare_parameter("loop_topic", "/perception/inputs/video_loop")
        self.declare_parameter("target_width", 1920)
        self.declare_parameter("target_height", 1080)
        self.declare_parameter("max_fps", 30.0)

        self._path = str(self.get_parameter("video_path").value)
        if not self._path:
            raise RuntimeError("video_path is required")
        metadata = probe_video_file(self._path)
        source_fps = max(1.0, float(metadata["fps"]))
        self._fps = min(float(self.get_parameter("max_fps").value), source_fps, 30.0)
        if not (1.0 <= self._fps <= 30.0):
            raise RuntimeError("effective video FPS must be within 1..30")
        self._target_width = int(self.get_parameter("target_width").value)
        self._target_height = int(self.get_parameter("target_height").value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._publisher = self.create_publisher(
            Image, str(self.get_parameter("output_topic").value), qos)
        self._loop_publisher = self.create_publisher(
            UInt64, str(self.get_parameter("loop_topic").value), qos)
        self._capture: Optional[cv2.VideoCapture] = None
        self._backend = ""
        self._loop_count = 0
        self._open_capture()
        self._timer = self.create_timer(1.0 / self._fps, self._publish_next)
        self.get_logger().info(
            f"video input ready: {Path(self._path).name} {metadata['width']}x{metadata['height']} "
            f"{metadata['codec']} -> {self._target_width}x{self._target_height}@{self._fps:.2f} "
            f"backend={self._backend}")

    def _open_capture(self) -> None:
        if self._capture is not None:
            self._capture.release()
        self._capture = None
        try:
            uri = Path(self._path).resolve().as_uri()
            pipeline = (
                f'uridecodebin uri="{uri}" ! videoconvert ! '
                'video/x-raw,format=BGR ! appsink max-buffers=2 drop=true sync=false'
            )
            candidate = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if candidate.isOpened():
                self._capture = candidate
                self._backend = "gstreamer"
            else:
                candidate.release()
        except Exception:
            self._capture = None
        if self._capture is None:
            candidate = cv2.VideoCapture(self._path)
            if not candidate.isOpened():
                candidate.release()
                raise RuntimeError("OpenCV could not decode the uploaded video")
            self._capture = candidate
            self._backend = "opencv"

    def _rewind(self) -> bool:
        assert self._capture is not None
        if self._capture.set(cv2.CAP_PROP_POS_FRAMES, 0):
            ok, frame = self._capture.read()
            if ok and frame is not None:
                self._loop_count += 1
                event = UInt64()
                event.data = self._loop_count
                self._loop_publisher.publish(event)
                self._publish_frame(frame)
                return True
        try:
            self._open_capture()
            ok, frame = self._capture.read()
        except Exception as exc:
            self.get_logger().error(f"video reopen failed: {exc}")
            return False
        if not ok or frame is None:
            return False
        self._loop_count += 1
        event = UInt64()
        event.data = self._loop_count
        self._loop_publisher.publish(event)
        self._publish_frame(frame)
        return True

    def _publish_next(self) -> None:
        if self._capture is None:
            return
        ok, frame = self._capture.read()
        if not ok or frame is None:
            if not self._rewind():
                self.get_logger().error("video reached EOS but could not rewind")
            return
        self._publish_frame(frame)

    def _publish_frame(self, frame: np.ndarray) -> None:
        canvas = fit_letterbox(frame, self._target_width, self._target_height)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "m4_video_input"
        msg.height = self._target_height
        msg.width = self._target_width
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = self._target_width * 3
        msg.data = array.array("B", canvas.tobytes())
        self._publisher.publish(msg)

    def destroy_node(self):
        if self._capture is not None:
            self._capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.parse_known_args(args)
    rclpy.init(args=args)
    node = VideoFilePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
