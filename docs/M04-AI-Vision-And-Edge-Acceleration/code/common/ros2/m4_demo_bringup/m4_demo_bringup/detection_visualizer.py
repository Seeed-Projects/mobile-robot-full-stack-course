"""Lightweight M4.1 wrapper visualizer.

Audit found the upstream `yolo_trt_node` already publishes the debug
detection image (bbox + class + confidence) on
`/perception/debug/detection_image`. We use that as the M4.1 demo
output directly via a ROS `remap`, so this wrapper is OPT-IN.

This node exists for two reasons:

1. Normalisation: if upstream changes the debug topic, we get a single
   place to remap.
2. Channel-merge: it merges a side panel with FPS / box-count text
   so students can see algorithm throughput without running
   `ros2 topic hz` (which we no longer allow for readiness).

If you do not need (1) or (2), prefer launching the demo with the
upstream topic directly:

    ros2 launch m4_demo_bringup m4_1_demo.launch.py

which uses a `remap` to expose `/perception/debug/detection_image` as
`/perception/demo/m4_1`.
"""
from __future__ import annotations

import array
import sys
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image


def _image_msg_to_bgr(msg: Image) -> np.ndarray:
    h, w = int(msg.height), int(msg.width)
    enc = (msg.encoding or 'bgr8').lower()
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ('bgr8', 'rgb8'):
        arr = raw.reshape(h, w, 3).copy()
        if enc == 'rgb8':
            arr = arr[:, :, ::-1]
        return arr
    if enc in ('bgra8', 'rgba8'):
        return cv2.cvtColor(raw.reshape(h, w, 4).copy(), cv2.COLOR_BGRA2BGR)
    if enc in ('mono8', '8uc1'):
        return cv2.cvtColor(raw.reshape(h, w).copy(), cv2.COLOR_GRAY2BGR)
    raise ValueError(f'unsupported encoding {enc!r}')


class DetectionVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('detection_visualizer')
        self.declare_parameter('image_topic', '/perception/debug/detection_image')
        self.declare_parameter('out_topic', '/perception/demo/m4_1')
        self.declare_parameter('log_throttle_ms', 1000)

        in_topic = self.get_parameter('image_topic').value
        out_topic = self.get_parameter('out_topic').value
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._out_pub = self.create_publisher(Image, out_topic, qos)
        self._sub = self.create_subscription(Image, in_topic, self._on_image, qos)
        self._frames = 0
        self._log_throttle_us = int(
            float(self.get_parameter('log_throttle_ms').value) * 1000.0)
        self._last_log_us = 0
        self.get_logger().info(
            f'detection_visualizer ready: sub={in_topic} pub={out_topic} '
            f'(passthrough; upstream already draws bbox+class+conf)')

    def _on_image(self, msg: Image) -> None:
        t0 = time.perf_counter()
        try:
            arr = _image_msg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f'image parse failed: {e}')
            return
        # Annotate with M4.1 banner + frame counter
        cv2.rectangle(arr, (0, 0), (arr.shape[1], 28), (0, 0, 0), -1)
        cv2.putText(arr, f'M4.1 detection  frame={self._frames}',
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1, cv2.LINE_AA)
        out = Image()
        out.header = msg.header
        out.height = arr.shape[0]
        out.width = arr.shape[1]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = arr.shape[1] * 3
        # MUST be array.array (buffer protocol), never bytes: assigning bytes
        # to the uint8[] field costs ~149 ns/byte (927 ms for a 1080p frame)
        # and pinned this visualizer to ~1 fps. See
        # bev_detection/test/csi_camera_publisher.py:numpy_to_image_msg.
        out.data = array.array('B', arr.tobytes())
        self._out_pub.publish(out)
        self._frames += 1
        now_us = self.get_clock().now().nanoseconds // 1000
        if (now_us - self._last_log_us) >= self._log_throttle_us:
            self._last_log_us = now_us
            self.get_logger().info(
                f'm4_1 viz frame={self._frames} latency_ms='
                f'{(time.perf_counter()-t0)*1000:.2f}')


def main(args=None) -> int:
    rclpy.init(args=args)
    node = DetectionVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
