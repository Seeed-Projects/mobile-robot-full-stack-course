#!/usr/bin/env python3
"""Compose a readable status strip over the raw BEV for RViz only."""
from __future__ import annotations

import time
import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class AvmStatusOverlay(Node):
    def __init__(self):
        super().__init__("avm_status_overlay")
        image_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.image_pub = self.create_publisher(Image, "/avm/bev/image_annotated", image_qos)
        self.create_subscription(Image, "/avm/bev/metric/image", self._on_image, image_qos)
        self.create_subscription(DiagnosticArray, "/avm/bev/diagnostics", self._on_bev_diag, 10)
        self.create_subscription(DiagnosticArray, "/avm/local_map/diagnostics", self._on_map_diag, 10)
        self.create_subscription(OccupancyGrid, "/avm/local_costmap", self._on_costmap, 2)
        self.create_subscription(Bool, "/avm/parking_permission", self._on_permission, 2)
        self.bev_values = {}
        self.map_values = {}
        self.permission = False
        self.latest_image = None
        self.latest_source = None
        self.last_image_time = 0.0
        self.fps = 0.0

    @staticmethod
    def _values(array):
        if not array.status:
            return {}
        return {item.key: item.value for item in array.status[0].values}

    @staticmethod
    def _decode(msg):
        if msg.encoding != "bgr8":
            return None
        try:
            return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
        except ValueError:
            return None

    @staticmethod
    def _image_msg(image, source):
        msg = Image()
        msg.header = source.header
        msg.height, msg.width = image.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(image.strides[0])
        msg.data = np.ascontiguousarray(image).tobytes()
        return msg

    @staticmethod
    def _draw_segments(image, segments, y, scale):
        x = 12
        for label, color in segments:
            cv2.putText(image, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                        scale, color, 1, cv2.LINE_AA)
            width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0]
            x += width + 14

    def _on_bev_diag(self, msg):
        self.bev_values = self._values(msg)
        self._publish_latest()

    def _on_map_diag(self, msg):
        self.map_values = self._values(msg)

    def _on_permission(self, msg):
        self.permission = bool(msg.data)

    def _on_costmap(self, msg):
        values = np.asarray(msg.data, dtype=np.int16)
        if values.size:
            self.map_values["unknown_percent"] = f"{100.0 * np.count_nonzero(values < 0) / values.size:.1f}"
            self.map_values["candidate_cells"] = str(np.count_nonzero((values >= 50) & (values < 100)))
            self.map_values["high_confidence_cells"] = str(np.count_nonzero(values >= 100))

    def _on_image(self, msg):
        image = self._decode(msg)
        if image is None:
            return
        now = time.monotonic()
        if self.last_image_time:
            dt = now - self.last_image_time
            if dt > 0.0:
                instant = 1.0 / dt
                self.fps = instant if not self.fps else 0.8 * self.fps + 0.2 * instant
        self.last_image_time = now
        self.latest_image = image
        self.latest_source = msg
        self._publish_latest()

    def _publish_latest(self):
        if self.latest_image is None or self.latest_source is None:
            return
        image = self.latest_image.copy()
        h, w = image.shape[:2]
        bar_h = max(58, min(76, h // 8))
        overlay = image.copy()
        cv2.rectangle(overlay, (0, h - bar_h), (w, h), (18, 22, 26), -1)
        image = cv2.addWeighted(overlay, 0.88, image, 0.12, 0.0)

        cameras = sum(self.bev_values.get("camera_" + d, "false") == "true"
                      for d in ("front", "back", "left", "right"))
        coverage = float(self.bev_values.get("coverage_unobserved_pct", "100"))
        candidate = (int(self.map_values.get("candidate_cells", "0")) +
                     int(self.map_values.get("high_confidence_cells", "0")))
        unknown = self.map_values.get("unknown_percent", "100")
        permission = "ENABLED" if self.permission else "DISABLED"
        green, amber, red, neutral = (75, 220, 100), (40, 190, 245), (35, 45, 245), (220, 225, 230)
        scale = max(0.34, min(0.52, w / 1350.0))
        self._draw_segments(image, [
            (f"Cameras {cameras}/4", green if cameras == 4 else red),
            (f"BEV {self.fps:4.1f} FPS", green if self.fps >= 3.0 else amber),
            (f"Coverage {max(0.0, 100.0 - coverage):4.1f}%", green if coverage < 5.0 else amber),
        ], h - bar_h + 24, scale)
        self._draw_segments(image, [
            (f"Candidates {candidate}", green if candidate == 0 else amber),
            (f"Unknown {unknown}%", neutral),
            (f"Parking {permission}", green if self.permission else red),
        ], h - 12, scale)
        lost = [d.upper() for d in ("front", "back", "left", "right")
                if self.bev_values.get("camera_" + d, "false") != "true"]
        if lost and self.bev_values:
            warning = "CAMERA LOST: " + ", ".join(lost)
            cv2.putText(image, warning, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        max(0.45, min(0.75, w / 900.0)), (20, 20, 245), 2, cv2.LINE_AA)
        self.image_pub.publish(self._image_msg(image, self.latest_source))


def main():
    rclpy.init()
    node = AvmStatusOverlay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
