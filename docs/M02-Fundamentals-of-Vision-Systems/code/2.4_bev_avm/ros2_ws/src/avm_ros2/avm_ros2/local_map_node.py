#!/usr/bin/env python3
"""Software-only local visual map for parking UI, intentionally not SLAM.

It publishes unknown outside calibrated coverage/vehicle footprint, free for
currently observed ground pixels, and conservative obstacle *candidates* where
the BEV changes substantially from a short stable visual background.  This
keeps the map useful in RViz without pretending a four-camera planar projection
can produce metric 3D obstacle geometry.
"""
from __future__ import annotations

import time
import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


class LocalVisualMap(Node):
    def __init__(self):
        super().__init__("avm_local_map")
        self.declare_parameter("view_m", 4.0)
        self.declare_parameter("resolution_m", 0.05)
        self.declare_parameter("difference_threshold", 42.0)
        self.declare_parameter("temporal_on_frames", 3)
        self.declare_parameter("temporal_off_frames", 5)
        self.declare_parameter("temporal_decay", 0.82)
        self.declare_parameter("baseline_frames", 15)
        self.declare_parameter("vehicle_width_m", 0.46)
        self.declare_parameter("vehicle_length_m", 0.46)
        self.declare_parameter("min_obstacle_cells", 3)
        self.view_m = float(self.get_parameter("view_m").value)
        self.resolution = float(self.get_parameter("resolution_m").value)
        self.cells = int(round(self.view_m / self.resolution))
        self.latest_image = None; self.latest_coverage = None; self.baseline = None
        self.occupancy_score = np.zeros((self.cells, self.cells), np.float32)
        self.samples = []; self.last_stamp = None
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "/avm/bev/metric/image", self._on_image, qos)
        self.create_subscription(Image, "/avm/bev/valid_mask", self._on_coverage, qos)
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/avm/local_costmap", 2)
        self.free_pub = self.create_publisher(OccupancyGrid, "/avm/free_space", 2)
        self.marker_pub = self.create_publisher(MarkerArray, "/avm/obstacles", 2)
        self.zone_pub = self.create_publisher(MarkerArray, "/avm/collision_zones", 2)
        self.permission_pub = self.create_publisher(Bool, "/avm/parking_permission", 1)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/avm/local_map/diagnostics", 2)
        self.timer = self.create_timer(0.2, self._tick)
        self.get_logger().info("Local AVM visual map ready: parking permission is hard-disabled")

    @staticmethod
    def _decode(msg):
        channels = 1 if msg.encoding == "mono8" else 3
        return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, channels)

    def _on_image(self, msg):
        if msg.encoding != "bgr8": return
        try: self.latest_image = self._decode(msg); self.last_stamp = msg.header.stamp
        except ValueError: pass

    def _on_coverage(self, msg):
        if msg.encoding != "mono8": return
        try: self.latest_coverage = self._decode(msg)[:, :, 0] > 0
        except ValueError: pass

    def _grid(self, stamp, data):
        info = MapMetaData(); info.resolution = self.resolution; info.width = self.cells; info.height = self.cells
        info.origin.position.x = -self.view_m / 2.0; info.origin.position.y = -self.view_m / 2.0; info.origin.orientation.w = 1.0
        grid = OccupancyGrid(); grid.header.stamp = stamp; grid.header.frame_id = "base_link"; grid.info = info
        grid.data = data.astype(np.int8).ravel().tolist()
        return grid

    def _sample_to_cells(self, mask):
        """Map RViz x-forward/y-left cells to image pixels (image x-right/y-up)."""
        h, w = mask.shape; scale = w / self.view_m
        xx = (np.arange(self.cells) + 0.5) * self.resolution - self.view_m / 2.0
        yy = (np.arange(self.cells) + 0.5) * self.resolution - self.view_m / 2.0
        x_ros, y_ros = np.meshgrid(xx, yy)
        px = np.clip(np.rint(w / 2.0 - y_ros * scale).astype(int), 0, w - 1)
        py = np.clip(np.rint(h / 2.0 - x_ros * scale).astype(int), 0, h - 1)
        return mask[py, px]

    def _vehicle_mask(self):
        coords = (np.arange(self.cells) + 0.5) * self.resolution - self.view_m / 2.0
        x, y = np.meshgrid(coords, coords)
        return ((np.abs(x) <= float(self.get_parameter("vehicle_length_m").value) / 2.0) &
                (np.abs(y) <= float(self.get_parameter("vehicle_width_m").value) / 2.0))

    def _markers(self, stamp, candidates, high_confidence):
        contours, _ = cv2.findContours((candidates.astype(np.uint8) * 255), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        markers = []; i = 0
        for contour in contours:
            x, y, width, height = cv2.boundingRect(contour)
            if width * height < int(self.get_parameter("min_obstacle_cells").value): continue
            marker = Marker(); marker.header.stamp = stamp; marker.header.frame_id = "base_link"
            marker.ns = "visual_obstacle_candidates"; marker.id = i; i += 1; marker.type = Marker.CUBE; marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = -self.view_m / 2.0 + (x + width / 2.0) * self.resolution
            marker.pose.position.y = -self.view_m / 2.0 + (y + height / 2.0) * self.resolution
            marker.pose.position.z = 0.08; marker.scale.x = width * self.resolution; marker.scale.y = height * self.resolution; marker.scale.z = 0.16
            is_high = bool(np.any(high_confidence[y:y + height, x:x + width]))
            marker.color.r = 1.0
            marker.color.g = 0.2 if is_high else 0.78
            marker.color.b = 0.05
            marker.color.a = 0.78 if is_high else 0.62
            markers.append(marker)
        return MarkerArray(markers=markers)

    def _zones(self, stamp):
        marker = Marker(); marker.header.stamp = stamp; marker.header.frame_id = "base_link"
        marker.ns = "parking_caution_zone"; marker.id = 0; marker.type = Marker.CYLINDER; marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0; marker.pose.position.z = 0.01
        marker.scale.x = marker.scale.y = 1.2; marker.scale.z = 0.02
        marker.color.r = 1.0; marker.color.g = 0.8; marker.color.b = 0.05; marker.color.a = 0.22
        return MarkerArray(markers=[marker])

    def _tick(self):
        stamp = self.get_clock().now().to_msg(); self.permission_pub.publish(Bool(data=False))
        if self.latest_image is None or self.latest_coverage is None: return
        # Normalize global exposure and suppress the small interpolation seams
        # introduced by the four-camera BEV before comparing frames.
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY).astype(np.float32)
        coverage = self.latest_coverage
        smooth = cv2.GaussianBlur(gray, (5, 5), 0)
        covered_values = smooth[coverage]
        if covered_values.size:
            smooth += float(np.median(covered_values)) - float(np.median(smooth[coverage]))
        if self.baseline is None:
            self.samples.append(smooth)
            if len(self.samples) >= int(self.get_parameter("baseline_frames").value):
                self.baseline = np.median(np.stack(self.samples), axis=0).astype(np.float32); self.samples.clear()
                self.get_logger().info("Visual map background initialized; occupancy means change candidate, not 3D ranging")
            return
        diff = np.abs(smooth - self.baseline)
        dynamic = (diff >= float(self.get_parameter("difference_threshold").value)) & coverage
        dynamic = cv2.morphologyEx(dynamic.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)) > 0
        dynamic = cv2.morphologyEx(dynamic.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)) > 0
        # Background only adapts at stable ground pixels, preventing a moving object from being learned immediately.
        self.baseline[(coverage & ~dynamic)] = 0.98 * self.baseline[(coverage & ~dynamic)] + 0.02 * smooth[(coverage & ~dynamic)]
        observed_cells = self._sample_to_cells(coverage)
        instant = self._sample_to_cells(dynamic)
        # Require persistence across several ticks. This prevents one-frame
        # exposure changes and BEV interpolation noise from painting the map.
        decay = float(self.get_parameter("temporal_decay").value)
        self.occupancy_score *= np.clip(decay, 0.0, 0.99)
        self.occupancy_score += instant.astype(np.float32)
        self.occupancy_score[~observed_cells] = 0.0
        on_frames = max(1, int(self.get_parameter("temporal_on_frames").value))
        off_frames = max(1, int(self.get_parameter("temporal_off_frames").value))
        candidates = self.occupancy_score >= max(1.0, float(on_frames) * 0.5)
        high_confidence = self.occupancy_score >= float(on_frames)
        # Keep the decay parameter explicit in the state policy.
        candidates &= self.occupancy_score >= (1.0 / float(off_frames))
        vehicle = self._vehicle_mask()
        candidates &= observed_cells & ~vehicle
        high_confidence &= candidates
        cost = np.full((self.cells, self.cells), -1, np.int8)
        cost[observed_cells] = 0
        cost[candidates] = 50
        cost[high_confidence] = 100
        cost[vehicle] = -1
        free = np.full((self.cells, self.cells), -1, np.int8)
        free[observed_cells & ~candidates & ~vehicle] = 0
        self.costmap_pub.publish(self._grid(stamp, cost)); self.free_pub.publish(self._grid(stamp, free))
        self.marker_pub.publish(self._markers(stamp, candidates, high_confidence)); self.zone_pub.publish(self._zones(stamp))
        status = DiagnosticStatus(); status.name = "avm/local_visual_map"; status.hardware_id = "software_only"
        status.level = DiagnosticStatus.WARN; status.message = "visual obstacle candidates only; parking control disabled"
        values = {
            "map_ready": "true", "free_cells": str(int(np.count_nonzero(free == 0))),
            "candidate_cells": str(int(np.count_nonzero((cost >= 50) & (cost < 100)))),
            "high_confidence_cells": str(int(np.count_nonzero(cost >= 100))),
            "unknown_cells": str(int(np.count_nonzero(cost < 0))),
            "unknown_percent": f"{100.0 * np.count_nonzero(cost < 0) / cost.size:.1f}",
            "resolution_m": str(self.resolution), "view_m": str(self.view_m),
            "visual_only": "true", "parking_permission": "false",
        }
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        diag = DiagnosticArray(); diag.header.stamp = stamp; diag.status = [status]; self.diag_pub.publish(diag)


def main():
    rclpy.init(); node = LocalVisualMap()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
