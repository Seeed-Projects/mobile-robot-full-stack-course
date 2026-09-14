#!/usr/bin/env python3
"""Publish the validated web AVM geometry into ROS2 without opening cameras.

Camera ownership remains with the ROS camera driver.  The renderer deliberately
imports the tested ``tools/calib_web.py`` geometry instead of the older preview
warp pipeline, so the RViz image has the same direct fish-eye mapping, valid
coverage masks, branch rejection and narrow seam weights as the web BEV page.
"""
from __future__ import annotations

import os
import json
import sys
import threading
import time
from pathlib import Path

_CUDA_CV = "/home/seeed/.local/opencv-4.14.0-cuda/lib/python3.10/dist-packages/cv2/python-3.10"
if os.environ.get("J501_AVM_USE_CUDA_OPENCV", "1") != "0" and Path(_CUDA_CV).is_dir() and _CUDA_CV not in sys.path:
    sys.path.insert(0, _CUDA_CV)
import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

DIRECTIONS = ("front", "back", "left", "right")
ROOT = "/home/seeed/workspace/ros2_bev"
TOOLS = ROOT + "/tools"


class _FrameStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None
        self.received = 0.0

    def update(self, frame):
        with self.lock:
            self.frame = frame.copy()
            self.received = time.monotonic()

    def latest(self):
        with self.lock:
            return (None if self.frame is None else self.frame.copy(), self.received)

    def latest_with_id(self):
        frame, received = self.latest()
        return frame, received, int(received * 1000)


def _image_msg(image, stamp, frame_id="base_link", encoding="bgr8"):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = image.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = int(image.strides[0])
    msg.data = np.ascontiguousarray(image).tobytes()
    return msg


def _quat_from_matrix(matrix):
    """Return xyzw from a 3x3 rotation matrix without scipy dependency."""
    m = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    tr = float(np.trace(m))
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        q = np.array([(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                      (m[1, 0] - m[0, 1]) / s, 0.25 * s])
    else:
        i = int(np.argmax(np.diag(m))); j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2.0
        q = np.zeros(4)
        q[i] = 0.25 * s
        q[3] = (m[k, j] - m[j, k]) / s
        q[j] = (m[j, i] + m[i, j]) / s
        q[k] = (m[k, i] + m[i, k]) / s
    return q / max(float(np.linalg.norm(q)), 1e-12)


class AvmBevNode(Node):
    def __init__(self):
        super().__init__("avm_bev")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("canvas_px", 600)
        self.declare_parameter("view_m", 4.0)
        self.declare_parameter("transition_m", 0.04)
        self.declare_parameter("camera_timeout_s", 0.7)
        self.declare_parameter("require_all_cameras", True)
        self.declare_parameter("calibration_root", ROOT + "/calib_results")
        self.declare_parameter("tools_dir", TOOLS)
        self.declare_parameter("config_dir", "/home/seeed/ros2_ws/src/j501_avm_calib/config")
        self.declare_parameter("use_cuda", True)
        self.declare_parameter("body_width_m", 0.46)
        self.declare_parameter("body_length_m", 0.46)

        calibration_root = self.get_parameter("calibration_root").value
        tools_dir = self.get_parameter("tools_dir").value
        os.environ["J501_AVM_CALIB_RESULTS_DIR"] = str(calibration_root)
        os.environ["J501_AVM_CALIB_CONFIG_DIR"] = str(self.get_parameter("config_dir").value)
        os.environ["J501_AVM_USE_CUDA_OPENCV"] = "1" if self.get_parameter("use_cuda").value else "0"
        if tools_dir not in sys.path:
            sys.path.insert(0, tools_dir)
        try:
            import calib_web
            self._web = calib_web
            self._stores = {d: _FrameStore() for d in DIRECTIONS}
            self.state = calib_web.CalibState(self._stores, calib_web.load_config())
            self.state.bev_canvas_px = int(self.get_parameter("canvas_px").value)
            ok, message = self.state.bev_set_view(
                float(self.get_parameter("view_m").value), 0.0, "blend",
                transition_m=float(self.get_parameter("transition_m").value),
                body_w_m=float(self.get_parameter("body_width_m").value),
                body_l_m=float(self.get_parameter("body_length_m").value), body_enabled=True)
            if not ok:
                raise RuntimeError(message)
        except Exception as exc:
            raise RuntimeError("Cannot load validated AVM calibration renderer: " + str(exc)) from exc

        self.expected_wh = (int(self.state.width), int(self.state.height))
        # Keep the full-resolution maps as the calibration reference.  The
        # standard ROS camera driver defaults to 0.5x DDS publication, which
        # remains geometrically valid when maps are scaled by the same factor.
        self._base_maps = {d: (self.state.bev_cache["mapx"][d].copy(),
                               self.state.bev_cache["mapy"][d].copy()) for d in DIRECTIONS}
        self._input_sizes = {d: None for d in DIRECTIONS}
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.subs = [self.create_subscription(Image, f"/cameras/{d}/image_raw",
                                                lambda m, direction=d: self._on_image(direction, m), qos)
                     for d in DIRECTIONS]
        self.metric_pub = self.create_publisher(Image, "/avm/bev/metric/image", pub_qos)
        self.bev_pub = self.create_publisher(Image, "/avm/bev/image", pub_qos)
        self.surround_pub = self.create_publisher(Image, "/avm/bev/surround/image", pub_qos)
        self.valid_pub = self.create_publisher(Image, "/avm/bev/valid_mask", pub_qos)
        self.coverage_pub = self.create_publisher(Image, "/avm/bev/coverage", pub_qos)
        self.owner_pub = self.create_publisher(Image, "/avm/bev/camera_mask", pub_qos)
        self.status_pub = self.create_publisher(String, "/avm/bev/status", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/avm/bev/diagnostics", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/avm/vehicle_footprint", 2)
        self.tfb = TransformBroadcaster(self)
        self._poses = self._load_poses(calibration_root)
        self.timer = self.create_timer(1.0 / max(0.5, float(self.get_parameter("rate_hz").value)), self._tick)
        self.get_logger().info("AVM ROS bridge ready: validated direct projection, no chassis interface")

    def _on_image(self, direction, msg):
        if msg.encoding not in ("bgr8", "rgb8"):
            self.get_logger().warn("%s encoding %s ignored; expected bgr8/rgb8" % (direction, msg.encoding), throttle_duration_sec=5.0)
            return
        if msg.width <= 0 or msg.height <= 0 or msg.width > self.expected_wh[0] or msg.height > self.expected_wh[1]:
            self.get_logger().warn("%s unsupported image size %dx%d; calibrated source is %dx%d" %
                                   (direction, msg.width, msg.height, *self.expected_wh), throttle_duration_sec=5.0)
            return
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            self._adapt_map_to_input(direction, msg.width, msg.height)
            self._stores[direction].update(frame)
        except ValueError:
            self.get_logger().warn("%s malformed image buffer ignored" % direction, throttle_duration_sec=5.0)

    def _adapt_map_to_input(self, direction, width, height):
        """Scale only the source-coordinate maps for a driver-published size."""
        size = (int(width), int(height))
        if self._input_sizes[direction] == size:
            return
        sx = size[0] / float(self.expected_wh[0]); sy = size[1] / float(self.expected_wh[1])
        if not (0.1 <= sx <= 1.0 and 0.1 <= sy <= 1.0):
            raise ValueError("input scaling outside supported calibrated range")
        base_x, base_y = self._base_maps[direction]
        with self.state._lock:
            self.state.bev_cache["mapx"][direction] = base_x * sx
            self.state.bev_cache["mapy"][direction] = base_y * sy
            # GPU maps need the same source-coordinate scaling.  Re-uploading
            # all static maps is rare (only on initial size/change).
            self.state._bev_upload_gpu(self.state.bev_cache)
        self._input_sizes[direction] = size
        self.get_logger().info("%s input %dx%d: applied source-map scale %.3fx%.3f" %
                               (direction, width, height, sx, sy))

    def _load_poses(self, calibration_root):
        try:
            import json
            with open(Path(calibration_root) / "extrinsics.json", encoding="utf-8") as f:
                return json.load(f).get("poses", {})
        except (OSError, ValueError, TypeError):
            return {}

    def _publish_tf(self, stamp):
        transforms = []
        for direction, pose in self._poses.items():
            try:
                r = np.asarray(pose["R"], dtype=np.float64).reshape(3, 3)
                t = np.asarray(pose["t"], dtype=np.float64).reshape(3)
                trans = -r.T @ t
                q = _quat_from_matrix(r.T)
                msg = TransformStamped()
                msg.header.stamp = stamp; msg.header.frame_id = "base_link"
                msg.child_frame_id = "camera_" + direction
                msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, trans)
                msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = map(float, q)
                transforms.append(msg)
            except (KeyError, TypeError, ValueError):
                continue
        if transforms:
            self.tfb.sendTransform(transforms)

    def _publish_markers(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp; marker.header.frame_id = "base_link"
        marker.ns = "vehicle"; marker.id = 0; marker.type = Marker.CUBE; marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(self.get_parameter("body_length_m").value)
        marker.scale.y = float(self.get_parameter("body_width_m").value)
        marker.scale.z = 0.08; marker.pose.position.z = 0.04
        marker.color.r = 0.15; marker.color.g = 0.75; marker.color.b = 0.95; marker.color.a = 0.75
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def _publish_diagnostics(self, stamp, available, render_ok):
        timeout = float(self.get_parameter("camera_timeout_s").value)
        cache = self.state.bev_cache
        coverage = cache.get("coverage", {}).get("outside_body_uncovered_pct", 100.0)
        healthy = render_ok and all(available.values()) and coverage < 5.0
        status = DiagnosticStatus()
        status.name = "avm/bev"; status.hardware_id = "four_camera_avm"
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN
        status.message = "visualization healthy; parking control disabled" if healthy else "incomplete or stale visual input"
        seam = self.state.bev_seam
        values = {"renderer": self.state.bev_backend, "coverage_unobserved_pct": coverage,
                  "seam_state": seam.get("state", "unknown"),
                  "calibration_fingerprint": self.state.bev_fingerprint[:12],
                  "parking_permission": "false", **{"camera_" + d: str(v).lower() for d, v in available.items()}}
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in values.items()]
        array = DiagnosticArray(); array.header.stamp = stamp; array.status = [status]
        self.diag_pub.publish(array)
        self.status_pub.publish(String(data=json.dumps({
            "healthy": bool(healthy), "metric": True, "surround_metric": False,
            "renderer": self.state.bev_backend,
            "coverage_unobserved_pct": float(coverage),
            "cameras": available, "seam": seam.get("state", "unknown"),
            "calibration_fingerprint": self.state.bev_fingerprint,
            "parking_permission": False,
        }, separators=(",", ":"))))

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        now = time.monotonic(); timeout = float(self.get_parameter("camera_timeout_s").value)
        available = {d: self._stores[d].latest()[0] is not None and now - self._stores[d].latest()[1] <= timeout for d in DIRECTIONS}
        if self.get_parameter("require_all_cameras").value and not all(available.values()):
            self._publish_diagnostics(stamp, available, False); self._publish_tf(stamp); self._publish_markers(stamp)
            return
        image = self.state.bev_render()
        if image is not None:
            cache = self.state.bev_cache
            coverage = (cache["valid_any"].astype(np.uint8) * 255)
            owner = np.where(cache["owner"] >= 0, cache["owner"] + 1, 0).astype(np.uint8)
            metric_msg = _image_msg(image, stamp)
            self.metric_pub.publish(metric_msg)
            self.bev_pub.publish(metric_msg)
            surround = self.state.bev_make_surround(image)
            self.surround_pub.publish(_image_msg(surround, stamp))
            self.valid_pub.publish(_image_msg(coverage, stamp, encoding="mono8"))
            self.coverage_pub.publish(_image_msg(coverage, stamp, encoding="mono8"))
            self.owner_pub.publish(_image_msg(owner, stamp, encoding="mono8"))
        self._publish_diagnostics(stamp, available, image is not None)
        self._publish_tf(stamp); self._publish_markers(stamp)


def main():
    rclpy.init(); node = AvmBevNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()
