"""m4_demo_bringup/m4_4_web_visualizer.py — web-hub overlay for M4.4 Isaac ROS FoundationPose.

Draws the estimated 6D pose from the Isaac ROS FoundationPose container
(pose topic /output, vision_msgs/Detection3DArray) onto the bag-replay RGB
frame (image topic /image_rect) and publishes the overlay to
/perception/demo/m4_4 for the web hub, plus a JSON stats string on
/perception/demo/m4_4/stats (consumed by /api/visualization/m4_4).

The container (m4-isaacros-foundationpose) runs with network/ipc/pid host
mode, so its topics are visible on the host DDS domain without any bridge.

Per-frame work is bounded to the pose math, 12 cv2.line calls, 3 axis lines
and the HUD text. The mesh bounding box is parsed ONCE at node startup from
the OBJ file. tf_transformations is not importable on the host python3, so
the quaternion->matrix conversion is inline (same math as the fallback in
the retired pose_visualizer).
"""

from __future__ import annotations

import array
import json
import math
import time
from collections import deque
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

# No cv_bridge on purpose: the host user-site numpy 2.x + cv2 5 stack breaks
# the system cv_bridge boost module (numpy 1.x ABI). Every production node in
# this package (segmentation_visualizer, web_demo_server) decodes Image
# messages manually via np.frombuffer; this node follows the same pattern.


def _image_msg_to_bgr(msg: Image) -> np.ndarray:
    """Decode an Image message into a HxWx3 uint8 BGR array."""
    h, w = int(msg.height), int(msg.width)
    enc = (msg.encoding or 'mono8').lower()
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ('bgr8', 'rgb8'):
        arr = raw.reshape(h, w, 3).copy()
        if enc == 'rgb8':
            # cv2 rejects negative-stride views ("Expected Ptr<cv::UMat>"),
            # so the flip MUST land in contiguous memory.
            arr = np.ascontiguousarray(arr[:, :, ::-1])
        return arr
    if enc in ('bgra8', 'rgba8'):
        return cv2.cvtColor(raw.reshape(h, w, 4).copy(), cv2.COLOR_BGRA2BGR)
    if enc in ('mono8', '8uc1'):
        gray = raw.reshape(h, w).copy()
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise ValueError(f'unsupported encoding for visualizer: {enc!r}')


def _bgr_to_image_msg(bgr: np.ndarray, header) -> Image:
    """Encode a HxWx3 uint8 BGR array into an Image message."""
    out_msg = Image()
    out_msg.header = header
    out_msg.height = bgr.shape[0]
    out_msg.width = bgr.shape[1]
    out_msg.encoding = 'bgr8'
    out_msg.is_bigendian = 0
    out_msg.step = bgr.shape[1] * 3
    # MUST be array.array (buffer protocol), never bytes: a bytes
    # assignment to the uint8[] field costs ~149 ns/byte and stalls the
    # publisher (same lesson as segmentation_visualizer).
    out_msg.data = array.array('B', bgr.tobytes())
    return out_msg


def extract_pose(message: Detection3DArray) -> Optional[dict]:
    """Return the first usable pose in the array, or None.

    Same acceptance gate as scripts/m4/verify_m4_4_pose.py: frame_id set,
    all values finite, p.z > 0, quaternion norm in [0.9, 1.1].
    """
    if not message.header.frame_id:
        return None
    for detection in message.detections:
        for result in detection.results:
            pose = result.pose.pose
            p, q = pose.position, pose.orientation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(v) for v in values):
                continue
            norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
            if p.z > 0 and 0.9 <= norm <= 1.1:
                return {
                    'position_m': (p.x, p.y, p.z),
                    'quaternion_xyzw': (q.x, q.y, q.z, q.w),
                    'quaternion_norm': norm,
                    'score': float(result.hypothesis.score),
                    'class_id': str(result.hypothesis.class_id),
                }
    return None


def quaternion_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Normalized (x, y, z, w) quaternion -> 4x4 rotation matrix (float32)."""
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-9:
        return np.eye(4, dtype=np.float32)
    q = q / n
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0.0],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0.0],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=np.float32)


def parse_obj_bbox(path: str, offset=(0.0, 0.0, 0.0)) -> np.ndarray:
    """Vertex lines of a Wavefront OBJ -> (8, 3) bbox corner array.

    Corner order matches the edge topology used in _draw_overlay:
    0-3 bottom face, 4-7 top face.
    """
    mins = np.full(3, np.inf)
    maxs = np.full(3, -np.inf)
    count = 0
    with open(path, 'r', errors='replace') as fh:
        for line in fh:
            if line.startswith('v '):
                parts = line.split()
                if len(parts) < 4:
                    continue
                try:
                    v = (float(parts[1]), float(parts[2]), float(parts[3]))
                except ValueError:
                    continue
                mins = np.minimum(mins, np.asarray(v))
                maxs = np.maximum(maxs, np.asarray(v))
                count += 1
    if count == 0:
        raise ValueError(f'no vertex lines in {path}')
    x0, y0, z0 = mins
    x1, y1, z1 = maxs
    corners = np.array([
        [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
        [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
    ], dtype=np.float32)
    return corners + np.asarray(offset, dtype=np.float32).reshape(1, 3)


_BOX_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
)


class M4_4WebVisualizer(Node):
    def __init__(self):
        super().__init__('m4_4_web_visualizer')

        # Defaults match the Isaac ROS FoundationPose quickstart bag topics
        # (root namespace, identical in official and adapted modes).
        self.declare_parameter('image_topic', '/image_rect')
        self.declare_parameter('camera_info_topic', '/camera_info_rect')
        self.declare_parameter('pose_topic', '/output')
        self.declare_parameter('output_topic', '/perception/demo/m4_4')
        self.declare_parameter('stats_topic', '/perception/demo/m4_4/stats')
        self.declare_parameter('mesh_obj_path', '')
        self.declare_parameter('mesh_center_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('axes_length_m', 0.05)
        self.declare_parameter('box_thickness', 2)
        self.declare_parameter('axes_thickness', 3)
        self.declare_parameter('max_output_fps', 30.0)
        self.declare_parameter('stats_period_s', 0.5)
        self.declare_parameter('pose_stale_s', 5.0)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        stats_topic = self.get_parameter('stats_topic').get_parameter_value().string_value
        mesh_obj_path = self.get_parameter('mesh_obj_path').get_parameter_value().string_value
        mesh_offset = self.get_parameter('mesh_center_offset').get_parameter_value().double_array_value

        self._axes_len = self.get_parameter('axes_length_m').get_parameter_value().double_value
        self._box_th = self.get_parameter('box_thickness').get_parameter_value().integer_value
        self._axes_th = self.get_parameter('axes_thickness').get_parameter_value().integer_value
        max_fps = self.get_parameter('max_output_fps').get_parameter_value().double_value
        stats_period = self.get_parameter('stats_period_s').get_parameter_value().double_value
        self._pose_stale_s = self.get_parameter('pose_stale_s').get_parameter_value().double_value
        self._min_interval = 1.0 / max(max_fps, 0.1)

        self._K: Optional[np.ndarray] = None
        self._pose: Optional[dict] = None
        self._pose_recv_s: Optional[float] = None
        self._pose_frame_id = ''
        self._pose_times: deque = deque(maxlen=64)
        self._draw_count = 0
        self._last_publish_s = 0.0

        # Bag frames and camera info flow at loop rate; keep depth 1 so the
        # executor never queues stale frames behind the throttle.
        qos1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._image_sub = self.create_subscription(Image, image_topic, self._on_image, qos1)
        self._info_sub = self.create_subscription(CameraInfo, info_topic, self._on_info, qos1)
        self._pose_sub = self.create_subscription(Detection3DArray, pose_topic, self._on_pose, 10)

        self._out_pub = self.create_publisher(Image, out_topic, 10)
        self._stats_pub = self.create_publisher(String, stats_topic, 10)

        self._corners: Optional[np.ndarray] = None
        if mesh_obj_path:
            try:
                self._corners = parse_obj_bbox(mesh_obj_path, mesh_offset)
                half = (self._corners.max(axis=0) - self._corners.min(axis=0)) / 2.0
                self.get_logger().info(
                    f'mesh bbox from {mesh_obj_path}: half-extents '
                    f'({half[0]:.4f}, {half[1]:.4f}, {half[2]:.4f}) m')
            except Exception as e:
                self.get_logger().warn(
                    f'mesh_obj_path {mesh_obj_path!r} unreadable ({e!r}); '
                    'drawing axes + HUD only')

        self._stats_timer = self.create_timer(stats_period, self._publish_stats)
        self.get_logger().info(
            f'm4_4_web_visualizer ready: image={image_topic} info={info_topic} '
            f'pose={pose_topic} out={out_topic} stats={stats_topic}')

    # ----------------------------------------------------------------
    def _on_info(self, msg: CameraInfo) -> None:
        self._K = np.asarray(msg.k, dtype=np.float32).reshape(3, 3)

    def _on_pose(self, msg: Detection3DArray) -> None:
        pose = extract_pose(msg)
        if pose is None:
            return
        self._pose = pose
        self._pose_frame_id = msg.header.frame_id
        now = time.monotonic()
        self._pose_recv_s = now
        self._pose_times.append(now)

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self._last_publish_s < self._min_interval:
            return
        try:
            bgr = _image_msg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f'image decode failed: {e!r}', throttle_duration_sec=5.0)
            return
        self._last_publish_s = now

        # Passthrough before the first pose: the hub video goes live during
        # the FoundationPose warm-up; the box appears once a pose is valid.
        out = bgr
        if self._pose is not None:
            out = self._draw_overlay(bgr)

        try:
            self._out_pub.publish(_bgr_to_image_msg(out, msg.header))
            self._draw_count += 1
        except Exception as e:
            self.get_logger().warn(f'output publish failed: {e!r}', throttle_duration_sec=5.0)

    # ----------------------------------------------------------------
    def _draw_overlay(self, bgr: np.ndarray) -> np.ndarray:
        pose = self._pose
        h, w = bgr.shape[:2]

        pose4 = quaternion_to_matrix(*pose['quaternion_xyzw'])
        pose4[0, 3] = pose['position_m'][0]
        pose4[1, 3] = pose['position_m'][1]
        pose4[2, 3] = pose['position_m'][2]

        try:
            if self._K is not None and self._corners is not None:
                pts_cam = (pose4[:3, :3] @ self._corners.T + pose4[:3, 3:4]).T  # (8, 3)
                if (pts_cam[:, 2] > 0.05).sum() >= 2:
                    uv = (self._K @ pts_cam.T).T
                    uv = uv[:, :2] / np.maximum(uv[:, 2:3], 1e-3)
                    uv_int = uv.astype(np.int32)
                    for a, b in _BOX_EDGES:
                        if (uv_int[a, 0] < 0 or uv_int[a, 0] >= w
                                or uv_int[a, 1] < 0 or uv_int[a, 1] >= h
                                or uv_int[b, 0] < 0 or uv_int[b, 0] >= w
                                or uv_int[b, 1] < 0 or uv_int[b, 1] >= h):
                            continue
                        cv2.line(bgr, (int(uv_int[a, 0]), int(uv_int[a, 1])),
                                 (int(uv_int[b, 0]), int(uv_int[b, 1])),
                                 (0, 220, 0), self._box_th, cv2.LINE_AA)
            # XYZ axes from the pose origin (drawn even without a mesh bbox)
            if self._K is not None:
                axes_obj = np.asarray([
                    [0.0, 0.0, 0.0],
                    [self._axes_len, 0.0, 0.0],
                    [0.0, self._axes_len, 0.0],
                    [0.0, 0.0, self._axes_len],
                ], dtype=np.float32)
                axes_cam = (pose4[:3, :3] @ axes_obj.T + pose4[:3, 3:4]).T
                if (axes_cam[:, 2] > 0.05).all():
                    uva = (self._K @ axes_cam.T).T
                    uva = uva[:, :2] / np.maximum(uva[:, 2:3], 1e-3)
                    uva_int = uva.astype(np.int32)
                    origin = (int(uva_int[0, 0]), int(uva_int[0, 1]))
                    cv2.line(bgr, origin, (int(uva_int[1, 0]), int(uva_int[1, 1])),
                             (0, 0, 220), self._axes_th, cv2.LINE_AA)
                    cv2.line(bgr, origin, (int(uva_int[2, 0]), int(uva_int[2, 1])),
                             (0, 220, 0), self._axes_th, cv2.LINE_AA)
                    cv2.line(bgr, origin, (int(uva_int[3, 0]), int(uva_int[3, 1])),
                             (220, 0, 0), self._axes_th, cv2.LINE_AA)
        except Exception as e:
            self.get_logger().warn(f'draw overlay failed: {e!r}', throttle_duration_sec=5.0)

        t = pose['position_m']
        q = pose['quaternion_xyzw']
        info_lines = [
            f'6D Pose: t=({t[0]:.2f},{t[1]:.2f},{t[2]:.2f})m',
            f'        q=({q[0]:.2f},{q[1]:.2f},{q[2]:.2f},{q[3]:.2f})',
        ]
        for i, line in enumerate(info_lines):
            cv2.putText(bgr, line, (10, h - 36 + 18 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 50, 50), 1, cv2.LINE_AA)
            cv2.putText(bgr, line, (10, h - 37 + 18 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return bgr

    # ----------------------------------------------------------------
    def _publish_stats(self) -> None:
        now = time.monotonic()
        if self._pose is None:
            status = 'waiting_pose'
        elif now - (self._pose_recv_s or now) > self._pose_stale_s:
            status = 'pose_stale'
        else:
            status = 'valid_pose'

        while self._pose_times and now - self._pose_times[0] > 5.0:
            self._pose_times.popleft()
        rate = len(self._pose_times) / 5.0 if self._pose_times else 0.0

        pose = self._pose
        payload = {
            'stamp_ns': self.get_clock().now().nanoseconds,
            'status': status,
            'position_m': list(pose['position_m']) if pose else None,
            'quaternion_xyzw': list(pose['quaternion_xyzw']) if pose else None,
            'quaternion_norm': pose['quaternion_norm'] if pose else None,
            'score': pose['score'] if pose else None,
            'class_id': pose['class_id'] if pose else None,
            'frame_id': self._pose_frame_id or None,
            'pose_age_ms': int((now - self._pose_recv_s) * 1000) if self._pose_recv_s else None,
            'pose_rate_hz': round(rate, 2),
            'draw_count': self._draw_count,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._stats_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = M4_4WebVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
