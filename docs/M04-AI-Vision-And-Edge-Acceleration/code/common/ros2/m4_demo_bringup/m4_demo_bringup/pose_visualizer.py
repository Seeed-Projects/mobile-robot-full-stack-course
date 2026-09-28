"""m4_demo_bringup/pose_visualizer.py — Render 6D pose overlay for the M4.4 demo.

CRITICAL DESIGN POINT
---------------------
The mesh geometry is loaded ONCE, at node startup, from the precomputed
.npz file published in /perception/mesh_meta and from the .npz file
directly. We DO NOT parse the OBJ file in any frame.

Per-frame work is bounded to:
    1. PoseStamped → 4x4 matrix (and quaternion normalisation).
    2. 8 bbox corners × 4x4 pose  → 8 image points.
    3. cv2.line on 12 box edges.
    4. cv2.projectPoints for the XYZ RGB axes.
    5. Overlay text with score / frame index.
    6. Publish to /perception/demo/m4_4 (WebRTC backend consumes this).

This keeps the overlay cheaper than the track() call itself.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import CameraInfo
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf_transformations import quaternion_matrix

log = logging.getLogger(__name__)


@dataclass
class VisualizationMesh:
    """Local mirror of mesh metadata used purely for visualization."""

    bbox_corners: np.ndarray        # (8, 3)
    diameter: float
    centroid: np.ndarray            # (3,)
    frame_id: str                   # tf2 child
    K: np.ndarray                   # (3, 3)
    image_size: tuple               # (w, h)


class PoseVisualizer(Node):
    def __init__(self):
        super().__init__('pose_visualizer')

        self.declare_parameter('rgb_topic', '/perception/cameras/front/image')
        self.declare_parameter('camera_info_topic',
                               '/perception/cameras/front/camera_info')
        self.declare_parameter('pose_topic', '/perception/object_pose')
        self.declare_parameter('output_topic', '/perception/demo/m4_4')
        self.declare_parameter('mesh_meta_topic', '/perception/mesh_meta')
        self.declare_parameter('mesh_npz_path', '')
        self.declare_parameter('axes_length_m', 0.08)
        self.declare_parameter('box_thickness', 2)
        self.declare_parameter('axes_thickness', 3)
        self.declare_parameter('publish_fps_stats', True)

        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        meta_topic = self.get_parameter('mesh_meta_topic').get_parameter_value().string_value
        mesh_npz_path = \
            self.get_parameter('mesh_npz_path').get_parameter_value().string_value

        self._axes_len = \
            self.get_parameter('axes_length_m').get_parameter_value().double_value
        self._box_th = \
            self.get_parameter('box_thickness').get_parameter_value().integer_value
        self._axes_th = \
            self.get_parameter('axes_thickness').get_parameter_value().integer_value

        self._bridge = CvBridge()
        self._last_rgb_msg: Optional[Image] = None
        self._last_rgb: Optional[np.ndarray] = None
        self._last_pose_msg: Optional[PoseStamped] = None
        self._mesh: Optional[VisualizationMesh] = None

        # Subscriptions (with sufficient queue for Phase 0 tolerance)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._rgb_sub = self.create_subscription(
            Image, rgb_topic, self._on_rgb, qos)
        self._pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self._on_pose, 10)
        self._info_sub = self.create_subscription(
            CameraInfo, info_topic, self._on_info, qos)
        # Latching: meta is published once at startup with transient_local
        meta_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._meta_sub = self.create_subscription(
            String, meta_topic, self._on_meta, meta_qos)

        self._out_pub = self.create_publisher(Image, out_topic, 10)

        # Load mesh metadata from the .npz file if the path is set.
        # This is a fast "if the visualizer starts before foundationpose_node,
        # still have geometry" path. The /perception/mesh_meta message will
        # overwrite the values once it arrives.
        if mesh_npz_path:
            try:
                from bev_pose.types import PreprocessedMesh
                m = PreprocessedMesh.from_npz(mesh_npz_path)
                self._mesh = VisualizationMesh(
                    bbox_corners=m.bbox_corners,
                    diameter=m.diameter,
                    centroid=m.centroid,
                    frame_id=m.frame_id or 'object',
                    K=np.eye(3, dtype=np.float32),
                    image_size=(0, 0),
                )
                self.get_logger().info(
                    f'mesh pre-loaded from {mesh_npz_path} (d={m.diameter:.3f}m)')
            except Exception as e:
                self.get_logger().warn(f'mesh_npz_path {mesh_npz_path} failed: {e!r}')

        self._draw_count = 0
        self.get_logger().info(
            f'pose_visualizer ready:\n'
            f'  rgb   = {rgb_topic}\n'
            f'  info  = {info_topic}\n'
            f'  pose  = {pose_topic}\n'
            f'  out   = {out_topic}\n'
            f'  meta  = {meta_topic}\n')

    # ----------------------------------------------------------------
    def _on_rgb(self, msg: Image) -> None:
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._last_rgb = img
            self._last_rgb_msg = msg
        except Exception as e:
            self.get_logger().warn(f'rgb decode failed: {e!r}')

        if self._last_pose_msg is not None and self._mesh is not None:
            self._draw_and_publish()

    def _on_pose(self, msg: PoseStamped) -> None:
        self._last_pose_msg = msg
        if self._last_rgb is not None and self._mesh is not None:
            self._draw_and_publish()

    def _on_info(self, msg: CameraInfo) -> None:
        K = np.asarray(msg.k, dtype=np.float32).reshape(3, 3)
        if self._mesh is None:
            self._mesh = VisualizationMesh(
                bbox_corners=np.zeros((8, 3), dtype=np.float32),
                diameter=0.0, centroid=np.zeros(3, dtype=np.float32),
                frame_id='object', K=K, image_size=(msg.width, msg.height),
            )
        else:
            self._mesh.K = K
            self._mesh.image_size = (msg.width, msg.height)

    def _on_meta(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
            bbox = np.asarray(d['bbox_corners'], dtype=np.float32)
            centroid = np.asarray(d['centroid'], dtype=np.float32)
            frame_id = d.get('frame_id', 'object')
            diameter = float(d.get('diameter_m', 0.0))
            # Keep the K from info_callback (image_size might not yet be known)
            K = self._mesh.K if (self._mesh is not None) else np.eye(3, dtype=np.float32)
            img_size = self._mesh.image_size if self._mesh is not None else (0, 0)
            self._mesh = VisualizationMesh(
                bbox_corners=bbox, diameter=diameter,
                centroid=centroid, frame_id=frame_id,
                K=K, image_size=img_size,
            )
            self.get_logger().info(
                f'mesh_meta arrived: diameter={diameter:.3f}m frame_id={frame_id}')
        except Exception as e:
            self.get_logger().warn(f'mesh_meta decode failed: {e!r}')

    # ----------------------------------------------------------------
    def _draw_and_publish(self) -> None:
        if self._last_rgb is None or self._last_pose_msg is None or self._mesh is None:
            return

        K = self._mesh.K
        if K is None or K.shape != (3, 3) or K[0, 0] == 0:
            return

        bgr = self._last_rgb.copy()
        h, w = bgr.shape[:2]

        pose = self._last_pose_msg
        try:
            pose4 = quaternion_matrix([
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ]).astype(np.float32)
        except Exception:
            # Fallback when tf_transformations isn't on PYTHONPATH
            q = np.array([
                pose.pose.orientation.x, pose.pose.orientation.y,
                pose.pose.orientation.z, pose.pose.orientation.w,
            ], dtype=np.float64)
            n = np.linalg.norm(q)
            q = q / max(n, 1e-9)
            x, y, z, ww = q
            pose4 = np.array([
                [1 - 2*(y*y + z*z), 2*(x*y - z*ww), 2*(x*z + y*ww), 0],
                [2*(x*y + z*ww), 1 - 2*(x*x + z*z), 2*(y*z - x*ww), 0],
                [2*(x*z - y*ww), 2*(y*z + x*ww), 1 - 2*(x*x + y*y), 0],
                [0, 0, 0, 1],
            ], dtype=np.float32)
        pose4[0, 3] = pose.pose.position.x
        pose4[1, 3] = pose.pose.position.y
        pose4[2, 3] = pose.pose.position.z
        pose4[3, 3] = 1.0

        # ---- BBox ----
        try:
            pts_cam = (pose4[:3, :3] @ self._mesh.bbox_corners.T
                       + pose4[:3, 3:4]).T  # (8, 3)
            z_ok = pts_cam[:, 2] > 0.05
            if z_ok.sum() >= 2:
                uv = (K @ pts_cam.T).T
                uv = uv[:, :2] / np.maximum(uv[:, 2:3], 1e-3)
                uv_int = uv.astype(np.int32)
                edges = [
                    (0, 1), (1, 2), (2, 3), (3, 0),
                    (4, 5), (5, 6), (6, 7), (7, 4),
                    (0, 4), (1, 5), (2, 6), (3, 7),
                ]
                for a, b in edges:
                    if uv_int[a, 0] < 0 or uv_int[a, 0] >= w: continue
                    if uv_int[a, 1] < 0 or uv_int[a, 1] >= h: continue
                    if uv_int[b, 0] < 0 or uv_int[b, 0] >= w: continue
                    if uv_int[b, 1] < 0 or uv_int[b, 1] >= h: continue
                    cv2.line(bgr,
                             (int(uv_int[a, 0]), int(uv_int[a, 1])),
                             (int(uv_int[b, 0]), int(uv_int[b, 1])),
                             (0, 220, 0), self._box_th, cv2.LINE_AA)
            # XYZ axes
            axes_obj = np.asarray([
                [0, 0, 0],
                [self._axes_len, 0, 0],
                [0, self._axes_len, 0],
                [0, 0, self._axes_len],
            ], dtype=np.float32)
            axes_cam = (pose4[:3, :3] @ axes_obj.T + pose4[:3, 3:4]).T
            if (axes_cam[:, 2] > 0.05).all():
                uva = (K @ axes_cam.T).T
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
            self.get_logger().warn(f'draw bbox failed: {e!r}')

        # ---- HUD ----
        info_lines = [
            f'6D Pose: t=({pose.pose.position.x:.2f},{pose.pose.position.y:.2f},{pose.pose.position.z:.2f})m',
            f'       q=({pose.pose.orientation.x:.2f},{pose.pose.orientation.y:.2f},'
            f'{pose.pose.orientation.z:.2f},{pose.pose.orientation.w:.2f})',
        ]
        for i, line in enumerate(info_lines):
            cv2.putText(bgr, line, (10, h - 36 + 18 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 50, 50), 1, cv2.LINE_AA)
            cv2.putText(bgr, line, (10, h - 37 + 18 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Publish
        try:
            msg = self._bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            msg.header = self._last_rgb_msg.header
            self._out_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'output publish failed: {e!r}')

        self._draw_count += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = PoseVisualizer()
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
