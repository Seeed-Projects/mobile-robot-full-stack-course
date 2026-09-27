"""bev_pose.object_mask_node — P0 single-object initialization helper.

SCOPE (deliberately narrow)
---------------------------
This node exists ONLY so that FoundationPose can be registered on a
"first frame" in the M4 demo. It is NOT a general segmentation module.

    * Input   : RGB + Depth + optional /set_roi service
    * Output  : /perception/object_mask  (mono8)

Behaviour:
    * If the user called /set_roi with a BoundingBox2D, restrict the
      mask to that rectangle.
    * Otherwise, the mask is the depth-foreground only
      (0 < depth < max_depth_m AND depth > min_depth_m).
    * The mask is published as mono8 (0 / 255).

Forbidden behaviours (hard limits):
    * NO multi-instance segmentation.
    * NO open-vocabulary class labels.
    * NO heavyweight model loading (SAM / GroundingDINO / YOLO-seg / ...).
    * NO tracking of any kind.

When this node has produced enough good masks to register, it SHOULD be
shut down via the lifecycle manager or just killed; further tracking
frames come directly from FoundationPose engine.track(), which does not
need a mask. The mask node is therefore optional after register().
"""
from __future__ import annotations

import json
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D

from rclpy.qos import QoSProfile, ReliabilityPolicy


def _qos() -> QoSProfile:
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class ObjectMaskNode(Node):
    def __init__(self):
        super().__init__('object_mask_node')

        self.declare_parameter('depth_topic', '/perception/cameras/front/depth')
        self.declare_parameter('rgb_topic', '/perception/cameras/front/image')
        self.declare_parameter('mask_topic', '/perception/object_mask')
        self.declare_parameter('min_depth_m', 0.10)
        self.declare_parameter('max_depth_m', 1.20)
        self.declare_parameter('default_pad_px', 8)
        # Static ROI: if [x_min,y_min,x_max,y_max] != [-1,-1,-1,-1], use it
        self.declare_parameter('static_roi', [-1, -1, -1, -1])
        # If true, shut down automatically once we publish N masks
        self.declare_parameter('auto_stop_after_masks', 0)
        # If true, disable subscriber + sink after first successful register
        self.declare_parameter('disable_after_register', False)
        # Allow the visualizer / web UI to push an ROI via topic
        self.declare_parameter('roi_topic', '/perception/mask_roi_hint')

        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        mask_topic = self.get_parameter('mask_topic').get_parameter_value().string_value
        roi_topic = self.get_parameter('roi_topic').get_parameter_value().string_value

        self._bridge = CvBridge()
        self._roi: Optional[BoundingBox2D] = None
        self._mask_count = 0
        self._stop_target = \
            self.get_parameter('auto_stop_after_masks').get_parameter_value().integer_value

        qos = _qos()
        self._depth_sub = self.create_subscription(
            Image, depth_topic, self._on_depth, qos)
        self._rgb_sub = self.create_subscription(
            Image, rgb_topic, self._on_rgb, qos)
        # ROI hint topic — lightweight std_msgs/String with JSON for ABI safety
        self._roi_sub = self.create_subscription(
            Image, roi_topic, self._on_roi_image, 1)

        self._mask_pub = self.create_publisher(Image, mask_topic, 10)

        # Last cached frame (we publish from depth + RGB)
        self._last_depth_msg: Optional[Image] = None
        self._last_depth: Optional[np.ndarray] = None
        self._last_rgb_msg: Optional[Image] = None
        self._last_rgb: Optional[np.ndarray] = None

        self.get_logger().info(
            f'object_mask_node ready:\n'
            f'  rgb   = {rgb_topic}\n'
            f'  depth = {depth_topic}\n'
            f'  mask  = {mask_topic}\n'
            f'  roi   = {roi_topic}\n'
            f'  static_roi={self.get_parameter("static_roi").get_parameter_value().integer_array_value}')

    # ----------------------- callbacks ------------------------
    def _on_depth(self, msg: Image) -> None:
        try:
            if msg.encoding in ('32FC1',):
                d = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            elif msg.encoding in ('16UC1',):
                d = self._bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1') * 0.001
            else:
                d = self._bridge.imgmsg_to_cv2(msg)
            self._last_depth = d.astype(np.float32)
            self._last_depth_msg = msg
        except Exception as e:
            self.get_logger().warn(f'depth decode failed: {e!r}')

        if self._last_depth is not None and self._last_rgb is not None:
            self._publish_mask()

    def _on_rgb(self, msg: Image) -> None:
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._last_rgb = img
            self._last_rgb_msg = msg
        except Exception as e:
            self.get_logger().warn(f'rgb decode failed: {e!r}')

    def _on_roi_image(self, msg: Image) -> None:
        # For convenience we accept an Image of full white with a small
        # coloured rectangle drawn on it; the rectangle bounding box is
        # the ROI. This avoids defining a new srv/topic type.
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ys, xs = np.where(gray > 128)
            if len(xs) > 4:
                x_min = int(xs.min()); x_max = int(xs.max())
                y_min = int(ys.min()); y_max = int(ys.max())
                bb = BoundingBox2D()
                # vision_msgs BoundingBox2D has center + size_x/ size_y
                cx = (x_min + x_max) / 2.0
                cy = (y_min + y_max) / 2.0
                bb.center.position.x = float(cx); bb.center.position.y = float(cy)
                bb.size_x = float(x_max - x_min); bb.size_y = float(y_max - y_min)
                self._roi = bb
                self.get_logger().info(
                    f'ROI hint received: center=({cx:.0f},{cy:.0f}) '
                    f'size=({x_max-x_min:.0f}x{y_max-y_min:.0f})')
        except Exception as e:
            self.get_logger().warn(f'roi hint decode failed: {e!r}')

    # ----------------------- mask gen ------------------------
    def _publish_mask(self) -> None:
        d = self._last_depth
        rgb_shape = self._last_rgb.shape[:2] if self._last_rgb is not None else None

        if d is None:
            return

        if rgb_shape is not None and d.shape != rgb_shape:
            d = cv2.resize(d, (rgb_shape[1], rgb_shape[0]),
                           interpolation=cv2.INTER_NEAREST)

        h, w = d.shape
        min_m = self.get_parameter('min_depth_m').get_parameter_value().double_value
        max_m = self.get_parameter('max_depth_m').get_parameter_value().double_value
        depth_mask = ((d > min_m) & (d < max_m) & np.isfinite(d)).astype(np.uint8) * 255

        # Apply ROI restriction
        roi_arr = self.get_parameter('static_roi').get_parameter_value().integer_array_value
        if roi_arr and len(roi_arr) == 4 and any(v >= 0 for v in roi_arr):
            x_min, y_min, x_max, y_max = roi_arr
            x_min = max(0, int(x_min)); y_min = max(0, int(y_min))
            x_max = min(w, int(x_max)); y_max = min(h, int(y_max))
            if x_max > x_min and y_max > y_min:
                roi_mask = np.zeros_like(depth_mask)
                roi_mask[y_min:y_max, x_min:x_max] = 255
                depth_mask = cv2.bitwise_and(depth_mask, roi_mask)
        elif self._roi is not None:
            cx = int(self._roi.center.position.x); cy = int(self._roi.center.position.y)
            sx = int(self._roi.size_x); sy = int(self._roi.size_y)
            x_min = max(0, cx - sx // 2); x_max = min(w, cx + sx // 2)
            y_min = max(0, cy - sy // 2); y_max = min(h, cy + sy // 2)
            if x_max > x_min and y_max > y_min:
                roi_mask = np.zeros_like(depth_mask)
                roi_mask[y_min:y_max, x_min:x_max] = 255
                depth_mask = cv2.bitwise_and(depth_mask, roi_mask)

        # Light morphological cleanup (P0 only)
        k = self.get_parameter('default_pad_px').get_parameter_value().integer_value
        if k and k > 0:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            depth_mask = cv2.morphologyEx(depth_mask, cv2.MORPH_OPEN, kernel)

        msg = self._bridge.cv2_to_imgmsg(depth_mask, encoding='mono8')
        msg.header = self._last_depth_msg.header
        self._mask_pub.publish(msg)
        self._mask_count += 1

        if self._stop_target and self._mask_count >= self._stop_target:
            self.get_logger().info(
                f'reached auto_stop_after_masks={self._stop_target}; '
                'object_mask_node will shut down. FoundationPose stays live.')
            raise SystemExit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = ObjectMaskNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
