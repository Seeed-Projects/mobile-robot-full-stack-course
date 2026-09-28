"""Segmentation visualizer (M4.3 demo overlay).

Subscribes to:
    /perception/cameras/front/image  (sensor_msgs/Image, bgr8)
    /perception/semantic_mask        (sensor_msgs/Image, mono8 / uint8 class ID)
    /perception/drivable_mask        (sensor_msgs/Image, mono8 / 0..255)

Publishes:
    /perception/demo/m4_3            (sensor_msgs/Image, bgr8) — side-by-side
                                     LEFT  : original image + semantic colour overlay
                                     RIGHT : original image + drivable area overlay

The visualizer NEVER modifies upstream algorithm nodes. It only reads
their outputs and republishes a teaching visualization.

Synchronization:
    BoundedStampCache with exact-stamp matching by default. If an
    audit-blessed upstream has stamp drift, `tolerance_ns > 0` may be
    configured; a WARNING is logged on first occurrence and the
    `fallback` counter is reported in the periodic status line.

Cityscapes palette:
    Derived from the open Cityscapes labels.csv (cityscapes.eu) — the
    same 19-class set used by `nvidia/segformer-b0-finetuned-cityscapes
    -512-1024` whose id2label mapping is also emitted into
    `models/m4/segmentation/labels/labels.json` by
    `scripts/m4/generate_labels_json.sh`. Colours are BGR tuples for
    direct cv2 consumption.
"""
from __future__ import annotations

import array
import json
import sys
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .sync_utils import BoundedStampCache


# Cityscapes 19-class palette (BGR).
# Source: well-known cityscapesScripts palette as documented in
# https://github.com/mcordts/cityscapesScripts/blob/master/cityscapesscripts/helpers/labels.py
# Verification against the checkpoint id2label mapping is the
# responsibility of the M4.3 Agent via generate_labels_json.sh.
CITYSCAPES_PALETTE_BGR: list[tuple[int, int, int]] = [
    (128,  64, 128),  # 0  road
    (244,  35, 232),  # 1  sidewalk
    ( 70,  70,  70),  # 2  building
    (102, 102, 156),  # 3  wall
    (190, 153, 153),  # 4  fence
    (153, 153, 153),  # 5  pole
    (250, 170,  30),  # 6  traffic light
    (220, 220,   0),  # 7  traffic sign
    (107, 142,  35),  # 8  vegetation
    (152, 251, 152),  # 9  terrain
    ( 70, 130, 180),  # 10 sky
    (220,  20,  60),  # 11 person
    (255,   0,   0),  # 12 rider
    (  0,   0, 142),  # 13 car
    (  0,   0,  70),  # 14 truck
    (  0,  60, 100),  # 15 bus
    (  0,  80, 100),  # 16 train
    (  0,   0, 230),  # 17 motorcycle
    (119,  11,  32),  # 18 bicycle
]

CITYSCAPES_NAMES: list[str] = [
    'road', 'sidewalk', 'building', 'wall', 'fence', 'pole',
    'traffic light', 'traffic sign', 'vegetation', 'terrain',
    'sky', 'person', 'rider', 'car', 'truck', 'bus', 'train',
    'motorcycle', 'bicycle',
]

# 256-entry BGR lookup table so the semantic overlay is ONE gather
# (`_SEMANTIC_LUT[cls]`) instead of 19 full-frame `cls == cid` comparisons.
# The old loop scanned a 1920x1080 mask 19 times per frame; class ids beyond
# the palette stay black.
_SEMANTIC_LUT = np.zeros((256, 3), dtype=np.uint8)
for _cid, _bgr in enumerate(CITYSCAPES_PALETTE_BGR):
    _SEMANTIC_LUT[_cid] = _bgr

def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _image_msg_to_bgr(msg: Image) -> np.ndarray:
    """Best-effort mono8 -> BGR."""
    h, w = int(msg.height), int(msg.width)
    enc = (msg.encoding or 'mono8').lower()
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ('bgr8', 'rgb8'):
        arr = raw.reshape(h, w, 3).copy()
        if enc == 'rgb8':
            arr = arr[:, :, ::-1]
        return arr
    if enc in ('bgra8', 'rgba8'):
        return cv2.cvtColor(raw.reshape(h, w, 4).copy(), cv2.COLOR_BGRA2BGR)
    if enc in ('mono8', '8uc1'):
        gray = raw.reshape(h, w).copy()
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise ValueError(f'unsupported encoding for visualizer: {enc!r}')


def _mono_mask_from_msg(msg: Image) -> np.ndarray:
    """Decode mono8 (or fallback) Image -> HxW uint8 array of class/0-255 IDs."""
    h, w = int(msg.height), int(msg.width)
    enc = (msg.encoding or 'mono8').lower()
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ('mono8', '8uc1'):
        return raw.reshape(h, w).copy()
    if enc == 'bgr8':
        return cv2.cvtColor(raw.reshape(h, w, 3), cv2.COLOR_BGR2GRAY)
    if enc == 'rgb8':
        return cv2.cvtColor(raw.reshape(h, w, 3), cv2.COLOR_RGB2GRAY)
    raise ValueError(f'unsupported encoding for mask: {enc!r}')


class SegmentationVisualizer(Node):
    """Subscribe image + semantic_mask + drivable_mask; publish demo image."""

    def __init__(self) -> None:
        super().__init__('segmentation_visualizer')

        # ---- Topics (defaults match upstream) ----
        self.declare_parameter('image_topic', '/perception/cameras/front/image')
        self.declare_parameter('semantic_topic', '/perception/semantic_mask')
        self.declare_parameter('drivable_topic', '/perception/drivable_mask')
        self.declare_parameter('debug_image_topic', '/perception/demo/m4_3')

        # ---- Sync ----
        self.declare_parameter('mask_cache_size', 10)
        # In Hub mode the TensorRT callback can be delayed behind other ROS
        # consumers.  Raw 1080p frames therefore need a longer history than
        # their small mono8 masks; 90 frames covers roughly three seconds at
        # 30 FPS without retaining data indefinitely.
        self.declare_parameter('image_cache_size', 90)
        self.declare_parameter('tolerance_ns', 0)
        self.declare_parameter('log_throttle_ms', 1000)

        # ---- Drawing ----
        self.declare_parameter('overlay_alpha', 0.45)
        self.declare_parameter('drivable_color_bgr', '00FF00')   # green BGR for drivable
        self.declare_parameter('view_mode', 'semantic')
        self.declare_parameter('max_width', 1920)
        self.declare_parameter('stats_topic', '/perception/demo/m4_3/stats')

        image_topic = self.get_parameter('image_topic').value
        semantic_topic = self.get_parameter('semantic_topic').value
        drivable_topic = self.get_parameter('drivable_topic').value
        debug_topic = self.get_parameter('debug_image_topic').value
        stats_topic = self.get_parameter('stats_topic').value

        cache_size = int(self.get_parameter('mask_cache_size').value)
        image_cache_size = int(self.get_parameter('image_cache_size').value)
        self._tolerance_ns = int(self.get_parameter('tolerance_ns').value)
        if self._tolerance_ns > 0:
            self.get_logger().warn(
                f'tolerance_ns={self._tolerance_ns} is non-zero; this should only '
                f'be enabled after an upstream stamp-bug audit. Lookups will '
                f'fall back to nearest-timestamp matching.'
            )

        # Rendering is mask-driven: retain only a small window of source
        # frames and publish when all three messages share a stamp.  This is
        # essential when the inference node intentionally runs below camera
        # rate in Hub mode; publishing from every camera callback would create
        # two out of three misleading "waiting" pictures.
        self._img_cache = BoundedStampCache(maxlen=image_cache_size, tolerance_ns=self._tolerance_ns)
        self._sem_cache = BoundedStampCache(maxlen=cache_size, tolerance_ns=self._tolerance_ns)
        self._driv_cache = BoundedStampCache(maxlen=cache_size, tolerance_ns=self._tolerance_ns)
        self._published_cache = BoundedStampCache(maxlen=cache_size, tolerance_ns=0)

        self._alpha = float(self.get_parameter('overlay_alpha').value)
        self._max_width = int(self.get_parameter('max_width').value)
        self._view_mode = str(self.get_parameter('view_mode').value)
        if self._view_mode not in ('original', 'semantic', 'drivable'):
            raise ValueError('view_mode must be original, semantic or drivable')
        s = self.get_parameter('drivable_color_bgr').value.strip().lstrip('#')
        self._drivable_bgr = (int(s[4:6], 16), int(s[2:4], 16), int(s[0:2], 16))

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._img_sub = self.create_subscription(Image, image_topic, self._on_image, qos)
        self._sem_sub = self.create_subscription(Image, semantic_topic, self._on_semantic, qos)
        self._driv_sub = self.create_subscription(Image, drivable_topic, self._on_drivable, qos)
        self._out_pub = self.create_publisher(Image, debug_topic, qos)
        self._stats_pub = self.create_publisher(String, stats_topic, qos)
        self._parameter_callback = self.add_on_set_parameters_callback(
            self._on_set_parameters)

        self._frames = 0
        self._exact = 0
        self._fallback = 0
        self._miss = 0
        self._last_log_us = 0
        self._log_throttle_us = int(
            float(self.get_parameter('log_throttle_ms').value) * 1000.0)
        self._last_stats_t = 0.0

        self.get_logger().info(
            f'segmentation_visualizer ready: '
            f'image={image_topic} sem={semantic_topic} drivable={drivable_topic} '
            f'pub={debug_topic} tolerance_ns={self._tolerance_ns} '
            f'image_cache={image_cache_size} mask_cache={cache_size} '
            f'view={self._view_mode} palette=CITYSCAPES_19_classes')

    def _on_set_parameters(self, params):
        result = SetParametersResult(successful=True, reason='')
        for param in params:
            if param.name == 'view_mode':
                value = str(param.value)
                if value not in ('original', 'semantic', 'drivable'):
                    result.successful = False
                    result.reason = 'view_mode must be original, semantic or drivable'
                    return result
                self._view_mode = value
                self.get_logger().info(f'M4.3 view switched to {value}')
        return result

    # ---- callbacks ----

    def _on_semantic(self, msg: Image) -> None:
        stamp = _stamp_ns(msg.header.stamp)
        self._sem_cache.push(stamp, msg)
        self._try_render(stamp)

    def _on_drivable(self, msg: Image) -> None:
        stamp = _stamp_ns(msg.header.stamp)
        self._driv_cache.push(stamp, msg)
        self._try_render(stamp)

    def _on_image(self, msg: Image) -> None:
        stamp = _stamp_ns(msg.header.stamp)
        self._img_cache.push(stamp, msg)
        self._try_render(stamp)

    def _try_render(self, stamp: int) -> None:
        """Publish exactly one visualisation when a full stamp triplet exists."""
        if self._published_cache.lookup(stamp)[0] == 'exact':
            return

        image_match = self._img_cache.lookup(stamp)
        sem_match = self._sem_cache.lookup(stamp)
        driv_match = self._driv_cache.lookup(stamp)
        if image_match[0] not in ('exact', 'nearest') or \
                sem_match[0] not in ('exact', 'nearest') or \
                driv_match[0] not in ('exact', 'nearest'):
            self._miss += 1
            return

        msg: Image = image_match[1]
        t0 = time.perf_counter()
        try:
            canvas = _image_msg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f'image parse failed: {e}')
            return

        sem_msg: Image = sem_match[1]
        driv_msg: Image = driv_match[1]
        if (image_match[0], sem_match[0], driv_match[0]) == ('exact', 'exact', 'exact'):
            self._exact += 1
        else:
            self._fallback += 1

        canvas = self._fit_source(canvas, False)
        try:
            cls = _mono_mask_from_msg(sem_msg)
            drivable = _mono_mask_from_msg(driv_msg)
        except Exception as exc:
            self.get_logger().warn(f'mask decode failed: {exc}')
            return
        if cls.shape[:2] != canvas.shape[:2]:
            cls = cv2.resize(
                cls, (canvas.shape[1], canvas.shape[0]), interpolation=cv2.INTER_NEAREST)
        if drivable.shape[:2] != canvas.shape[:2]:
            drivable = cv2.resize(
                drivable, (canvas.shape[1], canvas.shape[0]), interpolation=cv2.INTER_NEAREST)

        if self._view_mode == 'original':
            out = canvas
        elif self._view_mode == 'drivable':
            out = self._draw_drivable_mask(canvas, drivable)
        else:
            out = self._draw_semantic_mask(canvas, cls)
        self._publish_stats(cls, drivable, stamp)

        from sensor_msgs.msg import Image as ImageMsg
        out_msg = ImageMsg()
        out_msg.header = msg.header
        out_msg.height = out.shape[0]
        out_msg.width = out.shape[1]
        out_msg.encoding = 'bgr8'
        out_msg.is_bigendian = 0
        out_msg.step = out.shape[1] * 3
        # MUST be array.array (buffer protocol), never bytes: a bytes
        # assignment to the uint8[] field costs ~149 ns/byte (927 ms for a
        # 1920x1080 frame) and held this visualizer at ~1 fps.
        out_msg.data = array.array('B', out.tobytes())
        self._out_pub.publish(out_msg)
        self._published_cache.push(stamp, True)

        self._frames += 1

        now_us = self.get_clock().now().nanoseconds // 1000
        if (now_us - self._last_log_us) >= self._log_throttle_us:
            self._last_log_us = now_us
            self.get_logger().info(
                f'm4_3 viz frame={self._frames} exact={self._exact} '
                f'fallback={self._fallback} miss={self._miss} '
                f'latency_ms={(time.perf_counter()-t0)*1000:.2f} '
                f'tolerance_ns={self._tolerance_ns}'
            )

    # ---- drawing ----

    def _draw_semantic_mask(self, bgr: np.ndarray, cls: np.ndarray) -> np.ndarray:
        color = _SEMANTIC_LUT[cls]
        return cv2.addWeighted(bgr, 1.0 - self._alpha, color, self._alpha, 0)

    def _draw_drivable_mask(self, bgr: np.ndarray, mask: np.ndarray) -> np.ndarray:
        out = bgr.copy()
        active = mask > 0
        if np.any(active):
            tint = np.empty_like(out)
            tint[:] = self._drivable_bgr
            blended = cv2.addWeighted(out, 1.0 - self._alpha, tint, self._alpha, 0)
            out[active] = blended[active]
        return out

    def _publish_stats(self, cls: np.ndarray, drivable: np.ndarray, stamp: int) -> None:
        now = time.monotonic()
        if now - self._last_stats_t < 0.5:
            return
        self._last_stats_t = now
        counts = np.bincount(cls.reshape(-1), minlength=len(CITYSCAPES_NAMES))
        total = max(1, int(cls.size))
        present = [
            (cid, int(counts[cid])) for cid in range(len(CITYSCAPES_NAMES))
            if counts[cid] > 0
        ]
        present.sort(key=lambda item: item[1], reverse=True)
        top = []
        for cid, count in present[:5]:
            b, g, r = CITYSCAPES_PALETTE_BGR[cid]
            top.append({
                'id': cid,
                'name': CITYSCAPES_NAMES[cid],
                'ratio': round(count / total, 6),
                'color': f'#{r:02x}{g:02x}{b:02x}',
            })
        payload = {
            'stamp_ns': stamp,
            'view': self._view_mode,
            'top_classes': top,
            'drivable_ratio': round(float(np.count_nonzero(drivable)) / max(1, drivable.size), 6),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self._stats_pub.publish(msg)

    def _fit_source(self, canvas: np.ndarray, _unused: bool = False) -> np.ndarray:
        """Downscale the input image so the PUBLISHED frame is at most
        `max_width` wide (0 disables).

        Everything downstream runs at the final single-view resolution.
        """
        w = int(self._max_width)
        if w <= 0:
            return canvas
        target = w
        if target <= 0 or canvas.shape[1] <= target:
            return canvas
        scale = target / float(canvas.shape[1])
        return cv2.resize(
            canvas, (target, max(1, int(round(canvas.shape[0] * scale)))),
            interpolation=cv2.INTER_AREA)

def main(args=None) -> int:
    rclpy.init(args=args)
    node = SegmentationVisualizer()
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
