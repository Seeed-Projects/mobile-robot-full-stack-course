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
import sys
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image

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

# HUD panel fill / accent (BGR), matching the M4.1 and M4.2 overlays.
_HUD_BG_BGR = (24, 28, 34)
_HUD_ACCENT_BGR = (120, 230, 255)

# Width of the separator column inserted between the two side-by-side panels.
_SEP_W = 4


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
        self.declare_parameter('tolerance_ns', 0)
        self.declare_parameter('log_throttle_ms', 1000)

        # ---- Drawing ----
        self.declare_parameter('overlay_alpha', 0.45)
        self.declare_parameter('drivable_color_bgr', '00FF00')   # green BGR for drivable
        self.declare_parameter('side_by_side', True)
        self.declare_parameter('max_width', 960)

        image_topic = self.get_parameter('image_topic').value
        semantic_topic = self.get_parameter('semantic_topic').value
        drivable_topic = self.get_parameter('drivable_topic').value
        debug_topic = self.get_parameter('debug_image_topic').value

        cache_size = int(self.get_parameter('mask_cache_size').value)
        self._tolerance_ns = int(self.get_parameter('tolerance_ns').value)
        if self._tolerance_ns > 0:
            self.get_logger().warn(
                f'tolerance_ns={self._tolerance_ns} is non-zero; this should only '
                f'be enabled after an upstream stamp-bug audit. Lookups will '
                f'fall back to nearest-timestamp matching.'
            )

        self._bridge = CvBridge()
        self._sem_cache = BoundedStampCache(maxlen=cache_size, tolerance_ns=self._tolerance_ns)
        self._driv_cache = BoundedStampCache(maxlen=cache_size, tolerance_ns=self._tolerance_ns)

        self._alpha = float(self.get_parameter('overlay_alpha').value)
        self._max_width = int(self.get_parameter('max_width').value)
        # Class mask of the most recent frame, for the legend (set by
        # _draw_semantic, consumed by _on_image after max_width fitting).
        self._last_cls: Optional[np.ndarray] = None
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

        self._frames = 0
        self._exact = 0
        self._fallback = 0
        self._miss = 0
        self._last_log_us = 0
        self._log_throttle_us = int(
            float(self.get_parameter('log_throttle_ms').value) * 1000.0)
        # Smoothed draw rate for the HUD (independent of the throttled log).
        self._hud_fps = 0.0
        self._last_draw_t = 0.0

        self.get_logger().info(
            f'segmentation_visualizer ready: '
            f'image={image_topic} sem={semantic_topic} drivable={drivable_topic} '
            f'pub={debug_topic} tolerance_ns={self._tolerance_ns} '
            f'palette=CITYSCAPES_19_classes')

    # ---- callbacks ----

    def _on_semantic(self, msg: Image) -> None:
        self._sem_cache.push(_stamp_ns(msg.header.stamp), msg)

    def _on_drivable(self, msg: Image) -> None:
        self._driv_cache.push(_stamp_ns(msg.header.stamp), msg)

    def _on_image(self, msg: Image) -> None:
        t0 = time.perf_counter()
        try:
            canvas = _image_msg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f'image parse failed: {e}')
            return

        image_stamp = _stamp_ns(msg.header.stamp)
        sem_match = self._sem_cache.lookup(image_stamp)
        driv_match = self._driv_cache.lookup(image_stamp)

        sem_msg: Optional[Image] = None
        driv_msg: Optional[Image] = None
        if sem_match[0] == 'exact':
            sem_msg = sem_match[1]
            self._exact += 1
        elif sem_match[0] == 'nearest':
            sem_msg = sem_match[1]
            self._fallback += 1
        else:
            self._miss += 1
        if driv_match[0] in ('exact', 'nearest'):
            driv_msg = driv_match[1]

        side_by_side = bool(self.get_parameter('side_by_side').value)
        # Downscale the SOURCE to the publish width FIRST. Every blend,
        # legend and mask resize then runs at the final resolution instead of
        # building a 3844x1080 side-by-side pair and throwing most of it away.
        # `_draw_semantic` / `_draw_drivable` resize the mask to whatever
        # canvas they are handed, so this also removes the separate mask
        # rescale that used to be needed for the legend.
        canvas = self._fit_source(canvas, side_by_side)
        left = self._draw_semantic(canvas, sem_msg)
        right = self._draw_drivable(canvas, driv_msg)
        out = self._side_by_side(left, right) if side_by_side else left
        if self._last_cls is not None:
            self._draw_class_legend(out, self._last_cls)

        # Smoothed draw rate for the HUD.
        _now = time.monotonic()
        if self._last_draw_t > 0.0:
            _dt = _now - self._last_draw_t
            if _dt > 1e-6:
                _inst = 1.0 / _dt
                self._hud_fps = (
                    _inst if self._hud_fps <= 0.0
                    else 0.9 * self._hud_fps + 0.1 * _inst
                )
        self._last_draw_t = _now

        # Opaque HUD panel: identifies each side, the mask sync mode and the frame
        # rate, matching the M4.1 / M4.2 overlays.
        split_x = left.shape[1] if side_by_side else 0
        self._draw_hud(out, split_x)

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

    def _draw_semantic(self, bgr: np.ndarray, sem_msg: Optional[Image]) -> np.ndarray:
        out = bgr.copy()
        self._last_cls = None
        if sem_msg is None:
            cv2.putText(out, 'semantic: waiting', (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            return out
        try:
            cls = _mono_mask_from_msg(sem_msg)
        except Exception as e:
            self.get_logger().warn(f'semantic decode failed: {e}')
            return out
        if cls.shape[:2] != out.shape[:2]:
            cls = cv2.resize(cls, (out.shape[1], out.shape[0]),
                             interpolation=cv2.INTER_NEAREST)
        color = _SEMANTIC_LUT[cls]
        # The legend is drawn by _on_image() AFTER any max_width downscale so
        # its text stays crisp; remember the mask for that.
        self._last_cls = cls
        return cv2.addWeighted(out, 1.0 - self._alpha, color, self._alpha, 0)

    def _draw_class_legend(self, canvas: np.ndarray, cls: np.ndarray) -> None:
        """Bottom-left legend of the classes actually present.

        One `bincount` pass over the mask gives every class count, so this
        stays cheap even at 1920x1080 (the per-class `cls == cid` loop it
        replaces would have been another 19 full-frame scans).
        """
        counts = np.bincount(cls.ravel(), minlength=len(CITYSCAPES_NAMES))
        total = int(cls.size)
        if total <= 0:
            return
        present = [(int(counts[i]), i) for i in range(len(CITYSCAPES_NAMES))
                   if counts[i] > 0]
        present.sort(reverse=True)
        present = present[:8]           # keep it compact
        if not present:
            return

        h, w = canvas.shape[:2]
        fs = max(0.4, min(w, h) / 2200.0)
        font = cv2.FONT_HERSHEY_DUPLEX
        pad = max(4, int(round(9.0 * fs)))
        row_h = int(round(22.0 * fs))
        sw = int(round(16.0 * fs))
        panel_w = int(round(210.0 * fs))
        panel_h = row_h * (len(present) + 1) + pad
        x0 = 0
        y0 = max(0, h - panel_h)

        cv2.rectangle(canvas, (x0, y0), (x0 + panel_w, y0 + panel_h),
                      _HUD_BG_BGR, -1)
        cv2.line(canvas, (x0, y0), (x0 + panel_w, y0),
                 _HUD_ACCENT_BGR, 2, cv2.LINE_AA)
        cv2.putText(canvas, f'classes ({len(present)})',
                    (x0 + pad, y0 + pad + row_h - int(round(7.0 * fs))),
                    font, fs, (255, 255, 255), 1, cv2.LINE_AA)

        for row, (count, cid) in enumerate(present, start=1):
            cy = y0 + pad + row * row_h
            cv2.rectangle(
                canvas,
                (x0 + pad, cy - int(round(12.0 * fs))),
                (x0 + pad + sw, cy - int(round(12.0 * fs)) + sw),
                tuple(int(v) for v in CITYSCAPES_PALETTE_BGR[cid]), -1)
            pct = 100.0 * count / total
            cv2.putText(
                canvas, f'{CITYSCAPES_NAMES[cid]} {pct:.0f}%',
                (x0 + pad + sw + pad, cy),
                font, fs, _HUD_ACCENT_BGR, 1, cv2.LINE_AA)

    def _draw_drivable(self, bgr: np.ndarray, driv_msg: Optional[Image]) -> np.ndarray:
        out = bgr.copy()
        if driv_msg is None:
            cv2.putText(out, 'drivable: waiting', (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            return out
        try:
            mask = _mono_mask_from_msg(driv_msg)
        except Exception as e:
            self.get_logger().warn(f'drivable decode failed: {e}')
            return out
        if mask.shape[:2] != out.shape[:2]:
            mask = cv2.resize(mask, (out.shape[1], out.shape[0]),
                              interpolation=cv2.INTER_NEAREST)
        binary = mask > 0
        if not binary.any():
            return out
        overlay = np.full_like(out, self._drivable_bgr)
        # NOTE: cv2.addWeighted has NO `mask` parameter. The previous code
        # passed mask=... and raised
        #   cv2.error: 'mask' is an invalid keyword argument for addWeighted()
        # the first time a real drivable mask arrived. That path had never
        # executed because the M4.3 engine was not built, so the bug sat
        # latent. Blend everywhere, then keep only the masked pixels.
        blended = cv2.addWeighted(out, 1.0 - self._alpha, overlay,
                                  self._alpha, 0)
        # np.copyto(where=...) is a C-level masked copy. The equivalent
        # `out[binary] = blended[binary]` boolean fancy-indexing measured
        # roughly an order of magnitude slower on a 1920x1080 frame.
        np.copyto(out, blended, where=binary[:, :, None])
        return out

    def _fit_source(self, canvas: np.ndarray, side_by_side: bool) -> np.ndarray:
        """Downscale the input image so the PUBLISHED frame is at most
        `max_width` wide (0 disables).

        In side-by-side mode the published frame holds two panels plus a
        `_SEP_W`-px separator, so each panel is scaled to (max_width - sep)/2.
        Everything downstream (blend, legend, mask decode) then runs at the
        final resolution, which is both faster and keeps annotation text crisp
        because it is never scaled after being drawn.
        """
        w = int(self._max_width)
        if w <= 0:
            return canvas
        target = (w - _SEP_W) // 2 if side_by_side else w
        if target <= 0 or canvas.shape[1] <= target:
            return canvas
        scale = target / float(canvas.shape[1])
        return cv2.resize(
            canvas, (target, max(1, int(round(canvas.shape[0] * scale)))),
            interpolation=cv2.INTER_AREA)

    def _draw_hud(self, canvas: np.ndarray, split_x: int = 0) -> np.ndarray:
        """Opaque top-left HUD; labels the drivable half in side-by-side mode."""
        h, w = canvas.shape[:2]
        fs = max(0.5, min(w, h) / 1600.0)
        font = cv2.FONT_HERSHEY_DUPLEX
        pad = max(6, int(round(14.0 * fs)))
        gap = max(2, int(round(6.0 * fs)))

        line1 = 'M4.3  Semantic Segmentation'
        line2 = f'FPS {self._hud_fps:.1f}    semantic / drivable'

        (t1w, t1h), b1 = cv2.getTextSize(line1, font, fs, 1)
        (t2w, t2h), b2 = cv2.getTextSize(line2, font, fs, 1)
        panel_w = min(max(t1w, t2w) + 2 * pad, w)
        panel_h = min(t1h + b1 + gap + t2h + b2 + 2 * pad, h)
        if panel_w < 4 or panel_h < 4:
            return canvas

        cv2.rectangle(canvas, (0, 0), (panel_w - 1, panel_h - 1),
                      _HUD_BG_BGR, -1)
        cv2.line(canvas, (0, panel_h - 1), (panel_w - 1, panel_h - 1),
                 _HUD_ACCENT_BGR, 2, cv2.LINE_AA)

        y = pad + t1h
        cv2.putText(canvas, line1, (pad, y), font, fs,
                    (255, 255, 255), 1, cv2.LINE_AA)
        y += b1 + gap + t2h
        cv2.putText(canvas, line2, (pad, y), font, fs,
                    _HUD_ACCENT_BGR, 1, cv2.LINE_AA)

        if split_x > 0:
            # Mark where the drivable half starts (side-by-side mode).
            x = min(split_x, w - 1)
            cv2.line(canvas, (x, 0), (x, panel_h - 1),
                     _HUD_ACCENT_BGR, 2, cv2.LINE_AA)
            cv2.putText(canvas, 'drivable', (x + pad, pad + t1h),
                        font, fs, _HUD_ACCENT_BGR, 1, cv2.LINE_AA)
        return canvas

    @staticmethod
    def _side_by_side(left: np.ndarray, right: np.ndarray) -> np.ndarray:
        if left.shape[0] != right.shape[0]:
            h = max(left.shape[0], right.shape[0])
            if left.shape[0] < h:
                pad = np.zeros((h - left.shape[0], left.shape[1], 3),
                               dtype=left.dtype)
                left = np.vstack([left, pad])
            if right.shape[0] < h:
                pad = np.zeros((h - right.shape[0], right.shape[1], 3),
                               dtype=right.dtype)
                right = np.vstack([right, pad])
        sep = np.full((left.shape[0], _SEP_W, 3), 32, dtype=np.uint8)
        return np.hstack([left, sep, right])


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
