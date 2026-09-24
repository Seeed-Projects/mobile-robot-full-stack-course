"""bev_tracking/tracking_visualizer.py

Lightweight ROS 2 node that overlays ByteTrack tracks onto a camera
image and republishes the result on /perception/tracking_debug_image.

The visualizer is intentionally separate from TrackingNode so that:

  * TrackingNode stays camera-independent (it consumes only
    Detection2DArray; this is a hard contract).
  * The visualization layer can be replaced (e.g. rqt_image_view,
    web viewer, no viewer at all) without touching tracking code.
  * Tests can drive the visualizer through its two callbacks
    without bringing up any DDS or GPU pipeline.

Pipeline contract:

    /perception/cameras/front/image  (sensor_msgs/Image)
        --> TrackingVisualizer.on_image
    /perception/tracks              (vision_msgs/Detection2DArray)
        --> TrackingVisualizer.on_tracks
        --> /perception/tracking_debug_image  (sensor_msgs/Image, bgr8)

The visualizer never runs detection or tracking itself; it only
draws whatever ByteTrack published.

Track/image alignment:

    Tracks carry the camera header they were computed against. We
    keep a small bounded cache of recent tracks (default 10 entries,
    ~1 s at 10 Hz). When an image arrives we pick the cache entry
    whose header stamp is closest to the image stamp but no older
    than max_track_age_ms (default 200 ms). If no entry qualifies we
    still publish the image so the viewer keeps streaming; we just
    skip overlay drawing.

Overlay style (see "overlay style" section below):

    * One bright, deterministic colour per track id (golden-angle HSV
      palette), not one flat green for every object.
    * Thick L-shaped corner brackets + a thin full outline, both scaled to
      the image resolution (4 px at 1920x1080 instead of a fixed 2 px).
    * A filled label chip in the track colour, clamped inside the frame on
      all four sides so a border box never loses its label.
    * An opaque top-left HUD with the draw rate and track count, drawn last.

    This deliberately mirrors the M4.1 C++ overlay in
    bev_detection/src/yolo_trt_node.cpp so both hub tabs look like one
    product.
"""

from __future__ import annotations

import array
import colorsys
import time
from collections import deque
from typing import Deque, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray


# Encodings we can decode. Mirrors screenshot_saver.py exactly so the
# demo accepts whatever YOLO / camera drivers actually emit.
_BGR_ENCODINGS = {'bgr8', 'rgb8'}
_BGRA_ENCODINGS = {'bgra8', 'rgba8'}
_MONO_ENCODINGS = {'mono8', '8UC1'}


def image_msg_to_numpy(msg: Image) -> np.ndarray:
    """Convert a sensor_msgs/Image into an HxWx3 uint8 BGR array.

    Hand-written decoder (no cv_bridge) — same set of encodings as
    bev_detection/test/screenshot_saver.py so the visualizer works
    on whatever the upstream nodes actually publish.
    """
    h, w = int(msg.height), int(msg.width)
    enc = (msg.encoding or 'bgr8').lower()
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    if enc in _BGR_ENCODINGS:
        arr = raw.reshape(h, w, 3).copy()
        if enc == 'rgb8':
            arr = arr[:, :, ::-1]
        return arr
    if enc in _BGRA_ENCODINGS:
        bgra = raw.reshape(h, w, 4).copy()
        return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
    if enc in _MONO_ENCODINGS:
        gray = raw.reshape(h, w).copy()
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if enc in ('16uc1', 'mono16'):
        arr16 = raw.reshape(h, w).copy()
        arr8 = cv2.convertScaleAbs(arr16, alpha=(255.0 / 65535.0))
        return cv2.cvtColor(arr8, cv2.COLOR_GRAY2BGR)
    if enc == '32fc1':
        arr32 = raw.reshape(h, w).copy()
        arr8 = cv2.normalize(arr32, None, 0, 255, cv2.NORM_MINMAX,
                             dtype=cv2.CV_8U)
        return cv2.cvtColor(arr8, cv2.COLOR_GRAY2BGR)
    if enc == 'bayer_rggb8':
        bayer = raw.reshape(h, w).copy()
        return cv2.cvtColor(bayer, cv2.COLOR_BayerRGGB2BGR)

    # Unknown encoding: try BGR and hope for the best. This is the
    # same fallback behaviour as screenshot_saver (which raises).
    raise ValueError(f'unsupported image encoding {enc!r}')


def _stamp_to_ns(stamp) -> int:
    """Convert a std_msgs/Header.stamp to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _stamp_diff_ns(image_ns: int, track_ns: int) -> int:
    """Signed difference (track - image) in nanoseconds.

    Positive => track is newer than image (rare; usually stale).
    Negative => track is older than image (typical, tracks computed
    from detections whose header is the image stamp).
    """
    return track_ns - image_ns


# ---- overlay style ------------------------------------------------------
# The previous overlay drew EVERY track as one flat green 2 px rectangle with
# a 0.55 font and a black label bar that was only clamped on the top/left
# edges. On a 1080p fisheye scene that was barely visible and multi-object
# frames were unreadable. The style below mirrors the M4.1 C++ overlay so both
# hub tabs look like one product.


def _build_id_palette(n: int = 64) -> list:
    """Bright, deterministic BGR palette for track ids.

    Hues step by the golden angle (137.508 deg) at S=0.85/V=1.0 so every id
    is saturated and clearly separable from its neighbours, and a given id
    always maps to the same colour across frames.
    """
    out = []
    for i in range(n):
        hue = (i * 137.508) % 360.0
        r, g, b = colorsys.hsv_to_rgb(hue / 360.0, 0.85, 1.0)
        out.append((int(b * 255), int(g * 255), int(r * 255)))
    return out


_ID_PALETTE = _build_id_palette()

# HUD panel fill (BGR), matching yolo_trt_node.cpp's drawHudPanel().
_HUD_BG_BGR = (24, 28, 34)
_HUD_ACCENT_BGR = (120, 230, 255)


def _style_for(
    canvas: np.ndarray,
    min_thickness: int = 0,
    min_font_scale: float = 0.0,
) -> Tuple[int, int, float, int]:
    """Resolution-adaptive overlay metrics.

    Returns (thickness, corner_len, font_scale, inner_thickness). At
    1920x1080 this yields a 4 px bracket and a 0.77 font instead of the old
    fixed 2 px / 0.55. `min_thickness` / `min_font_scale` are the
    ROS-parameter overrides (0 = auto).
    """
    h, w = canvas.shape[:2]
    short = float(min(w, h))
    thickness = max(2, int(round(short / 300.0)), int(min_thickness))
    corner_len = max(12, int(round(short / 45.0)))
    font_scale = max(0.55, short / 1400.0, float(min_font_scale))
    return thickness, corner_len, font_scale, max(1, thickness // 3)


def _draw_corner_brackets(
    canvas: np.ndarray, x1: int, y1: int, x2: int, y2: int,
    color: Tuple[int, int, int], thickness: int, length: int,
) -> None:
    """Four thick L-shaped corners: prominent without hiding the object."""
    lx = max(4, min(length, (x2 - x1) // 2))
    ly = max(4, min(length, (y2 - y1) // 2))
    for ax, ay, bx, by in (
        (x1, y1, x1 + lx, y1), (x1, y1, x1, y1 + ly),
        (x2, y1, x2 - lx, y1), (x2, y1, x2, y1 + ly),
        (x1, y2, x1 + lx, y2), (x1, y2, x1, y2 - ly),
        (x2, y2, x2 - lx, y2), (x2, y2, x2, y2 - ly),
    ):
        cv2.line(canvas, (ax, ay), (bx, by), color, thickness, cv2.LINE_AA)


class TrackingVisualizer(Node):
    """Subscribe image + tracks, publish image + drawing overlays."""

    def __init__(self) -> None:
        super().__init__('tracking_visualizer')

        # Topics — all parameterised so the launch file can remap.
        self.declare_parameter('image_topic', '/perception/cameras/front/image')
        self.declare_parameter('tracks_topic', '/perception/tracks')
        self.declare_parameter('debug_image_topic',
                               '/perception/tracking_debug_image')

        # Cache / sync tuning.
        self.declare_parameter('track_cache_size', 10)
        self.declare_parameter('max_track_age_ms', 200.0)
        self.declare_parameter('log_throttle_ms', 1000)

        # Drawing style. Hex strings ("00FF00") so the launch file can
        # override without YAML-quoting headache.
        self.declare_parameter('bbox_color_bgr', '00FF00')  # green
        # 0 / 0.0 mean "auto": the overlay scales with the image resolution
        # (4 px + 0.77 font at 1920x1080). A positive value raises the
        # adaptive value to at least that much, so old configs that pin a
        # large thickness still work.
        self.declare_parameter('bbox_thickness', 0)
        self.declare_parameter('label_text_color_bgr', 'FFFFFF')  # white
        # HUD panel fill (RRGGBB). Default #181C22 = the same dark navy the
        # M4.1 C++ HUD uses.
        self.declare_parameter('label_bg_color_bgr', '221C18')
        self.declare_parameter('label_font_scale', 0.0)
        self.declare_parameter('label_thickness', 2)
        # Per-track colour from _ID_PALETTE. Set false to fall back to the
        # single `bbox_color_bgr` (pre-existing behaviour).
        self.declare_parameter('color_by_id', True)

        image_topic = self.get_parameter('image_topic').value
        tracks_topic = self.get_parameter('tracks_topic').value
        debug_topic = self.get_parameter('debug_image_topic').value

        cache_size = int(self.get_parameter('track_cache_size').value)
        self._max_age_ns = int(
            float(self.get_parameter('max_track_age_ms').value) * 1e6)
        self._log_throttle_us = int(
            float(self.get_parameter('log_throttle_ms').value) * 1000.0)
        self._last_log_us = 0

        self._bbox_color = self._hex_to_bgr(
            self.get_parameter('bbox_color_bgr').value)
        self._label_text_color = self._hex_to_bgr(
            self.get_parameter('label_text_color_bgr').value)
        self._label_bg_color = self._hex_to_bgr(
            self.get_parameter('label_bg_color_bgr').value)
        self._bbox_thickness = int(self.get_parameter('bbox_thickness').value)
        self._label_font_scale = float(
            self.get_parameter('label_font_scale').value)
        self._label_thickness = int(
            self.get_parameter('label_thickness').value)
        self._color_by_id = bool(self.get_parameter('color_by_id').value)

        # QoS — match M4.1 / M4.2 (BEST_EFFORT, KEEP_LAST, depth=10).
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._track_cache: Deque[Tuple[int, Detection2DArray]] = deque(
            maxlen=max(cache_size, 1))

        self._image_sub = self.create_subscription(
            Image, image_topic, self.on_image, qos)
        self._tracks_sub = self.create_subscription(
            Detection2DArray, tracks_topic, self.on_tracks, qos)
        self._debug_pub = self.create_publisher(
            Image, debug_topic, qos)

        # Diagnostics.
        self._frame_count = 0
        self._tracks_seen = 0
        self._tracks_with_id = 0
        self._last_latency_ms = 0.0
        self._hud_fps = 0.0
        self._last_draw_t = 0.0

        self.get_logger().info(
            f'tracking_visualizer ready: '
            f'sub_image={image_topic} sub_tracks={tracks_topic} '
            f'pub={debug_topic} max_track_age_ms='
            f'{self._max_age_ns / 1e6:.0f} cache={cache_size}'
        )

    # ---- helpers ---------------------------------------------------------

    @staticmethod
    def _hex_to_bgr(spec: str) -> Tuple[int, int, int]:
        """Convert 'RRGGBB' hex into a (b, g, r) tuple for cv2."""
        s = spec.strip().lstrip('#')
        if len(s) != 6:
            raise ValueError(f'colour must be RRGGBB hex, got {spec!r}')
        r = int(s[0:2], 16)
        g = int(s[2:4], 16)
        b = int(s[4:6], 16)
        return (b, g, r)

    def _find_matching_track(
        self, image_ns: int,
    ) -> Optional[Detection2DArray]:
        """Pick the most recent track whose stamp is not too far
        behind the image.

        Returns None if the cache is empty or every entry is older
        than max_track_age_ms. We DO NOT pick a track that is newer
        than the image (positive diff) — those are forecasts / future
        detections we never want to draw on a current frame.
        """
        if not self._track_cache:
            return None
        best = None
        best_age_ns = None
        for stamp_ns, tracks in self._track_cache:
            diff = _stamp_diff_ns(image_ns, stamp_ns)
            if diff > 0:
                # Track newer than image; skip. We do not draw future
                # tracks on a past frame.
                continue
            age = -diff  # how far in the past the track is
            if age > self._max_age_ns:
                continue
            if best_age_ns is None or age < best_age_ns:
                best_age_ns = age
                best = tracks
        return best

    def _color_for(self, det) -> Tuple[int, int, int]:
        """Stable per-track colour; falls back to bbox_color_bgr when
        `color_by_id` is disabled."""
        if not self._color_by_id:
            return self._bbox_color
        raw = str(det.id or '')
        if raw:
            try:
                key = int(raw)
            except ValueError:
                key = abs(hash(raw))
        else:
            # Detection not yet assigned an id: keep it neutral-ish but
            # still from the palette so it stays visible.
            key = 0
        return _ID_PALETTE[key % len(_ID_PALETTE)]

    def _draw_tracks(
        self,
        canvas: np.ndarray,
        tracks: Detection2DArray,
    ) -> int:
        """Draw all tracks on `canvas` (in-place). Returns drawn count.

        A track with an empty `id` is drawn (we have a detection, just
        no id yet) but the label omits the "#N" prefix. This matches
        the empirical observation that ByteTrack occasionally emits
        a frame of detections with no id while it's still warming up
        (e.g. minimum_consecutive_frames > 1).
        """
        drawn = 0
        h, w = canvas.shape[:2]
        thickness, corner_len, font_scale, inner_thickness = _style_for(
            canvas, self._bbox_thickness, self._label_font_scale)

        for det in tracks.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            sx = det.bbox.size_x
            sy = det.bbox.size_y
            x1 = int(round(cx - sx * 0.5))
            y1 = int(round(cy - sy * 0.5))
            x2 = int(round(cx + sx * 0.5))
            y2 = int(round(cy + sy * 0.5))

            x1 = max(0, min(w - 1, x1))
            y1 = max(0, min(h - 1, y1))
            x2 = max(0, min(w - 1, x2))
            y2 = max(0, min(h - 1, y2))
            if x2 - x1 < 2 or y2 - y1 < 2:
                continue

            color = self._color_for(det)

            # Thin full outline for extent + thick bright corners for
            # prominence (same visual language as the M4.1 overlay).
            cv2.rectangle(
                canvas, (x1, y1), (x2, y2), color, inner_thickness,
                lineType=cv2.LINE_AA,
            )
            _draw_corner_brackets(
                canvas, x1, y1, x2, y2, color, thickness, corner_len,
            )

            cls = ''
            score = 0.0
            if det.results:
                cls = str(det.results[0].hypothesis.class_id or '')
                score = float(det.results[0].hypothesis.score or 0.0)

            if det.id:
                label = f'{cls} #{det.id} {score:.2f}'
            else:
                label = f'{cls} {score:.2f}'

            self._draw_label(canvas, label, x1, y1, x2, y2, color, font_scale)
            drawn += 1
        return drawn

    def _draw_label(
        self,
        canvas: np.ndarray,
        text: str,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        color: Tuple[int, int, int],
        font_scale: float,
    ) -> None:
        """Filled label chip in the track's colour, clamped inside the frame.

        The chip is drawn above the box when there is room and below it
        otherwise, and is clamped on all four sides. The old version only
        clamped the top/left edges, so a box against the right border had its
        label cut off by the frame edge.
        """
        font = cv2.FONT_HERSHEY_DUPLEX
        (tw, th), baseline = cv2.getTextSize(text, font, font_scale, 1)
        pad_x = max(4, int(round(font_scale * 8.0)))
        pad_y = max(3, int(round(font_scale * 5.0)))
        chip_w = tw + 2 * pad_x
        chip_h = th + baseline + 2 * pad_y

        h, w = canvas.shape[:2]
        ty = y1 - chip_h
        if ty < 0:
            ty = y2                       # no room above: flip below
        ty = max(0, min(ty, max(0, h - chip_h)))
        tx = max(0, min(x1, max(0, w - chip_w)))

        # -1 fill (LINE_8) so the chip exactly covers its rect.
        cv2.rectangle(canvas, (tx, ty), (tx + chip_w, ty + chip_h), color, -1)
        cv2.putText(
            canvas, text, (tx + pad_x, ty + pad_y + th),
            font, font_scale, self._label_text_color, self._label_thickness,
            lineType=cv2.LINE_AA,
        )

    def _draw_hud(
        self, canvas: np.ndarray, drawn: int, note: str = '',
    ) -> None:
        """Opaque top-left HUD, drawn LAST so nothing bleeds through it.

        Replaces the old 0.55-scale "no tracks (waiting)" banner, which was
        both tiny and only shown when there were no tracks at all.
        """
        h, w = canvas.shape[:2]
        short = float(min(w, h))
        fs = max(0.5, short / 1600.0)
        font = cv2.FONT_HERSHEY_DUPLEX
        pad = max(6, int(round(14.0 * fs)))
        gap = max(2, int(round(6.0 * fs)))

        line1 = 'M4.2  ByteTrack'
        line2 = f'FPS {self._hud_fps:.1f}    tracks {drawn}'
        if note:
            line2 += f'    {note}'

        (t1w, t1h), b1 = cv2.getTextSize(line1, font, fs, 1)
        (t2w, t2h), b2 = cv2.getTextSize(line2, font, fs, 1)
        panel_w = max(t1w, t2w) + 2 * pad
        panel_h = t1h + b1 + gap + t2h + b2 + 2 * pad
        panel_w = min(panel_w, w)
        panel_h = min(panel_h, h)
        if panel_w < 4 or panel_h < 4:
            return

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

    # ---- callbacks -------------------------------------------------------

    def on_tracks(self, msg: Detection2DArray) -> None:
        """Cache the latest tracks (overwrite, dedup by stamp)."""
        stamp_ns = _stamp_to_ns(msg.header.stamp)
        # Overwrite an existing entry with the same stamp (latest
        # tracker output for that frame wins).
        for i, (existing_ns, _) in enumerate(self._track_cache):
            if existing_ns == stamp_ns:
                del self._track_cache[i]
                break
        self._track_cache.append((stamp_ns, msg))
        self._tracks_seen += 1
        self._tracks_with_id += sum(1 for d in msg.detections if d.id)

    def on_image(self, msg: Image) -> None:
        """Draw tracks on the incoming image and republish."""
        t0 = time.perf_counter()
        try:
            canvas = image_msg_to_numpy(msg)
        except Exception as e:
            self.get_logger().warn(f'image parse failed: {e}')
            return

        image_ns = _stamp_to_ns(msg.header.stamp)
        tracks = self._find_matching_track(image_ns)
        drawn = 0
        note = ''
        if tracks is not None and len(tracks.detections) > 0:
            drawn = self._draw_tracks(canvas, tracks)
        else:
            note = 'waiting for tracks'

        # Smoothed draw rate for the HUD (independent of the throttled log).
        now = time.monotonic()
        if self._last_draw_t > 0.0:
            dt = now - self._last_draw_t
            if dt > 1e-6:
                inst = 1.0 / dt
                self._hud_fps = (
                    inst if self._hud_fps <= 0.0
                    else 0.9 * self._hud_fps + 0.1 * inst
                )
        self._last_draw_t = now
        self._draw_hud(canvas, drawn, note)

        out = self._numpy_to_image_msg(canvas, msg.header)
        self._debug_pub.publish(out)

        t1 = time.perf_counter()
        self._last_latency_ms = (t1 - t0) * 1000.0
        self._frame_count += 1

        now_us = self.get_clock().now().nanoseconds // 1000
        if (now_us - self._last_log_us) >= self._log_throttle_us:
            self._last_log_us = now_us
            self.get_logger().info(
                f'viz frame={self._frame_count} '
                f'drawn={drawn} cache={len(self._track_cache)} '
                f'latency_ms={self._last_latency_ms:.2f}'
            )

    def _numpy_to_image_msg(
        self, canvas: np.ndarray, header_in: Header,
    ) -> Image:
        """Wrap an HxWx3 uint8 BGR array back into sensor_msgs/Image."""
        out = Image()
        out.header = header_in
        out.height = int(canvas.shape[0])
        out.width = int(canvas.shape[1])
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = int(canvas.shape[1]) * 3
        # MUST be array.array (buffer protocol), never bytes. Assigning a
        # bytes object to the uint8[] field converts element-by-element at
        # ~149 ns/byte, which measured 812 ms/frame at 1920x1080 and held
        # this node at ~1.2 fps / 95% CPU. array.array costs ~0.6 ms.
        out.data = array.array('B', canvas.tobytes())
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackingVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
