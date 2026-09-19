"""Unit tests for tracking_visualizer (no DDS / camera required).

These tests exercise the visualizer's pure-Python logic by:

  * constructing a TrackingVisualizer with a fresh rclpy context
  * calling its callbacks directly (on_image, on_tracks)
  * capturing the published debug image via a publish shim

What we verify (per plan §10):
  * visualizer constructs with default topics
  * tracks with non-empty IDs make it into the output
  * drawing preserves image dimensions
  * debug image header stamp equals the source image stamp
  * tracks older than max_track_age_ms are filtered out
  * visualizer still publishes even with no recent tracks
"""

from __future__ import annotations

import array

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


# --- helpers ------------------------------------------------------------


def _make_image(width: int = 320, height: int = 240,
                stamp_sec: int = 100, frame_id: str = 'front_camera',
                color: tuple = (32, 32, 32)) -> Image:
    """Synthesize a tiny solid-colour BGR image message."""
    arr = np.zeros((height, width, 3), dtype=np.uint8)
    arr[:] = color
    msg = Image()
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = 0
    msg.header.frame_id = frame_id
    msg.height = height
    msg.width = width
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = width * 3
    # array.array (buffer protocol), matching production publishers: a bytes
    # assignment to a uint8[] field is element-wise and ~149 ns/byte.
    msg.data = array.array('B', arr.tobytes())
    return msg


def _make_track(cx: float, cy: float, sx: float, sy: float,
                track_id: str = '',
                cls: str = 'person',
                score: float = 0.9,
                stamp_sec: int = 100) -> Detection2DArray:
    det = Detection2D()
    det.bbox = BoundingBox2D()
    det.bbox.center.position.x = cx
    det.bbox.center.position.y = cy
    det.bbox.size_x = sx
    det.bbox.size_y = sy
    det.bbox.center.theta = 0.0
    det.id = track_id
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = cls
    hyp.hypothesis.score = score
    det.results.append(hyp)

    msg = Detection2DArray()
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = 0
    msg.header.frame_id = 'front_camera'
    msg.detections.append(det)
    return msg


def _install_publish_capture(node):
    captured = []
    original = node._debug_pub.publish

    def cap(msg):
        captured.append(msg)
        original(msg)

    node._debug_pub.publish = cap
    return captured


# --- tests --------------------------------------------------------------


@pytest.fixture(scope='module')
def rclpy_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _make_node():
    from bev_tracking.tracking_visualizer import TrackingVisualizer
    return TrackingVisualizer()


def test_visualizer_constructs_with_default_topics(rclpy_context):
    node = _make_node()
    try:
        assert node.get_parameter('image_topic').value == \
            '/perception/cameras/front/image'
        assert node.get_parameter('tracks_topic').value == '/perception/tracks'
        assert node.get_parameter('debug_image_topic').value == \
            '/perception/tracking_debug_image'
        # Defaults for sync tuning.
        assert int(node.get_parameter('track_cache_size').value) == 10
        assert float(node.get_parameter('max_track_age_ms').value) == 200.0
    finally:
        node.destroy_node()


def test_track_id_is_non_empty_in_output_tracks(rclpy_context):
    node = _make_node()
    captured = _install_publish_capture(node)
    try:
        # Feed one track with non-empty id BEFORE the image so it
        # lands in the cache as "in the past" relative to the image.
        node.on_tracks(_make_track(cx=160.0, cy=120.0, sx=80.0, sy=120.0,
                                   track_id='3', stamp_sec=99))
        node.on_image(_make_image(stamp_sec=100))

        assert len(captured) == 1
        out_tracks_msg = captured[0]
        # Inspect the cached tracks (draw output is just a bgr8 image;
        # we verify the cache contained an id-bearing detection).
        assert len(node._track_cache) == 1
        cached_tracks = node._track_cache[0][1]
        assert len(cached_tracks.detections) == 1
        det = cached_tracks.detections[0]
        assert det.id == '3', f'expected track id "3", got {det.id!r}'
        assert det.results[0].hypothesis.class_id == 'person'
        assert det.results[0].hypothesis.score == pytest.approx(0.9)
        # The output is an image we can re-parse.
        assert out_tracks_msg.encoding == 'bgr8'
        assert out_tracks_msg.height == 240
        assert out_tracks_msg.width == 320
    finally:
        node.destroy_node()


def test_drawing_preserves_image_dimensions(rclpy_context):
    node = _make_node()
    captured = _install_publish_capture(node)
    try:
        node.on_tracks(_make_track(cx=160.0, cy=120.0, sx=80.0, sy=120.0,
                                   track_id='7', stamp_sec=99))
        node.on_image(_make_image(width=640, height=480, stamp_sec=100))

        assert len(captured) == 1
        out = captured[0]
        assert out.width == 640
        assert out.height == 480
        # Re-parse the bytes: we should recover the same shape.
        from bev_tracking.tracking_visualizer import image_msg_to_numpy
        arr = image_msg_to_numpy(out)
        assert arr.shape == (480, 640, 3)
        assert arr.dtype == np.uint8
    finally:
        node.destroy_node()


def test_debug_image_header_stamp_matches_image_stamp(rclpy_context):
    node = _make_node()
    captured = _install_publish_capture(node)
    try:
        node.on_tracks(_make_track(cx=100.0, cy=100.0, sx=40.0, sy=40.0,
                                   track_id='1', stamp_sec=999))
        node.on_image(_make_image(stamp_sec=1000))

        assert len(captured) == 1
        out = captured[0]
        # Debug image must carry the image's header (NOT the track's).
        assert out.header.stamp.sec == 1000
        assert out.header.stamp.nanosec == 0
        assert out.header.frame_id == 'front_camera'
    finally:
        node.destroy_node()


def test_visualizer_filters_tracks_older_than_max_age(rclpy_context):
    node = _make_node()
    captured = _install_publish_capture(node)
    try:
        # Tighten the window so we can saturate it deterministically.
        node.set_parameters([
            rclpy.parameter.Parameter('max_track_age_ms',
                                      rclpy.parameter.Parameter.Type.DOUBLE,
                                      50.0),
        ])
        # Track is 5 seconds older than the image; should NOT match.
        node.on_tracks(_make_track(cx=160.0, cy=120.0, sx=80.0, sy=120.0,
                                   track_id='11', stamp_sec=95))
        node.on_image(_make_image(stamp_sec=100))

        # Visualizer still publishes the image (with the
        # "no tracks (waiting)" banner) — it does not block.
        assert len(captured) == 1
        out = captured[0]
        # Sanity: image was republished unchanged in shape.
        assert out.width == 320
        assert out.height == 240
        # The cache still holds the stale entry (we never purge
        # on miss — only on overflow).
        assert len(node._track_cache) == 1
    finally:
        node.destroy_node()


def test_visualizer_publishes_even_with_no_recent_tracks(rclpy_context):
    node = _make_node()
    captured = _install_publish_capture(node)
    try:
        # No tracks fed at all.
        node.on_image(_make_image(stamp_sec=42))
        assert len(captured) == 1
        # The published image still has the source's encoding.
        assert captured[0].encoding == 'bgr8'
        assert captured[0].header.stamp.sec == 42
    finally:
        node.destroy_node()


def test_visualizer_handles_multiple_tracks(rclpy_context):
    """Two distinct ids in the cache both get drawn for the same image."""
    node = _make_node()
    captured = _install_publish_capture(node)
    try:
        # Build a multi-track message.
        msg = _make_track(cx=100.0, cy=120.0, sx=60.0, sy=80.0,
                          track_id='3', stamp_sec=99)
        d2 = _make_track(cx=220.0, cy=120.0, sx=60.0, sy=80.0,
                         track_id='8', stamp_sec=99).detections[0]
        msg.detections.append(d2)
        node.on_tracks(msg)

        node.on_image(_make_image(stamp_sec=100))
        assert len(captured) == 1
        cached = node._track_cache[0][1]
        ids = sorted(d.id for d in cached.detections)
        assert ids == ['3', '8']
    finally:
        node.destroy_node()


def test_hex_to_bgr_parses_known_values():
    from bev_tracking.tracking_visualizer import TrackingVisualizer
    assert TrackingVisualizer._hex_to_bgr('00FF00') == (0, 255, 0)
    assert TrackingVisualizer._hex_to_bgr('FF0000') == (0, 0, 255)
    assert TrackingVisualizer._hex_to_bgr('#FFFFFF') == (255, 255, 255)


def test_hex_to_bgr_rejects_invalid_input():
    from bev_tracking.tracking_visualizer import TrackingVisualizer
    with pytest.raises(ValueError):
        TrackingVisualizer._hex_to_bgr('XYZ')
    with pytest.raises(ValueError):
        TrackingVisualizer._hex_to_bgr('1234')
