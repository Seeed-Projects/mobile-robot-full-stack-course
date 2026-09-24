"""Integration test: tracking_node end-to-end with rclpy.

Pattern: drive the TrackingNode callback directly. We bypass DDS
altogether by calling `node._on_detections(msg)` synchronously, which
is exactly what the subscription callback would do. We then read back
from the publisher by intercepting publishes via a capture shim that
records every published message.

This is the same pattern used by the bev_detection engine_smoke test:
verify node wiring, parameter loading, tracker wiring, and ROS message
serialization without spinning a full DDS roundtrip.

Run with:
    colcon test --packages-select bev_tracking --ctest-args -V
"""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip('rclpy')

from std_msgs.msg import Header
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


def _make_detection(cx: float, cy: float, sx: float, sy: float,
                    cls: str = 'person', score: float = 0.9,
                    det_id: str = '') -> Detection2D:
    det = Detection2D()
    det.bbox = BoundingBox2D()
    det.bbox.center.position.x = cx
    det.bbox.center.position.y = cy
    det.bbox.size_x = sx
    det.bbox.size_y = sy
    det.bbox.center.theta = 0.0
    det.id = det_id
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = cls
    hyp.hypothesis.score = score
    det.results.append(hyp)
    return det


def _make_msg(stamp_sec: int, detections, frame_id: str = 'front_camera') -> Detection2DArray:
    m = Detection2DArray()
    m.header = Header()
    m.header.stamp.sec = stamp_sec
    m.header.frame_id = frame_id
    for d in detections:
        m.detections.append(d)
    return m


@pytest.fixture(scope='module')
def rclpy_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _make_node():
    from bev_tracking.tracking_node import TrackingNode
    return TrackingNode()


def _install_publish_capture(node):
    """Wrap the node's publisher so we can read what gets sent.

    Returns a list that gets appended with every published message.
    """
    captured = []
    original_publish = node._pub.publish

    def _publish(msg):
        captured.append(msg)
        original_publish(msg)

    node._pub.publish = _publish
    return captured


def test_first_frame_assigns_tracker_id(rclpy_context):
    node = _make_node()
    captured = _install_publish_capture(node)

    msg = _make_msg(stamp_sec=100, detections=[
        _make_detection(cx=100.0, cy=200.0, sx=50.0, sy=80.0),
    ])
    node._on_detections(msg)

    assert len(captured) == 1, 'expected exactly one published tracks message'
    out = captured[0]
    assert len(out.detections) == 1
    assert out.detections[0].id != '', f'expected non-empty track id, got {out.detections[0].id!r}'
    # Header preserved.
    assert out.header.frame_id == 'front_camera'
    assert out.header.stamp.sec == 100
    # Bbox affine: cx=100, sx=50 => xyxy = [75, 160, 125, 240]
    bb = out.detections[0].bbox
    assert bb.center.position.x == pytest.approx(100.0)
    assert bb.center.position.y == pytest.approx(200.0)
    assert bb.size_x == pytest.approx(50.0)
    assert bb.size_y == pytest.approx(80.0)
    node.destroy_node()


def test_stable_id_across_motion(rclpy_context):
    node = _make_node()
    captured = _install_publish_capture(node)

    for i in range(5):
        msg = _make_msg(stamp_sec=200 + i, detections=[
            _make_detection(cx=100.0 + 5 * i, cy=200.0, sx=50.0, sy=80.0),
        ])
        node._on_detections(msg)

    assert len(captured) == 5
    ids = [captured[i].detections[0].id for i in range(5) if captured[i].detections]
    assert all(i for i in ids), 'every output must carry a non-empty track id'
    assert len(set(ids)) == 1, f'id should be stable across motion, got {ids}'
    node.destroy_node()


def test_empty_input_still_publishes(rclpy_context):
    node = _make_node()
    captured = _install_publish_capture(node)

    # First, seed the tracker with one detection so it is not idle.
    seed = _make_msg(stamp_sec=300, detections=[
        _make_detection(cx=100.0, cy=200.0, sx=50.0, sy=80.0),
    ])
    node._on_detections(seed)

    # Now publish an empty detection array.
    empty = _make_msg(stamp_sec=301, detections=[])
    node._on_detections(empty)

    assert len(captured) == 2
    # The empty input must still produce a tracks message (possibly empty
    # or possibly the still-remembered track).
    assert isinstance(captured[1], Detection2DArray)
    # Header must still be forwarded.
    assert captured[1].header.stamp.sec == 301
    assert captured[1].header.frame_id == 'front_camera'
    node.destroy_node()


def test_multiple_objects_get_distinct_ids(rclpy_context):
    node = _make_node()
    captured = _install_publish_capture(node)

    msg = _make_msg(stamp_sec=400, detections=[
        _make_detection(cx=50.0, cy=200.0, sx=50.0, sy=80.0, cls='person'),
        _make_detection(cx=500.0, cy=200.0, sx=50.0, sy=80.0, cls='car'),
    ])
    node._on_detections(msg)

    assert len(captured) == 1
    assert len(captured[0].detections) == 2
    ids = [d.id for d in captured[0].detections]
    assert len(set(ids)) == 2, f'expected two distinct ids, got {ids}'
    # class_ids preserved.
    classes = sorted(d.results[0].hypothesis.class_id for d in captured[0].detections)
    assert classes == ['car', 'person']
    node.destroy_node()


def test_confidence_below_threshold_is_filtered(rclpy_context):
    node = _make_node()
    captured = _install_publish_capture(node)

    msg = _make_msg(stamp_sec=500, detections=[
        _make_detection(cx=50.0, cy=200.0, sx=50.0, sy=80.0, score=0.1),
    ])
    node._on_detections(msg)

    assert len(captured) == 1
    # The default track_activation_threshold is 0.25; 0.1 is below it,
    # so the tracker should reject the detection.
    assert len(captured[0].detections) == 0, \
        f'low-confidence detection should not produce a track, got {len(captured[0].detections)}'
    node.destroy_node()
