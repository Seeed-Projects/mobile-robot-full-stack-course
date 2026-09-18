"""Unit tests for the ROS <-> supervision adapter (no rclpy required).

These tests cover the bbox affine conversion in both directions plus
header / tracker_id forwarding. They MUST NOT require a running ROS
daemon; they import only pure-Python message types.

Run with:
    colcon test --packages-select bev_tracking --ctest-args -V
"""

from __future__ import annotations

import numpy as np
import pytest
import supervision as sv

from std_msgs.msg import Header
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from bev_tracking.adapter import detection_to_tracker, tracker_to_tracks


def _make_detection(cx: float, cy: float, sx: float, sy: float,
                    class_id: str = 'person', score: float = 0.9) -> Detection2D:
    det = Detection2D()
    det.bbox = BoundingBox2D()
    det.bbox.center.position.x = cx
    det.bbox.center.position.y = cy
    det.bbox.size_x = sx
    det.bbox.size_y = sy
    det.bbox.center.theta = 0.0
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = class_id
    hyp.hypothesis.score = score
    det.results.append(hyp)
    return det


def _make_header(stamp_sec: int = 1, frame_id: str = 'front_camera') -> Header:
    h = Header()
    h.stamp.sec = stamp_sec
    h.frame_id = frame_id
    return h


def test_empty_input_returns_empty_detections():
    msg = Detection2DArray()
    msg.header = _make_header()
    det = detection_to_tracker(msg)
    assert isinstance(det, sv.Detections)
    assert len(det) == 0


def test_single_detection_xyxy_round_trip():
    msg = Detection2DArray()
    msg.header = _make_header(stamp_sec=42, frame_id='cam')
    msg.detections.append(_make_detection(cx=150.0, cy=200.0, sx=100.0, sy=80.0))

    det = detection_to_tracker(msg)
    assert len(det) == 1
    # center 150, size 100 => xyxy = [100, 160, 200, 240]
    assert det.xyxy.shape == (1, 4)
    assert det.xyxy.dtype == np.float32
    np.testing.assert_allclose(det.xyxy[0], [100.0, 160.0, 200.0, 240.0], atol=1e-5)
    assert abs(det.confidence[0] - 0.9) < 1e-6
    assert det.class_id[0] == 'person'


def test_detection_without_results_uses_zero_confidence_and_empty_class():
    msg = Detection2DArray()
    msg.header = _make_header()
    det_in = Detection2D()
    det_in.bbox = BoundingBox2D()
    det_in.bbox.center.position.x = 50.0
    det_in.bbox.center.position.y = 50.0
    det_in.bbox.size_x = 10.0
    det_in.bbox.size_y = 20.0
    msg.detections.append(det_in)

    det = detection_to_tracker(msg)
    assert len(det) == 1
    assert det.confidence[0] == 0.0
    assert det.class_id[0] == ''


def test_tracker_id_forwarded_when_present():
    tracks = sv.Detections(
        xyxy=np.array([[10.0, 20.0, 30.0, 40.0]], dtype=np.float32),
        confidence=np.array([0.9], dtype=np.float32),
        class_id=np.array(['person'], dtype=object),
    )
    # supervision sets tracker_id via update_with_detections; emulate.
    tracks.tracker_id = np.array([7])

    header = _make_header(stamp_sec=99, frame_id='cam_front')
    out = tracker_to_tracks(tracks, header)

    assert len(out.detections) == 1
    det = out.detections[0]
    # center = (10+30)/2 = 20, (20+40)/2 = 30
    assert det.bbox.center.position.x == pytest.approx(20.0)
    assert det.bbox.center.position.y == pytest.approx(30.0)
    assert det.bbox.size_x == pytest.approx(20.0)
    assert det.bbox.size_y == pytest.approx(20.0)
    assert det.id == '7'
    assert det.results[0].hypothesis.class_id == 'person'
    assert det.results[0].hypothesis.score == pytest.approx(0.9)
    # Header is forwarded verbatim, not synthesised.
    assert out.header.frame_id == 'cam_front'
    assert out.header.stamp.sec == 99


def test_empty_tracker_output_yields_empty_array_with_header():
    tracks = sv.Detections.empty()
    header = _make_header(stamp_sec=7)
    out = tracker_to_tracks(tracks, header)
    assert len(out.detections) == 0
    assert out.header.frame_id == 'front_camera'
    assert out.header.stamp.sec == 7


def test_no_tracker_id_is_emitted_as_empty_string():
    tracks = sv.Detections(
        xyxy=np.array([[0.0, 0.0, 5.0, 5.0]], dtype=np.float32),
        confidence=np.array([0.5], dtype=np.float32),
        class_id=np.array(['person'], dtype=object),
    )
    # tracker_id is None when never updated
    assert tracks.tracker_id is None

    out = tracker_to_tracks(tracks, _make_header())
    assert len(out.detections) == 1
    assert out.detections[0].id == ''


def test_multi_object_round_trip_preserves_order():
    msg = Detection2DArray()
    msg.header = _make_header()
    msg.detections.append(_make_detection(cx=50.0, cy=50.0, sx=10.0, sy=10.0,
                                          class_id='a', score=0.9))
    msg.detections.append(_make_detection(cx=200.0, cy=200.0, sx=40.0, sy=40.0,
                                          class_id='b', score=0.7))

    det = detection_to_tracker(msg)
    assert len(det) == 2
    np.testing.assert_allclose(det.xyxy[0], [45.0, 45.0, 55.0, 55.0], atol=1e-5)
    np.testing.assert_allclose(det.xyxy[1], [180.0, 180.0, 220.0, 220.0], atol=1e-5)
    assert det.class_id[0] == 'a'
    assert det.class_id[1] == 'b'
