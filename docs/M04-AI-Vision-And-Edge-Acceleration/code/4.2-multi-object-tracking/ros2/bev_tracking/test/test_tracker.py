"""Unit tests for ByteTrack behaviour under representative scenarios.

We DO NOT spin a real ROS node here; instead we drive supervision.ByteTrack
through the same call sequence that tracking_node.py uses. This is the
right level of test because the only behaviour the ROS node adds is
threading and message marshalling, both of which are tested at the
rclpy integration layer (test_integration.py).

Run with:
    colcon test --packages-select bev_tracking --ctest-args -V
"""

from __future__ import annotations

import warnings

import numpy as np
import pytest
import supervision as sv

warnings.filterwarnings('ignore', category=UserWarning)


@pytest.fixture
def tracker():
    # 30 Hz detections, 90-frame lost buffer => ~3 s occlusion tolerance.
    # Keep these in step with config/bytetrack.yaml.
    return sv.ByteTrack(
        track_activation_threshold=0.25,
        lost_track_buffer=90,
        minimum_matching_threshold=0.8,
        frame_rate=30,
        minimum_consecutive_frames=1,
    )


def _det(x1: float, y1: float, x2: float, y2: float,
         cls: str = 'person', score: float = 0.9) -> sv.Detections:
    return sv.Detections(
        xyxy=np.array([[x1, y1, x2, y2]], dtype=np.float32),
        confidence=np.array([score], dtype=np.float32),
        class_id=np.array([cls], dtype=object),
    )


def test_first_appearance_gets_id_one(tracker):
    tracks = tracker.update_with_detections(_det(10, 10, 30, 30))
    assert tracks.tracker_id is not None
    assert int(tracks.tracker_id[0]) == 1


def test_stationary_object_keeps_same_id(tracker):
    t1 = tracker.update_with_detections(_det(10, 10, 30, 30))
    t2 = tracker.update_with_detections(_det(10, 10, 30, 30))
    t3 = tracker.update_with_detections(_det(10, 10, 30, 30))
    assert int(t1.tracker_id[0]) == int(t2.tracker_id[0]) == int(t3.tracker_id[0])


def test_motion_keeps_id_under_50px_per_frame(tracker):
    ids = []
    for i in range(10):
        tracks = tracker.update_with_detections(_det(100 + 5 * i, 100, 120 + 5 * i, 130))
        ids.append(int(tracks.tracker_id[0]))
    assert len(set(ids)) == 1, f'ID changed across frames: {ids}'


def test_short_occlusion_preserves_id(tracker):
    t1 = tracker.update_with_detections(_det(100, 100, 150, 150))
    id1 = int(t1.tracker_id[0])
    # 5 empty frames (well within lost_track_buffer=90)
    for _ in range(5):
        empty = tracker.update_with_detections(sv.Detections.empty())
        assert len(empty) == 0
    t2 = tracker.update_with_detections(_det(110, 105, 160, 155))
    assert int(t2.tracker_id[0]) == id1, f'ID lost across 5-frame occlusion: {id1} -> {t2.tracker_id[0]}'


def test_long_occlusion_creates_new_id(tracker):
    t1 = tracker.update_with_detections(_det(100, 100, 150, 150))
    id1 = int(t1.tracker_id[0])
    # 200 empty frames >> lost_track_buffer=90; old track is dropped.
    for _ in range(200):
        tracker.update_with_detections(sv.Detections.empty())
    # Re-acquire the object: ByteTrack 0.27 needs minimum_consecutive_frames
    # confirmations before exposing a tracker_id. We feed enough frames to
    # cover the warm-up window without artificially short tests.
    final = None
    for _ in range(5):
        final = tracker.update_with_detections(_det(110, 105, 160, 155))
        if final.tracker_id is not None and len(final) > 0:
            break
    assert final is not None and len(final) > 0, \
        'ByteTrack failed to re-acquire the object after a long occlusion'
    assert int(final.tracker_id[0]) != id1, \
        f'ID should be recycled after long occlusion, got {final.tracker_id[0]}'


def test_two_objects_get_distinct_ids(tracker):
    det = sv.Detections(
        xyxy=np.array([[10, 10, 30, 30], [500, 500, 520, 520]], dtype=np.float32),
        confidence=np.array([0.9, 0.9], dtype=np.float32),
        class_id=np.array(['a', 'b'], dtype=object),
    )
    t = tracker.update_with_detections(det)
    ids = sorted(int(i) for i in t.tracker_id)
    assert ids == [1, 2], f'Expected ids [1, 2], got {ids}'


def test_two_objects_motion_keeps_distinct_ids(tracker):
    det1 = sv.Detections(
        xyxy=np.array([[10, 10, 30, 30], [500, 500, 520, 520]], dtype=np.float32),
        confidence=np.array([0.9, 0.9], dtype=np.float32),
        class_id=np.array(['a', 'b'], dtype=object),
    )
    t1 = tracker.update_with_detections(det1)
    first_ids = sorted(int(i) for i in t1.tracker_id)
    for i in range(10):
        det = sv.Detections(
            xyxy=np.array(
                [[10 + 2 * i, 10, 30 + 2 * i, 30],
                 [500 + 2 * i, 500, 520 + 2 * i, 520]],
                dtype=np.float32,
            ),
            confidence=np.array([0.9, 0.9], dtype=np.float32),
            class_id=np.array(['a', 'b'], dtype=object),
        )
        t = tracker.update_with_detections(det)
        ids = sorted(int(i) for i in t.tracker_id)
        assert ids == first_ids, f'Frame {i}: IDs diverged {ids} != {first_ids}'


def test_low_confidence_detection_rejected(tracker):
    # Below track_activation_threshold (0.25) => tracker should ignore it.
    t = tracker.update_with_detections(_det(10, 10, 30, 30, score=0.1))
    assert len(t) == 0


def test_empty_input_returns_empty_detections(tracker):
    # Need a tracker that has seen something, to test the empty path.
    tracker.update_with_detections(_det(10, 10, 30, 30))
    t = tracker.update_with_detections(sv.Detections.empty())
    assert len(t) == 0
