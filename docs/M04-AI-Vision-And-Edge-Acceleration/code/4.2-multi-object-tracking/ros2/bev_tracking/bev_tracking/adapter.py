"""Adapter: ROS Detection2DArray <-> supervision.Detections (ByteTrack).

This module is the ONLY place that touches both ROS message types and the
supervision library. It must remain thin: no tracker state, no per-frame
logic. The two conversions are:

    detection_to_tracker(msg) -> sv.Detections
    tracker_to_tracks(tracks, header) -> Detection2DArray

supervision.ByteTrack uses xyxy coordinates; vision_msgs::BoundingBox2D
uses (center, size). The adapter does the affine conversion in both
directions. Header (stamp + frame_id) is preserved verbatim on output.
"""

from __future__ import annotations

import numpy as np
import supervision as sv

from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from std_msgs.msg import Header


def detection_to_tracker(msg: Detection2DArray) -> sv.Detections:
    """Convert a ROS Detection2DArray into a supervision.Detections.

    Returns an empty Detections for empty input. class_id is forwarded as
    a string (supervision accepts str or int); tracking_node.py is
    responsible for any numeric mapping needed downstream.

    Note: supervision.Detections uses float32 xyxy / confidence arrays
    with shape (N, 4) and (N,). We construct them once and trust the
    downstream tracker to copy what it needs.
    """
    n = len(msg.detections)
    if n == 0:
        return sv.Detections.empty()

    xyxy = np.empty((n, 4), dtype=np.float32)
    confidence = np.empty(n, dtype=np.float32)
    class_id = np.empty(n, dtype=object)

    for i, det in enumerate(msg.detections):
        cx = det.bbox.center.position.x
        cy = det.bbox.center.position.y
        sx = det.bbox.size_x
        sy = det.bbox.size_y
        xyxy[i, 0] = cx - sx * 0.5
        xyxy[i, 1] = cy - sy * 0.5
        xyxy[i, 2] = cx + sx * 0.5
        xyxy[i, 3] = cy + sy * 0.5

        if det.results:
            confidence[i] = float(det.results[0].hypothesis.score)
            class_id[i] = det.results[0].hypothesis.class_id
        else:
            confidence[i] = 0.0
            class_id[i] = ""

    return sv.Detections(
        xyxy=xyxy,
        confidence=confidence,
        class_id=class_id,
    )


def tracker_to_tracks(
    tracks: sv.Detections,
    header: Header,
) -> Detection2DArray:
    """Convert a supervision.Detections (post-tracker) to a Detection2DArray.

    Each track is emitted as one Detection2D. The tracker_id is written to
    the standard `id` field of vision_msgs::Detection2D (Humble schema).
    The output header is the input header verbatim; we never synthesise
    timestamps here because downstream consumers (FoundationPose, depth,
    fusion) rely on upstream image-time.
    """
    out = Detection2DArray()
    out.header = header

    if len(tracks) == 0:
        return out

    tracker_ids = tracks.tracker_id
    xyxy = tracks.xyxy
    confidence = tracks.confidence
    class_id = tracks.class_id

    for i in range(len(tracks)):
        det = Detection2D()
        det.header = header

        x1, y1, x2, y2 = xyxy[i]
        det.bbox.center.position.x = float((x1 + x2) * 0.5)
        det.bbox.center.position.y = float((y1 + y2) * 0.5)
        det.bbox.size_x = float(x2 - x1)
        det.bbox.size_y = float(y2 - y1)
        det.bbox.center.theta = 0.0

        hyp = ObjectHypothesisWithPose()
        if class_id is not None and i < len(class_id):
            hyp.hypothesis.class_id = str(class_id[i])
        if confidence is not None and i < len(confidence):
            hyp.hypothesis.score = float(confidence[i])
        det.results.append(hyp)

        if tracker_ids is not None and i < len(tracker_ids) and tracker_ids[i] is not None:
            det.id = str(int(tracker_ids[i]))

        out.detections.append(det)

    return out
