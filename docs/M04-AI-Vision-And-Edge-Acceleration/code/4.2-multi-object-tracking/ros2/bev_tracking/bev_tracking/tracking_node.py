"""bev_tracking/tracking_node.py

ROS2 rclpy node that wraps a supervision.ByteTrack tracker.

Subscribes:
    /perception/detections  (vision_msgs/Detection2DArray)
        QoS: BEST_EFFORT, depth=10  (matches bev_detection M4.1 contract)

Publishes:
    /perception/tracks      (vision_msgs/Detection2DArray)
        QoS: same SensorDataQoS as M4.1

The node is intentionally minimal:
    - adapter converts ROS <-> supervision types
    - tracker is single-instance, owned by the node
    - empty detection arrays still call update_with_detections so that
      ByteTrack's lost-track buffer progresses naturally
    - thread/process model = rclpy.spin() default

The tracker choice is fixed at runtime: we always use supervision's
ByteTrack. Ultralytics' BYTETracker was rejected because it pulls in
the entire ultralytics package and conceptually conflates tracking
with detection. Naive re-implementations of Kalman / Hungarian are
out of scope.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from vision_msgs.msg import Detection2DArray

from supervision import ByteTrack

from .adapter import detection_to_tracker, tracker_to_tracks


class TrackingNode(Node):
    def __init__(self) -> None:
        super().__init__('tracking_node')

        # Parameters
        self.declare_parameter('input_topic', '/perception/detections')
        self.declare_parameter('output_topic', '/perception/tracks')

        # ByteTrack parameters (must match config/bytetrack.yaml)
        self.declare_parameter('track_activation_threshold', 0.25)
        self.declare_parameter('lost_track_buffer', 30)
        self.declare_parameter('minimum_matching_threshold', 0.8)
        self.declare_parameter('frame_rate', 10)
        self.declare_parameter('minimum_consecutive_frames', 1)
        self.declare_parameter('publish_log_throttle_ms', 1000)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self._tracker = ByteTrack(
            track_activation_threshold=self.get_parameter('track_activation_threshold').value,
            lost_track_buffer=self.get_parameter('lost_track_buffer').value,
            minimum_matching_threshold=self.get_parameter('minimum_matching_threshold').value,
            frame_rate=self.get_parameter('frame_rate').value,
            minimum_consecutive_frames=self.get_parameter('minimum_consecutive_frames').value,
        )

        # QoS: matches bev_detection (BEST_EFFORT, KEEP_LAST, depth=10)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._sub = self.create_subscription(
            Detection2DArray,
            self.input_topic,
            self._on_detections,
            qos,
        )
        self._pub = self.create_publisher(
            Detection2DArray,
            self.output_topic,
            qos,
        )

        self._log_throttle_us = self.get_parameter('publish_log_throttle_ms').value * 1000
        self._last_log_us = 0
        self._frame_counter = 0
        self._last_latency_ms = 0.0

        self.get_logger().info(
            f'tracking_node ready: sub={self.input_topic} pub={self.output_topic} '
            f'frame_rate={self.get_parameter("frame_rate").value}'
        )

    def _on_detections(self, msg: Detection2DArray) -> None:
        t0 = time.perf_counter()
        detections = detection_to_tracker(msg)
        # CRITICAL: empty input still updates the tracker so that
        # ByteTrack's lost_track_buffer progresses naturally.
        tracks = self._tracker.update_with_detections(detections)
        out = tracker_to_tracks(tracks, msg.header)
        self._pub.publish(out)

        t1 = time.perf_counter()
        self._last_latency_ms = (t1 - t0) * 1000.0
        self._frame_counter += 1

        now_us = self.get_clock().now().nanoseconds // 1000
        if (now_us - self._last_log_us) >= self._log_throttle_us:
            self.get_logger().info(
                f'frame={self._frame_counter} in_dets={len(msg.detections)} '
                f'out_tracks={len(out.detections)} latency_ms={self._last_latency_ms:.2f}'
            )
            self._last_log_us = now_us


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackingNode()
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
