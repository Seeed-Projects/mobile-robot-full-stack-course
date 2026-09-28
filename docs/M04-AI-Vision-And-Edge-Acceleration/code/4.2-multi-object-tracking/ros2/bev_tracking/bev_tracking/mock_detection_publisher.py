"""Mock Detection2DArray publisher for standalone bev_tracking development.

This node lets M4.2 be developed and tested without bringing up:
    - bev_detection (YOLO + TensorRT)
    - Camera / sensor drivers

It publishes synthetic Detection2DArray messages on /perception/detections
using the same QoS as bev_detection (BEST_EFFORT, KEEP_LAST, depth=10).

The default scene simulates one "person" moving horizontally across frames
with periodic occlusion. Trajectory is deterministic so that ByteTrack
behaviour can be inspected by id alone (no random seeding needed).

Usage:
    ros2 run bev_tracking mock_detection_publisher
    ros2 run bev_tracking mock_detection_publisher --ros-args \
        -p rate_hz:=10 -p num_objects:=2 -p occlude_after_frame:=3
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Header
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


class MockDetectionPublisher(Node):
    def __init__(self) -> None:
        super().__init__('mock_detection_publisher')

        self.declare_parameter('output_topic', '/perception/detections')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('num_objects', 1)
        self.declare_parameter('frame_id', 'front_camera')
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('start_x', 100.0)
        self.declare_parameter('step_x', 8.0)
        self.declare_parameter('bbox_size', 80.0)
        self.declare_parameter('class_id', 'person')
        self.declare_parameter('confidence', 0.9)
        self.declare_parameter('occlude_after_frame', -1)
        self.declare_parameter('occlude_duration_frames', 3)

        self._topic = self.get_parameter('output_topic').value
        self._rate_hz = float(self.get_parameter('rate_hz').value)
        self._num_objects = max(0, int(self.get_parameter('num_objects').value))
        self._frame_id = self.get_parameter('frame_id').value
        self._img_w = int(self.get_parameter('image_width').value)
        self._img_h = int(self.get_parameter('image_height').value)
        self._start_x = float(self.get_parameter('start_x').value)
        self._step_x = float(self.get_parameter('step_x').value)
        self._bbox_size = float(self.get_parameter('bbox_size').value)
        self._class_id = self.get_parameter('class_id').value
        self._confidence = float(self.get_parameter('confidence').value)
        self._occlude_after = int(self.get_parameter('occlude_after_frame').value)
        self._occlude_duration = int(self.get_parameter('occlude_duration_frames').value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(Detection2DArray, self._topic, qos)

        period = 1.0 / max(self._rate_hz, 0.1)
        self._timer = self.create_timer(period, self._tick)
        self._frame_idx = 0
        self.get_logger().info(
            f'mock publishing on {self._topic} at {self._rate_hz} Hz, '
            f'{self._num_objects} object(s)'
        )

    def _in_occlusion(self) -> bool:
        if self._occlude_after < 0 or self._occlude_duration <= 0:
            return False
        return (
            self._frame_idx >= self._occlude_after
            and self._frame_idx < self._occlude_after + self._occlude_duration
        )

    def _build_detection(self, cx: float, cy: float) -> Detection2D:
        det = Detection2D()
        det.bbox = BoundingBox2D()
        det.bbox.center.position.x = cx
        det.bbox.center.position.y = cy
        det.bbox.center.theta = 0.0
        det.bbox.size_x = self._bbox_size
        det.bbox.size_y = self._bbox_size
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = self._class_id
        hyp.hypothesis.score = self._confidence
        det.results.append(hyp)
        return det

    def _tick(self) -> None:
        msg = Detection2DArray()
        msg.header = Header()
        msg.header.frame_id = self._frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        if not self._in_occlusion() and self._num_objects > 0:
            # Spread the objects across the image so they remain distinguishable.
            for k in range(self._num_objects):
                cx = self._start_x + self._step_x * self._frame_idx + 200.0 * k
                # Wrap around horizontally so it never escapes the image.
                if cx > self._img_w - self._bbox_size:
                    cx = self._start_x + 200.0 * k
                cy = self._img_h * 0.5 + 50.0 * k
                msg.detections.append(self._build_detection(cx, cy))

        self._pub.publish(msg)
        self._frame_idx += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockDetectionPublisher()
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
