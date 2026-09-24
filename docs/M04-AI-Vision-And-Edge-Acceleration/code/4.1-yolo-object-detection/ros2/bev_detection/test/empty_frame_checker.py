#!/usr/bin/env python3
"""Subscribe to /perception/detections and check the empty-frame contract.

Used by scripts/m4/test_empty_frame_contract.sh.

Contract:
  - yolo_trt_node publishes exactly one Detection2DArray per processed frame
  - For frames with no detections, detections.size() == 0 but the message
    IS still published, with header inherited from the input image.
"""
import sys
import time
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


class EmptyFrameChecker(Node):
    def __init__(self, output_path, timeout_s):
        super().__init__('empty_frame_checker')
        self.output_path = output_path
        self.timeout_s = timeout_s
        self.total = 0
        self.empty = 0
        self.start_ns = None
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        # yolo_trt_node publishes with SensorDataQoS (BEST_EFFORT, depth=10).
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.sub = self.create_subscription(
            Detection2DArray,
            '/perception/detections',
            self.on_msg,
            qos,
        )

    def on_msg(self, msg):
        if self.start_ns is None:
            self.start_ns = self.get_clock().now().nanoseconds
        self.total += 1
        if len(msg.detections) == 0:
            self.empty += 1
        with open(self.output_path, 'a') as f:
            f.write(f'frame={self.total} stamp_ns='
                    f'{msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec} '
                    f'bboxes={len(msg.detections)} frame_id={msg.header.frame_id}\n')

    def spin_until_timeout(self):
        end_time = time.time() + self.timeout_s
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
        with open(self.output_path, 'a') as f:
            f.write(f'__total__ {self.total}\n')
            f.write(f'__empty__ {self.empty}\n')


def main():
    if len(sys.argv) < 3:
        print('usage: empty_frame_checker.py OUTPUT_PATH TIMEOUT_S', file=sys.stderr)
        sys.exit(2)
    output_path = sys.argv[1]
    timeout_s = float(sys.argv[2])

    open(output_path, 'w').close()

    rclpy.init()
    node = EmptyFrameChecker(output_path, timeout_s)
    try:
        node.spin_until_timeout()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # Exit code: 0 if at least one empty-frame message received.
    sys.exit(0 if node.empty >= 1 else 1)


if __name__ == '__main__':
    main()
