#!/usr/bin/env python3
"""Subscribe to /perception/detections and write received messages to a file.

Used by scripts/m4/test_yolo_node.sh to validate the YOLO node end-to-end
without interactive ros2 topic echo.
"""
import sys
import time
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


class DetectionCounter(Node):
    def __init__(self, output_path, timeout_s):
        super().__init__('detection_counter')
        self.output_path = output_path
        self.timeout_s = timeout_s
        self.count = 0
        self.start_ns = None
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        # yolo_trt_node uses SensorDataQoS() (BEST_EFFORT) on the publisher.
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
        self.count += 1
        with open(self.output_path, 'a') as f:
            f.write(f'frame={self.count} stamp_ns={msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec} '
                    f'bboxes={len(msg.detections)}\n')

    def spin_until_timeout(self):
        end_time = time.time() + self.timeout_s
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
        # Write final summary
        with open(self.output_path, 'a') as f:
            f.write(f'__total__ {self.count}\n')


def main():
    if len(sys.argv) < 3:
        print('usage: count_detections.py OUTPUT_PATH TIMEOUT_S', file=sys.stderr)
        sys.exit(2)
    output_path = sys.argv[1]
    timeout_s = float(sys.argv[2])

    # Truncate output file
    open(output_path, 'w').close()

    rclpy.init()
    node = DetectionCounter(output_path, timeout_s)
    try:
        node.spin_until_timeout()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # Print final count on stdout
    print(node.count)
    sys.exit(0)


if __name__ == '__main__':
    main()
