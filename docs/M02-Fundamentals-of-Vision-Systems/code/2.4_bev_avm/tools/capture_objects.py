#!/usr/bin/env python3
"""Capture /bev/objects during one bag pass and save JSON for GT comparison."""
import json
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from autoware_perception_msgs.msg import DetectedObjects


class Capture(Node):
    def __init__(self, out_path, duration_s):
        super().__init__("objects_capture")
        self.out_path = out_path
        self.records = []
        self.sub = self.create_subscription(
            DetectedObjects,
            "/bev/objects",
            self.cb,
            QoSProfile(depth=5,
                       reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.deadline = self.get_clock().now() + rclpy.duration.Duration(
            seconds=float(duration_s))
        print(f"capturing for {duration_s}s", flush=True)

    def cb(self, m):
        self.records.append({
            "t_ns": m.header.stamp.sec * 10**9 + m.header.stamp.nanosec,
            "frame_id": m.header.frame_id,
            "objects": [
                {
                    "x": o.kinematics.pose_with_covariance.pose.position.x,
                    "y": o.kinematics.pose_with_covariance.pose.position.y,
                    "z": o.kinematics.pose_with_covariance.pose.position.z,
                    "q": [
                        o.kinematics.pose_with_covariance.pose.orientation.x,
                        o.kinematics.pose_with_covariance.pose.orientation.y,
                        o.kinematics.pose_with_covariance.pose.orientation.z,
                        o.kinematics.pose_with_covariance.pose.orientation.w,
                    ],
                    "score": o.existence_probability,
                    "label": o.classification[0].label if o.classification else -1,
                    "l": o.shape.dimensions.x,
                    "w": o.shape.dimensions.y,
                    "h": o.shape.dimensions.z,
                }
                for o in m.objects
            ],
        })

    def spin_until(self):
        while self.get_clock().now() < self.deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        with open(self.out_path, "w") as f:
            json.dump(self.records, f, indent=1)
        print(f"saved {len(self.records)} frames -> {self.out_path}", flush=True)


def main():
    rclpy.init()
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/objects_capture.json"
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0
    cap = Capture(out, dur)
    try:
        cap.spin_until()
    finally:
        cap.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()