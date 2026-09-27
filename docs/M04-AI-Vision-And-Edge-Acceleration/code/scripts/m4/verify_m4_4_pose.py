#!/usr/bin/env python3
"""Wait for a usable Isaac ROS FoundationPose Detection3DArray."""

import argparse
import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from vision_msgs.msg import Detection3DArray


def valid_pose(message):
    if not message.header.frame_id:
        return None
    for detection in message.detections:
        for result in detection.results:
            pose = result.pose.pose
            p = pose.position
            q = pose.orientation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(value) for value in values):
                continue
            norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
            if p.z > 0 and 0.9 <= norm <= 1.1:
                return {
                    'frame_id': message.header.frame_id,
                    'stamp_sec': message.header.stamp.sec,
                    'class_id': result.hypothesis.class_id,
                    'score': result.hypothesis.score,
                    'position_m': [p.x, p.y, p.z],
                    'quaternion_xyzw': [q.x, q.y, q.z, q.w],
                    'quaternion_norm': norm,
                }
    return None


def self_test():
    msg = Detection3DArray()
    msg.header.frame_id = 'camera'
    detection = msg.detections.add() if hasattr(msg.detections, 'add') else None
    if detection is None:
        from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose
        detection = Detection3D()
        detection.results.append(ObjectHypothesisWithPose())
        msg.detections.append(detection)
    pose = detection.results[0].pose.pose
    pose.position.z = 0.5
    assert valid_pose(msg) is not None
    pose.orientation.w = 0.0
    assert valid_pose(msg) is None
    pose.orientation.w = 1.0
    pose.position.x = math.nan
    assert valid_pose(msg) is None
    print('pose validator self-test passed')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/output')
    parser.add_argument('--timeout', type=float, default=240.0)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0

    rclpy.init()
    node = rclpy.create_node('m4_4_pose_verifier')
    accepted = []
    counts = {'messages': 0, 'rejected': 0}

    def callback(message):
        counts['messages'] += 1
        pose = valid_pose(message)
        if pose is None:
            counts['rejected'] += 1
        else:
            accepted.append(pose)

    node.create_subscription(Detection3DArray, args.topic, callback, 10)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and not accepted:
            rclpy.spin_once(node, timeout_sec=0.5)
        if not accepted:
            print(json.dumps({'result': 'timeout', 'topic': args.topic, **counts}))
            return 1
        print(json.dumps({'result': 'valid_pose', 'topic': args.topic, **counts,
                          'pose': accepted[0]}, allow_nan=False))
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
