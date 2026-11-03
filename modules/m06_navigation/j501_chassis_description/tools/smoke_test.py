#!/usr/bin/env python3
"""Exercise a running simulation; send only simulated ROS velocity commands."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from tf2_msgs.msg import TFMessage


class SmokeTest(Node):
    def __init__(self):
        super().__init__('j501_chassis_smoke_test')
        self.latest = {}
        self.counts = {}
        self.seen_ground_truth_frames = set()
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        for msg_type, topic in [(Odometry, '/odom'), (JointState, '/joint_states'),
                                (LaserScan, '/scan'), (Clock, '/clock'), (TFMessage, '/simulation/ground_truth')]:
            self.create_subscription(msg_type, topic, lambda msg, name=topic: self.receive(name, msg), qos_profile_sensor_data)

    def receive(self, topic, msg):
        self.latest[topic] = msg
        self.counts[topic] = self.counts.get(topic, 0) + 1
        if topic == '/simulation/ground_truth':
            for transform in msg.transforms:
                self.seen_ground_truth_frames.add(transform.child_frame_id)
                if transform.child_frame_id == 'j501_chassis':
                    self.latest['actual'] = transform.transform

    def sim_time(self):
        stamp = self.latest['/clock'].clock
        return stamp.sec + stamp.nanosec * 1e-9

    def command(self, v, w, duration):
        start = self.sim_time()
        deadline = time.monotonic() + max(30, duration * 10)
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        while self.sim_time() - start < duration:
            if time.monotonic() > deadline:
                raise RuntimeError('Simulation time stopped or is too slow')
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def run(node):
    deadline = time.monotonic() + 45
    required = ['/odom', '/joint_states', '/scan', '/clock', 'actual']
    while not all(key in node.latest for key in required):
        if time.monotonic() > deadline:
            raise RuntimeError(f'Missing inputs: {[key for key in required if key not in node.latest]}; ground truth frames: {sorted(node.seen_ground_truth_frames)}')
        rclpy.spin_once(node, timeout_sec=0.1)
    node.command(0.0, 0.0, 1.0)
    initial = node.latest['actual']
    node.command(0.1, 0.0, 3.0)
    node.command(0.0, 0.0, 1.0)
    after_forward = node.latest['actual']
    odom_before_turn = node.latest['/odom'].pose.pose.orientation
    distance = math.hypot(after_forward.translation.x-initial.translation.x, after_forward.translation.y-initial.translation.y)
    # Allow passive casters to reorient after straight travel before judging the turn.
    node.command(0.0, 0.3, 3.0)
    node.command(0.0, 0.0, 1.0)
    final = node.latest['actual']
    turn = math.atan2(math.sin(yaw(final.rotation)-yaw(after_forward.rotation)), math.cos(yaw(final.rotation)-yaw(after_forward.rotation)))
    tilt = math.acos(max(-1, min(1, 1-2*(final.rotation.x**2+final.rotation.y**2))))
    scan = node.latest['/scan']
    hits = [d for d in scan.ranges if math.isfinite(d) and scan.range_min <= d <= scan.range_max]
    odom = node.latest['/odom']
    odom_turn = math.atan2(math.sin(yaw(odom.pose.pose.orientation)-yaw(odom_before_turn)),
                          math.cos(yaw(odom.pose.pose.orientation)-yaw(odom_before_turn)))
    checks = {
        'forward_ground_truth_over_0_1m': distance > 0.1,
        'positive_ground_truth_turn_over_0_2rad': turn > 0.2,
        'body_tilt_under_0_1rad': tilt < 0.1,
        'ground_pose_height_under_0_02m': abs(final.translation.z) < 0.02,
        'stopped_linear_speed_under_0_02m_s': abs(odom.twist.twist.linear.x) < 0.02,
        'stopped_angular_speed_under_0_02rad_s': abs(odom.twist.twist.angular.z) < 0.02,
        'scan_has_obstacle_hits': len(hits) > 0,
        'scan_frame_is_laser_link': scan.header.frame_id == 'laser_link',
        'odometry_frames': odom.header.frame_id == 'odom' and odom.child_frame_id == 'base_footprint',
        'drive_joints_present': {'left_wheel_joint', 'right_wheel_joint'} <= set(node.latest['/joint_states'].name),
    }
    return {'passed': all(checks.values()), 'checks': checks, 'forward_distance_m': distance,
            'turn_rad': turn, 'encoder_odom_turn_rad': odom_turn,
            'commands': {'forward_m_s': 0.1, 'forward_duration_s': 3.0, 'turn_rad_s': 0.3, 'turn_duration_s': 3.0},
            'final_tilt_rad': tilt, 'final_model_z_m': final.translation.z,
            'finite_scan_hits': len(hits), 'scan_frame': scan.header.frame_id, 'scan_beams': len(scan.ranges),
            'topic_counts': node.counts, 'simulation_time_s': node.sim_time()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = SmokeTest()
    try:
        report = run(node)
        print(json.dumps(report, indent=2))
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2) + '\n')
        if not report['passed']:
            raise SystemExit(1)
    finally:
        for _ in range(5):
            node.pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.03)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
