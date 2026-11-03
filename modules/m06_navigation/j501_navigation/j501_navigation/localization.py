"""Check AMCL against live laser/map agreement before activating Nav2 navigation."""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener


def scan_agreement(grid, transform, scan, tolerance=0.20):
    """Fraction of finite scan endpoints near occupied map cells, independent of simulator truth."""
    angles = scan.angle_min+np.arange(len(scan.ranges))*scan.angle_increment
    ranges = np.array(scan.ranges)
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges < scan.range_max)
    angles, ranges = angles[valid][::3], ranges[valid][::3]
    if len(ranges) < 20:
        return 0.0
    q = transform.rotation
    heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    x = transform.translation.x + ranges*np.cos(angles+heading)
    y = transform.translation.y + ranges*np.sin(angles+heading)
    origin = grid.info.origin
    q = origin.orientation
    origin_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    dx, dy = x-origin.position.x, y-origin.position.y
    ix = np.floor((np.cos(origin_yaw)*dx+np.sin(origin_yaw)*dy)/grid.info.resolution).astype(int)
    iy = np.floor((-np.sin(origin_yaw)*dx+np.cos(origin_yaw)*dy)/grid.info.resolution).astype(int)
    pixels = np.array(grid.data).reshape(grid.info.height, grid.info.width)
    matched = np.zeros(len(ix), dtype=bool)
    radius = math.ceil(tolerance/grid.info.resolution)
    for ox in range(-radius, radius+1):
        for oy in range(-radius, radius+1):
            if math.hypot(ox, oy)*grid.info.resolution > tolerance:
                continue
            xx, yy = ix+ox, iy+oy
            inside = (xx >= 0) & (yy >= 0) & (xx < grid.info.width) & (yy < grid.info.height)
            matched[inside] |= pixels[yy[inside], xx[inside]] >= 65
    return float(matched.mean())


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')
        self.start_navigation = self.declare_parameter('start_navigation', True).value
        metadata_file = self.declare_parameter('metadata_file', '').value
        demo = self.declare_parameter('demo_map', False).value
        self.metadata = json.loads(Path(metadata_file).read_text()) if metadata_file and Path(metadata_file).is_file() else {}
        if demo:
            self.metadata = dict(world_from_map=[0., 0., 0.], evaluation_only=True, source='Prebuilt map benchmark')
        self.grid, self.scan, self.pose = None, None, None
        self.ready, self.pending, self.started = False, None, False
        self.stable, self.previous_pose_stamp = 0, None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(Bool, '/localization_ready', durable)
        self.metadata_publisher = self.create_publisher(String, '/lab/map_metadata', durable)
        self.metrics = self.create_publisher(String, '/localization/metrics', 5)
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'grid', m), durable)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: setattr(self, 'pose', m), durable)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.reset_estimate, 10)
        self.manager = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        self.create_timer(0.25, self.check)

    def reset_estimate(self, msg):
        self.pose, self.previous_pose_stamp = None, None
        self.ready, self.stable = False, 0
        self.status.publish(Bool(data=False))

    def check(self):
        self.metadata_publisher.publish(String(data=json.dumps(self.metadata)))
        if self.pending:
            if self.pending.done():
                result = self.pending.result()
                self.started = bool(result and result.success)
                self.pending = None
                if self.started:
                    self.get_logger().info('Localization verified; Nav2 navigation activated.')
            return
        if self.grid is None or self.scan is None or self.pose is None:
            self.status.publish(Bool(data=False))
            return
        try:
            transform = self.buffer.lookup_transform('map', self.scan.header.frame_id,
                                                     Time.from_msg(self.scan.header.stamp)).transform
        except Exception:
            return
        cov = self.pose.pose.covariance
        position_std = math.sqrt(max(cov[0], cov[7], 0))
        yaw_std = math.sqrt(max(cov[35], 0))
        age = (self.get_clock().now()-Time.from_msg(self.scan.header.stamp)).nanoseconds/1e9
        pose_age = (self.get_clock().now()-Time.from_msg(self.pose.header.stamp)).nanoseconds/1e9
        agreement = scan_agreement(self.grid, transform, self.scan)
        acceptable = position_std < 0.20 and yaw_std < 0.30 and agreement >= 0.55 and 0 <= age < 2 and 0 <= pose_age < 5
        stamp = Time.from_msg(self.pose.header.stamp).nanoseconds
        if stamp != self.previous_pose_stamp:
            self.stable = self.stable+1 if acceptable else 0
            self.previous_pose_stamp = stamp
        if not acceptable:
            self.stable = 0
        self.ready = self.stable >= 5
        self.metrics.publish(String(data=json.dumps(dict(position_std_m=position_std, yaw_std_rad=yaw_std,
            scan_map_agreement=agreement, stable_amcl_updates=self.stable, ready=self.ready,
            amcl_stamp_sec=stamp/1e9))))
        self.status.publish(Bool(data=self.ready))
        if self.ready and not self.start_navigation:
            self.started = True
        elif self.ready and not self.started and self.manager.service_is_ready():
            self.get_logger().info('AMCL passed covariance and scan-map checks; starting Nav2 lifecycle nodes.')
            self.pending = self.manager.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP))


def monitor_main():
    rclpy.init()
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def relocalize_main():
    parser = argparse.ArgumentParser(description='Seed AMCL and rotate slowly until live scan/map localization passes.')
    parser.add_argument('--pose', nargs=3, type=float, default=[0., 0., 0.], help='Approximate pose in the SAVED map frame')
    parser.add_argument('--global', dest='global_search', action='store_true', help='Use AMCL global particle initialization instead of a seed')
    parser.add_argument('--timeout', type=float, default=70)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Node('relocalize', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    ready, metrics = [False], [{}]
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Bool, '/localization_ready', lambda m: ready.__setitem__(0, m.data), durable)
    node.create_subscription(String, '/localization/metrics', lambda m: metrics.__setitem__(0, json.loads(m.data)), 5)
    initial = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    velocity = node.create_publisher(Twist, '/cmd_vel', 10)
    beginning = time.monotonic()
    try:
        amcl = node.create_client(GetState, '/amcl/get_state')
        deadline = time.monotonic()+30
        while True:
            if time.monotonic() > deadline:
                raise TimeoutError('AMCL did not become active')
            if not amcl.wait_for_service(timeout_sec=0.5):
                rclpy.spin_once(node, timeout_sec=0.05)
                continue
            future = amcl.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2)
            if future.done() and future.result().current_state.id == 3:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        seed_time = node.get_clock().now().nanoseconds/1e9
        ready[0], metrics[0] = False, {}
        if args.global_search:
            client = node.create_client(Empty, '/reinitialize_global_localization')
            if not client.wait_for_service(timeout_sec=20):
                raise TimeoutError('AMCL global localization service unavailable')
            future = client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done() or future.exception():
                raise RuntimeError('Global localization request failed')
        else:
            deadline = time.monotonic()+20
            while initial.get_subscription_count() == 0:
                if time.monotonic() > deadline:
                    raise TimeoutError('AMCL is not listening to /initialpose')
                rclpy.spin_once(node, timeout_sec=0.1)
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose.position.x, msg.pose.pose.position.y = args.pose[:2]
            msg.pose.pose.orientation.z = math.sin(args.pose[2]/2)
            msg.pose.pose.orientation.w = math.cos(args.pose[2]/2)
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25**2
            msg.pose.covariance[35] = math.radians(15)**2
            initial.publish(msg)
        while not (ready[0] and metrics[0].get('ready', False)
                   and metrics[0].get('stable_amcl_updates', 0) >= 5
                   and metrics[0].get('amcl_stamp_sec', -1) >= seed_time):
            if time.monotonic()-beginning > args.timeout:
                raise TimeoutError(f'Localization did not converge: {metrics[0]}')
            command = Twist(); command.angular.z = 0.25
            velocity.publish(command)
            rclpy.spin_once(node, timeout_sec=0.1)
        report = dict(succeeded=True, seed=None if args.global_search else args.pose,
            wall_time_s=time.monotonic()-beginning, **metrics[0])
        print(json.dumps(report, indent=2))
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2)+'\n')
    finally:
        for _ in range(5):
            velocity.publish(Twist()); rclpy.spin_once(node, timeout_sec=0.03)
        node.destroy_node()
        rclpy.shutdown()
