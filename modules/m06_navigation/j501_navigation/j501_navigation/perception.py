"""ROS 2 adapter for LiDAR-only ground separation, obstacle detection, and navigation observations."""
import json
import itertools
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .lidar_algorithms import euclidean_clusters, ground_plane, project_scan, voxel_filter


def transform_points(points, transform):
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    t = transform.translation
    return points @ rotation.T + np.array([t.x, t.y, t.z])


class LidarPerception(Node):
    def __init__(self):
        super().__init__('lidar_perception')
        defaults = dict(input_topic='/livox/lidar', base_frame='base_footprint', max_range=10., voxel_size=0.06,
            ground_threshold=0.04, ground_max_tilt_deg=15., ground_iterations=45,
            obstacle_min_height=0.07, obstacle_max_height=1.5, cluster_tolerance=0.18, cluster_min_points=5,
            scan_min_height=0.12, scan_max_height=0.42, scan_bins=720)
        self.p = {name: self.declare_parameter(name, value).value for name, value in defaults.items()}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.obstacles = self.create_publisher(PointCloud2, '/perception/obstacles', qos_profile_sensor_data)
        self.ground = self.create_publisher(PointCloud2, '/perception/ground', qos_profile_sensor_data)
        self.markers = self.create_publisher(MarkerArray, '/perception/boxes', 2)
        self.detections = self.create_publisher(String, '/perception/detections', 2)
        self.create_subscription(PointCloud2, self.p['input_topic'], self.process,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.last_processed = -math.inf
        self.last_warning = 0.0

    def process(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds-self.last_processed < 180_000_000:
            return  # Process at most ~5 Hz; retain depth-one sensor input behavior.
        try:
            transform = self.buffer.lookup_transform(self.p['base_frame'], msg.header.frame_id, stamp)
        except Exception as error:
            if time.monotonic()-self.last_warning > 5:
                self.get_logger().warn(f'Waiting for sensor TF at cloud timestamp: {error}')
                self.last_warning = time.monotonic()
            return
        self.last_processed = stamp.nanoseconds
        beginning = time.perf_counter()
        raw = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        raw = np.array([[p[0], p[1], p[2]] for p in raw], dtype=np.float64).reshape(-1, 3)
        raw = raw[np.isfinite(raw).all(axis=1)]
        points = transform_points(raw, transform.transform)
        radial = np.linalg.norm(points[:, :2], axis=1)
        valid = np.isfinite(points).all(axis=1) & (radial >= 0.35) & (radial <= self.p['max_range'])
        valid &= (points[:, 2] >= -0.15) & (points[:, 2] <= self.p['obstacle_max_height'])
        points = points[valid]
        header = Header(stamp=msg.header.stamp, frame_id=self.p['base_frame'])
        scan = LaserScan(header=header, angle_min=-math.pi, angle_max=math.pi-2*math.pi/self.p['scan_bins'],
            angle_increment=2*math.pi/self.p['scan_bins'], range_min=0.35, range_max=self.p['max_range'], scan_time=0.2)
        scan.ranges = project_scan(points, self.p['scan_bins'], self.p['scan_min_height'],
                                   self.p['scan_max_height'], max_range=self.p['max_range']).tolist()
        self.scan.publish(scan)
        reduced = voxel_filter(points, self.p['voxel_size'])
        plane, distances, fitted = ground_plane(reduced, self.p['ground_threshold'],
            self.p['ground_max_tilt_deg'], self.p['ground_iterations'])
        ground = reduced[np.abs(distances) <= self.p['ground_threshold']]
        obstacles = reduced[(distances >= self.p['obstacle_min_height']) &
                            (reduced[:, 2] <= self.p['obstacle_max_height'])]
        self.ground.publish(point_cloud2.create_cloud_xyz32(header, ground.tolist()))
        # All non-ground points feed collision checking; cluster-size thresholds only affect detections.
        self.obstacles.publish(point_cloud2.create_cloud_xyz32(header, obstacles.tolist()))
        clusters = euclidean_clusters(obstacles, self.p['cluster_tolerance'], self.p['cluster_min_points'])
        boxes, markers = [], [Marker(action=Marker.DELETEALL)]
        for i, cluster in enumerate(clusters):
            cloud = obstacles[cluster]
            low, high = cloud.min(axis=0), cloud.max(axis=0)
            center, size = (low+high)/2, np.maximum(high-low, 0.05)
            boxes.append(dict(center=center.tolist(), size=size.tolist(), points=len(cluster)))
            marker = Marker(header=header, ns='lidar_obstacles', id=i, type=Marker.LINE_LIST, action=Marker.ADD)
            marker.pose.orientation.w = 1.
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, center)
            corners = list(itertools.product([-1, 1], repeat=3))
            for a, b in itertools.combinations(corners, 2):
                if sum(x != y for x, y in zip(a, b)) == 1:
                    for corner in [a, b]:
                        xyz = np.array(corner)*size/2
                        marker.points.append(Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])))
            marker.scale.x = 0.015
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., 0.35, 0.05, 0.85
            marker.lifetime = Duration(seconds=0.6).to_msg()
            markers.append(marker)
        self.markers.publish(MarkerArray(markers=markers))
        report = dict(frame_id=header.frame_id, stamp_sec=stamp.nanoseconds/1e9,
            input_points=len(raw), voxel_points=len(reduced), ground_points=len(ground),
            obstacle_points=len(obstacles), ground_plane=plane.tolist(), ground_fitted=fitted,
            boxes=boxes, processing_ms=(time.perf_counter()-beginning)*1000)
        self.detections.publish(String(data=json.dumps(report)))


def main():
    rclpy.init()
    node = LidarPerception()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
