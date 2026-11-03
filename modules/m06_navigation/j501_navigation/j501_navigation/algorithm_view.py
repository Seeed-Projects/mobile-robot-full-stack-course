"""Persistent RViz overlays and plots of real localization / LiDAR detection output.

No scene geometry or simulator ground truth is consumed by this visualizer.
Cluster numbers identify one frame only; they are not tracking identities.
"""
import colorsys
import itertools
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .perception import transform_points


def cluster_color(index):
    return colorsys.hsv_to_rgb((0.06+index*0.61803398875) % 1., .8, 1.)


def box_corners(box):
    return np.array(box['center']) + np.array(list(itertools.product([-1, 1], repeat=3)))*np.array(box['size'])/2


def compact(box):
    """Show compact clusters clearly; keep raw wall clusters available in the original topic."""
    sx, sy, sz = box['size']
    return .10 <= sx <= 1.2 and .10 <= sy <= 1.2 and sz >= .15


class AlgorithmView(Node):
    def __init__(self):
        super().__init__('algorithm_view')
        self.output = Path(self.declare_parameter('output', 'build/m06_results/algorithm_view').value).expanduser()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.ready, self.grid = False, None
        self.planner, self.controller = 'Nav2', 'Nav2'
        self.trajectory, self.reference, self.latest_plan = [], [], []
        self.snapshot, self.snapshot_score = None, -math.inf
        self.cloud, self.last_cloud_stamp = [], -math.inf
        self.cloud_frame = None
        self.path_frame = 'map'
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.trace_pub = self.create_publisher(RosPath, '/lab/estimated_path', durable)
        self.reference_pub = self.create_publisher(RosPath, '/lab/reference_path', durable)
        self.markers_pub = self.create_publisher(MarkerArray, '/lab/algorithm_markers', 2)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, durable)
        self.create_subscription(Bool, '/localization_ready', self.localization_callback, durable)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.reset_localization, 10)
        self.create_subscription(RosPath, '/plan', self.plan_callback, 10)
        self.create_subscription(String, '/planner_selector', lambda m: setattr(self, 'planner', m.data), durable)
        self.create_subscription(String, '/controller_selector', lambda m: setattr(self, 'controller', m.data), durable)
        self.create_subscription(PointCloud2, '/perception/obstacles', self.cloud_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String, '/perception/detections', self.detection_callback, 2)
        self.create_service(Trigger, '/algorithm_view/save', self.save)
        self.create_timer(.2, self.record_pose)
        self.get_logger().info('Algorithm view ready. Save PNG + JSON with: '
            'ros2 service call /algorithm_view/save std_srvs/srv/Trigger "{}"')

    def localization_callback(self, msg):
        # A stationary AMCL does not keep emitting new estimates. Retain the
        # established frame for display; this flag never controls navigation.
        if msg.data:
            self.ready = True

    def reset_localization(self, msg):
        self.ready = False
        self.trajectory, self.reference = [], []
        self.snapshot, self.snapshot_score = None, -math.inf

    def map_callback(self, msg):
        q = msg.info.origin.orientation
        angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.grid = dict(width=msg.info.width, height=msg.info.height, resolution=msg.info.resolution,
            origin=[msg.info.origin.position.x, msg.info.origin.position.y, angle], data=list(msg.data))

    def plan_callback(self, msg):
        if len(msg.poses) < 2 or msg.header.frame_id != 'map':
            return
        self.latest_plan = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        # A changed goal begins a new demonstration, with a new reference and trace.
        if not self.reference or math.dist(self.reference[-1], self.latest_plan[-1]) > .4:
            self.reference = self.latest_plan.copy()
            self.trajectory = self.trajectory[-1:]
            self.snapshot, self.snapshot_score = None, -math.inf
        self.publish_paths()

    def record_pose(self):
        if not self.ready:
            return
        try:
            t = self.buffer.lookup_transform('map', 'base_footprint', Time()).transform
        except Exception:
            return
        q = t.rotation
        point = [t.translation.x, t.translation.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
        if not self.trajectory or math.dist(point[:2], self.trajectory[-1][:2]) >= .025:
            self.trajectory.append(point)
            self.trajectory = self.trajectory[-6000:]
        self.publish_paths()

    def path_message(self, points):
        path = RosPath(header=Header(frame_id='map', stamp=self.get_clock().now().to_msg()))
        for point in points:
            p = PoseStamped(header=path.header)
            p.pose.position.x, p.pose.position.y = map(float, point[:2])
            p.pose.orientation.w = 1.
            path.poses.append(p)
        return path

    def publish_paths(self):
        self.trace_pub.publish(self.path_message(self.trajectory))
        self.reference_pub.publish(self.path_message(self.reference))

    def cloud_callback(self, msg):
        points = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        self.cloud = np.array([[p[0], p[1], p[2]] for p in points], dtype=float).reshape(-1, 3)[::2]
        self.last_cloud_stamp = Time.from_msg(msg.header.stamp).nanoseconds/1e9
        self.cloud_frame = msg.header.frame_id

    def detection_callback(self, msg):
        if not self.ready:
            return
        report = json.loads(msg.data)
        stamp = Time(seconds=report['stamp_sec'])
        try:
            t = self.buffer.lookup_transform('map', report['frame_id'], stamp).transform
        except Exception:
            return
        boxes = []
        for i, box in enumerate(report['boxes']):
            corners = transform_points(box_corners(box), t)
            center = transform_points(np.array([box['center']]), t)[0]
            boxes.append(dict(id=i, center=center.tolist(), corners=corners.tolist(),
                size=box['size'], points=box['points'], compact=compact(box)))
        markers = [Marker(action=Marker.DELETEALL)]
        header = Header(frame_id='map', stamp=stamp.to_msg())
        for box in boxes:
            if not box['compact']:
                continue
            color = cluster_color(box['id'])
            marker = Marker(header=header, ns='compact_clusters', id=box['id'], type=Marker.LINE_LIST, action=Marker.ADD)
            marker.pose.orientation.w = 1.
            corners = box['corners']
            for a, b in itertools.combinations(range(8), 2):
                if bin(a ^ b).count('1') == 1:
                    marker.points.extend([Point(x=corners[j][0], y=corners[j][1], z=corners[j][2]) for j in [a, b]])
            marker.scale.x = .025
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 1.
            marker.lifetime = Duration(seconds=.8).to_msg()
            markers.append(marker)
            label = Marker(header=header, ns='cluster_labels', id=box['id'], type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            label.pose.orientation.w = 1.
            label.pose.position.x, label.pose.position.y = box['center'][:2]
            label.pose.position.z = max(p[2] for p in corners)+.18
            label.scale.z = .16
            label.color.r, label.color.g, label.color.b, label.color.a = *color, 1.
            label.text = f"C{box['id']} | {box['points']} pts"
            label.lifetime = Duration(seconds=.8).to_msg()
            markers.append(label)
        if self.reference:
            goal = Marker(header=header, ns='navigation_goal', id=0, type=Marker.CYLINDER, action=Marker.ADD)
            goal.pose.orientation.w = 1.
            goal.pose.position.x, goal.pose.position.y = self.reference[-1]
            goal.scale.x, goal.scale.y, goal.scale.z = .18, .18, .06
            goal.color.r, goal.color.g, goal.color.b, goal.color.a = 1., .15, .2, 1.
            markers.append(goal)
        self.markers_pub.publish(MarkerArray(markers=markers))
        if len(self.trajectory) < 5 or not self.reference:
            return
        robot = transform_points(np.zeros((1, 3)), t)[0]
        candidates = [b for b in boxes if b['compact'] and b['points'] >= 10
                      and .6 <= math.dist(b['center'][:2], robot[:2]) <= 2.2]
        if not candidates:
            return
        reference = np.array(self.reference)
        def score(box):
            route_distance = float(np.linalg.norm(reference-np.array(box['center'][:2]), axis=1).min())
            return (min(box['points'], 160)/160 - route_distance*3
                    - abs(math.dist(box['center'][:2], robot[:2])-1.15))
        selected = max(candidates, key=score)
        value = score(selected)
        if value < self.snapshot_score:
            return
        points = []
        if (abs(self.last_cloud_stamp-report['stamp_sec']) < .25 and len(self.cloud)
                and self.cloud_frame == report['frame_id']):
            points = transform_points(self.cloud, t).tolist()
        self.snapshot_score = value
        q = t.rotation
        robot_pose = [float(robot[0]), float(robot[1]), math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
        self.snapshot = dict(stamp_sec=report['stamp_sec'], robot_pose=robot_pose,
            selected_cluster=selected['id'], boxes=boxes, obstacle_points=points,
            processing_ms=report['processing_ms'], ground_fitted=report['ground_fitted'])

    def save(self, request, response):
        if self.grid is None or not self.reference or not self.trajectory or self.snapshot is None:
            response.success = False
            response.message = 'Need a map, planned route, localization trajectory, and a nearby compact LiDAR cluster. Navigate past an obstacle first.'
            return response
        output = self.output.resolve()
        report = dict(map=self.grid, reference_path=self.reference, latest_plan=self.latest_plan,
            planner=self.planner, controller=self.controller,
            estimated_trajectory=self.trajectory, detection_snapshot=self.snapshot,
            capture_stamp_sec=self.get_clock().now().nanoseconds/1e9,
            trajectory_source='AMCL + wheel/IMU EKF map -> base_footprint TF; estimated, not ground truth',
            detection_source='Live LiDAR ground separation and Euclidean clusters; no scene geometry',
            note='Trajectory covers the recorded run; points and boxes are one timestamped detection frame. Cluster IDs are frame-local, not tracked objects.')
        try:
            from .plot_algorithm_view import render
            output.parent.mkdir(parents=True, exist_ok=True)
            output.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
            render(report, output.with_suffix('.png'))
        except Exception as error:
            response.success = False
            response.message = f'Could not export visualization: {error}'
            self.get_logger().error(response.message)
            return response
        response.success = True
        response.message = f"Saved {output.with_suffix('.png')} and {output.with_suffix('.json')}"
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = AlgorithmView()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
