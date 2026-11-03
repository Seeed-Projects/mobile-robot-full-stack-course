"""Executable Nav2 global-planning, local-control, and combined navigation clients."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
import yaml

from .geometry import clearance

PLANNERS = {'astar': 'AStar', 'dijkstra': 'Dijkstra', 'theta_star': 'ThetaStar'}
CONTROLLERS = ['DWB', 'MPPI', 'RPP']


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def pose(node, xytheta):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x, msg.pose.position.y = map(float, xytheta[:2])
    angle = xytheta[2] if len(xytheta) > 2 else 0.0
    msg.pose.orientation.z, msg.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
    return msg


def wait(node, future, timeout):
    deadline = time.monotonic()+timeout
    while not future.done():
        if time.monotonic() > deadline:
            raise TimeoutError('ROS request timed out')
        rclpy.spin_once(node, timeout_sec=0.05)
    return future.result()


def active(node, name, timeout=60):
    client = node.create_client(GetState, f'/{name}/get_state')
    beginning = time.monotonic()
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=1.0):
            response = wait(node, client.call_async(GetState.Request()), 3)
            if response.current_state.id == 3:
                return
            if (time.monotonic()-beginning >= 3 and response.current_state.id == 1
                    and getattr(node, 'localization_ready', None) is False):
                raise RuntimeError(
                    f'{name} is {response.current_state.label}; navigation is waiting for AMCL relocalization. '
                    'Run: ros2 run j501_navigation relocalize --pose 0 0 0 . '
                    'Inspect /localization/metrics if it does not converge; do not activate Nav2 manually to bypass localization.')
        elif 'slam_toolbox' in node.get_node_names():
            raise RuntimeError(
                'Current mode is mapping (SLAM Toolbox); AMCL and Nav2 are not running. '
                '1) Save: ros2 run j501_navigation save_map --output build/m06_maps/office . '
                '2) Ctrl+C the mapping launch, then ./modules/m06_navigation/run_navigation.sh . '
                '3) ros2 run j501_navigation relocalize --pose 0 0 0 . '
                '4) Retry navigation after relocalization succeeds. Do not run mapping and navigation together.')
        rclpy.spin_once(node, timeout_sec=0.1)
    raise TimeoutError(f'{name} did not reach active state; inspect Nav2 launch output')


class Lab(Node):
    def __init__(self, extra_obstacle=None):
        super().__init__('j501_navigation_lab', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.start_wall = time.monotonic()
        self.actual = []
        self.motion_started = False
        self.localization_ready = None
        self.create_subscription(Bool, '/localization_ready', lambda msg: setattr(self, 'localization_ready', msg.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.map_alignment = None
        self.create_subscription(String, '/lab/map_metadata', self.map_metadata,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.min_clearance = math.inf
        self.collision_samples = 0
        self.local_obstacle_peak = -1
        self.global_obstacle_peak = -1
        self.extra_obstacle = extra_obstacle
        scene_file = Path(get_package_share_directory('j501_navigation')) / 'scenes/office.yaml'
        scene = yaml.safe_load(scene_file.read_text())
        self.scene_obstacles = scene.get('unmapped_cones', [])
        self.boxes = scene['boxes'] + self.scene_obstacles
        if extra_obstacle:
            self.boxes.append({'center': extra_obstacle[:2], 'size': extra_obstacle[2:]})
        self.path_publisher = self.create_publisher(RosPath, '/lab/actual_path', 10)
        self.stop_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(TFMessage, '/simulation/ground_truth', self.record, qos_profile_sensor_data)
        for topic in ['local_costmap', 'global_costmap']:
            self.create_subscription(OccupancyGrid, f'/{topic}/costmap',
                lambda msg, kind=topic: self.observe_costmap(msg, kind),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def current_pose(self):
        deadline = time.monotonic()+30
        while time.monotonic() < deadline:
            try:
                t = self.buffer.lookup_transform('map', 'base_footprint', Time()).transform
                return [t.translation.x, t.translation.y, yaw(t.rotation)]
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError('No map -> base_footprint transform; wait for AMCL and scan data')

    def map_metadata(self, msg):
        self.map_alignment = json.loads(msg.data).get('world_from_map')

    def world_to_map(self, point):
        if self.map_alignment is None:
            return None
        x0, y0, angle = self.map_alignment
        x, y = point[0]-x0, point[1]-y0
        return [math.cos(angle)*x+math.sin(angle)*y, -math.sin(angle)*x+math.cos(angle)*y, point[2]-angle]

    def record(self, msg):
        for t in msg.transforms:
            if t.child_frame_id != 'j501_chassis':
                continue
            xyz = t.transform.translation
            p = [xyz.x, xyz.y, yaw(t.transform.rotation)]
            # Evaluate and store at a bounded rate, independently of encoder odometry.
            if self.actual and math.dist(p[:2], self.actual[-1][:2]) < 0.025 and abs(p[2]-self.actual[-1][2]) < 0.05:
                continue
            self.actual.append(p)
            distance = clearance(p, self.boxes)
            self.min_clearance = min(self.min_clearance, distance)
            self.collision_samples += distance <= 0.0001
            path = RosPath()
            path.header.frame_id = 'map'
            path.header.stamp = self.get_clock().now().to_msg()
            if self.map_alignment is not None:
                path.poses = [pose(self, self.world_to_map(point)) for point in self.actual]
                self.path_publisher.publish(path)

    def observe_costmap(self, msg, kind):
        if not self.extra_obstacle:
            return
        x, y = self.extra_obstacle[:2]
        if msg.header.frame_id != 'map':
            try:
                transform = self.buffer.lookup_transform(msg.header.frame_id, 'map', Time()).transform
                angle = yaw(transform.rotation)
                x, y = (math.cos(angle)*x-math.sin(angle)*y+transform.translation.x,
                        math.sin(angle)*x+math.cos(angle)*y+transform.translation.y)
            except Exception:
                return
        ix = math.floor((x-msg.info.origin.position.x)/msg.info.resolution)
        iy = math.floor((y-msg.info.origin.position.y)/msg.info.resolution)
        if 0 <= ix < msg.info.width and 0 <= iy < msg.info.height:
            value = msg.data[iy*msg.info.width+ix]
            attr = 'local_obstacle_peak' if kind == 'local_costmap' else 'global_obstacle_peak'
            setattr(self, attr, max(getattr(self, attr), value))

    def summary(self):
        return {'actual_trajectory': self.actual, 'actual_path_length_m': sum(math.dist(a[:2], b[:2]) for a, b in zip(self.actual, self.actual[1:])),
                'actual_trajectory_frame': 'simulation_world',
                'actual_trajectory_map': [self.world_to_map(point) for point in self.actual] if self.map_alignment is not None else None,
                'min_footprint_clearance_m': self.min_clearance if math.isfinite(self.min_clearance) else None,
                'footprint_contact_samples': self.collision_samples, 'wall_time_s': time.monotonic()-self.start_wall,
                'temporary_obstacle': self.extra_obstacle, 'local_obstacle_peak_cost': self.local_obstacle_peak,
                'global_obstacle_peak_cost': self.global_obstacle_peak,
                'scene_unmapped_obstacles': self.scene_obstacles}


def request_path(node, planner, goal, start=None):
    active(node, 'planner_server')
    client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    if not client.wait_for_server(timeout_sec=15):
        raise TimeoutError('Planner Action unavailable')
    request = ComputePathToPose.Goal()
    request.goal, request.planner_id = pose(node, goal), PLANNERS[planner]
    request.use_start = start is not None
    if start:
        request.start = pose(node, start)
    handle = wait(node, client.send_goal_async(request), 10)
    if not handle.accepted:
        raise RuntimeError('Planner goal rejected')
    response = wait(node, handle.get_result_async(), 30)
    if response.status != GoalStatus.STATUS_SUCCEEDED or len(response.result.path.poses) < 2:
        raise RuntimeError(f'Planning failed, Action status {response.status}')
    path = response.result.path
    points = [[p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)] for p in path.poses]
    duration = response.result.planning_time
    report = {'planner': planner, 'planner_id': PLANNERS[planner], 'goal': goal, 'frame_id': path.header.frame_id,
              'points': points, 'length_m': sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:])),
              'planning_time_s': duration.sec+duration.nanosec*1e-9, 'source': 'Nav2 ComputePathToPose Action'}
    return path, report


def run_motion(node, action_type, action_name, request, timeout):
    client = ActionClient(node, action_type, action_name)
    if not client.wait_for_server(timeout_sec=30):
        raise TimeoutError(f'{action_name} unavailable')
    last_feedback = [0.0]
    def feedback(message):
        if time.monotonic()-last_feedback[0] >= 5:
            data = message.feedback
            distance = getattr(data, 'distance_remaining', getattr(data, 'distance_to_goal', None))
            node.get_logger().info(f'{action_name}: distance to goal = {distance}')
            last_feedback[0] = time.monotonic()
    handle = wait(node, client.send_goal_async(request, feedback_callback=feedback), 15)
    if not handle.accepted:
        raise RuntimeError(f'{action_name} rejected goal')
    node.motion_started = True
    try:
        response = wait(node, handle.get_result_async(), timeout)
    except (TimeoutError, KeyboardInterrupt):
        wait(node, handle.cancel_goal_async(), 5)
        raise
    return response.status


def write(report, output):
    print(json.dumps(report, indent=2))
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + '\n')


def global_main():
    parser = argparse.ArgumentParser(description='Execute a Nav2 global planner and export its path.')
    parser.add_argument('--planner', choices=list(PLANNERS), default='astar')
    parser.add_argument('--goal', nargs=3, type=float, default=[4.5, 2.7, 0.0])
    parser.add_argument('--start', nargs=3, type=float)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rclpy.init(); node = Lab()
    try:
        _, report = request_path(node, args.planner, args.goal, args.start)
        write(report, args.output)
    finally:
        node.destroy_node(); rclpy.shutdown()


def motion_main(follow=False):
    parser = argparse.ArgumentParser(description='Execute local FollowPath or complete Nav2 navigation.')
    parser.add_argument('--planner', choices=list(PLANNERS), default='astar')
    parser.add_argument('--controller', choices=CONTROLLERS, default='MPPI')
    parser.add_argument('--goal', nargs=3, type=float, default=[4.5, 2.7, 0.0])
    parser.add_argument('--path', type=Path, help='Use a saved global_plan path for isolated local control')
    parser.add_argument('--obstacle', nargs=4, type=float, help='Known evaluation geometry: x y width height; not navigation input')
    parser.add_argument('--timeout', type=float, default=240)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rclpy.init(); node = Lab(args.obstacle)
    try:
        active(node, 'controller_server' if follow else 'bt_navigator')
        if follow:
            if args.path:
                saved = json.loads(args.path.read_text())
                path = RosPath(); path.header.frame_id = saved['frame_id']; path.header.stamp = node.get_clock().now().to_msg()
                path.poses = [pose(node, point) for point in saved['points']]
                args.goal = saved['goal']
            else:
                path, _ = request_path(node, args.planner, args.goal)
            request = FollowPath.Goal()
            request.path, request.controller_id, request.goal_checker_id = path, args.controller, 'general_goal_checker'
            status = run_motion(node, FollowPath, '/follow_path', request, args.timeout)
        else:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            selectors = [node.create_publisher(String, '/planner_selector', qos), node.create_publisher(String, '/controller_selector', qos)]
            for _ in range(10):
                selectors[0].publish(String(data=PLANNERS[args.planner])); selectors[1].publish(String(data=args.controller))
                rclpy.spin_once(node, timeout_sec=0.05)
            request = NavigateToPose.Goal(); request.pose = pose(node, args.goal)
            status = run_motion(node, NavigateToPose, '/navigate_to_pose', request, args.timeout)
        current = node.current_pose()
        report = {**node.summary(), 'action_status': status, 'succeeded': status == GoalStatus.STATUS_SUCCEEDED,
                  'mode': 'local FollowPath' if follow else 'NavigateToPose', 'planner': args.planner,
                  'controller': args.controller, 'goal': args.goal, 'final_localized_pose': current,
                  'localized_goal_error_m': math.dist(current[:2], args.goal[:2])}
        if node.actual and node.map_alignment is not None:
            report['actual_goal_error_m'] = math.dist(node.world_to_map(node.actual[-1])[:2], args.goal[:2])
        write(report, args.output)
        if not report['succeeded']:
            raise SystemExit(1)
    except (TimeoutError, RuntimeError) as error:
        write({**node.summary(), 'succeeded': False, 'action_status': None,
               'controller': args.controller, 'planner': args.planner,
               'goal': args.goal, 'error': str(error)}, args.output)
        raise SystemExit(1)
    finally:
        if node.motion_started:
            for _ in range(5):
                node.stop_publisher.publish(Twist()); rclpy.spin_once(node, timeout_sec=0.03)
        node.destroy_node(); rclpy.shutdown()
