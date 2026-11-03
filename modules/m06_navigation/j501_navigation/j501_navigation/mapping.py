"""Survey using online SLAM feedback, then save actual occupancy and pose graph."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SerializePoseGraph
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def heading(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def survey_main():
    parser = argparse.ArgumentParser(description='Simulation-only SLAM-guided survey; no prebuilt map or ground truth.')
    parser.add_argument('--timeout', type=float, default=480)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Node('mapping_survey', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    odom, scan, grid = [None], [None], [None]
    node.create_subscription(Odometry, '/odom', lambda m: odom.__setitem__(0, m), qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', lambda m: scan.__setitem__(0, m), qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, '/map', lambda m: grid.__setitem__(0, m),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    def current_pose():
        t = buffer.lookup_transform('map', 'base_footprint', Time()).transform
        return t.translation.x, t.translation.y, heading(t.rotation)
    beginning = time.monotonic()
    # Manually selected survey waypoints relative to the first SLAM pose.
    # This is a reproducible teaching drive, not autonomous exploration.
    outward = [(2.4, 2.6), (2.9, 3.65), (3.2, 4.3), (4.5, 4.3), (6.3, 3.5), (9.3, 3.5), (9.3, 6.5)]
    route = outward+outward[-2::-1]+[(0., 0.)]
    try:
        while odom[0] is None or scan[0] is None or grid[0] is None:
            if time.monotonic()-beginning > 40:
                raise TimeoutError('Wait for SLAM /map, /scan, and /odom before surveying')
            rclpy.spin_once(node, timeout_sec=0.1)
        while not buffer.can_transform('map', 'base_footprint', Time()):
            if time.monotonic()-beginning > 45:
                raise TimeoutError('SLAM has not published map -> base_footprint')
            rclpy.spin_once(node, timeout_sec=0.1)
        x0, y0, yaw0 = current_pose()
        trajectory = []
        stalled = None
        for index, (dx, dy) in enumerate(route):
            target = x0+math.cos(yaw0)*dx-math.sin(yaw0)*dy, y0+math.sin(yaw0)*dx+math.cos(yaw0)*dy
            node.get_logger().info(f'Survey waypoint {index+1}/{len(route)}: map {target}')
            while True:
                if time.monotonic()-beginning > args.timeout:
                    raise TimeoutError('Survey time limit reached')
                rclpy.spin_once(node, timeout_sec=0.04)
                if (node.get_clock().now()-Time.from_msg(scan[0].header.stamp)).nanoseconds > 2_000_000_000:
                    raise RuntimeError('Laser data stale; stopping survey')
                x, y, angle = current_pose()
                distance = math.hypot(target[0]-x, target[1]-y)
                if distance < 0.10:
                    break
                error = math.atan2(math.sin(math.atan2(target[1]-y, target[0]-x)-angle),
                                   math.cos(math.atan2(target[1]-y, target[0]-x)-angle))
                cmd = Twist()
                cmd.angular.z = max(-0.35, min(0.35, 1.0*error))
                if abs(error) < 0.45:
                    cmd.linear.x = min(0.18, max(0.05, distance*0.4))*max(0., math.cos(error))
                ranges = scan[0].ranges
                near = [r for i, r in enumerate(ranges)
                        if abs(scan[0].angle_min+i*scan[0].angle_increment) < 0.45 and math.isfinite(r)]
                if near and min(near) < 0.60 and cmd.linear.x > 0:
                    cmd = Twist()
                    # A short, laser-only gap turn helps the survey recover from
                    # moderate SLAM/odometry error; it is not a global planner.
                    left = [r for i, r in enumerate(ranges) if 0.5 < scan[0].angle_min+i*scan[0].angle_increment < 1.2]
                    right = [r for i, r in enumerate(ranges) if -1.2 < scan[0].angle_min+i*scan[0].angle_increment < -0.5]
                    if left and right:
                        cmd.angular.z = 0.22 if min(left) > min(right) else -0.22
                    stalled = stalled or time.monotonic()
                    if time.monotonic()-stalled > 12:
                        raise RuntimeError('Survey route blocked; stop and use manual teleoperation')
                else:
                    stalled = None
                pub.publish(cmd)
                if not trajectory or math.dist([x, y], trajectory[-1][:2]) > 0.05:
                    trajectory.append([x, y, angle])
        pub.publish(Twist())
        # Let the final loop closure reach the published occupancy grid.
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        values = grid[0].data
        report = dict(succeeded=True, waypoints=len(route), wall_time_s=time.monotonic()-beginning,
            slam_trajectory=trajectory, map_frame=grid[0].header.frame_id,
            map_origin=[grid[0].info.origin.position.x, grid[0].info.origin.position.y],
            observed_cells=sum(v >= 0 for v in values), occupied_cells=sum(v >= 65 for v in values),
            note='Survey uses online SLAM pose and live scan; no prebuilt map or simulator truth. Not autonomous exploration.')
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k!='slam_trajectory'}, indent=2))
    finally:
        for _ in range(5):
            pub.publish(Twist()); rclpy.spin_once(node, timeout_sec=0.03)
        node.destroy_node()
        rclpy.shutdown()


def save_main():
    parser = argparse.ArgumentParser(description='Save the observed SLAM map and its reusable pose graph.')
    parser.add_argument('--output', type=Path, default=Path('build/m06_maps/office'))
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(output),
        '--ros-args', '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=15.0',
        '-p', 'free_thresh_default:=0.25', '-p', 'occupied_thresh_default:=0.65'], check=True, timeout=35)
    rclpy.init()
    node = Node('save_slam_graph')
    try:
        client = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Occupancy saved, but SLAM Toolbox serialize_map service is unavailable')
        request = SerializePoseGraph.Request(filename=str(output)+'_posegraph')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        if not future.done() or future.result().result != 0:
            raise RuntimeError('Occupancy saved, but pose-graph serialization failed')
        scene = yaml.safe_load((Path(get_package_share_directory('j501_navigation'))/'scenes/office.yaml').read_text())
        # SLAM fixes its first base pose at map (0,0,0). This known simulation
        # placement is used ONLY to compare ground truth with the saved map.
        metadata = dict(world_from_map=scene['initial_pose'], evaluation_only=True,
                        source='SLAM Toolbox /map and serialize_map; world placement only for evaluation',
                        note='Valid for a fresh simulation survey starting at its configured spawn.')
        Path(str(output)+'.metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
        print(f'Saved {output}.yaml + .pgm and {output}_posegraph.posegraph + .data')
    finally:
        node.destroy_node()
        rclpy.shutdown()
