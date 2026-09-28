"""Launch file: bev_detection (M4.1) + bev_tracking (M4.2) end-to-end.

This launch file composes the upstream bev_detection launch file as a
sub-launch and adds the tracking_node on top of it. It does NOT modify
the bev_detection package or its launch file; M4.1 is fully owned by
the bev_detection Agent and is treated as a black-box upstream producer
of /perception/detections.

Pipeline (single-process on the same ROS_DOMAIN_ID):
    /perception/cameras/front/image
        -- bev_detection/yolo_trt_node -->
    /perception/detections
        -- bev_tracking/tracking_node -->
    /perception/tracks
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('bev_tracking')
    pkg_detection = get_package_share_directory('bev_detection')

    config_path = os.path.join(pkg_share, 'config', 'bytetrack.yaml')
    detection_launch = os.path.join(pkg_detection, 'launch', 'm4_detection.launch.py')

    detection_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(detection_launch),
        launch_arguments=[('detections_topic', '/perception/detections')],
    )

    tracking_node = Node(
        package='bev_tracking',
        executable='tracking_node',
        name='tracking_node',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription([
        detection_launch_action,
        tracking_node,
    ])
