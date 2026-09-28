"""Launch file: standalone bev_tracking node.

Brings up:
    tracking_node  (subscribes to /perception/detections,
                    publishes /perception/tracks)

Does NOT start:
    Camera / sensor drivers
    bev_detection (YOLO + TensorRT)
    bev_perception / BEV pipeline

For integration with bev_detection, see tracking_with_detection.launch.py
in this same directory.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('bev_tracking')
    config_path = os.path.join(pkg_share, 'config', 'bytetrack.yaml')

    tracking_node = Node(
        package='bev_tracking',
        executable='tracking_node',
        name='tracking_node',
        output='screen',
        parameters=[config_path],
    )

    mock_publisher = Node(
        package='bev_tracking',
        executable='mock_detection_publisher',
        name='mock_detection_publisher',
        output='screen',
        parameters=[{
            'output_topic': '/perception/detections',
            'rate_hz': 10.0,
            'num_objects': 1,
            'class_id': 'person',
            'confidence': 0.9,
        }],
    )

    return LaunchDescription([
        tracking_node,
        mock_publisher,
    ])
