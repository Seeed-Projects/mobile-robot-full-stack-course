"""Launch file: M4.2 real-time demo pipeline.

Composes the existing M4.1 detection + M4.2 tracking modules and adds
the new tracking_visualizer on top. No code from bev_detection is
copied; the upstream launch is included as-is.

Pipeline:

    /perception/cameras/front/image
       (upstream) bev_detection/yolo_trt_node
       -> /perception/detections
       -> bev_tracking/tracking_node
       -> /perception/tracks
       -> bev_tracking/tracking_visualizer  (NEW)
       -> /perception/tracking_debug_image

Usage:

    ros2 launch bev_tracking tracking_demo.launch.py
    ros2 launch bev_tracking tracking_demo.launch.py \
        camera_topic:=/my/camera/image \
        use_monitor:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_tracking = get_package_share_directory('bev_tracking')
    pkg_detection = get_package_share_directory('bev_detection')

    tracking_config = os.path.join(pkg_tracking, 'config', 'bytetrack.yaml')
    detection_launch = os.path.join(pkg_detection, 'launch',
                                    'm4_detection.launch.py')

    # Camera topic (input) — exposed as a launch arg with the M4.1
    # default preserved.
    camera_topic = LaunchConfiguration('camera_topic')
    detections_topic = LaunchConfiguration('detections_topic')
    tracks_topic = LaunchConfiguration('tracks_topic')
    debug_image_topic = LaunchConfiguration('debug_image_topic')
    use_monitor = LaunchConfiguration('use_monitor')

    detection_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(detection_launch),
        launch_arguments=[
            ('use_camera_sync', 'false'),
            ('use_yolo', 'true'),
            ('use_bev', 'false'),
            ('use_viz', 'false'),
            ('use_monitor', use_monitor),
            ('yolo_image_topic', camera_topic),
            ('yolo_model_path', LaunchConfiguration('yolo_model_path')),
        ],
    )

    tracking_node = Node(
        package='bev_tracking',
        executable='tracking_node',
        name='tracking_node',
        output='screen',
        parameters=[tracking_config, {
            'input_topic': detections_topic,
            'output_topic': tracks_topic,
        }],
    )

    visualizer_node = Node(
        package='bev_tracking',
        executable='tracking_visualizer',
        name='tracking_visualizer',
        output='screen',
        parameters=[{
            'image_topic': camera_topic,
            'tracks_topic': tracks_topic,
            'debug_image_topic': debug_image_topic,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/perception/cameras/front/image',
            description='Camera image topic (input to the whole pipeline)',
        ),
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/perception/detections',
            description='Detection2DArray output of bev_detection',
        ),
        DeclareLaunchArgument(
            'tracks_topic',
            default_value='/perception/tracks',
            description='Detection2DArray output of bev_tracking (with ids)',
        ),
        DeclareLaunchArgument(
            'debug_image_topic',
            default_value='/perception/tracking_debug_image',
            description='Image topic with overlays drawn (visualizer output)',
        ),
        DeclareLaunchArgument(
            'use_monitor',
            default_value='true',
            description='Run the upstream bev_system_monitor node',
        ),
        DeclareLaunchArgument(
            'yolo_model_path',
            default_value=os.path.join(os.environ.get('M4_CODE_ROOT', ''), 'models/m4/detection/engines/yolo11n_fp16.engine'),
            description='TensorRT engine path (forwarded to bev_detection)',
        ),
        detection_launch_action,
        tracking_node,
        visualizer_node,
    ])
