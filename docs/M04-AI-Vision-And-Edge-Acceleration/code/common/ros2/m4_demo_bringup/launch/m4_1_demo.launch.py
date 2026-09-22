"""M4.1 demo launch.

Camera mode (camera_source arg, passed from bash wrapper):
  v4l2   : Start the installed csi_camera_publisher with v4l2src → YOLO
  existing: /perception/cameras/front/image already live; start YOLO only
  test   : GStreamer videotestsrc (synthetic, always works)
  gmsl   : (deprecated) legacy nvargus mode — use v4l2 instead

Usage (from bash wrapper):
  ros2 launch m4_demo_bringup m4_1_demo.launch.py camera_source:=v4l2
  ros2 launch m4_demo_bringup m4_1_demo.launch.py camera_source:=test

Standalone:
  ros2 launch m4_demo_bringup m4_1_demo.launch.py camera_source:=existing
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_demo = get_package_share_directory('m4_demo_bringup')
    pkg_detection = get_package_share_directory('bev_detection')

    config_path = os.path.join(pkg_demo, 'config', 'demo.yaml')

    # camera_source arg: v4l2 | existing | test | gmsl (deprecated)
    camera_source = LaunchConfiguration('camera_source')

    # ---- YOLO detection (always started) ----
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_detection, 'launch', 'yolo.launch.py')),
        launch_arguments={
            'image_topic': '/perception/cameras/front/image',
            'detections_topic': '/perception/detections',
            'debug_image_topic': '/perception/demo/m4_1',
            'publish_debug_image': 'true',
        }.items(),
    )

    # ---- Optional FPS-banner wrapper ----
    detection_wrapper = Node(
        package='m4_demo_bringup',
        executable='detection_visualizer',
        name='detection_visualizer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('wrapper_on')),
        parameters=[config_path],
        arguments=[
            '--ros-args', '--remap',
            'image_topic:=/perception/debug/detection_image',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_source', default_value='csi',
            description='csi|existing|test  (csi = GMSL via /dev/video0)'),
        DeclareLaunchArgument(
            'wrapper_on', default_value='false',
            description='Add detection_visualizer FPS banner'),
        yolo_launch,
        detection_wrapper,
        LogInfo(msg='m4_1_demo.launch.py: camera_source={}'.format(camera_source)),
    ])
