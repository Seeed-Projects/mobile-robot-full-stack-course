"""M4.4 web-hub visualizer launch — the host-side pose overlay node.

Starts the single node that turns the Isaac ROS FoundationPose container
output (bag replay of the NGC Mustard example) into /perception/demo/m4_4
plus /perception/demo/m4_4/stats for the web hub.

The container (m4-isaacros-foundationpose) runs with network/ipc/pid host
mode, so /image_rect, /camera_info_rect and /output are visible on the host
DDS domain as-is. This launch owns no inference and no camera; the container
pipeline itself is driven by scripts/m4/run_m4_4_isaacros_quickstart.sh in
hub-hold mode (M44_VIEW_SECONDS=-1) from the hub module wrapper.

Usage:
  ros2 launch m4_demo_bringup m4_4_web.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_mesh = os.environ.get('M44_MESH_OBJ_PATH', '')

    visualizer = Node(
        package='m4_demo_bringup',
        executable='m4_4_web_visualizer',
        name='m4_4_web_visualizer',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'pose_topic': LaunchConfiguration('pose_topic'),
            'mesh_obj_path': LaunchConfiguration('mesh_obj_path'),
            'max_output_fps': LaunchConfiguration('max_output_fps'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic', default_value='/image_rect',
            description='Bag-replay RGB topic from the Isaac ROS container'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera_info_rect',
            description='Bag-replay camera intrinsics topic'),
        DeclareLaunchArgument(
            'pose_topic', default_value='/output',
            description='FoundationPose Detection3DArray output topic'),
        DeclareLaunchArgument(
            'mesh_obj_path', default_value=default_mesh,
            description='OBJ mesh used for the bounding-box overlay'),
        DeclareLaunchArgument(
            'max_output_fps', default_value='30.0',
            description='Output overlay cap (the looping bag publishes faster)'),
        visualizer,
        LogInfo(msg='m4_4_web: host overlay node for the Isaac ROS FoundationPose container'),
    ])
