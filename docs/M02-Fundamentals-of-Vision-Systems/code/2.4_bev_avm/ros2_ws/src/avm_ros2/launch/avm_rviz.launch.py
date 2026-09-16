"""Start software-only AVM visualization. No chassis/CAN node is launched."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('avm_ros2')
    start_driver = LaunchConfiguration('start_camera_driver')
    return LaunchDescription([
        DeclareLaunchArgument('start_camera_driver', default_value='false', description='Start j501 camera driver. Stop calib_web first because V4L2 devices are exclusive.'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        Node(package='j501_avm_calib', executable='camera_driver', name='avm_camera_driver', output='screen', condition=IfCondition(start_driver)),
        Node(package='avm_ros2', executable='avm_bev_node', name='avm_bev', output='screen'),
        Node(package='avm_ros2', executable='avm_local_map_node', name='avm_local_map', output='screen'),
        Node(package='avm_ros2', executable='avm_status_overlay_node', name='avm_status_overlay', output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'config', 'avm_rviz.rviz')], output='screen', condition=IfCondition(LaunchConfiguration('start_rviz'))),
    ])
