from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    share = Path(get_package_share_directory('j501_chassis_description'))
    cad = LaunchConfiguration('cad_pose').perform(context).lower() == 'true'
    mappings = {'simulation': 'false', 'add_lidar': 'false'}
    if cad:
        mappings['suspension_compression'] = '0'
    description = xacro.process_file(str(share / 'urdf/j501_chassis.urdf.xacro'), mappings=mappings).toxml()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'rviz/chassis.rviz')]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('cad_pose', default_value='false'), OpaqueFunction(function=start)])
