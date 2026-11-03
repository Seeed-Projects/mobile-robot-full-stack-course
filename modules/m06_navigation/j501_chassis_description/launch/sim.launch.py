"""ROS 2 Humble + Gazebo Fortress; independent of the ROS 1 STEP converter."""

from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    share = Path(get_package_share_directory('j501_chassis_description'))
    description = xacro.process_file(
        str(share / 'urdf/j501_chassis.urdf.xacro'),
        mappings={'mesh_prefix': (share / 'meshes').as_uri() + '/',
                  'lidar_mode': LaunchConfiguration('lidar').perform(context),
                  'odometry_source': LaunchConfiguration('odometry_source').perform(context),
                  'suspension_compression': LaunchConfiguration('suspension_compression').perform(context)},
    ).toxml()
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    cmd = ['ign', 'gazebo', '-r', '-v', '2', LaunchConfiguration('world').perform(context)]
    if headless:
        cmd.extend(['-s', '--headless-rendering'])
    return [
        ExecuteProcess(cmd=cmd, output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(share / 'config/bridge.yaml'), 'use_sim_time': True}],
             output='screen'),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', 'navigation', '-topic', 'robot_description',
                        '-name', 'j501_chassis', '-z', '0.005',
                        '-x', LaunchConfiguration('x').perform(context),
                        '-y', LaunchConfiguration('y').perform(context),
                        '-Y', LaunchConfiguration('yaw').perform(context)], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', LaunchConfiguration('rviz_config')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    import yaml
    share = Path(get_package_share_directory('j501_chassis_description'))
    geom = yaml.safe_load((share / 'config/geometry.yaml').read_text())
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value=str(share / 'worlds/navigation.sdf')),
        DeclareLaunchArgument('rviz_config', default_value=str(share / 'rviz/chassis.rviz')),
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('yaw', default_value='0'),
        DeclareLaunchArgument('lidar', default_value='2d'),
        DeclareLaunchArgument('odometry_source', default_value='wheel'),
        DeclareLaunchArgument('suspension_compression', default_value=str(geom['suspension_compression_m'])),
        OpaqueFunction(function=start),
    ])
