"""Full-system launch: perception + (Phase 5-7: tracker, occupancy, collision,
vehicle interface). Currently launches the perception stack and reserves the
switches for the remaining nodes.

Phase 2 usage: identical to perception.launch.py (tracker etc. land in P5-7).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('bev_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('enable_tracking', default_value='false'),
        DeclareLaunchArgument('enable_occupancy', default_value='false'),
        DeclareLaunchArgument('enable_collision', default_value='false'),
        DeclareLaunchArgument('enable_vehicle_control', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', 'perception.launch.py']))),
        LogInfo(msg='vehicle.launch.py: perception core; P5-7 nodes are gated by flags (coming)'),
    ])