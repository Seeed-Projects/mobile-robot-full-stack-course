#!/usr/bin/env python3
"""BEV 拼接 Launch 文件"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """生成 launch 描述"""
    return LaunchDescription([
        # BEV 参数
        DeclareLaunchArgument('bev_width', default_value='1000'),
        DeclareLaunchArgument('bev_height', default_value='1000'),
        DeclareLaunchArgument('bev_resolution', default_value='0.01'),
        DeclareLaunchArgument('num_cameras', default_value='4'),

        # BEV 拼接节点
        Node(
            package='j501_vision',
            executable='bev_stitch_node',
            name='bev_stitch',
            parameters=[{
                'bev_width': LaunchConfiguration('bev_width'),
                'bev_height': LaunchConfiguration('bev_height'),
                'bev_resolution': LaunchConfiguration('bev_resolution'),
                'num_cameras': LaunchConfiguration('num_cameras'),
            }],
            output='screen',
        ),
    ])
