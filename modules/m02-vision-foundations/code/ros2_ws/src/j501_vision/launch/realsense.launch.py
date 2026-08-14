#!/usr/bin/env python3
"""RealSense D455 参数化 launch（realsense2_camera 封装）

用法：
  ros2 launch j501_vision realsense.launch.py \
      depth_profile:=1280x720x30 pointcloud_enable:=true

注意：本文件未在实机验证（无 RealSense 硬件），参数名以
realsense-ros (ros2 分支) 官方 rs_launch.py 为准。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depth_profile = LaunchConfiguration('depth_profile')
    rgb_profile = LaunchConfiguration('rgb_profile')
    pointcloud_enable = LaunchConfiguration('pointcloud_enable')
    align_depth_enable = LaunchConfiguration('align_depth_enable')

    return LaunchDescription([
        DeclareLaunchArgument(
            'depth_profile', default_value='1280x720x30',
            description='深度流分辨率x帧率'),
        DeclareLaunchArgument(
            'rgb_profile', default_value='1280x720x30',
            description='彩色流分辨率x帧率'),
        DeclareLaunchArgument(
            'pointcloud_enable', default_value='false',
            description='是否发布点云'),
        DeclareLaunchArgument(
            'align_depth_enable', default_value='true',
            description='是否将深度对齐到彩色帧'),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'depth_module.profile': depth_profile,
                'rgb_module.profile': rgb_profile,
                'pointcloud.enable': pointcloud_enable,
                'align_depth.enable': align_depth_enable,
            }],
        ),
    ])
