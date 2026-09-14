# -*- coding: utf-8 -*-
"""启动四路相机驱动。用法: ros2 launch j501_avm_calib driver.launch.py"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="j501_avm_calib",
            executable="camera_driver",
            name="camera_driver",
            output="screen",
        ),
    ])