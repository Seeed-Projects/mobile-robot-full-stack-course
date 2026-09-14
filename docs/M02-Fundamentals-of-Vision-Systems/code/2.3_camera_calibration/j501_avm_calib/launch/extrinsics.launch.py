# -*- coding: utf-8 -*-
"""外参标定向导。用法: ros2 launch j501_avm_calib extrinsics.launch.py"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="j501_avm_calib",
            executable="extrinsic_calibrator",
            name="extrinsic_calibrator",
            output="screen",
        ),
    ])