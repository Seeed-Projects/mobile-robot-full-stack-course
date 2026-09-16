# -*- coding: utf-8 -*-
"""常驻评估节点 + 诊断发布。用法: ros2 launch j501_avm_calib evaluate.launch.py"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="j501_avm_calib",
            executable="evaluator",
            name="calib_evaluator",
            output="screen",
        ),
    ])