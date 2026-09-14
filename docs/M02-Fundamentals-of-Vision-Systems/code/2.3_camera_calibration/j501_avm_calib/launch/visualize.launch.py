# -*- coding: utf-8 -*-
"""可视化 + 评估（RViz）。

用法: ros2 launch j501_avm_calib visualize.launch.py [rate:=5.0]
前置: calib_results/{front,back,left,right}.json 与 extrinsics.json 已生成。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rate = LaunchConfiguration("rate")
    rviz_cfg = PathJoinSubstitution([
        get_package_share_directory("j501_avm_calib"),
        "config", "rviz", "avm_calib.rviz",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("rate", default_value="5.0"),
        Node(
            package="j501_avm_calib",
            executable="bev_publisher",
            name="bev_publisher",
            output="screen",
            arguments=["--rate", rate],
        ),
        Node(
            package="j501_avm_calib",
            executable="evaluator",
            name="calib_evaluator",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_cfg],
        ),
    ])