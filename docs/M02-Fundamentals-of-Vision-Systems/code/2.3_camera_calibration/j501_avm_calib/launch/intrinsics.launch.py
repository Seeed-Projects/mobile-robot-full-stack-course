# -*- coding: utf-8 -*-
"""单路内参标定（GUI cameracalibrator / 无头自动二选一）。

用法:
    ros2 launch j501_avm_calib intrinsics.launch.py direction:=front
    ros2 launch j501_avm_calib intrinsics.launch.py direction:=back auto:=true
GUI 操作:
    1) GUI 窗口内把底部 trackbar「Camera type」拖到 1（fisheye）
    2) 缓慢移动棋盘覆盖各区域，CALIBRATE 可点后点击
    3) SAVE（生成 /tmp/calibrationdata.tar.gz）→ COMMIT（保存到驱动 YAML）
注意: GUI 需要显示器（本地/X 转发/局域网 PC 均可）；无显示器用 auto:=true。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

DIRECTIONS = ("front", "back", "left", "right")


def generate_launch_description():
    direction = LaunchConfiguration("direction")
    auto = LaunchConfiguration("auto")
    chess_cols = LaunchConfiguration("chess_cols")
    chess_rows = LaunchConfiguration("chess_rows")
    square = LaunchConfiguration("square")
    return LaunchDescription([
        DeclareLaunchArgument("direction", default_value="front",
                              choices=DIRECTIONS),
        DeclareLaunchArgument("auto", default_value="false",
                              description="true=无头自动标定节点"),
        DeclareLaunchArgument("chess_cols", default_value="8"),
        DeclareLaunchArgument("chess_rows", default_value="6"),
        DeclareLaunchArgument("square", default_value="0.025"),
        Node(
            package="camera_calibration",
            executable="cameracalibrator",
            name="cameracalibrator",
            output="screen",
            condition=IfCondition(PythonExpression(
                ["'", auto, "' == 'false'"])),
            remappings=[
                ("image", ["/cameras/", direction, "/image_raw"]),
                ("camera", ["/cameras/", direction]),
            ],
            arguments=["--size", [chess_cols, "x", chess_rows],
                       "--square", square,
                       "--no-service-check",
                       "--fisheye-check-conditions",
                       "--fisheye-fix-skew",
                       "--fisheye-recompute-extrinsicsts"],
        ),
        Node(
            package="j501_avm_calib",
            executable="intrinsics_auto",
            name=["intrinsics_auto_", direction],
            output="screen",
            condition=IfCondition(PythonExpression(["'", auto, "' == 'true'"])),
            arguments=["--direction", direction],
        ),
    ])