# launch/m4_segmentation.launch.py
#
# 启动 image_republisher + segmentation_node (test pipeline) 或
# 仅 segmentation_node (real pipeline, 由 bringup 提供 /perception/cameras/front/image).

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = "bev_segmentation"


def generate_launch_description():
    # 是否启动内置 test image republisher (true 用于 offline smoke; false 用于真实 pipeline)
    use_test_image = LaunchConfiguration("use_test_image", default="false")
    test_image_path = LaunchConfiguration(
        "test_image_path",
        default="/home/seeed/mobile-robot-full-stack-course/modules/m04-ai-vision-and-edge-acceleration/output/m4/4.3/test_input.png",
    )

    seg_node = Node(
        package=PACKAGE_NAME,
        executable="segmentation_node",
        name="segmentation_node",
        parameters=[
            os.path.join(
                os.path.dirname(__file__), "..", "config", "segmentation.yaml"
            )
        ],
        output="screen",
    )

    image_repub = Node(
        package=PACKAGE_NAME,
        executable="image_republisher.py",
        name="image_republisher",
        arguments=["--image", test_image_path],
        condition=None,  # 由 include 决定
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_test_image", default_value="false"),
        DeclareLaunchArgument(
            "test_image_path",
            default_value="/home/seeed/mobile-robot-full-stack-course/modules/m04-ai-vision-and-edge-acceleration/output/m4/4.3/test_input.png",
        ),
        seg_node,
    ])
