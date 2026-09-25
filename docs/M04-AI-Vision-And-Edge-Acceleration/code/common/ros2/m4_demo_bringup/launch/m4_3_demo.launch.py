"""M4.3 demo launch — SegFormer semantic segmentation.

Camera mode (camera_source arg, passed from bash wrapper):
  gmsl    : start camera_sync_node + camera_adapter_node, then
             segmentation_node + segmentation_visualizer
  existing/csi/usb/test: camera topic already live; start
             segmentation_node + segmentation_visualizer

Usage (from bash wrapper):
  ros2 launch m4_demo_bringup m4_3_demo.launch.py camera_source:=gmsl
  ros2 launch m4_demo_bringup m4_3_demo.launch.py camera_source:=test
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_demo = get_package_share_directory('m4_demo_bringup')
    pkg_detection = get_package_share_directory('bev_detection')
    pkg_seg = get_package_share_directory('bev_segmentation')

    config_path = os.path.join(pkg_demo, 'config', 'demo.yaml')
    seg_config = os.path.join(pkg_seg, 'config', 'segmentation.yaml')
    camera_source = LaunchConfiguration('camera_source')
    segmentation_max_fps = LaunchConfiguration('segmentation_max_fps')
    segmentation_view_mode = LaunchConfiguration('segmentation_view_mode')

    # ---- GMSL: camera_sync_node + camera_adapter_node ----
    gmsl_camera_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_detection, 'launch', 'm4_detection.launch.py')),
        # ROS 2 has no $(eval ...) substitution - that is roslaunch syntax.
        # PythonExpression evaluates the concatenated substitutions as Python.
        condition=IfCondition(
            PythonExpression(["'", camera_source, "' == 'gmsl'"])),
        launch_arguments={
            'use_camera_sync': 'true',
            'use_yolo': 'false',
            'use_bev': 'false',
            'use_viz': 'false',
            'use_monitor': 'false',
        }.items(),
    )

    # ---- Segmentation node ----
    seg_node = Node(
        package='bev_segmentation',
        executable='segmentation_node',
        name='segmentation_node',
        output='screen',
        parameters=[seg_config, {
            'max_inference_fps': segmentation_max_fps,
            'selected_image_topic': '/perception/segmentation/source_image',
        }],
    )

    # ---- Segmentation visualizer ----
    seg_visualizer = Node(
        package='m4_demo_bringup',
        executable='segmentation_visualizer',
        name='segmentation_visualizer',
        output='screen',
        parameters=[config_path, {
            'image_topic': '/perception/segmentation/source_image',
            'view_mode': segmentation_view_mode,
            'max_width': 1920,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_source', default_value='auto',
            description='gmsl|existing|csi|usb|test  (passed from bash wrapper)'),
        DeclareLaunchArgument(
            'segmentation_max_fps', default_value='30.0',
            description='M4.3 standalone inference cap (1..30)'),
        DeclareLaunchArgument(
            'segmentation_view_mode', default_value='semantic',
            description='M4.3 view: original|semantic|drivable'),
        gmsl_camera_include,
        seg_node,
        seg_visualizer,
        LogInfo(msg='m4_3_demo.launch.py: camera_source={}'.format(camera_source)),
    ])
