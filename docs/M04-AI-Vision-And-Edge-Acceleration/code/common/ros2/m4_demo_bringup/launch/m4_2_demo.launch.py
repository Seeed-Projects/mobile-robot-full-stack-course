"""M4.2 demo launch — ByteTrack tracking.

Camera mode (camera_source arg, passed from bash wrapper):
  gmsl    : start camera_sync_node + camera_adapter_node, then
             include tracking_demo.launch.py (YOLO + tracking + visualizer)
  existing/csi/usb/test: camera topic already live; include tracking_demo.launch.py
             (which starts YOLO + tracking + visualizer; use_camera_sync:=false)

Usage (from bash wrapper):
  ros2 launch m4_demo_bringup m4_2_demo.launch.py camera_source:=gmsl
  ros2 launch m4_demo_bringup m4_2_demo.launch.py camera_source:=test
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description() -> LaunchDescription:
    pkg_demo = get_package_share_directory('m4_demo_bringup')
    pkg_detection = get_package_share_directory('bev_detection')
    pkg_tracking = get_package_share_directory('bev_tracking')

    camera_source = LaunchConfiguration('camera_source')

    # ---- GMSL: camera_sync_node + camera_adapter_node ----
    # Start the full GMSL bridge (nvargus→frameset→/image).
    # If camera_sync_node is already running externally this is harmless.
    gmsl_camera_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_detection, 'launch', 'm4_detection.launch.py')),
        # ROS 2 has no $(eval ...) substitution - that is roslaunch syntax.
        # PythonExpression evaluates the concatenated substitutions as Python.
        condition=IfCondition(
            PythonExpression(["'", camera_source, "' == 'gmsl'"])),
        launch_arguments={
            'use_camera_sync': 'true',
            'use_yolo': 'false',       # YOLO is started by tracking_demo_include below
            'use_bev': 'false',
            'use_viz': 'false',
            'use_monitor': 'false',
        }.items(),
    )

    # ---- Tracking demo: YOLO + tracking_node + tracking_visualizer ----
    #   - use_camera_sync: false (camera_sync is started by gmsl branch above,
    #     or not needed for non-GMSL modes where /image is already live)
    #   - use_yolo: true (YOLO)
    #   - debug_image_topic remapped to /perception/demo/m4_2
    tracking_demo_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tracking, 'launch', 'tracking_demo.launch.py')),
        launch_arguments={
            'camera_topic': '/perception/cameras/front/image',
            'detections_topic': '/perception/detections',
            'tracks_topic': '/perception/tracks',
            'debug_image_topic': '/perception/demo/m4_2',
            'use_camera_sync': 'false',
            'use_yolo': 'true',
            'use_monitor': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_source', default_value='auto',
            description='gmsl|existing|csi|usb|test  (passed from bash wrapper)'),
        gmsl_camera_include,
        tracking_demo_include,
        LogInfo(msg='m4_2_demo.launch.py: camera_source={}'.format(camera_source)),
    ])
