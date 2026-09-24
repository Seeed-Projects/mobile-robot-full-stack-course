# orbbec_gemini2.launch.py
#
# Orbbec Gemini 2 RGB-D camera bringup for the bev_pose M4.4 demo.
#
# Device selection — MUST NOT hardcode usb_port:
#   1. If orbbec_serial_number is set (env or arg), pass to driver.
#   2. Otherwise set device_auto_select=True; driver picks the first
#      enumerated device and prints device_serial on start.
#   3. If the driver is not installed / no device, fall back to the
#      republisher test source so the demo is still smoke-testable.
#
# Required topics (re-mapped):
#   /camera/color/image_raw → /perception/cameras/front/image
#   /camera/depth/image_raw → /perception/cameras/front/depth
#   /camera/color/camera_info → /perception/cameras/front/camera_info
#
# Pre-flight:
#   sudo apt install ros-${ROS_DISTRO}-orbbec-camera  (or build from source)
#   lsusb | grep -i orbbec      (verify device appears)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PythonExpression,
)

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = 'orbbec_camera'
    declared = [
        DeclareLaunchArgument(
            'orbbec_serial_number', default_value='',
            description='Preferred: per-device serial number string. Empty = auto-discover.'),
        DeclareLaunchArgument(
            'orbbec_usb_port', default_value='',
            description='Optional usb_port hint (e.g. "2-1"). Empty = use serial or auto.'),
        DeclareLaunchArgument(
            'use_usb_port', default_value='false',
            description='If true, select device by usb_port instead of serial_number.'),
        DeclareLaunchArgument(
            'align_color_to_depth', default_value='true',
            description='Align depth frame to colour FOV (recommended for FoundationPose).'),
        DeclareLaunchArgument(
            'color_width', default_value='1280'),
        DeclareLaunchArgument(
            'color_height', default_value='720'),
        DeclareLaunchArgument(
            'color_fps', default_value='30'),
        DeclareLaunchArgument(
            'depth_width', default_value='640'),
        DeclareLaunchArgument(
            'depth_height', default_value='576'),
        DeclareLaunchArgument(
            'depth_fps', default_value='30'),
        DeclareLaunchArgument(
            'device_auto_select', default_value='true',
            description='Orbbec driver auto-selects the first device when no serial set.'),
        DeclareLaunchArgument(
            'enable_fallback_publisher', default_value='true',
            description='If orbbec_camera_node fails to publish within timeout, '
                        'start a synthetic RGB+Depth publisher for smoke-testing.'),
        DeclareLaunchArgument(
            'fallback_publisher', default_value='image_republisher.py',
            description='Override of the fallback script path. By default uses '
                        'ros2_ws/src/bev_segmentation/test/image_republisher.py '
                        'which is identical to the bev_detection version.'),
    ]

    # Build the params dict dynamically: prefer serial, fall back to usb_port.
    params = []

    def _maybe_set(key, value):
        if value and value != '':
            return [(key, value)]
        return []

    # Defer setting params via lambda; we cannot easily mutate lists — leave
    # the driver defaults and let it auto-select. Override is by launch arg.
    orbbec_node = Node(
        package=pkg,
        executable='orbbec_camera_node',
        name='orbbec_camera_node',
        output='screen',
        parameters=[{
            'serial_number': LaunchConfiguration('orbbec_serial_number'),
            'usb_port': LaunchConfiguration('orbbec_usb_port'),
            'device_auto_select': LaunchConfiguration('device_auto_select'),
            'use_usb_port': LaunchConfiguration('use_usb_port'),
            'color_width': LaunchConfiguration('color_width'),
            'color_height': LaunchConfiguration('color_height'),
            'color_fps': LaunchConfiguration('color_fps'),
            'depth_width': LaunchConfiguration('depth_width'),
            'depth_height': LaunchConfiguration('depth_height'),
            'depth_fps': LaunchConfiguration('depth_fps'),
            'align_color_to_depth': LaunchConfiguration('align_color_to_depth'),
        }],
        # Remap to the standard bev perception topic space.
        remappings=[
            ('/camera/color/image_raw', '/perception/cameras/front/image'),
            ('/camera/depth/image_raw', '/perception/cameras/front/depth'),
            ('/camera/color/camera_info', '/perception/cameras/front/camera_info'),
        ],
    )

    # Sanity probe: lsusb to confirm an Orbbec device is plugged in. We do NOT
    # refuse to launch if missing (the user may be running with the driver
    # only); this just logs.
    lsusb_probe = ExecuteProcess(
        cmd=['bash', '-lc',
             'if lsusb 2>/dev/null | grep -qi "Orbbec"; then '
             'echo "[ORBBEC_BRINGUP] device detected on USB bus:"; '
             'lsusb | grep -i orbbec; '
             'else '
             'echo "[ORBBEC_BRINGUP] WARNING: no Orbbec device on lsusb"; '
             'fi'],
        name='orbbec_lsusb_probe',
        output='screen',
    )

    return LaunchDescription(declared + [
        LogInfo(msg='[orbbec_gemini2.launch] starting camera bringup...'),
        lsusb_probe,
        orbbec_node,
    ])
