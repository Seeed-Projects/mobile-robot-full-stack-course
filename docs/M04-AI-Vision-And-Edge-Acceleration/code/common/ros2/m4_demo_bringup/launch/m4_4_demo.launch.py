# m4_4_demo.launch.py
#
# Top-level M4.4 demo launch:
#   1. Orbbec Gemini 2 camera         (bev_pose/launch/orbbec_gemini2.launch.py)
#   2. object_mask_node (P0 helper)  (bev_pose/launch/m4_pose_estimation.launch.py)
#   3. foundationpose_node           (bev_pose/launch/m4_pose_estimation.launch.py)
#   4. pose_visualizer               (this package, m4_demo_bringup)
#
# The web_server is started separately by run_m4_4_demo.sh, because the
# bash supervisor wants to own its lifecycle (PID/PGID/metadata).

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    FindPackageShare,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bev_pose_share = FindPackageShare('bev_pose')
    m4_bringup_share = FindPackageShare('m4_demo_bringup')

    return LaunchDescription([
        # ---- arguments ----
        DeclareLaunchArgument(
            'mesh_npz_path',
            default_value=os.path.join(os.environ.get('M4_CODE_ROOT', ''), 'models/m4/pose/processed/cup.npz'),
            description='Pre-computed mesh .npz.'),
        DeclareLaunchArgument(
            'frame_id', default_value='cup',
            description='tf2 child frame_id of the tracked object.'),
        DeclareLaunchArgument(
            'orbbec_serial_number', default_value='',
            description='Orbbec Gemini 2 serial number (preferred).'),
        DeclareLaunchArgument(
            'orbbec_usb_port', default_value='',
            description='Orbbec USB port hint (e.g. 2-1).'),

        # ---- 1. Orbbec camera bringup ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bev_pose_share, 'launch',
                                      'orbbec_gemini2.launch.py'])),
            launch_arguments={
                'orbbec_serial_number': LaunchConfiguration('orbbec_serial_number'),
                'orbbec_usb_port': LaunchConfiguration('orbbec_usb_port'),
                'align_color_to_depth': 'true',
            }.items(),
        ),

        # ---- 2 + 3. foundationpose + object_mask nodes ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bev_pose_share, 'launch',
                                      'm4_pose_estimation.launch.py'])),
            launch_arguments={
                'mesh_npz_path': LaunchConfiguration('mesh_npz_path'),
                'frame_id': LaunchConfiguration('frame_id'),
                'mask_auto_stop': '0',
            }.items(),
        ),

        # ---- 4. pose_visualizer ----
        Node(
            package='m4_demo_bringup',
            executable='pose_visualizer',
            name='pose_visualizer',
            output='screen',
            parameters=[{
                'rgb_topic': '/perception/cameras/front/image',
                'pose_topic': '/perception/object_pose',
                'output_topic': '/perception/demo/m4_4',
                'mesh_npz_path': LaunchConfiguration('mesh_npz_path'),
            }],
        ),
    ])
