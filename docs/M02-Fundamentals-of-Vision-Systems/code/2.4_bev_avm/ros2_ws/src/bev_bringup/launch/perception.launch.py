"""Perception stack launch: camera sync -> BEVDet -> monitor -> visualization.

Phase 2 usage (nuScenes bag):
  ros2 bag play datasets/bags/nuscenes_mini_60f --loop      (terminal 1)
  ros2 launch bev_bringup perception.launch.py               (terminal 2)
Phase 3+ usage (real cameras): use_real_camera:=true (bev_camera node publishes
the same /camera/* topics; sync + perception unchanged).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bev_perception_share = get_package_share_directory('bev_perception')  # noqa
    pkg_share = get_package_share_directory('bev_bringup')

    engine_path = LaunchConfiguration('engine_path')
    model_cfg = LaunchConfiguration('model_config')
    onnx_path = LaunchConfiguration('onnx_path')
    precision = LaunchConfiguration('precision')
    score_thr = LaunchConfiguration('score_threshold')
    allowed_delta = LaunchConfiguration('allowed_max_delta_ms')
    enable_viz = LaunchConfiguration('enable_viz')
    enable_monitor = LaunchConfiguration('enable_monitor')

    engine_default = PathJoinSubstitution([
        '/home/seeed/workspace/ros2_bev/models/engines',
        'bevdet_one_lt_d_r50_256x704_fp16_trt10.3_sm87.engine'])
    cfg_default = '/home/seeed/workspace/ros2_bev/ros2_ws/src/bevdet_vendor/cfgs/bevdet_lt_depth.yaml'
    onnx_default = '/home/seeed/workspace/ros2_bev/models/onnx/bevdet_one_lt_d.onnx'

    return LaunchDescription([
        DeclareLaunchArgument('engine_path', default_value=engine_default),
        DeclareLaunchArgument('model_config', default_value=cfg_default),
        DeclareLaunchArgument('onnx_path', default_value=onnx_default),
        DeclareLaunchArgument('precision', default_value='fp16'),
        DeclareLaunchArgument('score_threshold', default_value='0.25'),
        DeclareLaunchArgument('allowed_max_delta_ms', default_value='5.0'),
        DeclareLaunchArgument('enable_viz', default_value='true'),
        DeclareLaunchArgument('enable_monitor', default_value='true'),

        Node(
            package='bev_camera_sync',
            executable='camera_sync_node',
            name='camera_sync_node',
            output='screen',
            parameters=[{'allowed_max_delta_ms': 5.0,
                         'stale_drop_ms': 150.0}],
        ),
        Node(
            package='bev_perception',
            executable='bevdet_node',
            name='bevdet_node',
            output='screen',
            parameters=[{
                'engine_path': engine_path,
                'onnx_path': onnx_path,
                'model_config': model_cfg,
                'precision': precision,
                'score_threshold': score_thr,
                'expected_trt_version': '10.3',
                'scene_gap_seconds': 2.0,
            }],
        ),
        Node(
            package='bev_system_monitor',
            executable='bev_system_monitor',
            name='bev_system_monitor',
            output='screen',
            condition=IfCondition(enable_monitor),
        ),
        Node(
            package='bev_visualization',
            executable='bev_visualization',
            name='bev_visualization',
            output='screen',
            condition=IfCondition(enable_viz),
        ),
        LogInfo(msg='perception stack launched; system state on /bev/system_status'),
    ])