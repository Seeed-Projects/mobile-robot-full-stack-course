# m4_pose_estimation.launch.py
#
# Internal launch for bev_pose (foundationpose_node + object_mask_node).
# This is NOT what the user launches; demo integration goes through
# m4_demo_bringup/launch/m4_4_demo.launch.py.
#
# All topics use /perception/* namespaces so multiple demos cannot
# fight over the same source topics.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindPackage,
    FindPackageShare,
)
from launch_ros.actions import Node, PushRosNamespace
import os


def launch_setup(context, *args, **kwargs):
    pkg = FindPackageShare('bev_pose')

    config = PathJoinSubstitution([pkg, 'config', 'pose_estimation.yaml'])

    # --- object_mask_node (P0 helper, optional via flag) ---
    object_mask_node = Node(
        package='bev_pose',
        executable='object_mask_node',
        name='object_mask_node',
        output='screen',
        parameters=[
            config,
            {
                'mask_topic': '/perception/object_mask',
                'auto_stop_after_masks': LaunchConfiguration('mask_auto_stop'),
            },
        ],
    )

    # --- foundationpose_node ---
    fp_node = Node(
        package='bev_pose',
        executable='foundationpose_node',
        name='foundationpose_node',
        output='screen',
        parameters=[
            config,
            {
                'rgb_topic': '/perception/cameras/front/image',
                'depth_topic': '/perception/cameras/front/depth',
                'camera_info_topic': '/perception/cameras/front/camera_info',
                'mask_topic': '/perception/object_mask',
                'pose_topic': '/perception/object_pose',
                'det3d_topic': '/perception/object_poses_3d',
                'mesh_npz_path': LaunchConfiguration('mesh_npz_path'),
                'mesh_obj': LaunchConfiguration('mesh_obj'),
                'model_dir': os.path.join(os.environ.get('FOUNDATIONPOSE_DIR', ''), 'weights'),
                'frame_id': LaunchConfiguration('frame_id'),
                'camera_frame_id': 'camera_front',
            },
        ],
    )

    return [
        object_mask_node,
        fp_node,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'mesh_npz_path',
            default_value=os.path.join(os.environ.get('M4_CODE_ROOT', ''), 'models/m4/pose/processed/cup.npz'),
            description='Pre-computed mesh .npz produced by mesh_preprocessor.'),
        DeclareLaunchArgument(
            'mesh_obj',
            default_value=os.path.join(os.environ.get('M4_CODE_ROOT', ''), 'models/m4/pose/obj_models/cup.obj'),
            description='Original OBJ (FoundationPose reads tex; the .npz covers geometry).'),
        DeclareLaunchArgument(
            'frame_id', default_value='cup',
            description='tf2 child_frame_id of the tracked object.'),
        DeclareLaunchArgument(
            'mask_auto_stop', default_value='0',
            description='object_mask_node auto-stops after publishing this many masks. '
                        '0 = keep running.'),
        OpaqueFunction(function=launch_setup),
    ])
