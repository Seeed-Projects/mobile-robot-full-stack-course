"""Isaac ROS 3.2 Mustard graph with a project-owned 42-hypothesis config."""

import importlib.util
import json
import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


_CORE_PATH = os.path.join(
    get_package_share_directory('isaac_ros_foundationpose'),
    'launch', 'isaac_ros_foundationpose_core.launch.py')
_SPEC = importlib.util.spec_from_file_location('m4_4_official_fragment', _CORE_PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
_FRAGMENT = _MODULE.IsaacROSFoundationPoseLaunchFragment


def _build_graph(context):
    specs_path = context.perform_substitution(LaunchConfiguration('interface_specs_file'))
    with open(specs_path, encoding='utf-8') as handle:
        interface_specs = json.load(handle)
    nodes = _FRAGMENT.get_composable_nodes(interface_specs)
    nodes['foundationpose_node'] = ComposableNode(
        name='foundationpose_node',
        package='isaac_ros_foundationpose',
        plugin='nvidia::isaac_ros::foundationpose::FoundationPoseNode',
        parameters=[{
            'configuration_file': LaunchConfiguration('configuration_file'),
            # Collapse the six in-plane rotations to one per icosphere view.
            # max_hypothesis alone is only a cap and does not shrink the grid.
            'fixed_axis_angles': ['z_0'],
            'mesh_file_path': LaunchConfiguration('mesh_file_path'),
            'texture_path': LaunchConfiguration('texture_path'),
            'refine_engine_file_path': LaunchConfiguration('refine_engine_file_path'),
            'refine_input_tensor_names': ['input_tensor1', 'input_tensor2'],
            'refine_input_binding_names': ['input1', 'input2'],
            'refine_output_tensor_names': ['output_tensor1', 'output_tensor2'],
            'refine_output_binding_names': ['output1', 'output2'],
            'score_engine_file_path': LaunchConfiguration('score_engine_file_path'),
            'score_input_tensor_names': ['input_tensor1', 'input_tensor2'],
            'score_input_binding_names': ['input1', 'input2'],
            'score_output_tensor_names': ['output_tensor'],
            'score_output_binding_names': ['output1'],
        }],
        remappings=[
            ('pose_estimation/depth_image', 'depth_image'),
            ('pose_estimation/image', 'rgb/image_rect_color'),
            ('pose_estimation/camera_info', 'rgb/camera_info'),
            ('pose_estimation/segmentation', 'segmentation'),
            ('pose_estimation/output', 'output'),
        ],
    )
    return [ComposableNodeContainer(
        package='rclcpp_components',
        name='container',
        namespace='isaac_ros_examples',
        executable='component_container_mt',
        composable_node_descriptions=list(nodes.values()),
        output='screen',
    )]


def generate_launch_description():
    args = list(_FRAGMENT.get_launch_actions({'camera_resolution': {
        'width': 640, 'height': 480}}).values())
    args.extend([
        DeclareLaunchArgument('interface_specs_file'),
        DeclareLaunchArgument('configuration_file'),
    ])
    return launch.LaunchDescription(args + [OpaqueFunction(function=_build_graph)])
