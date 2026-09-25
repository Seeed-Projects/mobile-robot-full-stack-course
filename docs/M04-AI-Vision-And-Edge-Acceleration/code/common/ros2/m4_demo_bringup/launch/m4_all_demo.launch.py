"""M4 unified demo launch — ONE shared pipeline for 4.1 + 4.2 + 4.3.

Why this file exists
--------------------
`m4_1_demo.launch.py`, `m4_2_demo.launch.py` and `m4_3_demo.launch.py` each
bring up their own pipeline, and the 4.2 include starts YOLO itself
(`bev_tracking/tracking_demo.launch.py` hardcodes `use_yolo:=true`). Running
the three demos side by side therefore duplicated YOLO inference on the same
camera and required three separate web servers on the same port.

This launch composes the SAME nodes exactly once:

    /perception/cameras/front/image
      └─ yolo_trt_node ──────────────► /perception/detections
         └─ debug_image_topic ───────► /perception/demo/m4_1     (4.1 view)
      └─ tracking_node ──────────────► /perception/tracks
         └─ tracking_visualizer ────► /perception/demo/m4_2     (4.2 view)
      └─ segmentation_node ──────────► /perception/semantic_mask
                                       /perception/drivable_mask
         └─ segmentation_visualizer ─► /perception/demo/m4_3     (4.3 view)

so 4.1 and 4.2 share a single detection pass, and one web hub server
(scripts/m4/run_m4_web_hub.sh) can preview all three overlay topics.

Camera ownership stays outside this file: pass camera_source:=existing and
let the bash supervisor own exactly one camera publisher.

Usage:
  ros2 launch m4_demo_bringup m4_all_demo.launch.py
  ros2 launch m4_demo_bringup m4_all_demo.launch.py \\
      enable_tracking:=true enable_segmentation:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from m4_demo_bringup.control_settings import load_settings


def generate_launch_description() -> LaunchDescription:
    pkg_demo = get_package_share_directory('m4_demo_bringup')
    pkg_detection = get_package_share_directory('bev_detection')
    pkg_tracking = get_package_share_directory('bev_tracking')
    pkg_seg = get_package_share_directory('bev_segmentation')

    config_path = os.path.join(pkg_demo, 'config', 'demo.yaml')
    tracking_config = os.path.join(pkg_tracking, 'config', 'bytetrack.yaml')
    seg_config = os.path.join(pkg_seg, 'config', 'segmentation.yaml')

    camera_topic = LaunchConfiguration('camera_topic')
    detections_topic = LaunchConfiguration('detections_topic')
    tracks_topic = LaunchConfiguration('tracks_topic')
    yolo_model_path = LaunchConfiguration('yolo_model_path')
    segmentation_max_fps = LaunchConfiguration('segmentation_max_fps')
    segmentation_view_mode = LaunchConfiguration('segmentation_view_mode')
    runtime_settings, runtime_warning = load_settings()
    if runtime_warning:
        print(f'm4_all_demo: {runtime_warning}; using course defaults')

    # ---- 4.1: YOLO detection (single instance, shared with 4.2) ----
    # Includes yolo.launch.py directly (not m4_detection.launch.py) because
    # only yolo.launch.py exposes debug_image_topic as an argument, which is
    # what makes /perception/demo/m4_1 the detection overlay.
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_detection, 'launch', 'yolo.launch.py')),
        launch_arguments={
            'image_topic': camera_topic,
            'detections_topic': detections_topic,
            'debug_image_topic': '/perception/demo/m4_1',
            'publish_debug_image': 'true',
            'model_path': yolo_model_path,
            'confidence_threshold': str(runtime_settings['confidence_threshold']),
            'nms_threshold': str(runtime_settings['nms_threshold']),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_detection')),
    )

    # ---- 4.2: tracking on the SAME detections ----
    # Started as bare nodes rather than via tracking_demo.launch.py, which
    # would unconditionally start a second YOLO instance.
    tracking_node = Node(
        package='bev_tracking',
        executable='tracking_node',
        name='tracking_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_tracking')),
        parameters=[tracking_config, {
            'input_topic': detections_topic,
            'output_topic': tracks_topic,
            'track_activation_threshold': runtime_settings['track_activation_threshold'],
            'lost_track_buffer': runtime_settings['lost_track_buffer'],
            'minimum_matching_threshold': runtime_settings['minimum_matching_threshold'],
            'minimum_consecutive_frames': runtime_settings['minimum_consecutive_frames'],
        }],
    )

    tracking_visualizer = Node(
        package='bev_tracking',
        executable='tracking_visualizer',
        name='tracking_visualizer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_tracking')),
        parameters=[{
            'image_topic': camera_topic,
            'tracks_topic': tracks_topic,
            'debug_image_topic': '/perception/demo/m4_2',
        }],
    )

    # ---- 4.3: segmentation ----
    seg_node = Node(
        package='bev_segmentation',
        executable='segmentation_node',
        name='segmentation_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_segmentation')),
        parameters=([seg_config] if os.path.isfile(seg_config) else []) + [{
            'input_image_topic': camera_topic,
            'semantic_mask_topic': '/perception/semantic_mask',
            'drivable_mask_topic': '/perception/drivable_mask',
            'selected_image_topic': '/perception/segmentation/source_image',
            # Hub shares CPU with detection, tracking and WebRTC.  The node
            # itself preserves original frame stamps for the frames it takes.
            'max_inference_fps': segmentation_max_fps,
        }],
    )

    seg_visualizer = Node(
        package='m4_demo_bringup',
        executable='segmentation_visualizer',
        name='segmentation_visualizer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_segmentation')),
        parameters=[config_path, {
            'image_topic': '/perception/segmentation/source_image',
            'semantic_topic': '/perception/semantic_mask',
            'drivable_topic': '/perception/drivable_mask',
            'debug_image_topic': '/perception/demo/m4_3',
            'view_mode': segmentation_view_mode,
            'max_width': 1920,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_topic', default_value='/perception/cameras/front/image',
            description='Input image topic (single source of truth)'),
        DeclareLaunchArgument(
            'detections_topic', default_value='/perception/detections',
            description='YOLO detections, shared by 4.1 and 4.2'),
        DeclareLaunchArgument(
            'tracks_topic', default_value='/perception/tracks',
            description='Tracker output with ids'),
        DeclareLaunchArgument(
            'yolo_model_path',
            default_value=os.path.join(os.environ.get('M4_CODE_ROOT', ''), 'models/m4/detection/engines/yolo11n_fp16.engine'),
            description='YOLO TensorRT engine path'),
        DeclareLaunchArgument(
            'enable_detection', default_value='true',
            description='Start the shared YOLO detector (required by 4.1 and 4.2)'),
        DeclareLaunchArgument(
            'enable_tracking', default_value='true',
            description='Start 4.2 tracking node + visualizer'),
        DeclareLaunchArgument(
            'enable_segmentation', default_value='true',
            description='Start 4.3 segmentation node + visualizer '
                        '(set false when the engine is not built yet)'),
        DeclareLaunchArgument(
            'segmentation_max_fps', default_value='10.0',
            description='M4.3 TensorRT inference cap in the shared Hub (1..30)'),
        DeclareLaunchArgument(
            'segmentation_view_mode', default_value='semantic',
            description='M4.3 view: original|semantic|drivable'),
        yolo_launch,
        tracking_node,
        tracking_visualizer,
        seg_node,
        seg_visualizer,
        LogInfo(msg='m4_all_demo: one YOLO, one tracking, one segmentation chain'),
    ])
