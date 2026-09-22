"""M4.1 complete launch: camera_sync + camera_adapter + YOLO detection.

Canonical owner: bev_detection (M4.1).

Usage:
  ros2 launch bev_detection m4_detection.launch.py

Notes:
  * GMSL path (use_camera_sync:=true) starts camera_sync_node from the legacy
    bev_camera_sync package, which lives in the pre-M4 BEV workspace and is
    NOT part of the M4 course workspace. Leave it false unless you have it.
  * use_viz / use_monitor default true and pull in the pre-M4 packages
    bev_visualization / bev_system_monitor. Pass them false for M4-only runs.
  * YOLO-only alternative: ros2 launch bev_detection yolo.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_camera_sync = LaunchConfiguration('use_camera_sync')
    use_yolo = LaunchConfiguration('use_yolo')
    use_bev = LaunchConfiguration('use_bev')
    use_viz = LaunchConfiguration('use_viz')
    use_monitor = LaunchConfiguration('use_monitor')

    # YOLO parameters
    yolo_model_path = LaunchConfiguration('yolo_model_path')
    yolo_image_topic = LaunchConfiguration('yolo_image_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_camera_sync', default_value='false',
                           description='Start camera_sync_node (set true if not running perception.launch.py)'),
        DeclareLaunchArgument('use_yolo', default_value='true',
                           description='Start YOLO detection'),
        DeclareLaunchArgument('use_bev', default_value='false',
                           description='Start BEVDet (default false for M4.1 YOLO-only)'),
        DeclareLaunchArgument('use_viz', default_value='true',
                           description='Start visualization'),
        DeclareLaunchArgument('use_monitor', default_value='true',
                           description='Start system monitor'),

        DeclareLaunchArgument('yolo_model_path',
                           default_value='/home/seeed/workspace/ros2_bev/modules/m04-ai-vision-and-edge-acceleration/models/m4/detection/engines/yolo11n_fp16.engine',
                           description='YOLO TensorRT engine path'),
        DeclareLaunchArgument('yolo_image_topic',
                           default_value='/perception/cameras/front/image',
                           description='YOLO input image topic'),

        # --- Camera Sync (optional) ---
        Node(
            package='bev_camera_sync',
            executable='camera_sync_node',
            name='camera_sync_node',
            output='screen',
            condition=IfCondition(use_camera_sync),
            parameters=[{
                'allowed_max_delta_ms': 5.0,
                'stale_drop_ms': 150.0,
            }],
        ),

        # --- Camera Adapter: FrameSet -> front camera Image ---
        Node(
            package='bev_detection',
            executable='camera_adapter_node',
            name='camera_adapter_node',
            output='screen',
            condition=IfCondition(use_yolo),
            parameters=[{
                'frameset_topic': '/bev/frameset',
                'front_image_topic': '/perception/cameras/front/image',
                'camera_id': 'front',
            }],
        ),

        # --- YOLO Detection ---
        Node(
            package='bev_detection',
            executable='yolo_trt_node',
            name='yolo_trt_node',
            output='screen',
            condition=IfCondition(use_yolo),
            parameters=[{
                'model_path': yolo_model_path,
                'class_names_path': '/home/seeed/workspace/ros2_bev/modules/m04-ai-vision-and-edge-acceleration/models/m4/detection/labels/coco.names',
                'image_topic': yolo_image_topic,
                'detections_topic': '/perception/detections',
                'debug_image_topic': '/perception/debug/detection_image',
                'input_width': 640,
                'input_height': 640,
                'num_classes': 80,
                'confidence_threshold': 0.25,
                'nms_threshold': 0.45,
                'publish_debug_image': True,
                'expected_trt_version': '10.3',
            }],
        ),

        # --- BEVDet (optional, disabled by default for M4.1) ---
        Node(
            package='bev_perception',
            executable='bevdet_node',
            name='bevdet_node',
            output='screen',
            condition=IfCondition(use_bev),
            parameters=[{
                'engine_path': '/home/seeed/workspace/ros2_bev/modules/m02-vision-foundations/2.4-bev-avm/models/engines/bevdet_one_lt_d_r50_256x704_fp16_trt10.3_sm87.engine',
                'onnx_path': '/home/seeed/workspace/ros2_bev/modules/m02-vision-foundations/2.4-bev-avm/models/onnx/bevdet_one_lt_d.onnx',
                'model_config': '/home/seeed/workspace/ros2_bev/modules/m02-vision-foundations/2.4-bev-avm/ros2/bevdet_vendor/cfgs/bevdet_lt_depth.yaml',
                'precision': 'fp16',
                'score_threshold': 0.25,
                'expected_trt_version': '10.3',
                'scene_gap_seconds': 2.0,
            }],
        ),

        # --- Visualization ---
        Node(
            package='bev_visualization',
            executable='bev_visualization',
            name='bev_visualization',
            output='screen',
            condition=IfCondition(use_viz),
        ),

        # --- System Monitor ---
        Node(
            package='bev_system_monitor',
            executable='bev_system_monitor',
            name='bev_system_monitor',
            output='screen',
            condition=IfCondition(use_monitor),
        ),

        LogInfo(msg='M4.1 YOLO detection launched'),
    ])
