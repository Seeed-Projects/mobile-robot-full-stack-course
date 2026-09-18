"""YOLO TensorRT detection node launch.

Usage:
  ros2 launch bev_detection yolo.launch.py

Or with remapped topics:
  ros2 launch bev_detection yolo.launch.py image_topic:=/camera/front/image
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('bev_detection')

    model_path = LaunchConfiguration('model_path')
    class_names_path = LaunchConfiguration('class_names_path')
    image_topic = LaunchConfiguration('image_topic')
    detections_topic = LaunchConfiguration('detections_topic')
    debug_image_topic = LaunchConfiguration('debug_image_topic')
    input_width = LaunchConfiguration('input_width')
    input_height = LaunchConfiguration('input_height')
    num_classes = LaunchConfiguration('num_classes')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    nms_threshold = LaunchConfiguration('nms_threshold')
    publish_debug_image = LaunchConfiguration('publish_debug_image')

    # Defaults
    default_model = '/home/seeed/mobile-robot-full-stack-course/modules/m04-ai-vision-and-edge-acceleration/models/m4/detection/engines/yolo11n_fp16.engine'
    default_class_names = '/home/seeed/mobile-robot-full-stack-course/modules/m04-ai-vision-and-edge-acceleration/models/m4/detection/labels/coco.names'

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=default_model,
                             description='Path to TensorRT engine file'),
        DeclareLaunchArgument('class_names_path', default_value=default_class_names,
                             description='Path to class names file'),
        DeclareLaunchArgument('image_topic', default_value='/perception/cameras/front/image',
                             description='Input image topic'),
        DeclareLaunchArgument('detections_topic', default_value='/perception/detections',
                             description='Output detections topic'),
        DeclareLaunchArgument('debug_image_topic', default_value='/perception/debug/detection_image',
                             description='Debug image topic'),
        DeclareLaunchArgument('input_width', default_value='640',
                             description='Model input width'),
        DeclareLaunchArgument('input_height', default_value='640',
                             description='Model input height'),
        DeclareLaunchArgument('num_classes', default_value='80',
                             description='Number of object classes'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.25',
                             description='Confidence threshold'),
        DeclareLaunchArgument('nms_threshold', default_value='0.45',
                             description='NMS threshold'),
        DeclareLaunchArgument('publish_debug_image', default_value='true',
                             description='Publish debug image with bbox'),

        Node(
            package='bev_detection',
            executable='yolo_trt_node',
            name='yolo_trt_node',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'class_names_path': class_names_path,
                'image_topic': image_topic,
                'detections_topic': detections_topic,
                'debug_image_topic': debug_image_topic,
                'input_width': input_width,
                'input_height': input_height,
                'num_classes': num_classes,
                'confidence_threshold': confidence_threshold,
                'nms_threshold': nms_threshold,
                'publish_debug_image': publish_debug_image,
                'expected_trt_version': '10.3',
            }],
        ),
    ])
