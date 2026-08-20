"""
Argus 相机 Launch 文件

启动 GMSL2 RAW 相机节点（Argus 路径），支持参数配置。

使用方式：
  # 默认参数启动
  ros2 launch j501_vision argus_camera.launch.py

  # 自定义参数启动
  ros2 launch j501_vision argus_camera.launch.py sensor_id:=0 width:=1280 height:=720 fps:=60
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成 Launch 描述"""

    # 声明可配置参数
    sensor_id_arg = DeclareLaunchArgument(
        'sensor_id', default_value='0',
        description='相机 ID (0 或 1)'
    )
    width_arg = DeclareLaunchArgument(
        'width', default_value='1920',
        description='图像宽度'
    )
    height_arg = DeclareLaunchArgument(
        'height', default_value='1080',
        description='图像高度'
    )
    fps_arg = DeclareLaunchArgument(
        'fps', default_value='30',
        description='帧率'
    )
    wbmode_arg = DeclareLaunchArgument(
        'wbmode', default_value='auto',
        description='白平衡模式'
    )

    # GMSL2 RAW 相机节点（Argus 路径）
    argus_camera_node = Node(
        package='j501_vision',
        executable='argus_camera_node',
        name='argus_camera',
        parameters=[{
            'sensor_id': LaunchConfiguration('sensor_id'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
            'wbmode': LaunchConfiguration('wbmode'),
            'publish_rate': 30.0,
            'frame_id': 'argus_camera_optical',
        }],
        output='screen',
    )

    return LaunchDescription([
        sensor_id_arg,
        width_arg,
        height_arg,
        fps_arg,
        wbmode_arg,
        argus_camera_node,
    ])
