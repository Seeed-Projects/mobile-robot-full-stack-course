"""
J501 GMSL2 RAW 相机 ROS 2 节点（Argus 路径）

发布话题：
  /camera/argus/image_raw  (sensor_msgs/Image)  — BGR 彩色图像
  /camera/argus/camera_info (sensor_msgs/CameraInfo) — 相机内参（标定后填充）

参数：
  sensor_id: 相机 ID（默认 0）
  width: 图像宽度（默认 1920）
  height: 图像高度（默认 1080）
  fps: 帧率（默认 30）
  publish_rate: 发布频率（默认 30.0 Hz）
  frame_id: TF 坐标系名称（默认 argus_camera_optical）
  wbmode: 白平衡模式（默认 auto）
  flip_method: 翻转方式（默认 0）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from .argus_camera import ArgusCamera, ArgusCameraError


class ArgusCameraNode(Node):
    """GMSL2 RAW 相机 ROS 2 节点（Argus 路径）"""

    def __init__(self):
        super().__init__('argus_camera')

        # 声明参数
        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('frame_id', 'argus_camera_optical')
        self.declare_parameter('wbmode', 'auto')
        self.declare_parameter('flip_method', 0)

        # 读取参数
        sensor_id = self.get_parameter('sensor_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        wbmode = self.get_parameter('wbmode').value
        flip_method = self.get_parameter('flip_method').value

        # 初始化相机
        try:
            self.cam = ArgusCamera(
                sensor_id=sensor_id, width=width, height=height,
                fps=fps, wbmode=wbmode, flip_method=flip_method
            )
            self.cam.start()
        except ArgusCameraError as e:
            self.get_logger().error(f"Argus 相机初始化失败: {e}")
            raise

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/argus/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/argus/camera_info', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_frame)
        self.get_logger().info(
            f'GMSL2 RAW 相机节点已启动: {width}x{height}@{fps}fps (sensor={sensor_id})'
        )

    def publish_frame(self):
        """定时回调：采集并发布一帧图像"""
        frame = self.cam.read()
        if frame is not None:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.pub.publish(msg)

    def destroy_node(self):
        """销毁节点时释放相机资源"""
        if hasattr(self, 'cam'):
            self.cam.stop()
        super().destroy_node()


def main(args=None):
    """节点入口函数"""
    rclpy.init(args=args)
    node = ArgusCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
