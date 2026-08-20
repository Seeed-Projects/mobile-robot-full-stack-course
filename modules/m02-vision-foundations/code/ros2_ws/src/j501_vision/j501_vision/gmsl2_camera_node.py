#!/usr/bin/env python3
"""GMSL2 YUV 相机 ROS 2 节点（V4L2 路径）

适用于 SG3S-ISX031C-GMSL2F 等内置 ISP 的 YUV 输出相机：
从 /dev/videoN（YUYV）读帧，发布 sensor_msgs/Image（bgr8）与 CameraInfo。

用法（每路相机一个实例，用命名空间区分）：
  ros2 run j501_vision gmsl2_camera_node --ros-args \
      -r __ns:=/camera_front -p video_device:=/dev/video0
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class GMSL2CameraNode(Node):
    """V4L2 YUV 相机节点"""

    def __init__(self):
        super().__init__('gmsl2_camera_node')

        # 可配置参数
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1536)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'camera')

        device = self.get_parameter('video_device').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value

        # 打开 V4L2 设备（YUYV 直通，不经过 Jetson Argus ISP）
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f'无法打开相机设备: {device}')
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f'已打开相机: {device}，实际输出 {actual_w}x{actual_h}'
            '（ISX031 会忽略高度请求，固定输出 1536）')

        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, 'image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)

        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('读帧失败，检查相机链路')
            return

        stamp = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id
        self.img_pub.publish(img_msg)

        # 未标定时 CameraInfo 仅携带分辨率，标定后由 camera_info_manager 加载
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = frame.shape[1]
        info.height = frame.shape[0]
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = GMSL2CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
