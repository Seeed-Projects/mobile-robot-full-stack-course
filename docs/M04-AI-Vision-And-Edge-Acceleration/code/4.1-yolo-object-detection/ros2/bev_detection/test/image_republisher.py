#!/usr/bin/env python3
"""Publish a single JPEG image as ROS topic for YOLO testing."""
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageRepublisher(Node):
    def __init__(self, image_path, topic='/perception/cameras/front/image'):
        super().__init__('image_republisher')
        self.pub = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()
        self.image = cv2.imread(image_path)
        if self.image is None:
            raise RuntimeError(f'Cannot load {image_path}')
        self.count = 0
        # Publish once per second, indefinite
        self.timer = self.create_timer(0.033, self.publish)  # ~30 FPS
        self.get_logger().info(f'Publishing {image_path} ({self.image.shape}) to {topic}')

    def publish(self):
        self.count += 1
        msg = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_front'
        self.pub.publish(msg)
        if self.count % 10 == 0:
            self.get_logger().info(f'Published frame {self.count}')


def main():
    if len(sys.argv) < 2:
        print('Usage: image_republisher.py <image_path>')
        sys.exit(1)
    rclpy.init()
    node = ImageRepublisher(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
