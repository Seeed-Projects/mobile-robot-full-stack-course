#!/usr/bin/env python3
"""Publish a uniform-grey 1920x1080 image to /perception/cameras/front/image.

Used by scripts/m4/test_empty_frame_contract.sh. YOLO11n at confidence=0.25
should produce zero detections on this image.
"""
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class BlankImagePublisher(Node):
    def __init__(self):
        super().__init__('blank_image_publisher')
        self.pub = self.create_publisher(Image,
                                         '/perception/cameras/front/image',
                                         10)
        self.bridge = CvBridge()
        # 1920x1080, BGR8, uniform grey (YOLO11n should produce no detections)
        self.image = np.full((1080, 1920, 3), 128, dtype=np.uint8)
        self.count = 0
        # ~10 FPS keeps the test under timeout while letting YOLO process
        # multiple frames.
        self.timer = self.create_timer(0.1, self.publish)
        self.get_logger().info('Publishing blank 1920x1080 grey image to '
                               '/perception/cameras/front/image')

    def publish(self):
        self.count += 1
        msg = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_front'
        self.pub.publish(msg)
        if self.count % 20 == 0:
            self.get_logger().info(f'Published frame {self.count}')


def main():
    rclpy.init()
    node = BlankImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
