#!/usr/bin/env python3
"""BEV 拼接 ROS 2 节点

订阅多路相机图像，进行 IPM 变换和 BEV 拼接，
发布拼接后的 BEV 图像。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque

from .ipm_calculator import IPMCalculator  # IPM 计算类，见 2.4.2 节

class BEVStitchNode(Node):
    """BEV 拼接 ROS 2 节点"""

    def __init__(self):
        """初始化节点"""
        super().__init__('bev_stitch_node')

        # 参数声明
        self.declare_parameter('bev_width', 1000)
        self.declare_parameter('bev_height', 1000)
        self.declare_parameter('bev_resolution', 0.01)
        self.declare_parameter('num_cameras', 4)

        self.bev_width = self.get_parameter('bev_width').value
        self.bev_height = self.get_parameter('bev_height').value
        self.bev_resolution = self.get_parameter('bev_resolution').value
        self.num_cameras = self.get_parameter('num_cameras').value

        # CV Bridge
        self.bridge = CvBridge()

        # 相机数据存储
        self.camera_data = {}
        camera_names = ['front', 'left', 'right', 'rear']

        for name in camera_names[:self.num_cameras]:
            self.camera_data[name] = {
                'image': None,
                'K': None,
                'H': None,  # IPM 单应性矩阵
                'weight': None,
            }

            # 订阅图像话题
            self.create_subscription(
                Image,
                f'/camera_{name}/image_raw',
                lambda msg, n=name: self.image_callback(msg, n),
                10
            )

            # 订阅相机信息话题
            self.create_subscription(
                CameraInfo,
                f'/camera_{name}/camera_info',
                lambda msg, n=name: self.camera_info_callback(msg, n),
                10
            )

        # BEV 图像发布者
        self.bev_pub = self.create_publisher(
            Image, '/bev/image_raw', 10
        )

        # 定时器：定期执行拼接
        self.timer = self.create_timer(0.033, self.stitch_timer_callback)

        self.get_logger().info(
            f'BEV 拼接节点已启动 '
            f'(BEV: {self.bev_width}×{self.bev_height}, '
            f'分辨率: {self.bev_resolution}m/px)'
        )

    def image_callback(self, msg, camera_name):
        """图像回调"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.camera_data[camera_name]['image'] = cv_image
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')

    def camera_info_callback(self, msg, camera_name):
        """相机信息回调"""
        K = np.array(msg.k).reshape(3, 3)
        self.camera_data[camera_name]['K'] = K

        # 如果已有内参，计算 IPM 矩阵
        # 实际应用中，安装参数应从 URDF 或参数服务器获取
        # 这里使用默认参数作为示例
        if self.camera_data[camera_name]['H'] is None:
            self._compute_ipm(camera_name, K)

    def _compute_ipm(self, camera_name, K):
        """计算 IPM 单应性矩阵

        Args:
            camera_name: 相机名称
            K: 内参矩阵
        """
        # 安装参数（实际应从参数或 URDF 获取）
        install_params = {
            'front': {'height': 1.2, 'pitch': -45, 'yaw': 0},
            'left':  {'height': 1.0, 'pitch': -45, 'yaw': 90},
            'right': {'height': 1.0, 'pitch': -45, 'yaw': -90},
            'rear':  {'height': 1.2, 'pitch': -45, 'yaw': 180},
        }

        params = install_params.get(camera_name)
        if params is None:
            return

        # 计算 IPM 矩阵（参考 2.4.2 节）
        ipm_calc = IPMCalculator(
            K, self.bev_width, self.bev_height, self.bev_resolution
        )
        H = ipm_calc.compute_homography(
            params['height'], params['pitch'], params['yaw']
        )
        self.camera_data[camera_name]['H'] = H

        # 创建权重图
        weight = self._create_weight_map()
        self.camera_data[camera_name]['weight'] = weight

    def _create_weight_map(self):
        """创建权重图"""
        h, w = self.bev_height, self.bev_width
        y, x = np.ogrid[:h, :w]
        center_y, center_x = h / 2, w / 2
        dist = np.sqrt((y - center_y)**2 + (x - center_x)**2)
        max_dist = np.sqrt(center_y**2 + center_x**2)
        weight = 1.0 - (dist / max_dist) * 0.5
        return np.clip(weight, 0, 1).astype(np.float32)

    def stitch_timer_callback(self):
        """定时拼接回调"""
        # 检查所有相机是否有图像
        ready_cameras = [
            name for name, data in self.camera_data.items()
            if data['image'] is not None and data['H'] is not None
        ]

        if not ready_cameras:
            return

        # 执行 IPM 变换和拼接
        result = np.zeros(
            (self.bev_height, self.bev_width, 3), dtype=np.float32
        )
        weight_sum = np.zeros(
            (self.bev_height, self.bev_width), dtype=np.float32
        )

        for name in ready_cameras:
            data = self.camera_data[name]
            # IPM 变换
            bev = cv2.warpPerspective(
                data['image'], data['H'],
                (self.bev_width, self.bev_height),
                flags=cv2.INTER_LINEAR
            )
            # 加权累加
            weight = data['weight']
            weight_3c = np.stack([weight] * 3, axis=-1)
            result += bev.astype(np.float32) * weight_3c
            weight_sum += weight

        # 归一化
        weight_sum = np.maximum(weight_sum, 1e-6)
        result = result / np.stack([weight_sum] * 3, axis=-1)
        result = result.astype(np.uint8)

        # 发布 BEV 图像
        bev_msg = self.bridge.cv2_to_imgmsg(result, 'bgr8')
        bev_msg.header.stamp = self.get_clock().now().to_msg()
        bev_msg.header.frame_id = 'bev_frame'
        self.bev_pub.publish(bev_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BEVStitchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
