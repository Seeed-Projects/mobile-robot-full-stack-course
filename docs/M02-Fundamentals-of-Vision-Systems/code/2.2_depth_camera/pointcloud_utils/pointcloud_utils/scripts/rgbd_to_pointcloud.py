"""Synchronize registered RGB-D images and publish PointXYZRGB."""

import numpy as np
import message_filters

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class RgbdToPointCloud(Node):
    def __init__(self):
        super().__init__('rgbd_to_pointcloud')
        self.declare_parameter('registered_depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('color_camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('output_topic', '/camera/points')
        self.declare_parameter('sync_slop_sec', 0.03)
        self.declare_parameter('depth_unit_m', 0.001)
        self.declare_parameter('pixel_stride', 2)

        self.bridge = CvBridge()
        self.color_info = None
        depth_topic = self.get_parameter('registered_depth_topic').value
        color_topic = self.get_parameter('color_topic').value
        info_topic = self.get_parameter('color_camera_info_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data)
        self.color_sub = message_filters.Subscriber(
            self, Image, color_topic, qos_profile=qos_profile_sensor_data)
        # Keep this object on self: a local synchronizer can be garbage-collected.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.color_sub], queue_size=10,
            slop=float(self.get_parameter('sync_slop_sec').value))
        self.sync.registerCallback(self.rgbd_callback)
        self.pc_pub = self.create_publisher(PointCloud2, output_topic, 10)

        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        self.point_dtype = np.dtype([
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])

    def info_callback(self, msg):
        self.color_info = msg

    def rgbd_callback(self, depth_msg, color_msg):
        if self.color_info is None:
            self.get_logger().warn('等待 color CameraInfo，跳过当前帧', throttle_duration_sec=2.0)
            return

        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='rgb8')
        if depth.ndim != 2 or depth.shape != color.shape[:2]:
            self.get_logger().error(
                f'深度 {depth.shape} 与 RGB {color.shape[:2]} 不一致；确认已启用 D2C 对齐。',
                throttle_duration_sec=2.0)
            return
        if self.color_info.width and (self.color_info.width, self.color_info.height) != (color.shape[1], color.shape[0]):
            self.get_logger().error('CameraInfo 与 RGB 分辨率不一致，拒绝发布点云', throttle_duration_sec=2.0)
            return

        if depth_msg.encoding == '16UC1':
            z = depth.astype(np.float32) * float(self.get_parameter('depth_unit_m').value)
        elif depth_msg.encoding == '32FC1':
            z = depth.astype(np.float32)
        else:
            self.get_logger().error(f'不支持的深度编码: {depth_msg.encoding}', throttle_duration_sec=2.0)
            return

        fx, fy = self.color_info.k[0], self.color_info.k[4]
        cx, cy = self.color_info.k[2], self.color_info.k[5]
        if fx <= 0 or fy <= 0:
            self.get_logger().error('CameraInfo 内参无效，拒绝发布点云', throttle_duration_sec=2.0)
            return

        stride = max(1, int(self.get_parameter('pixel_stride').value))
        sampled_z = z[::stride, ::stride]
        sampled_rgb = color[::stride, ::stride]
        valid = np.isfinite(sampled_z) & (sampled_z > 0)
        if not np.any(valid):
            self.get_logger().warn('当前帧没有有效深度，跳过发布', throttle_duration_sec=2.0)
            return

        v, u = np.mgrid[0:z.shape[0]:stride, 0:z.shape[1]:stride]
        zs = sampled_z[valid]
        points = np.empty(zs.size, dtype=self.point_dtype)
        points['x'] = (u[valid].astype(np.float32) - cx) * zs / fx
        points['y'] = (v[valid].astype(np.float32) - cy) * zs / fy
        points['z'] = zs
        rgb = sampled_rgb[valid].astype(np.uint32)
        points['rgb'] = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]

        header = Header()
        header.stamp = color_msg.header.stamp
        header.frame_id = self.color_info.header.frame_id or color_msg.header.frame_id
        cloud = point_cloud2.create_cloud(header, self.fields, points)
        cloud.is_dense = True
        self.pc_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = RgbdToPointCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
