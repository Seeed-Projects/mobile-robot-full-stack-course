"""Open3D viewer that accepts either packed-rgb or separate RGB fields."""

import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudViewer(Node):
    def __init__(self):
        import open3d as o3d
        super().__init__('open3d_viewer')
        self.o3d = o3d
        self.declare_parameter('pointcloud_topic', '/camera/depth_registered/points')
        topic = self.get_parameter('pointcloud_topic').value
        self.sub = self.create_subscription(PointCloud2, topic, self.pc_callback, qos_profile_sensor_data)
        self.lock = threading.Lock()
        self.latest_xyz = None
        self.latest_colors = None
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='RGB-D PointCloud', width=960, height=720)
        self.geometry = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.geometry)

    def pc_callback(self, msg):
        names = {field.name for field in msg.fields}
        if not {'x', 'y', 'z'} <= names:
            self.get_logger().error('点云缺少 x/y/z 字段', throttle_duration_sec=2.0)
            return
        data = point_cloud2.read_points(msg, skip_nans=True)
        if data.size == 0:
            return
        xyz = np.column_stack((data['x'], data['y'], data['z'])).astype(np.float64)
        valid = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)
        xyz = xyz[valid]
        if xyz.size == 0:
            return
        colors = None
        if 'rgb' in names:
            raw = np.asarray(data['rgb'][valid])
            packed = raw.view(np.uint32) if np.issubdtype(raw.dtype, np.floating) else raw.astype(np.uint32)
            colors = np.column_stack(((packed >> 16) & 255, (packed >> 8) & 255, packed & 255)) / 255.0
        elif {'r', 'g', 'b'} <= names:
            colors = np.column_stack((data['r'][valid], data['g'][valid], data['b'][valid])) / 255.0
        with self.lock:
            self.latest_xyz = xyz
            self.latest_colors = colors

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            with self.lock:
                if self.latest_xyz is not None:
                    self.geometry.points = self.o3d.utility.Vector3dVector(self.latest_xyz)
                    if self.latest_colors is not None:
                        self.geometry.colors = self.o3d.utility.Vector3dVector(self.latest_colors)
                    self.vis.update_geometry(self.geometry)
            self.vis.poll_events()
            self.vis.update_renderer()
        self.vis.destroy_window()


def main(args=None):
    rclpy.init(args=args)
    viewer = PointCloudViewer()
    try:
        viewer.run()
    finally:
        viewer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
