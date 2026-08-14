#!/usr/bin/env python3
"""RGB-D 点云生成与 Open3D 可视化

从 RealSense 深度图和彩色图生成点云，并实时可视化。
"""

import open3d as o3d
import numpy as np
import pyrealsense2 as rs

class PointCloudVisualizer:
    """RGB-D 点云可视化器"""

    def __init__(self, width=1280, height=720, fps=30):
        """初始化 RealSense 管线

        Args:
            width: 图像宽度
            height: 图像高度
            fps: 帧率
        """
        self.pipeline = rs.pipeline()
        config = rs.config()

        # 配置深度流
        config.enable_stream(
            rs.stream.depth, width, height, rs.format.z16, fps
        )
        # 配置彩色流
        config.enable_stream(
            rs.stream.color, width, height, rs.format.rgb8, fps
        )

        # 启动管线
        self.profile = self.pipeline.start(config)

        # 获取深度传感器内参
        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print(f"深度比例: {depth_scale} (m per unit)")

        # 获取内参
        self.depth_intrinsics = (
            self.profile.get_stream(rs.stream.depth)
            .get_video_stream_profile()
            .get_intrinsics()
        )
        self.color_intrinsics = (
            self.profile.get_stream(rs.stream.color)
            .get_video_stream_profile()
            .get_intrinsics()
        )

        # 创建对齐器（深度对齐到彩色）
        self.align = rs.align(rs.stream.color)

        # Open3D 可视化器
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window("Point Cloud", width=1280, height=720)
        self.pcd = o3d.geometry.PointCloud()

        # 设置渲染选项
        opt = self.vis.get_render_option()
        opt.background_color = np.array([0.1, 0.1, 0.1])
        opt.point_size = 1.0

    def rs_intrinsics_to_matrix(self, intrinsics):
        """将 RealSense 内参转换为 3x3 矩阵

        Args:
            intrinsics: rs.intrinsics 对象

        Returns:
            3x3 numpy 数组
        """
        K = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ])
        return K

    def depth_to_pointcloud(self, depth_image, color_image, intrinsics):
        """从深度图和彩色图生成点云

        Args:
            depth_image: 深度图（uint16，单位 mm 或 m）
            color_image: 彩色图（uint8，RGB）
            intrinsics: rs.intrinsics 对象

        Returns:
            open3d.geometry.PointCloud
        """
        # 获取内参
        K = self.rs_intrinsics_to_matrix(intrinsics)
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        h, w = depth_image.shape

        # 创建像素坐标网格
        u, v = np.meshgrid(np.arange(w), np.arange(h))

        # 深度值转换为米
        z = depth_image.astype(np.float64) * 0.001  # mm → m

        # 过滤无效深度
        valid = z > 0
        z = z[valid]
        u = u[valid]
        v = v[valid]

        # 反投影到 3D
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # 组合点云
        points = np.stack([x, y, z], axis=-1)

        # 获取颜色
        colors = color_image[valid].astype(np.float64) / 255.0

        # 创建 Open3D 点云
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        return pcd

    def run(self):
        """运行实时可视化"""
        try:
            while True:
                # 等待帧
                frames = self.pipeline.wait_for_frames(5000)
                if not frames:
                    continue

                # 对齐深度到彩色
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                # 转换为 numpy 数组
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # 生成点云
                pcd = self.depth_to_pointcloud(
                    depth_image, color_image, self.color_intrinsics
                )

                # 降采样（提高渲染速度）
                pcd = pcd.voxel_down_sample(voxel_size=0.005)

                # 更新可视化
                self.vis.update_geometry(pcd)
                if not self.vis.poll_events():
                    break
                self.vis.update_renderer()

        except KeyboardInterrupt:
            print("\n停止可视化")
        finally:
            self.pipeline.stop()
            self.vis.destroy_window()


if __name__ == '__main__':
    visualizer = PointCloudVisualizer()
    visualizer.run()
