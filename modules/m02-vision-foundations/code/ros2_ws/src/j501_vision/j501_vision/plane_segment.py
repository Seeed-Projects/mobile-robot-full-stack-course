#!/usr/bin/env python3
"""RANSAC 平面分割 - 地面提取

从点云中分割出地面平面，用于导航和障碍物检测。
"""

import open3d as o3d
import numpy as np

class PlaneSegmenter:
    """RANSAC 平面分割器"""

    def __init__(self, distance_threshold=0.01, ransac_n=1000):
        """初始化平面分割器

        Args:
            distance_threshold: 点到平面的距离阈值（米）
            ransac_n: RANSAC 迭代次数
        """
        self.distance_threshold = distance_threshold
        self.ransac_n = ransac_n

    def segment_plane(self, pcd):
        """分割地面平面

        Args:
            pcd: Open3D 点云

        Returns:
            tuple: (plane_model, inliers)
                plane_model: [a, b, c, d] 平面方程系数
                inliers: 内点索引列表
        """
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=self.distance_threshold,
            ransac_n=self.ransac_n,
            num_iterations=100
        )

        a, b, c, d = plane_model
        print(f"平面方程: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")
        print(f"内点数: {len(inliers)} / {len(pcd.points)} "
              f"({len(inliers)/len(pcd.points)*100:.1f}%)")

        return plane_model, inliers

    def separate_ground(self, pcd):
        """分离地面和非地面点云

        Args:
            pcd: 原始点云

        Returns:
            tuple: (ground_pcd, obstacle_pcd)
        """
        plane_model, inliers = self.segment_plane(pcd)

        # 提取地面点云
        ground_pcd = pcd.select_by_index(inliers)
        ground_pcd.paint_uniform_color([0, 1, 0])  # 绿色

        # 提取障碍物点云
        obstacle_pcd = pcd.select_by_index(inliers, invert=True)
        obstacle_pcd.paint_uniform_color([1, 0, 0])  # 红色

        return ground_pcd, obstacle_pcd

    def visualize(self, ground_pcd, obstacle_pcd):
        """可视化分割结果"""
        # 合并点云用于显示
        combined = ground_pcd + obstacle_pcd

        # 创建坐标系
        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.5, origin=[0, 0, 0]
        )

        o3d.visualization.draw_geometries(
            [combined, coord],
            window_name="Ground Segmentation (Green=Ground, Red=Obstacle)"
        )


if __name__ == '__main__':
    # 加载点云
    pcd = o3d.io.read_point_cloud("/tmp/depth_pointcloud.ply")
    print(f"原始点云: {len(pcd.points)} 个点")

    # 降采样
    pcd = pcd.voxel_down_sample(voxel_size=0.01)
    print(f"降采样后: {len(pcd.points)} 个点")

    # 分割地面
    segmenter = PlaneSegmenter(distance_threshold=0.02, ransac_n=1000)
    ground, obstacle = segmenter.separate_ground(pcd)

    # 可视化
    segmenter.visualize(ground, obstacle)
