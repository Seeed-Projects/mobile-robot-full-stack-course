"""NumPy LiDAR perception: voxel reduction, constrained ground RANSAC, and Euclidean clustering."""
import math

import numpy as np
from scipy.spatial import cKDTree


def voxel_filter(points, size):
    if len(points) == 0:
        return points
    cells = np.floor(points/size).astype(np.int64)
    _, indices, inverse = np.unique(cells, axis=0, return_index=True, return_inverse=True)
    sums = np.zeros((len(indices), 3))
    np.add.at(sums, inverse, points)
    return sums / np.bincount(inverse)[:, None]


def ground_plane(points, threshold=0.04, max_tilt_deg=15, iterations=45):
    """Fit only a near-horizontal plane close to the base's ground height.

    If ground returns are sparse, use the calibrated horizontal ground plane,
    explicitly returning fitted=False instead of inventing a RANSAC success.
    """
    candidates = points[(points[:, 2] > -0.15) & (points[:, 2] < 0.20)]
    best, count, fitted = np.array([0., 0., 1., 0.]), 0, False
    rng = np.random.default_rng(7)
    if len(candidates) >= 15:
        # Bound work independently of the sensor resolution.
        candidates = candidates[::max(1, len(candidates)//1500)]
        for _ in range(iterations):
            a, b, c = candidates[rng.choice(len(candidates), 3, replace=False)]
            normal = np.cross(b-a, c-a)
            norm = np.linalg.norm(normal)
            if norm < 1e-8:
                continue
            normal /= norm
            if normal[2] < 0:
                normal *= -1
            d = -normal @ a
            if normal[2] < math.cos(math.radians(max_tilt_deg)) or abs(d) > 0.12:
                continue
            inliers = np.abs(candidates @ normal+d) < threshold
            if inliers.sum() > count:
                best, count = np.r_[normal, d], int(inliers.sum())
        fitted = count >= 15
    distances = points @ best[:3]+best[3]
    return best, distances, fitted


def euclidean_clusters(points, tolerance=0.18, min_points=5):
    """Radius-connected components using a KD-tree, without an O(N²) distance matrix."""
    if not len(points):
        return []
    neighbors = cKDTree(points).query_ball_point(points, tolerance)
    visited = np.zeros(len(points), dtype=bool)
    clusters = []
    for seed in range(len(points)):
        if visited[seed]:
            continue
        visited[seed] = True
        queue, cluster = [seed], []
        while queue:
            i = queue.pop()
            cluster.append(i)
            near = np.array(neighbors[i])
            near = near[~visited[near]]
            visited[near] = True
            queue.extend(near.tolist())
        if len(cluster) >= min_points:
            clusters.append(np.array(cluster))
    return clusters


def project_scan(points, bins=720, min_height=0.12, max_height=0.42, min_range=0.35, max_range=12.0):
    """Height-slice projection in base_footprint; empty angular bins remain infinity."""
    ranges = np.full(bins, np.inf, dtype=np.float32)
    radial = np.linalg.norm(points[:, :2], axis=1)
    valid = (points[:, 2] >= min_height) & (points[:, 2] <= max_height) & (radial >= min_range) & (radial <= max_range)
    angles = np.arctan2(points[valid, 1], points[valid, 0])
    # LaserScan angles refer to bin centers; flooring biases every ray clockwise.
    indices = np.floor((angles+math.pi)*bins/(2*math.pi)+0.5).astype(int) % bins
    np.minimum.at(ranges, indices, radial[valid])
    return ranges
