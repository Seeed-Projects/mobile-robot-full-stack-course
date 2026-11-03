import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from j501_navigation.lidar_algorithms import euclidean_clusters, ground_plane, project_scan, voxel_filter


class LidarTests(unittest.TestCase):
    def test_ground_fit_rejects_elevated_objects_and_clusters_remain_separate(self):
        rng = np.random.default_rng(42)
        ground = rng.uniform(-4, 4, (800, 3))
        ground[:, 2] = .02*ground[:, 0] + rng.normal(0, .005, 800)
        left = rng.normal([1., 1., .6], [.06, .06, .06], (70, 3))
        right = rng.normal([-1., 1., .6], [.06, .06, .06], (70, 3))
        points = voxel_filter(np.concatenate([ground, left, right]), .04)
        plane, distances, fitted = ground_plane(points)
        self.assertTrue(fitted)
        self.assertGreater(plane[2], math.cos(math.radians(5)))
        above = points[distances > .07]
        clusters = euclidean_clusters(above, .18, 5)
        self.assertEqual(len(clusters), 2)
        centers = sorted(above[indices].mean(axis=0)[0] for indices in clusters)
        np.testing.assert_allclose(centers, [-1, 1], atol=.06)

    def test_sparse_ground_is_explicitly_reported_as_calibrated_fallback(self):
        plane, distances, fitted = ground_plane(np.array([[1., 1., .5], [1., 1., .6]]))
        self.assertFalse(fitted)
        np.testing.assert_allclose(plane, [0, 0, 1, 0])
        self.assertTrue(np.all(distances > .4))

    def test_projection_preserves_nearest_hit_and_excludes_ground_and_overhead(self):
        points = np.array([[2, 0, .2], [1, 0, .2], [0, 1, 0], [0, -1, 1.2], [-1, 0, .2]])
        ranges = project_scan(points)
        self.assertEqual(ranges[360], 1)
        self.assertEqual(ranges[0], 1)
        self.assertTrue(np.isinf(ranges[180]))
        self.assertTrue(np.isinf(ranges[540]))
        self.assertTrue(np.isinf(project_scan(np.empty((0, 3)))).all())

    def test_projection_uses_the_nearest_ray_angle_without_clockwise_bias(self):
        angles = np.radians([0.4, -0.4, 179.9, -179.9])
        points = np.column_stack([2*np.cos(angles), 2*np.sin(angles), np.full(4, .2)])
        scan = project_scan(points, bins=360)
        self.assertEqual(np.isfinite(scan).sum(), 2)
        self.assertAlmostEqual(float(scan[180]), 2.)
        self.assertAlmostEqual(float(scan[0]), 2.)
