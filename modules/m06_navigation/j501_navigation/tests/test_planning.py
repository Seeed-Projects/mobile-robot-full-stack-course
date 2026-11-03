import math
from pathlib import Path
import sys
import unittest

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from j501_navigation.grid_search import Grid
from j501_navigation.geometry import polygon_distance, rectangle


def small_grid(blocked):
    grid = Grid.__new__(Grid)
    grid.blocked = np.array(blocked, dtype=bool)
    grid.resolution, grid.origin = 1.0, [0.0, 0.0]
    return grid


class SearchTests(unittest.TestCase):
    def test_cannot_cross_a_blocked_diagonal_corner(self):
        grid = small_grid([[0, 1], [1, 0]])
        self.assertFalse(grid.visible((0, 0), (1, 1)))
        for algorithm in ['astar', 'dijkstra', 'theta_star']:
            with self.assertRaises(ValueError):
                grid.search([0.5, 0.5], [1.5, 1.5], algorithm)

    def test_complete_wall_is_unreachable(self):
        blocked = np.zeros((5, 5), dtype=bool)
        blocked[:, 2] = True
        grid = small_grid(blocked)
        for algorithm in ['astar', 'dijkstra', 'theta_star']:
            with self.assertRaises(ValueError):
                grid.search([0.5, 0.5], [4.5, 4.5], algorithm)

    def test_office_requires_a_detour_and_astar_matches_dijkstra(self):
        grid = Grid(PACKAGE / 'maps/office.yaml')
        start, goal = [-4.5, -3.0], [4.5, 2.7]
        self.assertFalse(grid.visible(grid.cell(start), grid.cell(goal)))
        rows = {a: grid.search(start, goal, a) for a in ['astar', 'dijkstra', 'theta_star']}
        self.assertAlmostEqual(rows['astar']['length_m'], rows['dijkstra']['length_m'], places=9)
        self.assertLess(rows['astar']['expanded'], rows['dijkstra']['expanded'])
        self.assertLessEqual(rows['theta_star']['length_m'], rows['astar']['length_m'])
        self.assertGreater(rows['astar']['length_m'], math.dist(start, goal))


class FootprintTests(unittest.TestCase):
    def test_axis_aligned_gap_and_overlap(self):
        self.assertAlmostEqual(polygon_distance(rectangle(0, 0, 0, 1, 1), rectangle(2, 0, 0, 1, 1)), 1)
        self.assertEqual(polygon_distance(rectangle(0, 0, 0, 1, 1), rectangle(0.2, 0.2, 0, 1, 1)), 0)

    def test_rotated_chassis_corner_changes_clearance(self):
        chassis = rectangle(0, 0, math.pi/4, 0.49, 0.49)
        obstacle = rectangle(1, 0, 0, 0.4, 0.4)
        self.assertAlmostEqual(polygon_distance(chassis, obstacle), 0.8-0.49/math.sqrt(2), places=9)


if __name__ == '__main__':
    unittest.main()
