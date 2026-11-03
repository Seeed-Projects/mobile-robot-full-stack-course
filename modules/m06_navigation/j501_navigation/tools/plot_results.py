#!/usr/bin/python3
"""Plot measured native planner output and the actual local-avoidance trajectory."""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from j501_navigation.grid_search import Grid


def main():
    grid = Grid(PACKAGE / 'maps/office.yaml', radius=0)
    records = json.loads((PACKAGE / 'validation/nav2_global_planners.json').read_text())
    local = json.loads((PACKAGE / 'validation/local_mppi_obstacle.json').read_text())
    scene = yaml.safe_load((PACKAGE / 'scenes/office.yaml').read_text())
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    extent = [grid.origin[0], grid.origin[0]+grid.pixels.shape[1]*grid.resolution,
              grid.origin[1], grid.origin[1]+grid.pixels.shape[0]*grid.resolution]
    for ax in axes:
        ax.imshow(grid.pixels, origin='lower', cmap='gray', vmin=0, vmax=255, extent=extent)
        ax.set(xlabel='map X (m)', ylabel='map Y (m)')
    for name, label in [('astar', 'A*'), ('dijkstra', 'Dijkstra'), ('theta_star', 'Theta*')]:
        row = next(r for r in records if r['planner'] == name)
        points = np.array(row['points'])
        axes[0].plot(points[:, 0], points[:, 1], linewidth=1.8, label=f"{label}: {row['length_m']:.2f} m")
    axes[0].scatter(*scene['initial_pose'][:2], color='green', label='Start')
    axes[0].scatter(*scene['goals']['cross_rooms'][:2], marker='x', color='red', label='Goal')
    axes[0].set_title('Native Nav2 global plans (fixed endpoints)')
    axes[0].legend(fontsize=8)
    x, y, sx, sy = local['temporary_obstacle']
    axes[1].add_patch(plt.Rectangle((x-sx/2, y-sy/2), sx, sy, color='orange', label='Uncharted obstacle'))
    axes[1].plot([-4.5, -2.0], [-3, -3], '--', color='green', label='Fixed reference')
    actual = np.array(local['actual_trajectory'])
    axes[1].plot(actual[:, 0], actual[:, 1], color='tab:blue', linewidth=2, label='Actual MPPI trajectory')
    axes[1].set(xlim=(-5.1, -1.5), ylim=(-4.1, -1.9), title=f"MPPI local avoidance; Action success={local['succeeded']}")
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    file = PACKAGE / 'assets/planning_results.png'
    fig.savefig(file, dpi=160)
    plt.close(fig)
    print(file)
    cone_record = PACKAGE / 'validation/navigation_cone_mppi.json'
    if cone_record.exists():
        result = json.loads(cone_record.read_text())
        cone = scene['unmapped_cones'][0]
        reference = np.array(next(r for r in records if r['planner'] == 'astar')['points'])
        actual = np.array(result['actual_trajectory'])
        fig, axes = plt.subplots(1, 2, figsize=(13, 6))
        for ax in axes:
            ax.imshow(grid.pixels, origin='lower', cmap='gray', vmin=0, vmax=255, extent=extent)
            ax.plot(reference[:, 0], reference[:, 1], '--', color='green', linewidth=2, label='Static-map A* reference')
            ax.plot(actual[:, 0], actual[:, 1], color='tab:blue', linewidth=2, label='Actual MPPI trajectory')
            x, y = cone['center']
            sx, sy = cone['size']
            ax.add_patch(plt.Rectangle((x-sx/2, y-sy/2), sx, sy, color='orange', label='Unmapped cone base'))
            ax.scatter(*scene['goals']['cross_rooms'][:2], marker='x', s=80, color='red', label='Goal')
            ax.set(xlabel='map X (m)', ylabel='map Y (m)')
        axes[0].scatter(*scene['initial_pose'][:2], color='green', label='Start')
        axes[0].set_title('Full route with the arrival cone')
        axes[0].legend(fontsize=8, loc='lower left')
        axes[1].set(xlim=(2.9, 5.5), ylim=(0.3, 3.3), title='Local avoidance before the goal')
        axes[1].legend(fontsize=8, loc='upper left')
        fig.suptitle(f"MPPI: success={result['succeeded']}, actual goal error={result['actual_goal_error_m']:.3f} m")
        fig.tight_layout()
        file = PACKAGE / 'assets/arrival_cone_avoidance.png'
        fig.savefig(file, dpi=160)
        plt.close(fig)
        print(file)


if __name__ == '__main__':
    main()
