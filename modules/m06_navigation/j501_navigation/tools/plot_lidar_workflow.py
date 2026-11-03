#!/usr/bin/python3
"""Plot the actually surveyed SLAM map, measured-map route, and cone detour."""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from j501_navigation.grid_search import Grid


def main():
    records = PACKAGE / 'validation/lidar_workflow'
    grid = Grid(records / 'office.yaml', radius=0)
    survey = json.loads((records / 'survey.json').read_text())
    navigation = json.loads((records / 'navigation.json').read_text())
    plan = json.loads((records / 'global_plan.json').read_text())
    actual = np.array(navigation['actual_trajectory_map'])
    reference = np.array(plan['points'])
    survey_path = np.array(survey['slam_trajectory'])
    extent = [grid.origin[0], grid.origin[0]+grid.pixels.shape[1]*grid.resolution,
              grid.origin[1], grid.origin[1]+grid.pixels.shape[0]*grid.resolution]
    fig, axes = plt.subplots(1, 3, figsize=(17, 5.5))
    for ax in axes:
        ax.imshow(grid.pixels, origin='lower', cmap='gray', vmin=0, vmax=255, extent=extent)
        ax.set(xlabel='SLAM map X (m)', ylabel='SLAM map Y (m)')
    axes[0].plot(survey_path[:, 0], survey_path[:, 1], color='tab:blue', linewidth=1.3, label='Online SLAM survey')
    axes[0].scatter(0, 0, color='green', label='First scan / start')
    axes[0].set_title('Map measured from LiDAR, not rasterized scene')
    axes[0].legend(fontsize=8)
    for ax in axes[1:]:
        ax.plot(reference[:, 0], reference[:, 1], '--', color='green', linewidth=2, label='Measured-map A* reference')
        ax.plot(actual[:, 0], actual[:, 1], color='tab:blue', linewidth=2, label='Actual MPPI trajectory')
        ax.add_patch(plt.Rectangle((8.05, 4.2), .6, .6, color='orange', label='Cone added after mapping'))
        ax.scatter(9, 5.7, marker='x', s=70, color='red', label='Goal')
    axes[1].set_title('Restart, AMCL relocalization, then Nav2')
    axes[1].legend(fontsize=8, loc='lower left')
    axes[2].set(xlim=(7.2, 10.1), ylim=(3.0, 6.3), title='LiDAR obstacle detection / local detour')
    axes[2].legend(fontsize=8, loc='upper left')
    fig.suptitle(f"Measured-map navigation success={navigation['succeeded']}; actual goal error {navigation['actual_goal_error_m']:.3f} m")
    fig.tight_layout()
    output = PACKAGE / 'assets/lidar_workflow.png'
    fig.savefig(output, dpi=150)
    plt.close(fig)
    print(output)


if __name__ == '__main__':
    main()
