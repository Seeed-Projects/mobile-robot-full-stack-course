#!/usr/bin/python3
"""Compare observed wheel-only and wheel/IMU EKF maps from separate simulation runs."""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from j501_navigation.grid_search import Grid


def main():
    folder = PACKAGE / 'validation/ekf_mapping'
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    for ax, name, title, metric in zip(axes, ['wheel_map', 'ekf_map'],
            ['Encoder-only map: observed wall distortion', 'Wheel velocity + IMU yaw-rate EKF'],
            ['map_quality_before.json', 'map_quality_after.json']):
        grid = Grid(folder / (name+'.yaml'), radius=0)
        result = json.loads((folder/metric).read_text())
        extent = [grid.origin[0], grid.origin[0]+grid.pixels.shape[1]*grid.resolution,
                  grid.origin[1], grid.origin[1]+grid.pixels.shape[0]*grid.resolution]
        ax.imshow(grid.pixels, origin='lower', cmap='gray', vmin=0, vmax=255, extent=extent)
        ax.set(xlim=(-1.8, 11.8), ylim=(-2.8, 9.0), xlabel='SLAM map X (m)', ylabel='SLAM map Y (m)',
               title=title+f"\nOccupied-cell wall-distance p95: {result['p95_wall_error_m']:.3f} m")
    fig.suptitle('Separate measured runs; scene geometry used only for offline evaluation')
    fig.tight_layout()
    output = PACKAGE / 'assets/mapping_drift_comparison.png'
    fig.savefig(output, dpi=150)
    plt.close(fig)
    print(output)


if __name__ == '__main__':
    main()
