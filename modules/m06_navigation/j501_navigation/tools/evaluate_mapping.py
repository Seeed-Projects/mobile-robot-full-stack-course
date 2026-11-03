#!/usr/bin/python3
"""Offline simulation evaluation: observed occupied cells versus known box surfaces.

Scene geometry is used ONLY here, never by SLAM or odometry estimation.
This one-way surface metric does not measure unseen-wall completeness.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from j501_navigation.grid_search import Grid


def evaluate(map_file):
    grid = Grid(map_file, radius=0)
    metadata = json.loads(map_file.with_suffix('.metadata.json').read_text())
    x0, y0, angle = metadata['world_from_map']
    iy, ix = np.where(grid.pixels == 0)
    x, y = grid.origin[0]+(ix+.5)*grid.resolution, grid.origin[1]+(iy+.5)*grid.resolution
    points = np.column_stack([x0+np.cos(angle)*x-np.sin(angle)*y, y0+np.sin(angle)*x+np.cos(angle)*y])
    distances = np.full(len(points), np.inf)
    scene = yaml.safe_load((PACKAGE / 'scenes/office.yaml').read_text())
    for box in scene['boxes']:
        delta = np.abs(points-np.array(box['center']))-np.array(box['size'])/2
        outside = np.linalg.norm(np.maximum(delta, 0), axis=1)
        inside = np.minimum(np.maximum(delta[:, 0], delta[:, 1]), 0)
        distances = np.minimum(distances, np.abs(outside+inside))
    if not len(points):
        raise ValueError('No occupied cells in observed map')
    return dict(map=str(map_file), occupied_cells=len(points), resolution_m=grid.resolution,
        median_wall_error_m=float(np.median(distances)), p95_wall_error_m=float(np.percentile(distances, 95)),
        cells_over_15cm_ratio=float(np.mean(distances > .15)),
        note='Simulation evaluation only. Occupied cells to nearest scene-box surface; does not measure completeness or continuous pose accuracy.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = evaluate(args.map)
    print(json.dumps(result, indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
