import argparse
import json
from pathlib import Path

from .grid_search import Grid


def main():
    parser = argparse.ArgumentParser(description='Run standalone Dijkstra / A* / Theta* on the office map.')
    parser.add_argument('--map', type=Path)
    parser.add_argument('--algorithm', choices=['astar', 'dijkstra', 'theta_star'], default='astar')
    parser.add_argument('--start', nargs=2, type=float, default=[-4.5, -3.0])
    parser.add_argument('--goal', nargs=2, type=float, default=[4.5, 2.7])
    parser.add_argument('--radius', type=float, default=0.36)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not args.map:
        from ament_index_python.packages import get_package_share_directory
        args.map = Path(get_package_share_directory('j501_navigation')) / 'maps/office.yaml'
    report = Grid(args.map, radius=args.radius).search(args.start, args.goal, args.algorithm)
    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + '\n')
