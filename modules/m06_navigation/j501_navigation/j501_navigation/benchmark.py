"""Compare native global planners with fixed endpoints; no robot motion is commanded."""
import argparse
import json
from pathlib import Path

import rclpy
from .clients import Lab, PLANNERS, request_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', nargs=3, type=float, default=[-4.5, -3.0, 0.0])
    parser.add_argument('--goal', nargs=3, type=float, default=[4.5, 2.7, 0.0])
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--output', type=Path, default=Path('build/m06_results/global_planners.json'))
    args = parser.parse_args()
    rclpy.init(); node = Lab(); reports = []
    try:
        for planner in PLANNERS:
            for trial in range(args.trials):
                _, row = request_path(node, planner, args.goal, args.start)
                row['trial'] = trial
                reports.append(row)
                print(f"{planner}: trial={trial}, length={row['length_m']:.3f}m, planning={row['planning_time_s']:.6f}s", flush=True)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(reports, indent=2)+'\n')
        print(args.output)
    finally:
        node.destroy_node(); rclpy.shutdown()
