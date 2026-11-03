#!/usr/bin/env python3
"""Offline Leader-Follower target geometry; publishes no ROS topic or action.

Leader pose is already expressed in a shared map. Offsets are in the
leader's body frame: x forward, y left. This is not a formation controller.
"""
import argparse
import json
import math


def formation_goal(leader, offset):
    x, y, yaw = leader
    dx, dy = offset
    if not all(math.isfinite(v) for v in [x, y, yaw, dx, dy]):
        raise ValueError('Pose and offset must be finite')
    return [x+math.cos(yaw)*dx-math.sin(yaw)*dy,
            y+math.sin(yaw)*dx+math.cos(yaw)*dy,
            math.atan2(math.sin(yaw), math.cos(yaw))]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--leader', nargs=3, type=float, required=True, metavar=('X', 'Y', 'YAW_RAD'))
    parser.add_argument('--offset', nargs=2, type=float, default=[-1.2, 0.0], metavar=('DX', 'DY'))
    args = parser.parse_args()
    x, y, yaw = formation_goal(args.leader, args.offset)
    print(json.dumps(dict(frame_id='map', x=x, y=y, yaw=yaw,
        note='Offline target geometry only; validate frames, free space, and freshness before sending any navigation goal.'), indent=2))


if __name__ == '__main__':
    main()
