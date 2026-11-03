"""Insert / remove uncharted Gazebo obstacles using native Fortress services."""
import argparse
import json
import math
import subprocess
import time


def service(name, request_type, request):
    result = subprocess.run(['ign', 'service', '-s', f'/world/navigation/{name}',
        '--reqtype', request_type, '--reptype', 'ignition.msgs.Boolean', '--timeout', '5000',
        '--req', request], capture_output=True, text=True, timeout=8)
    if result.returncode or 'data: true' not in result.stdout:
        raise RuntimeError(f'Gazebo {name} failed: {result.stdout} {result.stderr}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default='temporary_obstacle')
    parser.add_argument('--x', type=float, default=-3.3)
    parser.add_argument('--y', type=float, default=-3.15)
    parser.add_argument('--size', nargs=2, type=float, default=[0.35, 0.35])
    parser.add_argument('--remove', action='store_true')
    parser.add_argument('--crossing', action='store_true', help='Move across the lane for a bounded experiment')
    parser.add_argument('--duration', type=float, default=20)
    args = parser.parse_args()
    if args.crossing and (not math.isfinite(args.duration) or args.duration <= 0):
        raise ValueError('Crossing duration must be finite and positive')
    if not args.name.replace('_', '').isalnum() or not all(math.isfinite(v) for v in [args.x, args.y, *args.size]):
        raise ValueError('Use a plain model name and finite geometry')
    if args.remove:
        service('remove', 'ignition.msgs.Entity', f'name: {json.dumps(args.name)} type: MODEL')
        print('Removed', args.name)
        return
    if min(args.size) <= 0:
        raise ValueError('Obstacle dimensions must be positive')
    sx, sy = args.size
    sdf = f'''<sdf version="1.8"><model name="{args.name}"><static>true</static>
      <pose>{args.x} {args.y} 0.4 0 0 0</pose><link name="link">
      <collision name="box"><geometry><box><size>{sx} {sy} 0.8</size></box></geometry></collision>
      <visual name="box"><geometry><box><size>{sx} {sy} 0.8</size></box></geometry>
      <material><ambient>1 .35 .05 1</ambient><diffuse>1 .35 .05 1</diffuse></material></visual>
      </link></model></sdf>'''
    service('create', 'ignition.msgs.EntityFactory', f'sdf: {json.dumps(sdf)} allow_renaming: false')
    print(f'Created {args.name} at ({args.x}, {args.y}); excluded from the static map.', flush=True)
    if args.crossing:
        beginning = time.monotonic()
        while time.monotonic()-beginning < args.duration:
            y = args.y+1.2*math.sin((time.monotonic()-beginning)*0.4)
            service('set_pose', 'ignition.msgs.Pose', f'name: {json.dumps(args.name)} position: {{x: {args.x} y: {y} z: 0.4}}')
            time.sleep(0.1)
