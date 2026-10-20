#!/usr/bin/env python3
"""Inspect a prepared Jetson project without importing models or opening devices."""
import argparse
import json
from pathlib import Path
import shutil
import socket
import subprocess
import sys


def preflight(project, chassis, task):
    project, chassis, task = map(lambda p: Path(p).resolve(), (project, chassis, task))
    result = {'project': str(project), 'disk_free_gib': shutil.disk_usage(project).free / 2**30,
              'ffmpeg': shutil.which('ffmpeg'), 'errors': [], 'ports': {}}
    errors = result['errors']
    if result['disk_free_gib'] < 3.5:
        errors.append('Demo start requires at least 3.5 GiB disk headroom')
    if Path('/proc/meminfo').exists():
        memory = next(line.split()[1] for line in Path('/proc/meminfo').read_text().splitlines() if line.startswith('MemAvailable:'))
        result['memory_available_gib'] = int(memory) * 1024 / 2**30
        if result['memory_available_gib'] < 3:
            errors.append('Less than 3 GiB available memory')
    for name, path in [('chassis', chassis), ('task', task)]:
        if not path.is_file():
            errors.append(f'Missing {name}: {path}')
    if chassis.is_file():
        config = json.loads(chassis.read_text())
        profile = Path(config['sdk_chassis_config'])
        if not profile.is_file():
            errors.append('Missing geometry JSON (not a requirement to run hardware SDK)')
        else:
            result['geometry_matches'] = config['geometry'] == json.loads(profile.read_text())['geometry']
            if not result['geometry_matches']:
                errors.append('Geometry JSON differs from chassis configuration')
        if not (chassis.parent / config['asset_manifest']).is_file():
            errors.append('Missing CAD manifest')
    metadata = "import sys,json,importlib.metadata as m; print(json.dumps({'python':sys.version.split()[0], 'packages':{n:m.version(n) for n in sys.argv[1:]}}))"
    for key, executable, packages in [
        ('model_environment', project / '.venv-lightnav-py310/bin/python', ['torch', 'torchvision', 'transformers', 'numpy']),
        ('demo_environment', project / 'mujoco_demo/.venv-cpu/bin/python', ['mujoco', 'numpy', 'aiohttp'])
    ]:
        if not executable.is_file():
            errors.append('Missing environment: ' + str(executable))
            continue
        process = subprocess.run([str(executable), '-c', metadata, *packages], capture_output=True, text=True, timeout=10)
        if process.returncode:
            errors.append('Environment metadata failed: ' + key)
            result[key] = process.stderr[-1000:]
        else:
            result[key] = json.loads(process.stdout)
    for port in [8050, 8091]:
        with socket.socket() as probe:
            probe.settimeout(.3)
            result['ports'][str(port)] = {'accepts_tcp': probe.connect_ex(('127.0.0.1', port)) == 0}
    result['ok'] = not errors
    result['notes'] = ['TCP open is not model readiness; inspect managed status and Demo health.',
                       'This check never imports torch/mujoco, initializes CUDA, logs into the model or controls motion.']
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project', type=Path, required=True)
    p.add_argument('--chassis-config', type=Path, required=True)
    p.add_argument('--task-config', type=Path, required=True)
    a = p.parse_args()
    result = preflight(a.project, a.chassis_config, a.task_config)
    print(json.dumps(result, indent=2))
    sys.exit(0 if result['ok'] else 1)
