#!/usr/bin/python3
"""Prepare an optional, unprivileged Nav2 dependency overlay without installing packages."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import gzip
import json
from pathlib import Path
import re
import subprocess
from urllib.request import urlopen


def prepare(output):
    output.mkdir(parents=True, exist_ok=True)
    (output / 'COLCON_IGNORE').touch()
    debs = output / 'debs'
    root = output / 'root'
    debs.mkdir(exist_ok=True)
    root.mkdir(exist_ok=True)
    plan = subprocess.run(
        ['apt-get', '--print-uris', '--yes', '--download-only', '-o', 'Debug::NoLocking=1',
         'install', 'ros-humble-navigation2', 'ros-humble-nav2-bringup', 'ros-humble-slam-toolbox', 'ros-humble-robot-localization', 'python3-scipy', 'python3-matplotlib'],
        check=True, capture_output=True, text=True,
    ).stdout
    packages = re.findall(r"^'([^']+)' (\S+) (\d+) MD5Sum:(\w+)$", plan, re.MULTILINE)
    # Installed APT indexes can refer to ROS builds that the rolling repository replaced.
    # Read fresh metadata through a TLS mirror rather than weakening TLS validation.
    mirror = 'https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/'
    architecture = subprocess.check_output(['dpkg', '--print-architecture'], text=True).strip()
    multiarch = {'amd64': 'x86_64-linux-gnu', 'arm64': 'aarch64-linux-gnu'}[architecture]
    cache_index = output / 'Packages.gz'
    errors = []
    for endpoint in [mirror, 'https://mirrors.ustc.edu.cn/ros2/ubuntu/']:
        try:
            with urlopen(endpoint + f'dists/jammy/main/binary-{architecture}/Packages.gz', timeout=15) as response:
                data = response.read()
            gzip.decompress(data)
            cache_index.write_bytes(data)
            mirror = endpoint
            break
        except Exception as exc:
            errors.append(str(exc))
    else:
        if not cache_index.exists():
            raise RuntimeError(f'Cannot fetch ROS package metadata: {errors}')
        print('Using previously downloaded package metadata.', flush=True)
    text = gzip.decompress(cache_index.read_bytes()).decode()
    index = {}
    for block in text.split('\n\n'):
        fields = dict(line.split(': ', 1) for line in block.splitlines() if ': ' in line and not line.startswith(' '))
        if 'Package' in fields:
            index[fields['Package']] = fields
    resolved = []
    for url, name, size, md5 in packages:
        package_name = name.split('_')[0]
        if package_name.startswith('ros-humble-'):
            entry = index[package_name]
            url = mirror + entry['Filename']
            name, size, md5 = Path(entry['Filename']).name, entry['Size'], entry['MD5sum']
            expected_sha = entry['SHA256']
        else:
            expected_sha = None
        resolved.append((url, name, size, md5, expected_sha))
    # New Nav2 builds use the out-of-line diagnostic_updater ABI. Older installed
    # Humble packages satisfy APT's unversioned dependency but lack that vtable.
    for package_name in ['ros-humble-diagnostic-updater']:
        entry = index[package_name]
        row = (mirror + entry['Filename'], Path(entry['Filename']).name, entry['Size'], entry['MD5sum'], entry['SHA256'])
        if not any(item[1] == row[1] for item in resolved):
            resolved.append(row)
    packages = resolved
    print(f'Preparing {len(packages)} packages in {output}; system packages are unchanged.', flush=True)

    def download(package):
        url, name, size, md5, expected_sha = package
        path = debs / name
        if not path.exists() or hashlib.md5(path.read_bytes()).hexdigest() != md5:
            errors = []
            candidates = [url.replace('http://', 'https://'), url]
            if 'cn.archive.ubuntu.com' in url:
                candidates.insert(0, url.replace('http://cn.archive.ubuntu.com', 'https://archive.ubuntu.com'))
            for candidate in candidates:
                try:
                    with urlopen(candidate, timeout=40) as response:
                        data = response.read()
                    if len(data) != int(size) or hashlib.md5(data).hexdigest() != md5:
                        raise ValueError(f'Package integrity check failed: {name}')
                    if expected_sha and hashlib.sha256(data).hexdigest() != expected_sha:
                        raise ValueError(f'SHA-256 mismatch: {name}')
                    path.write_bytes(data)
                    break
                except Exception as exc:
                    errors.append(str(exc))
            else:
                raise RuntimeError(f'Cannot download {name}: {errors}')
        return {'file': name, 'url': url, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}

    records = []
    with ThreadPoolExecutor(max_workers=6) as pool:
        for row in pool.map(download, packages):
            records.append(row)
            print(f"Downloaded {len(records)}/{len(packages)}: {row['file']}", flush=True)
    for row in records:
        subprocess.run(['dpkg-deb', '-x', str(debs / row['file']), str(root)], check=True)
    # Base ROS is already installed; extend its search paths with the extracted packages.
    prefix = root / 'opt/ros/humble'
    script = f'''# Generated optional Nav2 overlay. Source after /opt/ros/humble/setup.bash.
export AMENT_PREFIX_PATH="{prefix}:${{AMENT_PREFIX_PATH:-}}"
export CMAKE_PREFIX_PATH="{prefix}:${{CMAKE_PREFIX_PATH:-}}"
export LD_LIBRARY_PATH="{prefix}/lib:{root}/usr/lib:{root}/usr/lib/{multiarch}:${{LD_LIBRARY_PATH:-}}"
export PYTHONPATH="{prefix}/local/lib/python3.10/dist-packages:{prefix}/lib/python3.10/site-packages:${{PYTHONPATH:-}}"
export PATH="{prefix}/bin:${{PATH}}"
'''
    (output / 'setup.bash').write_text(script)
    (prefix / 'local_setup.bash').write_text(script)
    (output / 'packages.json').write_text(json.dumps(records, indent=2) + '\n')
    print(f"Ready: source {output / 'setup.bash'}", flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    prepare(args.output.resolve())
