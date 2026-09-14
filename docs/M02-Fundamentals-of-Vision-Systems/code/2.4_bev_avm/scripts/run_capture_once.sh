#!/usr/bin/env bash
# One-shot capture driver: restart bag in loop mode + capture objects.
# Writing everything to files so it can run detached and inspected later.
set -eo pipefail
cd /home/seeed/workspace/ros2_bev
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

echo "[driver] killing old players" >> /tmp/capture_driver.log
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 2

echo "[driver] starting bag loop" >> /tmp/capture_driver.log
setsid nohup ros2 bag play datasets/bags/nuscenes_mini_60f --loop --rate 1.0 \
  > /tmp/bagplay3.log 2>&1 < /dev/null &
echo "  bag pid $!" >> /tmp/capture_driver.log

sleep 12
echo "[driver] capturing" >> /tmp/capture_driver.log
python3 tools/capture_objects.py /tmp/objects_loop1.json 40 >> /tmp/capture_driver.log 2>&1
echo "[driver] capture rc=$?" >> /tmp/capture_driver.log

echo "[driver] analyzing" >> /tmp/capture_driver.log
python3 - <<'EOF' >> /tmp/capture_driver.log 2>&1
import json
d = json.load(open('/tmp/objects_loop1.json'))
print("frames:", len(d))
nobj = [len(f['objects']) for f in d]
if d:
    print("objs/frame min/max/avg:", min(nobj), max(nobj), round(sum(nobj)/len(d), 2))
    print("total:", sum(nobj))
    for f in d[:4]:
        print("  f t=%.1f:" % (f['t_ns']/1e9), [(round(o['x'],1), round(o['y'],1), round(o['score'],2), o['label']) for o in f['objects']])
EOF
echo "[driver] done" >> /tmp/capture_driver.log