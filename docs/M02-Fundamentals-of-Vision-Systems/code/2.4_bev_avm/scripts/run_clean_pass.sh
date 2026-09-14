#!/usr/bin/env bash
# Clean full-pass validation: fresh stack + single bag pass + full capture.
set -eo pipefail
cd /home/seeed/workspace/ros2_bev
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export PATH=/usr/local/cuda-12.6/bin:$PATH

echo "[driver] killing all" >> /tmp/clean_pass.log
pkill -9 -f "ros2 bag play" 2>/dev/null || true
pkill -9 -f bevdet_node 2>/dev/null || true
pkill -9 -f camera_sync_node 2>/dev/null || true
pkill -9 -f bev_system_monitor 2>/dev/null || true
pkill -9 -f bev_visualization 2>/dev/null || true
pkill -9 -f "ros2 launch" 2>/dev/null || true
sleep 3

echo "[driver] starting stack" >> /tmp/clean_pass.log
export BEV_DEBUG_DUMP
export SYNC_DEBUG_DUMP
setsid nohup ros2 launch bev_bringup perception.launch.py model_config:=${MODEL_CFG:-/home/seeed/workspace/ros2_bev/ros2_ws/src/bevdet_vendor/cfgs/bevdet_lt_depth.yaml} > /tmp/clean_pass_launch.log 2>&1 < /dev/null &
STACK_PID=$!
sleep 6

echo "[driver] starting capture (45s)" >> /tmp/clean_pass.log
python3 tools/capture_objects.py /tmp/objects_clean.json 45 \
  > /tmp/capture_clean.log 2>&1 &
CAP_PID=$!

echo "[driver] playing bag once at rate 1.0" >> /tmp/clean_pass.log
ros2 bag play datasets/bags/nuscenes_mini_60f --rate 1.0 \
  > /tmp/clean_pass_bag.log 2>&1 || true

wait $CAP_PID || true
echo "[driver] capture done" >> /tmp/clean_pass.log
python3 - <<'EOF' >> /tmp/clean_pass.log 2>&1
import json
d = json.load(open('/tmp/objects_clean.json'))
print("frames:", len(d))
if not d:
    print("NO DATA")
else:
    counts = [len(f['objects']) for f in d]
    scores = [o['score'] for f in d for o in f['objects']]
    print("objs/frame:", min(counts), max(counts), round(sum(counts)/len(d),1))
    if scores:
        print("score max:", max(scores), "| >=0.4:", sum(1 for s in scores if s>=0.4))
    # per-frame top boxes
    for f in d[:8]:
        tops = sorted(f['objects'], key=lambda o:-o['score'])[:3]
        print(" t=%.1f" % (f['t_ns']/1e9), [(round(o['x'],1),round(o['y'],1),round(o['score'],2),o['label']) for o in tops])
EOF
echo "[driver] done" >> /tmp/clean_pass.log
kill $STACK_PID 2>/dev/null || true