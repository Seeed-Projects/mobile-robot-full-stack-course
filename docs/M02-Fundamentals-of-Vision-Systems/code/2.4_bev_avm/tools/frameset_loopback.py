#!/usr/bin/env python3
"""FrameSet CDR round-trip integrity test (python pub/sub, both sides)."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from bev_interfaces.msg import FrameSet
from sensor_msgs.msg import Image, CameraInfo

rclpy.init()
pub_node = Node('fs_pub')
sub_node = Node('fs_sub')

qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
pub = pub_node.create_publisher(FrameSet, '/fs_test', qos)
got = []
sub_node.create_subscription(FrameSet, '/fs_test', lambda m: got.append(m), qos)

fs = FrameSet()
fs.header.frame_id = 'base_link'
for i in range(6):
    img = Image()
    img.header.stamp.sec = 1
    img.height = 900
    img.width = 400
    img.encoding = 'bgr8'
    img.step = 400 * 3
    arr = np.zeros((900, 400, 3), dtype=np.uint8)
    for c in range(3):
        arr[:, :, c] = 10 * i + c
    img.data = arr.tobytes()
    fs.camera_ids.append(f'cam{i}')
    fs.images.append(img)
    ci = CameraInfo()
    ci.header.stamp.sec = 1
    fs.camera_infos.append(ci)

end = time.time() + 15
while time.time() < end and not got:
    pub.publish(fs)
    rclpy.spin_once(sub_node, timeout_sec=0.1)

if not got:
    print("NO MESSAGE RECEIVED")
else:
    m = got[-1]
    print("received frameset, images:", len(m.images))
    ok = True
    for i in range(6):
        data = bytes(m.images[i].data)
        arr = np.frombuffer(data, np.uint8)
        unique = set(np.unique(arr).tolist())
        expect = {10 * i, 10 * i + 1, 10 * i + 2}
        good = (unique == expect) and (len(arr) == 900 * 400 * 3)
        ok &= good
        print(f"  img{i}: size={len(arr)} unique={sorted(unique)[:6]} match={good}")
    print("ROUNDTRIP OK" if ok else "ROUNDTRIP CORRUPTED")