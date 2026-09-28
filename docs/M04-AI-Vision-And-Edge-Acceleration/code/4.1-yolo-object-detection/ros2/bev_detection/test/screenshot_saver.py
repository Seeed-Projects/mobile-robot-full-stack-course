#!/usr/bin/env python3
"""Subscribe to a ROS Image topic and save the first N received frames as PNG.

No cv_bridge: we parse sensor_msgs/Image directly into a numpy array.

CLI:
  screenshot_saver.py --out <PATH> [--topic /perception/debug/detection_image]
                      [--timeout 30] [--min-frames 1] [--all]
"""
import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy
)
from sensor_msgs.msg import Image


_BGR_ENCODINGS = {'bgr8', 'rgb8'}
_BGRA_ENCODINGS = {'bgra8', 'rgba8'}
_MONO_ENCODINGS = {'mono8', '8UC1'}
_GRAY8 = 'mono8'


def image_msg_to_numpy(msg):
    """Convert sensor_msgs/Image to a HxWx3 uint8 BGR numpy array."""
    h, w = msg.height, msg.width
    enc = msg.encoding or 'bgr8'
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in _BGR_ENCODINGS:
        if enc == 'rgb8':
            arr = raw.reshape(h, w, 3)[:, :, ::-1].copy()
        else:
            arr = raw.reshape(h, w, 3).copy()
    elif enc in _BGRA_ENCODINGS:
        bgra = raw.reshape(h, w, 4).copy()
        arr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
    elif enc in _MONO_ENCODINGS:
        gray = raw.reshape(h, w).copy()
        arr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    elif enc == '16UC1' or enc == 'mono16':
        arr16 = raw.reshape(h, w).copy()
        arr8 = cv2.convertScaleAbs(arr16, alpha=(255.0 / 65535.0))
        arr = cv2.cvtColor(arr8, cv2.COLOR_GRAY2BGR)
    elif enc == '32FC1':
        arr32 = raw.reshape(h, w).copy()
        arr8 = cv2.normalize(arr32, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        arr = cv2.cvtColor(arr8, cv2.COLOR_GRAY2BGR)
    elif enc == 'bayer_rggb8':
        bayer = raw.reshape(h, w).copy()
        arr = cv2.cvtColor(bayer, cv2.COLOR_BayerRGGB2BGR)
    else:
        raise ValueError(f'unsupported encoding {enc!r}')
    return arr


class ScreenshotSaver(Node):
    def __init__(self, out_path, topic, timeout_s, min_frames, save_all):
        super().__init__('yolo_screenshot_saver')
        self.out_path = out_path
        self.timeout_s = timeout_s
        self.min_frames = min_frames
        self.save_all = save_all
        self.frames = []
        self.start = time.monotonic()
        self.saved = False

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.sub = self.create_subscription(Image, topic, self.on_msg, qos)
        self.get_logger().info(
            f'Subscribing to {topic} (BEST_EFFORT), '
            f'will save to {out_path}, timeout={timeout_s}s'
        )

    def on_msg(self, msg):
        age = time.monotonic() - self.start
        try:
            cv_img = image_msg_to_numpy(msg)
        except Exception as e:
            self.get_logger().warn(f'image parse failed: {e}')
            return

        if self.save_all:
            base, ext = self.out_path.rsplit('.', 1)
            path = f'{base}_{len(self.frames):03d}.{ext}'
        else:
            path = self.out_path

        cv2.imwrite(path, cv_img)
        self.frames.append(path)
        self.get_logger().info(
            f'Saved frame {len(self.frames)} ({cv_img.shape[1]}x{cv_img.shape[0]}) '
            f'@ t={age:.1f}s -> {path}'
        )

        if not self.save_all:
            self.saved = True
            time.sleep(0.1)
            raise SystemExit(0)

    def run_until_exit(self):
        # timeout_s <= 0 means infinite
        if self.timeout_s <= 0:
            self.get_logger().info('running until process is interrupted')
            try:
                while rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.2)
                    if self.saved:
                        return
            except KeyboardInterrupt:
                return
            return
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.saved:
                return
        if not self.frames and self.min_frames > 0:
            self.get_logger().error(
                f'No frames received in {self.timeout_s}s, exiting FAIL'
            )
            sys.exit(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    parser.add_argument('--topic', default='/perception/debug/detection_image')
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--min-frames', type=int, default=1)
    parser.add_argument('--all', action='store_true',
                        help='Save every received frame as out_NNN.png')
    args = parser.parse_args()

    rclpy.init()
    node = ScreenshotSaver(
        args.out, args.topic, args.timeout, args.min_frames, args.all
    )
    try:
        node.run_until_exit()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
