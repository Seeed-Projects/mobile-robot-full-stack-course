#!/usr/bin/env python3
"""Loop-publish a folder of jpg images to /perception/cameras/front/image.

Used by scripts/m4/visualize_yolo.sh.

No cv_bridge dependency: we construct sensor_msgs/Image manually
from a numpy array. Works under any numpy / cv_bridge version.

Two modes:
  --mode loop    Publish all images, then loop forever (until --timeout).
                 This keeps the publisher alive so subscribers have time
                 to spin up after it.
  --mode finite  Publish --num-frames images, then exit.

CLI:
  image_loop_publisher.py --dir <DIR> [--fps 2] [--num-frames 3]
                          [--timeout 30] [--mode loop|finite]
                          [--topic /perception/cameras/front/image]
"""
import argparse
import array
import os
import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def numpy_to_image_msg(arr, frame_id, stamp):
    """Convert a HxWxC uint8 BGR numpy array to sensor_msgs/Image without cv_bridge.

    `msg.data` is assigned an `array.array('B')` on purpose: assigning a
    `bytes` object to the `uint8[]` field is converted element-by-element by
    rclpy (~149 ns/byte), which costs ~927 ms for a 1920x1080 frame.
    """
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(arr.shape[0])
    msg.width = int(arr.shape[1])
    if arr.ndim == 2:
        msg.encoding = 'mono8'
        msg.step = arr.shape[1]
        msg.data = array.array('B', arr.tobytes())
    else:
        channels = arr.shape[2]
        if channels == 3:
            msg.encoding = 'bgr8'
            msg.step = arr.shape[1] * 3
            msg.data = array.array('B', arr.tobytes())
        elif channels == 4:
            msg.encoding = 'bgra8'
            msg.step = arr.shape[1] * 4
            msg.data = array.array('B', arr.tobytes())
        else:
            raise ValueError(f'unsupported channel count {channels}')
    msg.is_bigendian = 0
    return msg


class LoopPublisher(Node):
    def __init__(self, image_dir, fps, num_frames, timeout_s, topic, mode):
        super().__init__('yolo_image_loop_publisher')
        self.pub = self.create_publisher(Image, topic, 10)
        self.image_dir = image_dir
        self.dt = 1.0 / max(fps, 0.1)
        self.num_frames = num_frames
        self.timeout_s = timeout_s
        self.mode = mode
        self.start_time = time.monotonic()

        exts = ('.jpg', '.jpeg', '.png')
        all_files = []
        for fn in sorted(os.listdir(image_dir)):
            if fn.lower().endswith(exts):
                all_files.append(os.path.join(image_dir, fn))
        self.files = all_files[:num_frames] if num_frames > 0 else all_files
        if not self.files:
            self.get_logger().fatal(f'No images in {image_dir}')
            sys.exit(1)
        self.idx = 0
        self.total_published = 0
        self.timer = self.create_timer(self.dt, self.publish_next)
        mode_desc = 'loop forever' if mode == 'loop' else f'{num_frames} frames'
        self.get_logger().info(
            f'Will publish {mode_desc} at {fps} fps from {image_dir}'
        )

    def publish_next(self):
        # Timeout check (only meaningful for finite mode)
        if self.mode == 'finite' and \
           (time.monotonic() - self.start_time) > self.timeout_s:
            self.get_logger().warn(f'Timeout {self.timeout_s}s reached')
            self.finish()
            return

        # Pick file
        if self.mode == 'loop':
            path = self.files[self.idx % len(self.files)]
        else:
            if self.idx >= len(self.files):
                self.get_logger().info('Done publishing')
                self.finish()
                return
            path = self.files[self.idx]

        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn(f'cv2.imread failed: {path}, skip')
            self.idx += 1
            self.total_published += 1
            return

        stamp = self.get_clock().now().to_msg()
        msg = numpy_to_image_msg(img, 'camera_front', stamp)
        self.pub.publish(msg)
        self.total_published += 1
        if self.mode == 'finite':
            self.get_logger().info(
                f'[{self.idx + 1}/{len(self.files)}] {os.path.basename(path)} '
                f'({img.shape[1]}x{img.shape[0]})'
            )
        elif self.total_published % 10 == 1:
            self.get_logger().info(
                f'[{self.total_published}] {os.path.basename(path)} '
                f'({img.shape[1]}x{img.shape[0]})'
            )
        self.idx += 1

    def finish(self):
        self.timer.cancel()
        time.sleep(0.2)
        raise SystemExit(0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', required=True)
    parser.add_argument('--fps', type=float, default=2.0)
    parser.add_argument('--num-frames', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--topic', default='/perception/cameras/front/image')
    parser.add_argument('--mode', choices=['loop', 'finite'], default='finite',
                        help='loop = keep cycling until timeout; '
                             'finite = publish N frames then exit (default)')
    args = parser.parse_args()

    rclpy.init()
    node = LoopPublisher(args.dir, args.fps, args.num_frames, args.timeout,
                         args.topic, args.mode)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
