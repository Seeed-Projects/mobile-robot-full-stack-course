#!/usr/bin/env python3
"""Standalone CLI entry point for scripts/m4/lib/m4_demo_lib.sh.

Usage:
    python3 -m m4_demo_bringup.wait_for_message_node \
        --topic /perception/cameras/front/image --timeout 5 \
        [--msg-type sensor_msgs.msg.Image]

Exit:
    0 = received >=1 message within timeout
    4 = timed out

Stdout line on success: WAIT_FOR_FIRST_MESSAGE:UP <topic> after <Ns>
"""
from __future__ import annotations

import argparse
import importlib
import sys
import time

import rclpy

from .wait_for_message import wait_for_first_message


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', required=True)
    parser.add_argument('--timeout', type=float, required=True)
    parser.add_argument('--msg-type', default='sensor_msgs.msg.Image')
    args = parser.parse_args(argv)

    try:
        mod_name, cls_name = args.msg_type.rsplit('.', 1)
        mod = importlib.import_module(mod_name)
        msg_type = getattr(mod, cls_name)
    except Exception as e:
        print(f'WAIT_FOR_FIRST_MESSAGE:ERROR {args.msg_type}: {e}', file=sys.stderr)
        return 5

    rclpy.init()
    t0 = time.monotonic()
    got = wait_for_first_message(None, args.topic, msg_type, args.timeout)
    elapsed = time.monotonic() - t0
    if got:
        print(f'WAIT_FOR_FIRST_MESSAGE:UP {args.topic} after {elapsed:.2f}s')
        rc = 0
    else:
        print(f'WAIT_FOR_FIRST_MESSAGE:TIMEOUT {args.topic} after {elapsed:.2f}s',
              file=sys.stderr)
        rc = 4
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return rc


if __name__ == '__main__':
    sys.exit(main())
