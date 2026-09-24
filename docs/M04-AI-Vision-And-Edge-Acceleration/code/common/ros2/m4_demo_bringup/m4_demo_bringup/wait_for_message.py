"""Bounded, QoS-aware, one-shot wait_for_first_message helper.

Used by the M4 demo bash wrapper (scripts/m4/lib/m4_demo_lib.sh) as the
replacement for `ros2 topic hz` in the readiness gate. The helper does
not sustain a subscription after returning, and never uses ros2 topic hz.

Public function:
    wait_for_first_message(node, topic, msg_type, timeout_s) -> bool
        node: an existing rclpy.node.Node
        topic: the ROS topic name (str)
        msg_type: the message class (e.g. sensor_msgs.msg.Image)
        timeout_s: deadline in seconds (float)

Returns True on first message, False on timeout. If `node` is None, a
private singleton node is created internally and shut down before
returning.
"""
from __future__ import annotations

import threading
from typing import Optional, Type

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


def sensor_data_qos(depth: int = 10) -> QoSProfile:
    """Same as rclpy.qos.SensorDataQoS but configurable depth."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def wait_for_first_message(
    node: Optional[Node],
    topic: str,
    msg_type: Type,
    timeout_s: float,
    qos: Optional[QoSProfile] = None,
) -> bool:
    """Subscribe once, return True on first message, False on timeout.

    This helper NEVER sustains a subscription after returning. It is the
    bounded replacement for `ros2 topic hz` used by the M4 demo helpers.

    If `node` is None a private node is spun up internally, shut down,
    and destroyed before returning.
    """
    if timeout_s <= 0:
        return False

    own_node = False
    if node is None:
        if not rclpy.ok():
            rclpy.init()
            own_node = True
        node = rclpy.create_node('wait_for_first_message')

    qos = qos or sensor_data_qos()
    done_event = threading.Event()
    state = {'received': False}

    def _cb(_msg):
        if not state['received']:
            state['received'] = True
            done_event.set()

    sub = node.create_subscription(msg_type, topic, _cb, qos)

    deadline = threading.Event()
    deadline.wait(timeout_s)
    # Use a small busy-spin via rclpy.spin_once so we actually pump callbacks
    # (sleeping on Event doesn't drain rclpy). We use a timer to break out.
    import time
    t0 = time.monotonic()
    while not done_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)
        if (time.monotonic() - t0) >= timeout_s:
            break
        if done_event.is_set():
            break

    try:
        node.destroy_subscription(sub)
    except Exception:
        pass
    if own_node:
        try:
            node.destroy_node()
        except Exception:
            pass

    return state['received']
