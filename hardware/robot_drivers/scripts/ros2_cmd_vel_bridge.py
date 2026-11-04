#!/usr/bin/env python3
"""Forward ROS 2 Twist commands to the local Simple AMR service; never open CAN."""
import argparse
import json
import math
import time
from urllib import request
from urllib.parse import urlparse


def twist_to_command(msg):
    values = (msg.linear.x, msg.linear.y, msg.linear.z,
              msg.angular.x, msg.angular.y, msg.angular.z)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Twist contains a non-finite value")
    if any(abs(v) > 1e-9 for v in (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)):
        raise ValueError("Only planar linear.x and angular.z are supported")
    linear, angular = float(msg.linear.x), float(msg.angular.z)
    if abs(linear) > 0.6 or abs(angular) > 2.0:
        raise ValueError("Twist exceeds chassis limits: 0.6 m/s, 2.0 rad/s")
    return {"linear": linear, "angular": angular}


class WebUIClient:
    def __init__(self, base_url="http://127.0.0.1:8765"):
        parsed = urlparse(base_url)
        if parsed.scheme != "http" or parsed.hostname not in ("127.0.0.1", "localhost", "::1") or parsed.path not in ("", "/"):
            raise ValueError("WebUI URL must be a local HTTP address")
        self.base_url = base_url.rstrip("/")

    def post(self, path, payload=None):
        body = json.dumps(payload or {}).encode("utf-8")
        req = request.Request(self.base_url + path, data=body,
                              headers={"Content-Type": "application/json"}, method="POST")
        with request.urlopen(req, timeout=2.0) as response:
            return json.load(response)


def main():
    parser = argparse.ArgumentParser(description="Simple AMR ROS 2 /cmd_vel bridge")
    parser.add_argument("--webui-url", default="http://127.0.0.1:8765")
    parser.add_argument("--topic", default="/cmd_vel")
    args = parser.parse_args()
    client = WebUIClient(args.webui_url)

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    class CmdVelBridge(Node):
        def __init__(self):
            super().__init__("simple_amr_cmd_vel_bridge")
            self.mode = "webui"
            self.armed = False
            self.connected = False
            self.last_error = 0.0
            self.create_subscription(Twist, args.topic, self.on_twist, 10)
            self.create_timer(1.0, self.heartbeat)
            self.heartbeat()
            self.get_logger().info(f"Listening to {args.topic}; CAN remains owned by WebUI")

        def warn(self, exc):
            if time.monotonic() - self.last_error > 2.0:
                self.get_logger().warn(str(exc))
                self.last_error = time.monotonic()

        def heartbeat(self):
            try:
                state = client.post("/api/ros2/heartbeat")
                self.mode, self.armed, self.connected = state["mode"], state["armed"], True
                if self.mode == "ros2" and not self.armed:
                    client.post("/api/ros2/command", {"linear": 0.0, "angular": 0.0})
                    self.armed = True
            except Exception as exc:
                self.connected = False
                self.warn(exc)

        def on_twist(self, msg):
            if not self.connected or self.mode != "ros2" or not self.armed:
                return
            try:
                client.post("/api/ros2/command", twist_to_command(msg))
            except Exception as exc:
                self.warn(exc)

    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            client.post("/api/ros2/stop")
        except Exception:
            pass
        node.destroy_node()
        # SIGTERM may already have shut down the context via rclpy's signal handler.
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
