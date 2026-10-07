#!/usr/bin/env python3
"""测试替身：模拟"与影子并存的旧导航"在 /cmd_vel 上发布。

节点名固定为 `controller_server`、命名空间 `/`，用来验证 shadow_guard 只禁止
**影子命名空间内**的节点发布真实控制话题，而不是禁止旧 Nav2 正常发布。
它发布零速度，不接车、不代表真实 Nav2 行为。
"""

import argparse

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SpoofNav2(Node):
    def __init__(self) -> None:
        super().__init__("controller_server")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.1, self.on_timer)
        self.get_logger().warn("spoof Nav2 controller_server publishing zero /cmd_vel "
                               "(coexistence test, not a real controller)")

    def on_timer(self) -> None:
        self.pub.publish(Twist())


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0)
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = SpoofNav2()
    end = rclpy.clock.Clock().now().nanoseconds * 1e-9 + args.duration
    try:
        while rclpy.ok() and rclpy.clock.Clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
