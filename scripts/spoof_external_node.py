#!/usr/bin/env python3
"""测试替身：模拟"实车原有节点"（默认名字 uart_node，命名空间 /）在运行。

用途：验证影子安全自检**不会**把实车原有节点误判成"影子启动了禁止节点"。
它只创建节点本身，不发布任何话题，也不接串口。
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="uart_node", help="节点名（默认 uart_node）")
    parser.add_argument("--namespace", default="/")
    parser.add_argument("--duration", type=float, default=60.0)
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Node(args.name, namespace=args.namespace)
    node.get_logger().warn("spoof external node '%s%s' running (test double, does nothing)"
                           % (args.namespace.rstrip('/'), args.name))
    end = time.time() + args.duration
    try:
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
