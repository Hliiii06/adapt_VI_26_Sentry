#!/usr/bin/env python3
"""现场暂停/恢复**影子侧**输入转发（只影响 /sentry_scan，不动实车节点）。

配合 `input_gate:=true` 启动的影子入口使用：

    python3 scripts/onsite_pause_inputs.py --pause     # 影子视为"输入失效"
    python3 scripts/onsite_pause_inputs.py --resume    # 影子视为"输入恢复"
    python3 scripts/onsite_pause_inputs.py --status    # 读取当前闸门状态（transient_local）

用途：验证"运行中的断流锁止"与"恢复后不自动续跑"——不需要重启影子，也不需要停实车节点。
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool

PAUSE_TOPIC = "/sentry_scan/test/pause_inputs"
STATE_TOPIC = "/sentry_scan/test/input_paused"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pause", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--status", action="store_true")
    parser.add_argument("--wait", type=float, default=5.0)
    args, ros_args = parser.parse_known_args(argv)

    if not (args.pause or args.resume or args.status):
        print("用法：--pause | --resume | --status")
        return 1

    rclpy.init(args=ros_args)
    node = Node("onsite_pause_inputs")
    state = {"value": None}

    def on_state(msg):
        state["value"] = bool(msg.data)

    latch = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Bool, STATE_TOPIC, on_state, latch)
    publisher = node.create_publisher(Bool, PAUSE_TOPIC, 10)

    deadline = time.time() + args.wait
    while time.time() < deadline and publisher.get_subscription_count() == 0 and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        print("失败：没有订阅者出现在 %s；影子入口是否用 input_gate:=true 启动？" % PAUSE_TOPIC)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    if args.status:
        end = time.time() + 2.0
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        print("影子输入当前状态: %s"
              % ("未知（未收到 latch 状态）" if state["value"] is None
                 else ("已暂停" if state["value"] else "正常转发")))
        node.destroy_node()
        rclpy.shutdown()
        return 0

    msg = Bool()
    msg.data = bool(args.pause)
    for _ in range(3):
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    print("已发送：%s 影子输入转发" % ("暂停" if args.pause else "恢复"))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
