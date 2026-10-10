#!/usr/bin/env python3
"""接管期命令闸门的现场操作：显式使能 / 取消使能 / 查看状态。

    python3 scripts/cmd_gate_control.py --status
    python3 scripts/cmd_gate_control.py --enable     # 显式使能（仍受健康/任务/限幅约束）
    python3 scripts/cmd_gate_control.py --disable     # 立即回到零输出

安全：只发布 `/sentry_scan/cmd_vel_enable`（Bool，transient_local）与读取状态话题；
**不发布任何速度**。闸门默认不输出，必须被显式使能。
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, String

ENABLE_TOPIC = "/sentry_scan/cmd_vel_enable"
STATE_TOPIC = "/sentry_scan/cmd_gate/state"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--enable", action="store_true")
    parser.add_argument("--disable", action="store_true")
    parser.add_argument("--status", action="store_true")
    parser.add_argument("--wait", type=float, default=5.0)
    args, ros_args = parser.parse_known_args(argv)
    if not (args.enable or args.disable or args.status):
        print("用法：--enable | --disable | --status")
        return 1

    rclpy.init(args=ros_args)
    node = Node("cmd_gate_control")
    state = {"value": None}
    latch = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, STATE_TOPIC, lambda m: state.__setitem__("value", m.data),
                             latch)
    publisher = node.create_publisher(Bool, ENABLE_TOPIC, latch)
    deadline = time.time() + args.wait
    while time.time() < deadline and publisher.get_subscription_count() == 0 and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        print("失败：没有订阅者出现在 %s；闸门是否用 cmd_gate:=true 启动？" % ENABLE_TOPIC)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    if args.status:
        end = time.time() + 2.0
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        print("闸门状态: %s" % (state["value"] or "未收到状态"))
        node.destroy_node()
        rclpy.shutdown()
        return 0

    msg = Bool()
    msg.data = bool(args.enable)
    for _ in range(3):
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    print("已发送：%s 命令闸门" % ("使能" if args.enable else "取消使能"))
    if args.enable:
        print("提醒：使能只是必要条件之一；仍需任务授权 + 输入健康 + 地图心跳 + 候选新鲜 + 非零限幅。")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
