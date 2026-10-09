#!/usr/bin/env python3
"""只读拓扑审计：把控制话题的**发布者**与**订阅者**分开列出（不再混在一起）。

为什么需要：`ros2 topic info -v` 的文本里 publisher/subscriber 段落交错，
用 grep 抓"Node name"会把订阅者（例如 `uart_node`）误显示成发布者。
本脚本用 rclpy 的 `get_publishers_info_by_topic()` / `get_subscriptions_info_by_topic()`
精确区分，并给出判据：

  * 影子命名空间（默认 `/sentry_scan`）里**任何**控制话题发布者 → FAIL（rc=2）；
  * 外部发布者只列出并统计（实车原有导航正常存在）。

用法：
    python3 scripts/onsite_control_audit.py
    python3 scripts/onsite_control_audit.py --topics /cmd_vel,/cmd_vel_remap
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

DEFAULT_TOPICS = ["/cmd_vel", "/cmd_vel_remap", "/cmd_vel_nav", "/cmd_vel_remap_unused"]


def _endpoint(info):
    namespace = (info.node_namespace or "/").rstrip("/")
    qos = info.qos_profile
    return "%s/%s  [%s/%s/depth=%s]" % (namespace or "", info.node_name,
                                        qos.reliability.name, qos.durability.name, qos.depth)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topics", default=",".join(DEFAULT_TOPICS))
    parser.add_argument("--shadow-namespace", default="/sentry_scan")
    args, ros_args = parser.parse_known_args(argv)

    topics = [t.strip() for t in args.topics.split(",") if t.strip()]
    own_ns = (args.shadow_namespace or "/").rstrip("/")

    rclpy.init(args=ros_args)
    node = Node("onsite_control_audit")
    available = dict(node.get_topic_names_and_types())

    failures = []
    for topic in topics:
        types = available.get(topic)
        print("=" * 72)
        print("%s   %s" % (topic, types[0] if types else "（当前不存在）"))
        if not types:
            continue
        publishers = node.get_publishers_info_by_topic(topic)
        subscribers = node.get_subscriptions_info_by_topic(topic)
        print("  发布者 (%d) —— 谁在真正下发速度：" % len(publishers))
        if not publishers:
            print("    无：该话题当前没有任何发布者（确认底盘速度是否由别的话题驱动）")
        for info in publishers:
            marker = ""
            if (info.node_namespace or "/").rstrip("/") == own_ns:
                marker = "   <== 影子节点！绝不允许"
                failures.append("%s 由影子节点 %s 发布" % (topic, info.node_name))
            print("    %s%s" % (_endpoint(info), marker))
        print("  订阅者 (%d) —— 谁在消费速度：" % len(subscribers))
        for info in subscribers:
            print("    %s" % _endpoint(info))

    node.destroy_node()
    rclpy.shutdown()

    print("=" * 72)
    if failures:
        print("FAIL:")
        for item in failures:
            print("  - %s" % item)
        return 2
    print("PASS: 影子命名空间没有出现在任何控制话题的发布者名单里")
    return 0


if __name__ == "__main__":
    sys.exit(main())
