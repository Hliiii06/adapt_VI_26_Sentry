#!/usr/bin/env python3
"""只读诊断：把影子为什么"不健康"直接打印出来（现场排查用）。

`/sentry_scan/health`（`diagnostic_msgs/DiagnosticArray`）里已经带有每通道的
`count / rejected / frame / age_s`，以及不健康的**具体原因**字符串。本脚本把它读出来，
省去在日志里翻找。

用法：
    python3 scripts/onsite_health_dump.py --duration 4

退出码：0 = 健康（level=OK 且 health_ok=True）；2 = 不健康（有问题）；3 = 收不到 health（影子没跑）。
"""

import argparse
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool

HEALTH_TOPIC = "/sentry_scan/health"
HEALTH_OK_TOPIC = "/sentry_scan/health_ok"
LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=4.0)
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Node("onsite_health_dump")
    last = {"array": None, "ok": None}
    node.create_subscription(DiagnosticArray, HEALTH_TOPIC,
                             lambda msg: last.__setitem__("array", msg), 10)
    latch = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Bool, HEALTH_OK_TOPIC,
                             lambda msg: last.__setitem__("ok", bool(msg.data)), latch)

    end = time.time() + args.duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if last["array"] is None:
        print("FAIL: %.1fs 内没有收到 %s —— 影子入口没有运行？" % (args.duration, HEALTH_TOPIC))
        return 3

    def level_of(status):
        # diagnostic_msgs 的 level 是 byte，rclpy 里可能是 bytes
        raw = status.level
        if isinstance(raw, (bytes, bytearray)):
            return int(raw[0]) if raw else 0
        return int(raw)

    print("health_ok = %s" % ("未收到（latch 缺失）" if last["ok"] is None else last["ok"]))
    unhealthy = []
    for status in last["array"].status:
        level = level_of(status)
        if level != 0:
            unhealthy.append("%s: %s" % (status.name, status.message or "无原因字符串"))
        print("-" * 72)
        print("通道: %s   级别: %s" % (status.name, LEVEL_NAMES.get(level, level)))
        print("原因: %s" % (status.message or "-"))
        values = {kv.key: kv.value for kv in status.values}
        for key in sorted(values):
            print("    %-22s %s" % (key, values[key]))
    print("-" * 72)
    print("-" * 72)
    print("提示：`rejected` 增长而 `count` 不增长的通道就是问题所在；"
          "`age_s=n/a` 表示从未收到有效数据，`age_s` 偏大表示中断或时间戳尺度不符。")
    if unhealthy or last["ok"] is False:
        print("结论：不健康。")
        for item in unhealthy:
            print("  - %s" % item)
        return 2
    print("结论：健康。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
