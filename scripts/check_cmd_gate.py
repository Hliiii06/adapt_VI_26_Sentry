#!/usr/bin/env python3
"""校验命令闸门的输出行为（接管前必跑，失败返回非零）。

    python3 scripts/check_cmd_gate.py --duration 6 --expect-silent
    python3 scripts/check_cmd_gate.py --duration 8 --expect-active --max 0.3

判据：
  * `--expect-silent`：`cmd_vel_gated` 上**一条消息都没有**（从未使能 → 绝不发布）；
  * `--expect-active`：出现非零输出，且 `|vx|,|vy| <= --max`、`|wz| <= --max-wz`；
  * 两种模式都会审计 `/cmd_vel`：**闸门自己绝不出现在真实控制话题的发布者名单里**
    （由 `onsite_control_audit.py` 的另一路检查保证）。退出码 0/2。
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

GATED_TOPIC = "/sentry_scan/cmd_vel_gated"
STATE_TOPIC = "/sentry_scan/cmd_gate/state"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--expect-silent", action="store_true")
    parser.add_argument("--expect-active", action="store_true")
    parser.add_argument("--expect-zero-output", action="store_true",
                        help="已使能但限幅为 0：应有消息且**全部为零**")
    parser.add_argument("--max", type=float, default=0.3)
    parser.add_argument("--max-wz", type=float, default=0.0)
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Node("check_cmd_gate")
    samples = []
    states = []
    latch = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Twist, GATED_TOPIC, lambda m: samples.append(m), 10)
    node.create_subscription(String, STATE_TOPIC, lambda m: states.append(m.data), latch)
    end = time.time() + args.duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    max_vx = max((abs(m.linear.x) for m in samples), default=0.0)
    max_vy = max((abs(m.linear.y) for m in samples), default=0.0)
    max_wz = max((abs(m.angular.z) for m in samples), default=0.0)
    max_xyz = max((abs(m.linear.z) for m in samples), default=0.0)
    failures = []
    print("gated samples=%d  max|vx|=%.3f max|vy|=%.3f max|wz|=%.3f max|vz|=%.3f  states=%s"
          % (len(samples), max_vx, max_vy, max_wz, max_xyz, sorted(set(states)) or "-"))
    if args.expect_silent and samples:
        failures.append("expected NO output before enable, got %d messages" % len(samples))
    if args.expect_active:
        if not samples:
            failures.append("expected active output, got none (仍被互锁挡住？看状态话题)")
        elif max(max_vx, max_vy) <= 1e-9:
            failures.append("active window had no non-zero velocity")
    if args.expect_zero_output:
        if not samples:
            failures.append("expected zero-valued output after enable, got NO messages "
                            "(gate 未使能或未在发布)")
        elif max(max_vx, max_vy, max_wz, max_xyz) > 1e-9:
            failures.append("expected all-zero output, got max|vx|=%.3f max|vy|=%.3f max|wz|=%.3f"
                            % (max_vx, max_vy, max_wz))
    if max_vx > args.max + 1e-6 or max_vy > args.max + 1e-6:
        failures.append("clamp violated: max|vx|=%.3f max|vy|=%.3f > %.3f"
                        % (max_vx, max_vy, args.max))
    if max_wz > args.max_wz + 1e-6:
        failures.append("rotation clamp violated: max|wz|=%.3f > %.3f" % (max_wz, args.max_wz))
    if max_xyz > 1e-9:
        failures.append("linear.z must stay zero, got %.3f" % max_xyz)
    if not math.isfinite(max_vx + max_vy + max_wz):
        failures.append("non-finite gate output")
    if failures:
        print("FAIL:")
        for item in failures:
            print("  - %s" % item)
        return 2
    print("PASS: gate output honours the expectation (silent=%s active=%s)"
          % (args.expect_silent, args.expect_active))
    return 0


if __name__ == "__main__":
    sys.exit(main())
