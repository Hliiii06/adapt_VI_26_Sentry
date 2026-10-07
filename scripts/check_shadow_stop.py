#!/usr/bin/env python3
"""失效/取消停车判据：事件前必须有非零候选速度，事件后限时归零并整窗保持为零。

为什么单独写：只断言"事后输出为零"无法区分"任务确实停了"和"本来就没动"。
本脚本从**事件前**开始连续记录，并检查：

  1. `[0, fault-at)` 内必须出现 ≥ `min-motion` 的候选速度（证明运行中的任务确有输出）；
  2. 最后一个非零样本必须出现在 `fault-at + stop-deadline` 之前（限时停车）；
  3. `[zero-from, duration]` 整个观察窗内必须持续为零（含输入恢复之后，覆盖"旧任务不恢复"）；
  4. 采样覆盖完整、无大间隔（避免"漏采"伪通过）；
  5. 可选 `--expect-healthy-after T`：T 秒后必须观察到 `health_ok=true`
     （用来证明门控确实在健康为真时也生效，例如地图心跳停更）。

退出码：0 = 通过；2 = 任一判据失败。
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
from std_msgs.msg import Bool

SHADOW_TOPIC = "/sentry_scan/cmd_vel_shadow"
HEALTH_TOPIC = "/sentry_scan/health_ok"


class StopChecker(Node):
    def __init__(self) -> None:
        super().__init__("check_shadow_stop")
        self.samples = []      # (t, vxy)
        self.health_seen = []  # (t, bool)
        self.start = self.get_clock().now().nanoseconds * 1e-9
        latch = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Twist, SHADOW_TOPIC, self.on_twist, 10)
        self.create_subscription(Bool, HEALTH_TOPIC, self.on_health, latch)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9 - self.start

    def on_twist(self, msg: Twist) -> None:
        self.samples.append((self.now(), math.hypot(msg.linear.x, msg.linear.y)))

    def on_health(self, msg: Bool) -> None:
        self.health_seen.append((self.now(), bool(msg.data)))


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--fault-at", type=float, required=True,
                        help="故障/取消注入时刻（相对本脚本启动，秒）")
    parser.add_argument("--stop-deadline", type=float, default=2.5)
    parser.add_argument("--zero-from", type=float, default=None,
                        help="从该时刻起必须持续为零；默认 fault-at + stop-deadline")
    parser.add_argument("--min-motion", type=float, default=0.2)
    parser.add_argument("--max-gap", type=float, default=0.5)
    parser.add_argument("--expect-healthy-after", type=float, default=None)
    args, ros_args = parser.parse_known_args(argv)

    zero_from = args.zero_from if args.zero_from is not None else args.fault_at + args.stop_deadline
    rclpy.init(args=ros_args)
    node = StopChecker()
    end = time.time() + args.duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    failures = []
    samples = sorted(node.samples)
    health = sorted(node.health_seen)
    if len(samples) < 10:
        print("FAIL: only %d shadow samples received" % len(samples))
        sys.exit(2)

    # 覆盖与间隔
    times = [s[0] for s in samples]
    gaps = [b - a for a, b in zip(times, times[1:])]
    max_gap = max(gaps) if gaps else 0.0
    if max_gap > args.max_gap:
        failures.append("sample gap %.3fs > %.3fs (observation not continuous)" % (max_gap, args.max_gap))

    # 1) 事件前必须有运动
    motion = [(t, v) for t, v in samples if t < args.fault_at]
    peak_motion = max((v for _, v in motion), default=0.0)
    if peak_motion < args.min_motion:
        failures.append("no motion before the fault: peak |v_xy| = %.4f < %.4f"
                        % (peak_motion, args.min_motion))

    # 2) 限时停车
    nonzero = [t for t, v in samples if v > 1e-9]
    last_nonzero = max(nonzero) if nonzero else 0.0
    if last_nonzero > args.fault_at + args.stop_deadline:
        failures.append("last non-zero sample at %.3fs > fault %.3fs + deadline %.3fs"
                        % (last_nonzero, args.fault_at, args.stop_deadline))

    # 3) 观察窗持续为零
    window = [(t, v) for t, v in samples if t >= zero_from]
    if not window:
        failures.append("no samples in the zero window [%.2f, %.2f]" % (zero_from, args.duration))
    else:
        worst = max(v for _, v in window)
        if worst > 1e-9:
            failures.append("non-zero output in the zero window: max |v_xy| = %.4f" % worst)

    # 5) 健康状态（可选）
    if args.expect_healthy_after is not None:
        after = [ok for t, ok in health if t >= args.expect_healthy_after]
        if not after:
            failures.append("no health_ok sample after %.2fs" % args.expect_healthy_after)
        elif not any(after):
            failures.append("health_ok never true after %.2fs (expected gate to act while healthy)"
                            % args.expect_healthy_after)

    print("samples=%d  peak|v_xy| before fault=%.4f  last nonzero=%.3fs  "
          "max|v_xy| in [%.2f,%.2f]=%.6f  max gap=%.3fs"
          % (len(samples), peak_motion, last_nonzero, zero_from, args.duration,
             max((v for _, v in window), default=0.0), max_gap))
    if failures:
        print("\nFAIL:")
        for item in failures:
            print("  - %s" % item)
        sys.exit(2)
    print("\nPASS: motion before the fault, stop within deadline, zero throughout the window")


if __name__ == "__main__":
    main()
