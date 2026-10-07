#!/usr/bin/env python3
"""失效/取消停车判据：事件前必须有非零候选速度，事件后限时归零并整窗保持为零。

为什么单独写：只断言"事后输出为零"无法区分"任务确实停了"和"本来就没动"。
本脚本从**事件前**开始连续记录，并检查：

  1. 覆盖完整：首帧在启动后 `--max-start-lag` 内、末帧覆盖到 `--zero-until`（默认 duration）、
     相邻样本间隔 ≤ `--max-gap`，且所有记录值有限；
     （只检查"已有样本之间"的间隔会在观察窗没采满时假通过，这里必须检查首尾覆盖。）
  2. `[0, 故障时刻)` 内必须出现 ≥ `--min-motion` 的候选速度（证明运行中的任务确有输出）；
  3. 最后一个非零样本必须出现在 `故障时刻 + --stop-deadline` 之前；
  4. `[--zero-from, --zero-until]` 整个观察窗内必须持续为零；
  5. 可选 `--expect-motion-after T`：T 之后必须重新出现运动（例如"新任务才恢复"）；
  6. 可选 `--expect-healthy-after T`：T 后必须观察到 `health_ok=true`
     （证明门控在输入健康时也生效，例如地图心跳停更）。

故障时刻优先取 `--fault-marker-topic` 上第一条标记的**本机接收时刻**（由注入方在真正注入时发布），
取不到才回退到 `--fault-at`。这样停车延迟是按实际事件时间算的，不靠固定假设。

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
from std_msgs.msg import Bool, Header

SHADOW_TOPIC = "/sentry_scan/cmd_vel_shadow"
HEALTH_TOPIC = "/sentry_scan/health_ok"


class StopChecker(Node):
    def __init__(self, marker_topic: str) -> None:
        super().__init__("check_shadow_stop")
        self.samples = []      # (t, vxy)
        self.health_seen = []  # (t, bool)
        self.markers = []      # (t, frame_id/marker note)
        self.start = self.get_clock().now().nanoseconds * 1e-9
        latch = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Twist, SHADOW_TOPIC, self.on_twist, 10)
        self.create_subscription(Bool, HEALTH_TOPIC, self.on_health, 10)
        self.create_subscription(Header, marker_topic, self.on_marker, 10)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9 - self.start

    def on_twist(self, msg: Twist) -> None:
        self.samples.append((self.now(), math.hypot(msg.linear.x, msg.linear.y)))

    def on_health(self, msg: Bool) -> None:
        self.health_seen.append((self.now(), bool(msg.data)))

    def on_marker(self, msg: Header) -> None:
        self.markers.append((self.now(), msg.frame_id or "fault"))


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--fault-at", type=float, required=True,
                        help="故障/取消注入时刻的回退值（相对本脚本启动，秒）")
    parser.add_argument("--fault-marker-topic", default="/sentry_scan/test/fault_marker")
    parser.add_argument("--stop-deadline", type=float, default=2.5)
    parser.add_argument("--zero-from", type=float, default=None,
                        help="从该时刻起必须持续为零；默认 fault + stop-deadline")
    parser.add_argument("--zero-until", type=float, default=None,
                        help="零观察窗终点；默认 duration")
    parser.add_argument("--min-motion", type=float, default=0.2)
    parser.add_argument("--max-gap", type=float, default=0.5)
    parser.add_argument("--max-start-lag", type=float, default=1.0)
    parser.add_argument("--expect-healthy-after", type=float, default=None)
    parser.add_argument("--expect-motion-after", type=float, default=None)
    parser.add_argument("--resume-at", type=float, default=None,
                        help="新任务允许恢复的最早时刻；限时停车检查只看该时刻之前的非零样本"
                             "（默认取 --expect-motion-after）")
    args, ros_args = parser.parse_known_args(argv)

    duration = args.duration
    zero_until = args.zero_until if args.zero_until is not None else duration
    rclpy.init(args=ros_args)
    node = StopChecker(args.fault_marker_topic)
    end = time.time() + duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    failures = []
    samples = sorted(node.samples)
    health = sorted(node.health_seen)
    markers = sorted(node.markers)
    if len(samples) < 10:
        print("FAIL: only %d shadow samples received" % len(samples))
        sys.exit(2)

    if markers:
        fault_at = markers[0][0]
        fault_source = "marker(%s)" % markers[0][1]
    else:
        fault_at = args.fault_at
        fault_source = "assumed --fault-at"
    zero_from = args.zero_from if args.zero_from is not None else fault_at + args.stop_deadline

    times = [s[0] for s in samples]
    values = [s[1] for s in samples]
    if any(not math.isfinite(v) for v in values):
        failures.append("non-finite shadow sample value recorded")
    if any(not math.isfinite(t) for t in times):
        failures.append("non-finite sample timestamp recorded")

    # 1) 覆盖：首尾都要覆盖到，不能只看样本之间的间隔。
    if times[0] > args.max_start_lag:
        failures.append("first sample at %.3fs > %.3fs: recording did not start early enough"
                        % (times[0], args.max_start_lag))
    if times[-1] < zero_until - args.max_gap:
        failures.append("last sample at %.3fs does not cover the window end %.3fs"
                        % (times[-1], zero_until))
    gaps = [b - a for a, b in zip(times, times[1:])]
    max_gap = max(gaps) if gaps else 0.0
    if max_gap > args.max_gap:
        failures.append("sample gap %.3fs > %.3fs (observation not continuous)" % (max_gap, args.max_gap))

    # 2) 事件前必须有运动
    motion = [(t, v) for t, v in samples if t < fault_at]
    peak_motion = max((v for _, v in motion), default=0.0)
    if peak_motion < args.min_motion:
        failures.append("no motion before the fault: peak |v_xy| = %.4f < %.4f"
                        % (peak_motion, args.min_motion))

    # 3) 限时停车（按实际事件时间）。
    # 若显式允许"新任务恢复"（--expect-motion-after T），则限时检查只看 T 之前的非零样本：
    # 那时之后的运动属于新任务，不是旧任务没停。
    nonzero = [t for t, v in samples if v > 1e-9]
    last_nonzero = max(nonzero) if nonzero else 0.0
    resume_at = args.resume_at if args.resume_at is not None else args.expect_motion_after
    deadline_scope = [t for t in nonzero if resume_at is None or t < resume_at]
    last_before_resume = max(deadline_scope) if deadline_scope else 0.0
    latency = last_before_resume - fault_at
    if last_before_resume > fault_at + args.stop_deadline:
        failures.append("last non-zero sample before the new task at %.3fs > fault %.3fs (%s) + "
                        "deadline %.3fs"
                        % (last_before_resume, fault_at, fault_source, args.stop_deadline))

    # 4) 观察窗持续为零
    window = [(t, v) for t, v in samples if zero_from <= t <= zero_until]
    if not window:
        failures.append("no samples in the zero window [%.2f, %.2f]" % (zero_from, zero_until))
    else:
        worst = max(v for _, v in window)
        if worst > 1e-9:
            failures.append("non-zero output in the zero window [%.2f, %.2f]: max |v_xy| = %.4f"
                            % (zero_from, zero_until, worst))

    # 5) 新任务恢复（可选）
    if args.expect_motion_after is not None:
        after = [v for t, v in samples if t >= args.expect_motion_after]
        if not after:
            failures.append("no samples after %.2fs to check new-task motion" % args.expect_motion_after)
        elif max(after) < args.min_motion:
            failures.append("no motion after %.2fs (max |v_xy| = %.4f); a new task did not resume"
                            % (args.expect_motion_after, max(after)))

    # 6) 健康状态（可选）
    if args.expect_healthy_after is not None:
        after_health = [ok for t, ok in health if t >= args.expect_healthy_after]
        if not after_health:
            failures.append("no health_ok sample after %.2fs" % args.expect_healthy_after)
        elif not any(after_health):
            failures.append("health_ok never true after %.2fs (expected gate to act while healthy)"
                            % args.expect_healthy_after)

    print("samples=%d [%.2f..%.2f]  fault=%.2fs (%s)  peak|v_xy| before=%.4f  "
          "last nonzero(<=resume)=%.3fs (latency %.3fs, overall last=%.3fs)  "
          "max|v_xy| in [%.2f,%.2f]=%.6f  max gap=%.3fs"
          % (len(samples), times[0], times[-1], fault_at, fault_source, peak_motion,
             last_before_resume, latency, last_nonzero, zero_from, zero_until,
             max((v for _, v in window), default=0.0), max_gap))
    if failures:
        print("\nFAIL:")
        for item in failures:
            print("  - %s" % item)
        sys.exit(2)
    print("\nPASS: motion before the fault, stop within deadline, zero throughout the window")


if __name__ == "__main__":
    main()
