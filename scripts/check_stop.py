#!/usr/bin/env python3
"""停车判据检查：从事件时刻起，命令必须在时限内归零并保持静止。

用于 cancel（取消）与 odom_loss（里程计断流）两类场景。与旧判据的区别：

- 事件时刻（取消/断流）由调用方显式给出，不再靠"第一次速度到零"反推。
- 既检查"时限内归零"，也检查"之后一整段观察窗内持续为零"。
- 失败时返回非零退出码，供编排脚本判断。

用法:
    check_stop.py --csv <cmdvel.csv> --event-t <秒> --deadline <秒>
                  [--observe <秒>] [--label cancel]
"""

import argparse
import csv
import sys

ZERO_EPS = 1e-3


def norm(vx, vy, wz):
    return max(abs(vx), abs(vy), abs(wz))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--event-t", type=float, required=True,
                        help="事件（取消/断流）在 CSV 时间轴上的秒数")
    parser.add_argument("--deadline", type=float, required=True,
                        help="要求在该时限内归零")
    parser.add_argument("--observe", type=float, default=5.0,
                        help="归零后需要持续静止的观察时长")
    parser.add_argument("--label", default="事件")
    args = parser.parse_args()

    rows = []
    with open(args.csv) as handle:
        for r in csv.DictReader(handle):
            rows.append((float(r["t"]), float(r["vx"]), float(r["vy"]), float(r["wz"])))
    if not rows:
        print("失败：%s 的 cmd_vel 记录为空" % args.csv)
        return 2
    rows.sort()

    before = [r for r in rows if r[0] < args.event_t]
    after = [r for r in rows if r[0] >= args.event_t]
    moving_before = [r for r in before if norm(r[1], r[2], r[3]) > 0.05]

    print("%s = t=%.2fs，cmd_vel 样本 %d（事件前 %d，事件后 %d）"
          % (args.label, args.event_t, len(rows), len(before), len(after)))

    if not moving_before:
        print("失败：事件前没有明显非零命令，本场景无效（无法证明是"
              "「先动后停」而不是「一直没动」）")
        return 2
    print("事件前非零样本 %d，最大命令 %.3f" % (len(moving_before),
          max(norm(*r[1:]) for r in moving_before)))

    if not after:
        print("失败：事件后没有采样，无法判断是否停车")
        return 2

    # 1) 时限内归零
    window = [r for r in after if r[0] <= args.event_t + args.deadline]
    stopped_at = None
    for r in after:
        if norm(r[1], r[2], r[3]) <= ZERO_EPS:
            stopped_at = r[0]
            break
    if stopped_at is None:
        print("失败：事件后从未出现零命令（观察 %.2fs）" % (after[-1][0] - args.event_t))
        return 1
    delay = stopped_at - args.event_t
    print("归零时刻 t=%.2fs，延迟 %.3fs（时限 %.2fs）" % (stopped_at, delay, args.deadline))
    if delay > args.deadline:
        print("失败：超出停车时限")
        return 1

    # 2) 归零后持续静止
    observe_end = stopped_at + args.observe
    tail = [r for r in rows if stopped_at <= r[0] <= observe_end]
    re_moved = [r for r in tail if norm(r[1], r[2], r[3]) > ZERO_EPS]
    print("归零后观察窗 [%.2f, %.2f]：%d 个样本，其中非零 %d 个"
          % (stopped_at, observe_end, len(tail), len(re_moved)))
    if len(tail) < 10:
        print("失败：归零后样本过少（%d），无法证明持续静止" % len(tail))
        return 1
    if re_moved:
        worst = max(re_moved, key=lambda r: norm(r[1], r[2], r[3]))
        print("失败：停车后又重新出现非零命令，t=%.2f 命令 %.3f（共 %d 个样本）"
              % (worst[0], norm(*worst[1:]), len(re_moved)))
        print("      这正是「取消后重新运动」的形态")
        return 1

    print("通过：%s 后 %.3fs 内归零，并在随后 %.1fs 内保持静止" % (args.label, delay, args.observe))
    return 0


if __name__ == "__main__":
    sys.exit(main())
