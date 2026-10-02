#!/usr/bin/env python3
"""判据：机器人**全程没有动**。

为什么不能复用 check_stop.py：那个判据要求"事件前有明显非零命令"，用来证明
"先动后停"。而"目标在已知地面之外"这类场景本来就**不应该动**，
用 check_stop 会把它判成"场景无效"。

本判据只做一件事：整段记录里 cmd_vel 必须始终为零，且记录必须覆盖完整时长、
中间不缺样。配合调用方对日志的检查（例如"越界拒绝"必须出现），
才能区分"拒绝规划"与"根本没收到目标"。
"""

import argparse
import csv
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--duration", type=float, required=True,
                        help="要求覆盖的时长（秒）")
    parser.add_argument("--threshold", type=float, default=1e-3)
    parser.add_argument("--label", default="no motion")
    args = parser.parse_args()

    rows = []
    with open(args.csv) as handle:
        for r in csv.DictReader(handle):
            rows.append((float(r["t"]), float(r["vx"]), float(r["vy"]), float(r["wz"])))
    rows.sort()
    if len(rows) < 50:
        print("失败：样本过少（%d），无法证明全程静止" % len(rows))
        return 2

    nonzero = [r for r in rows if max(abs(r[1]), abs(r[2]), abs(r[3])) > args.threshold]
    peak = (max(abs(r[1]) for r in rows), max(abs(r[2]) for r in rows),
            max(abs(r[3]) for r in rows))
    covered = rows[-1][0] - rows[0][0]
    times = [r[0] for r in rows]
    dts = sorted(times[i + 1] - times[i] for i in range(len(times) - 1))
    median_dt = dts[len(dts) // 2] if dts else 0.0
    worst_gap = dts[-1] if dts else 0.0

    print("%s：%d 个样本，覆盖 %.2fs（要求 >= %.2fs）" % (args.label, len(rows), covered, args.duration))
    print("命令峰值 |vx|=%.4f |vy|=%.4f |wz|=%.4f（阈值 %.4f）"
          % (peak[0], peak[1], peak[2], args.threshold))
    print("采样间隔 中位 %.4fs，最大 %.4fs" % (median_dt, worst_gap))

    failures = []
    if covered < args.duration:
        failures.append("记录只覆盖 %.2fs，不足要求的 %.2fs" % (covered, args.duration))
    if worst_gap > max(0.25, 5.0 * median_dt):
        failures.append("记录中间有 %.3fs 缺样，该时段未被观测" % worst_gap)
    if nonzero:
        t, vx, vy, wz = nonzero[0]
        failures.append("全程并非静止：%d 个非零命令，首个 t=%.2f (vx=%.3f vy=%.3f wz=%.3f)"
                        % (len(nonzero), t, vx, vy, wz))

    if failures:
        print("\n失败：")
        for f in failures:
            print("  - " + f)
        return 1
    print("\n通过：全程无运动且观测完整")
    return 0


if __name__ == "__main__":
    sys.exit(main())
