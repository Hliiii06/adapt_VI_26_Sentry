#!/usr/bin/env python3
"""通过/拒绝判据（供 scenario.sh 使用），失败返回非零。

两种模式：
  --expect reach    终点必须在目标容差内（Mode 1 到达类场景）
  --expect pass     终点必须越过 gate-y（例如通过一段通道）
  --expect refuse   终点必须停在 gate-y 之前（例如通道窄于车体时必须拒绝）

对 refuse 场景，"没动"和"动了一段后停下"都算拒绝，但会分别报出来，
以便区分"规划器拒绝执行"和"走到通道口被拦下"。
"""

import argparse
import csv
import math
import sys


def load(path):
    rows = []
    with open(path) as handle:
        for r in csv.DictReader(handle):
            rows.append((float(r["t"]), float(r["x"]), float(r["y"])))
    rows.sort()
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--expect", required=True,
                        choices=["reach", "pass", "refuse", "none"])
    parser.add_argument("--gate-y", type=float, default=0.0)
    parser.add_argument("--goal-x", type=float, default=0.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--tolerance", type=float, default=0.20)
    args = parser.parse_args()

    rows = load(args.csv)
    if not rows:
        print("失败：轨迹为空")
        return 2
    x0, y0 = rows[0][1], rows[0][2]
    x1, y1 = rows[-1][1], rows[-1][2]
    print("起点 (%.3f, %.3f) -> 终点 (%.3f, %.3f)" % (x0, y0, x1, y1))

    if args.expect == "none":
        return 0

    if args.expect == "reach":
        d = math.dist((x1, y1), (args.goal_x, args.goal_y))
        print("终点到目标距离 %.3f m（容差 %.2f）" % (d, args.tolerance))
        if d > args.tolerance:
            print("失败：未到达目标")
            return 1
        print("通过：到达目标")
        return 0

    if args.expect == "pass":
        print("gate y=%.2f；要求终点越过 gate" % args.gate_y)
        if y1 <= args.gate_y:
            print("失败：未通过通道（终点 y=%.3f <= gate %.3f）" % (y1, args.gate_y))
            return 1
        print("通过：越过了通道")
        return 0

    # refuse
    print("gate y=%.2f；要求终点停在 gate 之前" % args.gate_y)
    if y1 >= args.gate_y:
        print("失败：车体进入了本应拒绝的通道（终点 y=%.3f >= gate %.3f）" % (y1, args.gate_y))
        return 1
    if abs(y1 - y0) < 0.05:
        print("通过：规划器拒绝执行，全程未移动（终点 y=%.3f）" % y1)
        return 0
    print("通过：走到 y=%.3f 后停下，未进入通道" % y1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
