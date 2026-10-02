#!/usr/bin/env python3
"""高度跟随判据：按**预期形状**分别检查上坡 / 下坡 / 跨越坡顶 / 平地。

为什么重写（Codex 复审指出）：
    旧判据用 `abs(rise) < min_rise` 判断"有没有变化"，且只统计"单帧下降超过 2 cm"。
    结果是**持续下降 10 cm、每帧降 1 cm** 的数据会被判成"通过……上升 -0.100 m"，
    方向完全错了也算通过。

现在的规则：
    1. 必须到达指定终点（若给了 --goal-x/--goal-y）；
    2. 按 --expect 检查**方向与幅度**：
         uphill   z 末 - z 首 >= --min-rise，且全程不得出现超容差的下降；
         downhill z 首 - z 末 >= --min-drop，且全程不得出现超容差的上升；
         crest    先升 >= --min-rise、后降 >= --min-drop，且最高点落在路径中段；
         flat     |z 末 - z 首| <= --flat-tolerance，且全程起伏不超过该值；
    3. "与地面网格的偏差"仍计算并打印，但**只作为一致性证据**，不作为通过依据
       （模拟器与判据用同一张网格时该值恒为 0，本身没有信息量）。

失败返回非零。
"""

import argparse
import csv
import math
import sys


def load_ground(path):
    cell = x0 = y0 = None
    nx = ny = None
    values = []
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                if cell is None:
                    tokens = line.lstrip("# ").split()
                    try:
                        for i, tok in enumerate(tokens):
                            if i + 1 >= len(tokens):
                                break
                            if tok == "cell":
                                cell = float(tokens[i + 1])
                            elif tok == "x0":
                                x0 = float(tokens[i + 1])
                            elif tok == "y0":
                                y0 = float(tokens[i + 1])
                            elif tok == "nx":
                                nx = int(tokens[i + 1])
                            elif tok == "ny":
                                ny = int(tokens[i + 1])
                    except ValueError:
                        cell = x0 = y0 = None
                        nx = ny = None
                continue
            values.extend(float(v) for v in line.split())
    if None in (cell, x0, y0, nx, ny) or len(values) != nx * ny:
        raise RuntimeError("地面网格格式不对: %s" % path)
    return cell, x0, y0, nx, ny, values


def make_lookup(path):
    cell, x0, y0, nx, ny, v = load_ground(path)

    def at(x, y):
        # 与 C++ GroundHeightMap::heightAt 一致：先夹取连续坐标，再插值
        fx = min(max((x - x0) / cell, 0.0), float(nx - 1))
        fy = min(max((y - y0) / cell, 0.0), float(ny - 1))
        i0 = min(int(math.floor(fx)), nx - 1)
        j0 = min(int(math.floor(fy)), ny - 1)
        i1 = min(i0 + 1, nx - 1)
        j1 = min(j0 + 1, ny - 1)
        tx = fx - i0
        ty = fy - j0
        val = lambda i, j: v[j * nx + i]
        a = val(i0, j0) * (1 - tx) + val(i1, j0) * tx
        b = val(i0, j1) * (1 - tx) + val(i1, j1) * tx
        return a * (1 - ty) + b * ty

    return at


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--ground-grid", required=True)
    parser.add_argument("--body-height", type=float, default=0.125)
    parser.add_argument("--expect", required=True,
                        choices=["uphill", "downhill", "crest", "flat"])
    parser.add_argument("--min-rise", type=float, default=0.10,
                        help="上坡/坡顶前段要求的最小上升（米）")
    parser.add_argument("--min-drop", type=float, default=0.10,
                        help="下坡/坡顶后段要求的最小下降（米）")
    parser.add_argument("--flat-tolerance", type=float, default=0.05)
    parser.add_argument("--monotonic-tolerance", type=float, default=0.03,
                        help="单调性容差：允许的逆向变化幅度（米）")
    parser.add_argument("--consistency-tolerance", type=float, default=0.06,
                        help="z 与 地面+机体高度 的允许偏差（仅作一致性证据）")
    parser.add_argument("--goal-x", type=float, default=None)
    parser.add_argument("--goal-y", type=float, default=None)
    parser.add_argument("--goal-tolerance", type=float, default=0.30)
    parser.add_argument("--min-samples", type=int, default=200)
    args = parser.parse_args()

    at = make_lookup(args.ground_grid)
    rows = []
    with open(args.csv) as handle:
        for r in csv.DictReader(handle):
            rows.append((float(r["t"]), float(r["x"]), float(r["y"]), float(r["z"])))
    rows.sort()
    if len(rows) < args.min_samples:
        print("失败：轨迹样本过少（%d < %d）" % (len(rows), args.min_samples))
        return 2

    xs = [r[1] for r in rows]
    ys = [r[2] for r in rows]
    zs = [r[3] for r in rows]
    z0, z1 = zs[0], zs[-1]
    moved = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])

    print("样本 %d，水平位移 %.2f m" % (len(rows), moved))
    print("z: %.3f -> %.3f m（变化 %+.3f），最低 %.3f 最高 %.3f" % (
        z0, z1, z1 - z0, min(zs), max(zs)))
    worst_dev = max(abs(z - (at(x, y) + args.body_height)) for (_, x, y, z) in rows)
    print("与地面网格的期望值偏差（一致性证据，非通过依据）: 最大 %.4f m（容差 %.3f）"
          % (worst_dev, args.consistency_tolerance))

    failures = []
    if moved < 0.5:
        failures.append("水平位移只有 %.2f m，机器人基本没动" % moved)

    if args.goal_x is not None and args.goal_y is not None:
        d = math.hypot(xs[-1] - args.goal_x, ys[-1] - args.goal_y)
        print("终点到指定目标 %.3f m（容差 %.2f）" % (d, args.goal_tolerance))
        if d > args.goal_tolerance:
            failures.append("未到达指定终点（差 %.3f m）" % d)

    tol = args.monotonic_tolerance

    def worst_reverse(values, forward=True):
        """全程逆向变化的最大幅度（forward=True 表示期望递增）。"""
        worst = 0.0
        for i in range(1, len(values)):
            delta = values[i] - values[i - 1]
            worse = -delta if forward else delta
            worst = max(worst, worse)
        return worst

    if args.expect == "uphill":
        rise = z1 - z0
        print("上坡检查：要求上升 >= %.3f m" % args.min_rise)
        if rise < args.min_rise:
            failures.append("上升不足（%+.3f m < %.3f m）" % (rise, args.min_rise))
        rev = worst_reverse(zs, forward=True)
        print("  全程最大逆向下降 %.4f m（容差 %.3f）" % (rev, tol))
        if rev > tol:
            failures.append("上坡途中出现 %.3f m 的下降，方向不一致" % rev)

    elif args.expect == "downhill":
        drop = z0 - z1
        print("下坡检查：要求下降 >= %.3f m" % args.min_drop)
        if drop < args.min_drop:
            failures.append("下降不足（%+.3f m < %.3f m）" % (drop, args.min_drop))
        rev = worst_reverse(zs, forward=False)
        print("  全程最大逆向上升 %.4f m（容差 %.3f）" % (rev, tol))
        if rev > tol:
            failures.append("下坡途中出现 %.3f m 的上升，方向不一致" % rev)

    elif args.expect == "crest":
        peak = max(zs)
        i_peak = zs.index(peak)
        # 用**累计路程**而不是采样序号来定位峰值：机器人在路线跑完后会原地停留，
        # 按序号会把峰值挤到前面（这是本轮修掉的一个判据缺陷）。
        cumulative = [0.0]
        for i in range(1, len(rows)):
            cumulative.append(cumulative[-1] + math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]))
        total_dist = cumulative[-1]
        frac = cumulative[i_peak] / total_dist if total_dist > 1e-6 else 0.0
        rise = peak - z0
        drop = peak - z1
        print("坡顶检查：升 %.3f m（要求 >= %.3f），降 %.3f m（要求 >= %.3f），"
              "最高点位于路程 %.0f%%（要求 15%%~85%%，总路程 %.2f m）"
              % (rise, args.min_rise, drop, args.min_drop, frac * 100, total_dist))
        if rise < args.min_rise:
            failures.append("坡前上升不足（%.3f m）" % rise)
        if drop < args.min_drop:
            failures.append("坡后下降不足（%.3f m）" % drop)
        if not 0.15 <= frac <= 0.85:
            failures.append("最高点位置异常（路程 %.0f%%），不像跨越坡顶" % (frac * 100))

    else:  # flat
        span = max(zs) - min(zs)
        print("平地检查：要求全程起伏 <= %.3f m" % args.flat_tolerance)
        if abs(z1 - z0) > args.flat_tolerance:
            failures.append("首末高度差 %.3f m 超过平地容差" % abs(z1 - z0))
        if span > args.flat_tolerance:
            failures.append("全程起伏 %.3f m 超过平地容差" % span)

    if failures:
        print("\n失败：")
        for f in failures:
            print("  - " + f)
        return 1

    print("\n通过：%s 形状符合预期，且到达指定终点" % args.expect)
    return 0


if __name__ == "__main__":
    sys.exit(main())
