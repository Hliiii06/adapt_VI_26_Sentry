#!/usr/bin/env python3
"""高度跟随判据：机器人 z 必须跟随地面高度网格，而不是保持固定。

检查项：
  1. 轨迹 z 与「地面(x,y) + body_height」的最大偏差在容差内；
  2. z 确实发生了变化（否则等于没跟随，可能是地形网格没加载）；
  3. 前进方向上 z 单调不减（上坡场景），排除"往回走"造成的假象。

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
        fx = (x - x0) / cell - 0.5
        fy = (y - y0) / cell - 0.5
        i0 = max(0, min(int(math.floor(fx)), nx - 1))
        j0 = max(0, min(int(math.floor(fy)), ny - 1))
        i1 = min(i0 + 1, nx - 1)
        j1 = min(j0 + 1, ny - 1)
        tx = min(max(fx - math.floor(fx), 0.0), 1.0)
        ty = min(max(fy - math.floor(fy), 0.0), 1.0)
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
    parser.add_argument("--tolerance", type=float, default=0.06,
                        help="z 与 地面+机体高度 的最大允许偏差（米）")
    parser.add_argument("--min-rise", type=float, default=0.05,
                        help="z 至少要有这么大的变化，否则视为没有跟随")
    args = parser.parse_args()

    at = make_lookup(args.ground_grid)
    rows = []
    with open(args.csv) as handle:
        for r in csv.DictReader(handle):
            rows.append((float(r["t"]), float(r["x"]), float(r["y"]), float(r["z"])))
    if len(rows) < 10:
        print("失败：轨迹样本过少（%d）" % len(rows))
        return 2
    rows.sort()

    errors = [abs(z - (at(x, y) + args.body_height)) for (_, x, y, z) in rows]
    worst = max(errors)
    rms = math.sqrt(sum(e * e for e in errors) / len(errors))

    z0, z1 = rows[0][3], rows[-1][3]
    rise = z1 - z0
    # 前进方向上的单调性（允许 2cm 抖动）
    decreasing = sum(1 for i in range(1, len(rows)) if rows[i][3] < rows[i - 1][3] - 0.02)

    print("轨迹样本: %d" % len(rows))
    print("z: %.3f -> %.3f m（变化 %+.3f m）" % (z0, z1, rise))
    print("期望 z = 地面(x,y) + %.3f" % args.body_height)
    print("偏差: 最大 %.4f m，RMS %.4f m（容差 %.3f）" % (worst, rms, args.tolerance))
    print("反向下降的采样点数: %d" % decreasing)

    failed = False
    if abs(rise) < args.min_rise:
        print("失败：z 几乎没有变化（%.4f m），不是高度跟随" % abs(rise))
        failed = True
    if worst > args.tolerance:
        print("失败：z 与地形期望偏差超过容差")
        failed = True
    if decreasing > max(3, len(rows) // 20):
        print("失败：z 在前进方向上反复下降，跟随不稳定")
        failed = True

    if failed:
        return 1
    print("通过：z 全程跟随地形（最大偏差 %.4f m，上升 %.3f m）" % (worst, rise))
    return 0


if __name__ == "__main__":
    sys.exit(main())
