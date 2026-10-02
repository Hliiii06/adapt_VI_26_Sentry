#!/usr/bin/env python3
"""把「原始点云 / 估计地面 / 分类障碍 / 机器人包络」叠在一张剖面上逐处核对。

Codex 第二轮复审的建议：不要因为"当前分类下该点附近有阻挡"就直接归因于
用户给的坐标有误；应把四者叠起来看，才能判断洞口内部是否存在**连续可行驶坡面**。

输出是一张沿某一方向的竖直剖面（默认沿 +y，即"穿过洞口"的方向）：
    '.' 原始点（未判为障碍）
    'o' 分类后的障碍点
    'g' 估计地面在该 x 上的高度线
    'R' 机器人包络（机体高度带 [地面, 地面+0.25]）在中心线上的范围
    '#' 原始点中高于地面 0.25 m 以上的结构（洞顶/墙）

用法：
    inspect_field_terrain.py --pcd <原始> --obstacles <分类后> --ground-grid <网格>
        --x 0.98 --y -6.18 --along y --half-span 3.0 --half-width 0.35
"""

import argparse
import array
import math
import sys

BODY = 0.125
ROBOT_H = 0.25


def read_ground(path):
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
                        pass
                continue
            values.extend(float(v) for v in line.split())
    if None in (cell, x0, y0, nx, ny) or len(values) != nx * ny:
        raise RuntimeError("地面网格格式不对: %s" % path)
    return cell, x0, y0, nx, ny, values


def make_lookup(path):
    cell, x0, y0, nx, ny, v = read_ground(path)

    def at(x, y):
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


def read_pcd(path):
    with open(path, "rb") as handle:
        count = None
        while True:
            line = handle.readline()
            if not line:
                raise RuntimeError("PCD 头部不完整")
            text = line.decode("ascii", "replace").strip()
            if not text or text.startswith("#"):
                continue
            key, _, value = text.partition(" ")
            if key.upper() == "POINTS":
                count = int(value)
            if key.upper() == "DATA":
                break
        raw = handle.read(count * 12)
    values = array.array("f")
    values.frombytes(raw[:(len(raw) // 12) * 12])
    return [(values[i], values[i + 1], values[i + 2]) for i in range(0, len(values), 3)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcd", required=True, help="原始点云")
    parser.add_argument("--obstacles", required=True, help="分类后的障碍点云")
    parser.add_argument("--ground-grid", required=True)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--along", choices=["x", "y"], default="y")
    parser.add_argument("--half-span", type=float, default=3.0)
    parser.add_argument("--half-width", type=float, default=0.35)
    parser.add_argument("--z-max", type=float, default=1.0)
    args = parser.parse_args()

    at = make_lookup(args.ground_grid)
    raw = read_pcd(args.pcd)
    obs = read_pcd(args.obstacles)
    obs_set = {(round(p[0], 3), round(p[1], 3), round(p[2], 3)) for p in obs}

    step = 0.05
    n = int(args.half_span / step)
    rows = int(args.z_max / 0.05)
    print("剖面：%s = %.2f .. %.2f（中心 %.3f, %.3f），横向 |Δ| <= %.2f，z 0..%.1f"
          % (args.along, -args.half_span, args.half_span, args.x, args.y,
             args.half_width, args.z_max))
    print("图例：'o'=分类障碍(会被判碰撞)  '.'=原始点(未判障碍)  "
          "'g'=估计地面  'T'=包络上沿(地面+0.25m)")

    accum = {}
    for px, py, pz in raw:
        if args.along == "y":
            d = py - args.y
            lat = abs(px - args.x)
        else:
            d = px - args.x
            lat = abs(py - args.y)
        if abs(d) > args.half_span or lat > args.half_width:
            continue
        col = int(round((d + args.half_span) / step))
        row = int(round(pz / 0.05))
        if row < 0 or row > rows:
            continue
        key = (col, row)
        is_obs = (round(px, 3), round(py, 3), round(pz, 3)) in obs_set
        # 同一格里有障碍点就标 o；否则记原始点
        if key not in accum or is_obs:
            accum[key] = "o" if is_obs else "."

    # 地面线与包络
    for col in range(n * 2 + 1):
        d = -args.half_span + col * step
        gx, gy = (args.x, args.y + d) if args.along == "y" else (args.x + d, args.y)
        g = at(gx, gy)
        rg = int(round(g / 0.05))
        rt = int(round((g + ROBOT_H) / 0.05))
        # 只画包络的**上下边界线**，不填充内部——填充会把点云盖住。
        if 0 <= rg <= rows and (col, rg) not in accum:
            accum[(col, rg)] = "g"
        if 0 <= rt <= rows and (col, rt) not in accum:
            accum[(col, rt)] = "T"

    # 打印
    for r in range(rows, -1, -1):
        line = "".join(accum.get((c, r), " ") for c in range(n * 2 + 1))
        print("z=%5.2f |%s" % (r * 0.05, line))
    print("        " + "".join("|" if c % 20 == 0 else " " for c in range(n * 2 + 1)))
    print("        每 20 列 = 1.0 m；中心在正中间（第 %d 列）" % n)

    # 地面高度剖面（判断洞内是否连续坡面）
    print("\n估计地面高度沿剖面（每 0.25 m 采样，用于看是否连续可行驶坡面）：")
    vals = []
    for i in range(0, n * 2 + 1, 5):
        d = -args.half_span + i * step
        gx, gy = (args.x, args.y + d) if args.along == "y" else (args.x + d, args.y)
        vals.append((d, at(gx, gy)))
    print("  " + "  ".join("%+.2f:%.3f" % (d, g) for d, g in vals))
    drops = [(vals[i][0], vals[i][1] - vals[i - 1][1])
             for i in range(1, len(vals)) if abs(vals[i][1] - vals[i - 1][1]) > 0.10]
    print("  相邻采样高差 > 0.10 m 的位置：%s"
          % ("无（地面连续）" if not drops else
             " ".join("%+.2f(Δ%+.3f)" % (d, dd) for d, dd in drops)))


if __name__ == "__main__":
    sys.exit(main())
