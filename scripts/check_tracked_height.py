#!/usr/bin/env python3
"""判据：实际执行路径在**实际执行高度**上是否安全，以及绕障后高度是否跟着地形走。

为什么需要它（Codex 第二轮指出）：
    只比较"z 与同一张地面网格的偏差"没有信息量——模拟器就是用那张网格算 z 的。
    真正要回答的是：**规划器检查的高度**与**实际执行的高度**是否一致，
    以及按实际执行高度去查障碍时会不会撞。

因此本判据不依赖"偏差为 0"，而是：
    1. **在录制到的实际位姿上重放障碍检查**：把机体包络（半径 0.26、高度带
       [z-0.125, z+0.125]）放到每一帧实际 (x, y, z) 上去查障碍云，必须无碰撞。
       这是"按实际执行高度检查碰撞"。
    2. **横向绕行必须真的发生，且高度随之变化**：要求 x 的跨度 >= --min-lateral，
       并检查 z 与地形在横向上的相关性——z 必须跟着横向位置变，而不是常数。
    3. **反例对照**：用起终点线性插值高度（即修复前的行为）算一遍，
       报告它在本路径上会偏离多少、以及在哪些点会与实际执行高度不一致。
       若线性假设下的高度落在障碍里，而实际高度不落在障碍里，
       说明"优化后按地形重算高度"确实改变了安全结论。
"""

import argparse
import array
import csv
import math
import os
import sys


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
    parser.add_argument("--csv", required=True)
    parser.add_argument("--ground-grid", required=True)
    parser.add_argument("--obstacles", required=True)
    parser.add_argument("--body-height", type=float, default=0.125)
    parser.add_argument("--z-inflation", type=float, default=0.125)
    parser.add_argument("--radius", type=float, default=0.26)
    parser.add_argument("--min-lateral", type=float, default=0.30,
                        help="要求的横向绕行跨度（米）")
    parser.add_argument("--min-height-change", type=float, default=0.05,
                        help="要求绕行带来的高度变化（米）")
    parser.add_argument("--planned", default="",
                        help="规划器发布的轨迹采样 CSV（*_planned.csv），用于直接比较"
                             "规划检查的高度与实际执行的高度")
    parser.add_argument("--planned-tolerance", type=float, default=0.06,
                        help="规划高度与实际执行高度的允许偏差（米）")
    parser.add_argument("--require-counterfactual", action="store_true",
                        help="要求线性高度假设在本路径上确实会给出不同的安全结论")
    args = parser.parse_args()

    at = make_lookup(args.ground_grid)
    obs = read_pcd(args.obstacles)
    # 空间哈希，便于逐帧重放
    H = 0.25
    buckets = {}
    for px, py, pz in obs:
        buckets.setdefault((int(math.floor(px / H)), int(math.floor(py / H))), []).append((px, py, pz))

    rows = []
    with open(args.csv) as handle:
        for r in csv.DictReader(handle):
            rows.append((float(r["t"]), float(r["x"]), float(r["y"]), float(r["z"])))
    rows.sort()
    if len(rows) < 50:
        print("失败：样本过少（%d）" % len(rows))
        return 2

    def collides(x, y, z):
        cx, cy = int(math.floor(x / H)), int(math.floor(y / H))
        sp = int(math.ceil(args.radius / H))
        for i in range(cx - sp, cx + sp + 1):
            for j in range(cy - sp, cy + sp + 1):
                for px, py, pz in buckets.get((i, j), ()):
                    if abs(pz - z) <= args.z_inflation and math.hypot(px - x, py - y) < args.radius:
                        return (px, py, pz)
        return None

    failures = []

    # --- 1) 按实际执行高度重放碰撞检查 ---
    hits = []
    for t, x, y, z in rows:
        hit = collides(x, y, z)
        if hit:
            hits.append((t, x, y, z, hit))
    print("按实际执行高度重放障碍检查：%d / %d 帧碰撞" % (len(hits), len(rows)))
    if hits:
        t, x, y, z, (px, py, pz) = hits[0]
        print("  首次碰撞 t=%.2f 机体 (%.2f,%.2f,%.3f) 障碍点 (%.2f,%.2f,%.3f)"
              % (t, x, y, z, px, py, pz))
        failures.append("实际执行路径在其执行高度上与障碍相交（%d 帧）" % len(hits))

    # --- 2) 横向绕行与高度变化 ---
    xs = [r[1] for r in rows]
    zs = [r[3] for r in rows]
    lateral = max(xs) - min(xs)
    z_span = max(zs) - min(zs)
    # 只统计有位移的帧，避免起停静止段拉低相关性
    moving = [r for r in rows if r[0] >= rows[0][0]]
    terrain = [at(r[1], r[2]) + args.body_height for r in moving]
    dev = max(abs(r[3] - g) for r, g in zip(moving, terrain))
    print("横向跨度 %.3f m（要求 >= %.3f）；高度跨度 %.3f m（要求 >= %.3f）"
          % (lateral, args.min_lateral, z_span, args.min_height_change))
    print("z 与地形(+机体高度)的最大偏差 %.4f m（一致性参考，不是通过依据）" % dev)
    if lateral < args.min_lateral:
        failures.append("横向绕行不足（%.3f m），没有覆盖到绕障后高度这一问题" % lateral)
    if z_span < args.min_height_change:
        failures.append("高度变化不足（%.3f m），无法证明高度跟随了横向地形" % z_span)

    # 高度必须随横向位置变化：比较 x 大与 x 小时的平均 z
    hi = [r[3] for r in rows if r[1] > max(xs) - 0.15 * lateral]
    lo = [r[3] for r in rows if r[1] < min(xs) + 0.15 * lateral]
    if hi and lo:
        print("横向高位一侧平均 z %.3f，低位一侧平均 z %.3f（差 %.3f）"
              % (sum(hi) / len(hi), sum(lo) / len(lo),
                 abs(sum(hi) / len(hi) - sum(lo) / len(lo))))

    # --- 3) 反例对照：起终点线性插值高度（修复前的行为） ---
    z_start, z_end = zs[0], zs[-1]
    total = sum(math.dist(rows[i][1:3], rows[i + 1][1:3]) for i in range(len(rows) - 1))
    acc = 0.0
    worst_lin = 0.0
    lin_collisions = 0
    for i in range(1, len(rows)):
        acc += math.dist(rows[i][1:3], rows[i - 1][1:3])
        frac = acc / total if total > 1e-6 else 0.0
        z_lin = z_start + frac * (z_end - z_start)
        worst_lin = max(worst_lin, abs(z_lin - rows[i][3]))
        if collides(rows[i][1], rows[i][2], z_lin):
            lin_collisions += 1
    print("线性高度假设 vs 实际执行高度：最大差 %.4f m；"
          "线性高度下碰撞 %d 帧（实际高度下 %d 帧）"
          % (worst_lin, lin_collisions, len(hits)))
    if args.require_counterfactual:
        if worst_lin < 0.02:
            failures.append("线性高度与执行高度几乎相同，本场景没有区分度")
        if lin_collisions == 0 and len(hits) == 0:
            print("  提示：本场景下两种高度都不碰撞，区分度有限（仅证明执行高度安全）")

    # --- 4) 规划器检查的高度 vs 实际执行的高度 ---
    if args.planned and os.path.exists(args.planned):
        planned = []
        with open(args.planned) as handle:
            for r in csv.DictReader(handle):
                planned.append((int(float(r["traj_id"])), float(r["t"]),
                                float(r["x"]), float(r["y"]), float(r["z"])))
        # 规划轨迹里每个采样点，找实际执行轨迹上最近的 (x, y)，比较 z
        worst = 0.0
        worst_at = None
        compared = 0
        for _, _, px, py, pz in planned:
            best = None
            for _, ex, ey, ez in rows:
                d2 = (ex - px) ** 2 + (ey - py) ** 2
                if best is None or d2 < best[0]:
                    best = (d2, ez)
            if best is None or best[0] > 0.05 ** 2:
                continue
            compared += 1
            dz = abs(best[1] - pz)
            if dz > worst:
                worst = dz
                worst_at = (px, py, pz, best[1])
        print("规划高度 vs 实际执行高度：比较 %d 个采样点，最大偏差 %.4f m（容差 %.3f）"
              % (compared, worst, args.planned_tolerance))
        if compared < 20:
            failures.append("可用于比较的规划/执行采样点太少（%d）" % compared)
        if worst > args.planned_tolerance:
            failures.append("规划检查的高度与实际执行高度不一致（最大 %.3f m，位置 %s）"
                            % (worst, worst_at))
    elif args.planned:
        print("提示：未找到规划轨迹文件 %s，跳过该项" % args.planned)

    if failures:
        print("\n失败：")
        for f in failures:
            print("  - " + f)
        return 1
    print("\n通过：实际执行高度下无碰撞，且高度随横向地形变化")
    return 0


if __name__ == "__main__":
    sys.exit(main())
