#!/usr/bin/env python3
"""地形分离：把地图拆成「地面」与「障碍」，并输出地面高度网格。

为什么不用按绝对高度删点（上一版做法）：
    那样会把地面和矮障碍一起删掉。`rmuc2026_field.pcd` 里地面本身是**平滑穹形**
    （边缘约 -6 cm，中心约 +14 cm），中央另有约 0.6 m 的结构；
    0.30 m 的绝对阈值把 0~30 cm 整层删除，正好删掉整片缓坡与低矮场地元素。

做法：
    1. 按网格取每格 z 的低分位数，作为该格的地面候选；
    2. 用**稳健迭代**拟合一个二次曲面 z = f(x,y)（Huber 式重加权：反复丢弃
       残差大的格再拟合）。这样被结构占满的格（例如中央结构）会被当作离群点剔除，
       而不是把结构顶面当成地面；
    3. 障碍 = z - ground(x, y) >= --obstacle-height 的点，其余算地面点。

输出（同一前缀）：
    <prefix>_obstacles.pcd   障碍点，喂给 SCAN（地面不能进 SCAN 的占据栅格）
    <prefix>_ground.pcd      地面网格点，仅用于 RViz 显示
    <prefix>_ground.txt      地面高度网格，供运动模拟器/规划器查询 z
    <prefix>_report.txt      统计与诊断

注意：这里得到的「地面」是**地图自身表达的地面**。它可能来自真实场地起伏，
也可能是建图过程中的曲率/回环伪影——需要场地owner确认。仿真一律以地图为准。
"""

import argparse
import array
import math
import os
import struct


# ---------------------------------------------------------------- PCD I/O

def read_pcd_xyz(path):
    with open(path, "rb") as handle:
        header = {}
        while True:
            line = handle.readline()
            if not line:
                raise RuntimeError("PCD 头部不完整")
            text = line.decode("ascii", "replace").strip()
            if not text or text.startswith("#"):
                continue
            key, _, value = text.partition(" ")
            header[key.upper()] = value
            if key.upper() == "DATA":
                break
        if header.get("DATA", "").strip() != "binary":
            raise RuntimeError("仅支持 binary PCD")
        fields = header["FIELDS"].split()
        if fields[:3] != ["x", "y", "z"]:
            raise RuntimeError("PCD 前三字段必须是 x y z")
        count = int(header["POINTS"])
        raw = handle.read(count * 12)
    values = array.array("f")
    values.frombytes(raw[: (len(raw) // 12) * 12])
    return [(values[i], values[i + 1], values[i + 2]) for i in range(0, len(values), 3)]


def write_pcd_xyz(path, points):
    with open(path, "wb") as handle:
        handle.write((
            "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
            "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
            % (len(points), len(points))).encode("ascii"))
        handle.write(struct.pack("<%df" % (3 * len(points)),
                                 *[c for p in points for c in p]))


# ---------------------------------------------------------------- 稳健二次曲面

def design(x, y):
    return (1.0, x, y, x * x, y * y, x * y)


def solve_normal_equations(rows, weights):
    """最小二乘解 6 参数二次曲面。rows: [(x,y,z)]。"""
    n = 6
    ata = [[0.0] * n for _ in range(n)]
    atb = [0.0] * n
    for (x, y, z), w in zip(rows, weights):
        if w <= 0.0:
            continue
        phi = design(x, y)
        for i in range(n):
            atb[i] += w * phi[i] * z
            for j in range(n):
                ata[i][j] += w * phi[i] * phi[j]
    # 高斯消元（带部分主元）
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(ata[r][col]))
        if abs(ata[piv][col]) < 1e-12:
            return None
        ata[col], ata[piv] = ata[piv], ata[col]
        atb[col], atb[piv] = atb[piv], atb[col]
        for r in range(col + 1, n):
            f = ata[r][col] / ata[col][col]
            for c in range(col, n):
                ata[r][c] -= f * ata[col][c]
            atb[r] -= f * atb[col]
    coef = [0.0] * n
    for r in range(n - 1, -1, -1):
        s = atb[r] - sum(ata[r][c] * coef[c] for c in range(r + 1, n))
        coef[r] = s / ata[r][r]
    return coef


def eval_surface(coef, x, y):
    phi = design(x, y)
    return sum(c * p for c, p in zip(coef, phi))


def robust_fit(rows, iterations=6, huber=0.10, verbose=True):
    """迭代重加权：先普通最小二乘，再反复丢弃残差大的格。"""
    weights = [1.0] * len(rows)
    coef = None
    for it in range(iterations):
        coef = solve_normal_equations(rows, weights)
        if coef is None:
            raise RuntimeError("二次曲面拟合失败（奇异）")
        residuals = [abs(z - eval_surface(coef, x, y)) for (x, y, z) in rows]
        scale = sorted(residuals)[int(0.5 * len(residuals))] or 1e-6
        # Huber：残差超过阈值的格降权，超过 3 倍的直接丢弃
        weights = []
        dropped = 0
        for r in residuals:
            if r > 3.0 * max(huber, 3.0 * scale):
                weights.append(0.0)
                dropped += 1
            elif r > huber:
                weights.append(huber / r)
            else:
                weights.append(1.0)
        used = sum(1 for w in weights if w > 0.0)
        if verbose:
            rms = math.sqrt(sum(r * r for r in residuals) / len(residuals))
            print("  迭代 %d: 使用 %d/%d 格, RMS 残差 %.4f m, 丢弃 %d"
                  % (it + 1, used, len(rows), rms, dropped))
    return coef


# ---------------------------------------------------------------- 主流程

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--out-prefix", required=True)
    parser.add_argument("--cell", type=float, default=0.5,
                        help="地面候选网格边长（米）")
    parser.add_argument("--ground-percentile", type=float, default=0.05,
                        help="每格地面候选取 z 的该分位数")
    parser.add_argument("--min-points", type=int, default=8,
                        help="格内点数少于此值不参与地面拟合")
    parser.add_argument("--obstacle-height", type=float, default=0.08,
                        help="高于拟合地面该值算障碍（米）")
    args = parser.parse_args()

    print("读取 %s ..." % args.input)
    points = read_pcd_xyz(args.input)
    print("点数 %d" % len(points))

    cell = args.cell
    cells = {}
    for x, y, z in points:
        key = (int(math.floor(x / cell)), int(math.floor(y / cell)))
        cells.setdefault(key, []).append(z)

    candidates = []
    for key, zs in cells.items():
        if len(zs) < args.min_points:
            continue
        zs.sort()
        idx = min(len(zs) - 1, int(args.ground_percentile * len(zs)))
        candidates.append(((key[0] + 0.5) * cell, (key[1] + 0.5) * cell, zs[idx]))
    print("参与地面拟合的格: %d / %d" % (len(candidates), len(cells)))

    print("稳健拟合二次地面曲面 ...")
    coef = robust_fit(candidates, verbose=True)
    print("地面曲面: z = %.4f + %.5f x + %.5f y + %.6f x^2 + %.6f y^2 + %.6f xy"
          % tuple(coef))

    ground_vals = [eval_surface(coef, x, y) for (x, y, _) in candidates]
    print("拟合地面高度范围: [%.3f, %.3f] m" % (min(ground_vals), max(ground_vals)))

    obstacles, ground_pts = [], []
    heights = []
    for x, y, z in points:
        g = eval_surface(coef, x, y)
        h = z - g
        heights.append(h)
        if h >= args.obstacle_height:
            obstacles.append((x, y, z))
        else:
            ground_pts.append((x, y, z))

    write_pcd_xyz(args.out_prefix + "_obstacles.pcd", obstacles)

    # 地面高度网格：供运动模拟器/规划器查询（双线性插值）
    gx = [k[0] for k in cells]
    gy = [k[1] for k in cells]
    x0, x1, y0, y1 = min(gx), max(gx), min(gy), max(gy)
    nx, ny = x1 - x0 + 1, y1 - y0 + 1
    with open(args.out_prefix + "_ground.txt", "w") as handle:
        handle.write("# terrain ground height grid (bilinear lookup)\n")
        handle.write("# cell %.6f x0 %.6f y0 %.6f nx %d ny %d\n"
                     % (cell, x0 * cell, y0 * cell, nx, ny))
        handle.write("# rows: j=0..ny-1 (y ascending), each row nx values (x ascending)\n")
        for j in range(ny):
            row = []
            for i in range(nx):
                row.append("%.5f" % eval_surface(coef, (x0 + i + 0.5) * cell,
                                                 (y0 + j + 0.5) * cell))
            handle.write(" ".join(row) + "\n")

    # 地形表面点云：每个地面网格一个点，仅用于 RViz 显示地形起伏。
    # 注意地面点绝不能喂给 SCAN 的占据栅格，因此走独立话题。
    surface = []
    for j in range(y0, y1 + 1):
        for i in range(x0, x1 + 1):
            cx = (i + 0.5) * cell
            cy = (j + 0.5) * cell
            surface.append((round(cx, 3), round(cy, 3), round(eval_surface(coef, cx, cy), 4)))
    write_pcd_xyz(args.out_prefix + "_surface.pcd", surface)

    heights.sort()
    report = [
        "输入: %s" % args.input,
        "总点数: %d" % len(points),
        "地面曲面系数 (1,x,y,x^2,y^2,xy): %s" % ", ".join("%.6f" % c for c in coef),
        "拟合地面高度范围: [%.3f, %.3f] m" % (min(ground_vals), max(ground_vals)),
        "高于地面的高度分布: p05=%.3f p50=%.3f p95=%.3f max=%.3f m" % (
            heights[len(heights) // 20], heights[len(heights) // 2],
            heights[len(heights) * 19 // 20], heights[-1]),
        "障碍阈值: %.3f m" % args.obstacle_height,
        "障碍点: %d (%.1f%%)" % (len(obstacles), 100.0 * len(obstacles) / len(points)),
        "地面点: %d (%.1f%%)，地形表面网格点: %d" % (
            len(ground_pts), 100.0 * len(ground_pts) / len(points), len(surface)),
        "",
        "对照：上一版按绝对高度删点保留 %d 点（删掉了 0~30cm 整层，含整片缓坡）",
        "本方法保留全部 %d 点，只把「高于地面 %.2f m」的点标为障碍。"
        % (len(points), args.obstacle_height),
    ]
    with open(args.out_prefix + "_report.txt", "w") as handle:
        handle.write("\n".join(report) + "\n")
    print()
    print("\n".join(report[:8]))
    print("\n输出: %s_{obstacles.pcd,surface.pcd,ground.txt,report.txt}" % args.out_prefix)


if __name__ == "__main__":
    main()
