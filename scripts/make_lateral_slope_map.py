#!/usr/bin/env python3
"""横向高度变化 + 需要绕障的场景：检验"规划检查的高度"与"实际执行的高度"是否一致。

为什么需要这个场景（Codex 第二轮指出）：
    高度修正在优化之前完成时，绕障会改变 XY，而 z 仍沿用优化前的位置。
    一条笔直坡道通过**覆盖不到**这个问题——必须在**横向有高度变化**的坡面上绕障。

几何（解析给定）：
    走廊沿 y：y ∈ [-2, 6]，两侧墙 x = ±1.5
    地面横向倾斜：z = tan(--angle) · x       （x 变化 -> 高度变化）
    中央障碍：x ∈ [-0.5, 0.5]、y ∈ [1.5, 2.0]，从地面到地面+0.5 m
              机器人半径 0.26 -> 必须绕到 |x| >= 0.76
    地面在 x=0 为 0，在 x=0.8 为 tan(角度)·0.8 -> 绕障必然带来高度变化

判据见 scripts/check_tracked_height.py。
"""

import argparse
import math
import os
import struct

CELL = 0.10
X_MIN, X_MAX = -1.60, 1.60
Y_MIN, Y_MAX = -2.50, 6.50
WALL_X = 1.50
BLOCK = (-0.50, 0.50, 1.50, 2.00, 0.50)   # x0,x1,y0,y1,height
WALL_H = 0.60


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--angle", type=float, default=12.0,
                        help="横向坡度（度）")
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    slope = math.tan(math.radians(args.angle))

    def ground(x, y):
        return slope * x

    pts = []
    step = 0.04
    # 侧墙
    y = Y_MIN
    while y <= Y_MAX + 1e-9:
        for xs in (-WALL_X, WALL_X):
            z = ground(xs, y)
            for k in range(int(WALL_H / step) + 1):
                pts.append((xs, round(y, 4), round(z + k * step, 4)))
        y += step
    # 中央障碍（逼迫横向绕行）
    x0, x1, y0, y1, h = BLOCK
    for i in range(int((x1 - x0) / step) + 1):
        x = x0 + i * step
        for j in range(int((y1 - y0) / step) + 1):
            yy = y0 + j * step
            g = ground(x, yy)
            for k in range(int(h / step) + 1):
                pts.append((round(x, 4), round(yy, 4), round(g + k * step, 4)))
            # 顶面
            pts.append((round(x, 4), round(yy, 4), round(g + h, 4)))

    pts = sorted(set(pts))
    with open(os.path.join(args.out_dir, "lateral_slope.pcd"), "wb") as handle:
        handle.write((
            "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
            "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
            % (len(pts), len(pts))).encode("ascii"))
        handle.write(struct.pack("<%df" % (3 * len(pts)), *[c for p in pts for c in p]))

    nx = int(round((X_MAX - X_MIN) / CELL)) + 1
    ny = int(round((Y_MAX - Y_MIN) / CELL)) + 1
    with open(os.path.join(args.out_dir, "lateral_slope_ground.txt"), "w") as handle:
        handle.write("# lateral slope scene ground grid\n")
        handle.write("# cell %.6f x0 %.6f y0 %.6f nx %d ny %d\n"
                     % (CELL, X_MIN, Y_MIN, nx, ny))
        handle.write("# rows: j=0..ny-1 (y ascending), each row nx values (x ascending)\n")
        for j in range(ny):
            y = Y_MIN + j * CELL
            handle.write(" ".join("%.5f" % ground(X_MIN + i * CELL, y)
                                  for i in range(nx)) + "\n")

    surf = []
    for j in range(ny):
        y = Y_MIN + j * CELL
        for i in range(nx):
            x = X_MIN + i * CELL
            surf.append((round(x, 3), round(y, 3), round(ground(x, y), 4)))
    with open(os.path.join(args.out_dir, "lateral_slope_ground.pcd"), "wb") as handle:
        handle.write((
            "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
            "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
            % (len(surf), len(surf))).encode("ascii"))
        handle.write(struct.pack("<%df" % (3 * len(surf)), *[c for p in surf for c in p]))

    print("横向坡度 %.0f°（tan=%.4f），走廊 x∈[±%.2f]，障碍 x∈[%.2f,%.2f] y∈[%.2f,%.2f] 高 %.2f m"
          % (args.angle, slope, WALL_X, BLOCK[0], BLOCK[1], BLOCK[2], BLOCK[3], BLOCK[4]))
    print("x=0 处地面 0.000 m；绕到 x=+0.80 处地面 %.3f m；绕到 x=-0.80 处地面 %.3f m"
          % (ground(0.8, 0), ground(-0.8, 0)))
    print("=> 横向绕障必然带来 >= %.3f m 的高度变化" % abs(ground(0.8, 0)))
    print("起点 (0.0, -1.5)；终点 (0.0, 5.5)")
    print("输出：lateral_slope.pcd / _ground.txt / _ground.pcd 于 %s" % args.out_dir)


if __name__ == "__main__":
    main()
