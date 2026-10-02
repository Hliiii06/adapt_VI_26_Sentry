#!/usr/bin/env python3
"""生成带已知坡度的合成地形地图（用于定量验证高度跟随）。

与 docs/testing/maps/ 里那批碰撞测试地图的区别：
    那批是平的、用来验证**碰撞包络**；这批带**已知角度的坡**，用来验证**高度跟随**。

每个场景输出两个文件：
    <out>/<name>.pcd        障碍点云（只含侧墙，坡面本身属于地面，不进障碍）
    <out>/<name>_ground.txt 解析式地面高度网格（不用拟合，角度是给定的）

地形形状（沿 +y 前进）：
    y < y_base        平地，z = 0
    y_base..y_top     坡，z = tan(angle) * (y - y_base)         （上坡）
    y > y_top         平台，z = 高度上限
侧墙保证机器人不会绕开坡面。

用法: make_terrain_maps.py --out-dir docs/testing/maps/terrain
"""

import argparse
import math
import os
import struct

CELL = 0.1          # 地面网格分辨率（细一点，坡面才平滑）
ROOM_X = 1.2        # 半宽（侧墙在 ±ROOM_X）
Y_MIN, Y_MAX = -3.0, 6.0
WALL_Z1 = 1.2
STEP = 0.04


def wall_z(x, y0, y1, z0=0.0, z1=WALL_Z1, step=STEP):
    pts = []
    n = max(1, int(abs(y1 - y0) / step))
    for i in range(n + 1):
        y = y0 + (y1 - y0) * i / n
        z = z0
        while z <= z1 + 1e-9:
            pts.append((round(x, 4), round(y, 4), round(z, 4)))
            z += step
    return pts


def write_pcd(path, points):
    with open(path, "wb") as handle:
        handle.write((
            "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
            "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
            % (len(points), len(points))).encode("ascii"))
        handle.write(struct.pack("<%df" % (3 * len(points)),
                                 *[c for p in points for c in p]))


def make_scenario(angle_deg, y_base=0.0, y_top=3.0, length=4.0):
    """返回 (地形函数, 最高点)。坡顶之后是长度 length 的平台。"""
    slope = math.tan(math.radians(angle_deg))
    top_z = slope * (y_top - y_base)

    def terrain(x, y):
        if y <= y_base:
            return 0.0
        if y <= y_top:
            return slope * (y - y_base)
        return top_z if y <= y_top + length else top_z

    return terrain, top_z


SCENARIOS = [
    ("ramp_10deg", 10.0, "10° 上坡，应被跟随"),
    ("ramp_20deg", 20.0, "20° 上坡，应被跟随"),
    ("ramp_30deg", 30.0, "30° 上坡；最陡，用于观察跟随极限与声明局限"),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    x0 = -ROOM_X
    nx = int(round(2 * ROOM_X / CELL)) + 1
    y0 = Y_MIN
    ny = int(round((Y_MAX - Y_MIN) / CELL)) + 1

    index = ["# 合成坡度地形（示例尺寸，非实车标定）",
             "# 侧墙 x=±%.2f，机器人半径 0.26 m -> 侧向净空 %.2f m" % (ROOM_X, ROOM_X - 0.26),
             "# 地面网格 cell=%.2f x0=%.2f y0=%.2f nx=%d ny=%d" % (CELL, x0, y0, nx, ny),
             "name\tangle_deg\ttop_z\tfile"]
    for name, angle, desc in SCENARIOS:
        terrain, top_z = make_scenario(angle)
        # 障碍：只有两侧的墙
        obstacles = wall_z(-ROOM_X, Y_MIN, Y_MAX) + wall_z(+ROOM_X, Y_MIN, Y_MAX)
        write_pcd(os.path.join(args.out_dir, name + ".pcd"), obstacles)

        # 地形表面点云：仅用于 RViz 显示坡度（地面绝不能进 SCAN 的占据栅格）
        surf = []
        for j in range(ny):
            y = y0 + j * CELL
            for i in range(nx):
                x = x0 + i * CELL
                surf.append((round(x, 3), round(y, 3), round(terrain(x, y), 4)))
        write_pcd(os.path.join(args.out_dir, name + "_ground.pcd"), surf)

        with open(os.path.join(args.out_dir, name + "_ground.txt"), "w") as handle:
            handle.write("# terrain ground height grid (analytic, angle=%.1f deg)\n" % angle)
            handle.write("# cell %.6f x0 %.6f y0 %.6f nx %d ny %d\n" % (CELL, x0, y0, nx, ny))
            handle.write("# rows: j=0..ny-1 (y ascending), each row nx values (x ascending)\n")
            for j in range(ny):
                y = y0 + j * CELL
                handle.write(" ".join("%.5f" % terrain(x0 + i * CELL, y) for i in range(nx)) + "\n")

        print("%-12s %4.0f°  坡顶 z=%.3f m  障碍 %d 点  %s"
              % (name, angle, top_z, len(obstacles), desc))
        index.append("%s\t%.0f\t%.3f\t%s.pcd" % (name, angle, top_z, name))

    with open(os.path.join(args.out_dir, "INDEX.txt"), "w") as handle:
        handle.write("\n".join(index) + "\n")
    print("\n起点建议 (0.0, -2.0, 0.125)；目标 (0.0, 5.0)")
    print("索引写入 %s/INDEX.txt" % args.out_dir)


if __name__ == "__main__":
    main()
