#!/usr/bin/env python3
"""可控洞口场景：平地 -> 上坡 -> 平台(带洞) -> 下坡 -> 平地。

用途（Codex 复审要求）：把"开口几何够不够"与"地图/判据有没有问题"分开。
本场景的地面高度与障碍都是**解析给定**的，不经过任何地图处理，因此结论唯一。

几何（沿 +y 前进，走廊沿 x 居中）：
    y ∈ [-2.0,  0.0]   平地    z = 0
    y ∈ [ 0.0,  1.5]   上坡    z = tan(20°) · y
    y ∈ [ 1.5,  4.0]   平台    z = 0.546
    y ∈ [ 4.0,  5.5]   下坡    z = 0.546 − tan(20°) · (y − 4.0)
    y ∈ [ 5.5,  7.0]   平地    z = 0
    两侧走廊墙 x = ±0.80（全高 1.0 m）
    洞：y ∈ [2.2, 2.8]，洞壁 x = ±0.45，洞顶 z = 平台 + --roof-height
    矮障碍：x ∈ [0.55, 0.75]、y ∈ [-1.6, -1.2]、高 0.12 m（在路线旁，不被路线挡住，
            用于验证"矮障碍没有被地图处理悄悄删掉"）

两个变体：
    tunnel_high  --roof-height 0.45  -> 机器人 0.25 m + 包络 z±0.125 应当能过
    tunnel_low   --roof-height 0.20  -> 洞顶侵入机体高度带，必须拒绝

输出：<name>.pcd（障碍）、<name>_ground.pcd（地形表面）、<name>_ground.txt（高度网格）、
      <name>_facts.txt（期望值，供判据核对）
"""

import argparse
import math
import os
import struct

CELL = 0.10
X_MIN, X_MAX = -1.00, 1.00
Y_MIN, Y_MAX = -2.50, 7.50
CORRIDOR_HALF = 0.80
TUNNEL_HALF = 0.45
TUNNEL_Y0, TUNNEL_Y1 = 2.20, 2.80
SLOPE = math.tan(math.radians(20.0))
PLATEAU_Z = SLOPE * 1.5
LOW_BOX = (0.55, 0.75, -1.60, -1.20, 0.12)   # x0,x1,y0,y1,height


def ground_profile(y):
    if y <= 0.0:
        return 0.0
    if y <= 1.5:
        return SLOPE * y
    if y <= 4.0:
        return PLATEAU_Z
    if y <= 5.5:
        return max(0.0, PLATEAU_Z - SLOPE * (y - 4.0))
    return 0.0


def wall_segment(p0, p1, z0, z1, step=0.04):
    pts = []
    length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    n = max(1, int(length / step))
    for i in range(n + 1):
        t = i / n
        x = p0[0] + t * (p1[0] - p0[0])
        y = p0[1] + t * (p1[1] - p0[1])
        # z 从 z0 到 z1 线性铺满（用于斜墙/斜洞壁）
        m = max(1, int(abs(z1 - z0) / step))
        for k in range(m + 1):
            z = z0 + (z1 - z0) * k / m
            pts.append((round(x, 4), round(y, 4), round(z, 4)))
    return pts


def slab(x0, x1, y0, y1, z, step=0.04):
    pts = []
    nx = max(1, int((x1 - x0) / step))
    ny = max(1, int((y1 - y0) / step))
    for i in range(nx + 1):
        for j in range(ny + 1):
            pts.append((round(x0 + (x1 - x0) * i / nx, 4),
                        round(y0 + (y1 - y0) * j / ny, 4), round(z, 4)))
    return pts


def box(x0, x1, y0, y1, z0, z1, step=0.04):
    pts = set()
    pts |= set(slab(x0, x1, y0, y1, z1, step))
    pts |= set(wall_segment((x0, y0), (x1, y0), z0, z1, step))
    pts |= set(wall_segment((x0, y1), (x1, y1), z0, z1, step))
    pts |= set(wall_segment((x0, y0), (x0, y1), z0, z1, step))
    pts |= set(wall_segment((x1, y0), (x1, y1), z0, z1, step))
    return sorted(pts)


def build(roof_height):
    pts = []
    # 走廊侧墙，随地面起伏（贴地建墙）
    step = 0.04
    y = Y_MIN
    while y <= Y_MAX + 1e-9:
        g = ground_profile(y)
        for xs in (-CORRIDOR_HALF, +CORRIDOR_HALF):
            z = g
            while z <= g + 1.0 + 1e-9:
                pts.append((xs, round(y, 4), round(z, 4)))
                z += step
        y += step
    # 洞壁（比洞顶再高一点，形成真正需要穿过的门）
    for xs in (-TUNNEL_HALF, +TUNNEL_HALF):
        pts += wall_segment((xs, TUNNEL_Y0), (xs, TUNNEL_Y1),
                            ground_profile(TUNNEL_Y0), PLATEAU_Z + roof_height + 0.10)
    # 洞顶
    pts += slab(-TUNNEL_HALF, TUNNEL_HALF, TUNNEL_Y0, TUNNEL_Y1, PLATEAU_Z + roof_height)
    # 矮障碍（路线旁）
    x0, x1, y0, y1, h = LOW_BOX
    pts += box(x0, x1, y0, y1, ground_profile((y0 + y1) / 2), ground_profile((y0 + y1) / 2) + h)
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    nx = int(round((X_MAX - X_MIN) / CELL)) + 1
    ny = int(round((Y_MAX - Y_MIN) / CELL)) + 1
    index = ["# 可控洞口场景（解析给定，示例尺寸，非实车标定）",
             "# 走廊半宽 %.2f m，洞半宽 %.2f m，洞 y∈[%.2f, %.2f]" % (
                 CORRIDOR_HALF, TUNNEL_HALF, TUNNEL_Y0, TUNNEL_Y1),
             "# 坡 20°，平台 z=%.3f m；机器人半径 0.26 m、高 0.25 m，包络 z±0.125" % PLATEAU_Z,
             "name\troof_height\tclear_height\tpoints\texpect"]

    for name, roof in (("tunnel_high", 0.45), ("tunnel_low", 0.20)):
        write_pcd(os.path.join(args.out_dir, name + ".pcd"), build(roof))

        with open(os.path.join(args.out_dir, name + "_ground.txt"), "w") as handle:
            handle.write("# controllable tunnel scene ground grid\n")
            handle.write("# cell %.6f x0 %.6f y0 %.6f nx %d ny %d\n"
                         % (CELL, X_MIN, Y_MIN, nx, ny))
            handle.write("# rows: j=0..ny-1 (y ascending), each row nx values (x ascending)\n")
            for j in range(ny):
                y = Y_MIN + j * CELL
                row = ["%.5f" % ground_profile(y) for _ in range(nx)]
                handle.write(" ".join(row) + "\n")

        surf = []
        for j in range(ny):
            y = Y_MIN + j * CELL
            for i in range(nx):
                surf.append((round(X_MIN + i * CELL, 3), round(y, 3),
                             round(ground_profile(y), 4)))
        write_pcd(os.path.join(args.out_dir, name + "_ground.pcd"), surf)

        expect = "pass" if roof > 0.125 + 0.25 else "refuse"
        with open(os.path.join(args.out_dir, name + "_facts.txt"), "w") as handle:
            handle.write("roof_height=%.3f\n" % roof)
            handle.write("clear_height=%.3f\n" % (PLATEAU_Z + roof))
            handle.write("plateau_z=%.3f\n" % PLATEAU_Z)
            handle.write("tunnel_y0=%.3f\ntunnel_y1=%.3f\n" % (TUNNEL_Y0, TUNNEL_Y1))
            handle.write("tunnel_half_width=%.3f\n" % TUNNEL_HALF)
            handle.write("low_box=%.2f %.2f %.2f %.2f %.2f\n" % LOW_BOX)
            handle.write("expect=%s\n" % expect)
            handle.write("robot_z_band=0.25\n")
            handle.write("z_inflation=0.125\n")

        print("%-12s 洞顶高 %.2f m（离平台）  洞内净空 z=%.3f m  预期 %s"
              % (name, roof, PLATEAU_Z + roof, expect))
        index.append("%s\t%.2f\t%.3f\t-\t%s" % (name, roof, PLATEAU_Z + roof, expect))

    with open(os.path.join(args.out_dir, "INDEX.txt"), "w") as handle:
        handle.write("\n".join(index) + "\n")
    print("\n起点 (0.0, -2.0)；终点 (0.0, 7.0)")
    print("预期：洞顶 0.45 m -> 通过；洞顶 0.20 m -> 拒绝（洞顶侵入 [地面, 地面+0.25] 高度带）")
    print("索引写入 %s/INDEX.txt" % args.out_dir)


if __name__ == "__main__":
    main()
