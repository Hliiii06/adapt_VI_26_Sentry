#!/usr/bin/env python3
"""从地面高度网格生成带高度剖面的 Mode 2 航点 / Mode 3 参考路线。

为什么要生成而不是手写 z：
    手写一组固定 z 在有坡度的场地上必然与实际地面不符；SCAN 的碰撞查询用的是轨迹 z，
    z 错了就会在错误的高度上做检查。这里让 z 直接来自同一个地面网格，
    保证「机器人高度 / 轨迹高度 / 地形」三者一致。

语义（与 SCAN 源码一致）：
    Mode 2  fsm.waypoints 的 z 被直接使用            -> 输出 ground + body_height
    Mode 3  参考路线 z 会再叠加 grid_map.body_height -> 输出 ground（不加）
    见 scan_replan_fsm.cpp 的 prepareReferenceWaypoints(..., body_height_, ...) 调用。

用法：
    make_terrain_route.py --ground-grid <_ground.txt> --mode 3 \
        --start -6 6 --goal -6 0 --out <yaml> [--obstacles <pcd>] [--max-clearance-check]
"""

import argparse
import math
import os
import sys


def load_ground_grid(path):
    cell = x0 = y0 = None
    nx = ny = None
    values = []
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                # 只认第一行含 "cell" 的头部；其它注释（例如 "each row nx values"）
                # 也含 nx/ny 这样的词，不能参与解析。
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
        raise RuntimeError("地面网格文件格式不对: %s" % path)
    return cell, x0, y0, nx, ny, values


class Ground:
    def __init__(self, path):
        self.cell, self.x0, self.y0, self.nx, self.ny, self.v = load_ground_grid(path)

    def at(self, x, y):
        # x0/y0 are sample coordinates, not lower cell edges (same as C++).
        fx = min(max((x - self.x0) / self.cell, 0.0), self.nx - 1)
        fy = min(max((y - self.y0) / self.cell, 0.0), self.ny - 1)
        i0 = max(0, min(int(math.floor(fx)), self.nx - 1))
        j0 = max(0, min(int(math.floor(fy)), self.ny - 1))
        i1 = min(i0 + 1, self.nx - 1)
        j1 = min(j0 + 1, self.ny - 1)
        tx = min(max(fx - math.floor(fx), 0.0), 1.0)
        ty = min(max(fy - math.floor(fy), 0.0), 1.0)

        def val(i, j):
            return self.v[j * self.nx + i]

        a = val(i0, j0) * (1 - tx) + val(i1, j0) * tx
        b = val(i0, j1) * (1 - tx) + val(i1, j1) * tx
        return a * (1 - ty) + b * ty


def load_obstacles_xyz(path):
    import array
    with open(path, "rb") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise RuntimeError("PCD 头部不完整")
            text = line.decode("ascii", "replace").strip()
            if text.startswith("#") or not text:
                continue
            key, _, value = text.partition(" ")
            if key.upper() == "POINTS":
                count = int(value)
            if key.upper() == "DATA":
                break
        raw = handle.read(count * 12)
    values = array.array("f")
    values.frombytes(raw[: (len(raw) // 12) * 12])
    pts = []
    for i in range(0, len(values), 3):
        pts.append((values[i], values[i + 1], values[i + 2]))
    return pts


def min_clearance(points, x, y, z_lo, z_hi, search=1.5):
    """只统计落在**机体高度带** [z_lo, z_hi] 内的障碍点。

    固定用绝对 z 区间是错的：洞顶在机器人头顶之上时并不构成碰撞，
    按绝对高度统计会把洞顶当成"净空 0"。高度带应由地面网格推导。
    """
    best = search
    for px, py, pz in points:
        if pz < z_lo or pz > z_hi:
            continue
        d = math.hypot(px - x, py - y)
        if d < best:
            best = d
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ground-grid", required=True)
    parser.add_argument("--mode", type=int, choices=[2, 3], required=True)
    parser.add_argument("--start", type=float, nargs=2, required=True, metavar=("X", "Y"))
    parser.add_argument("--goal", type=float, nargs=2, required=True, metavar=("X", "Y"))
    parser.add_argument("--body-height", type=float, default=0.125)
    parser.add_argument("--step", type=float, default=0.5, help="采样间距（米）")
    parser.add_argument("--out", required=True)
    parser.add_argument("--obstacles", default="",
                        help="可选：障碍 PCD，用于沿线净空检查并报警")
    parser.add_argument("--radius", type=float, default=0.26)
    parser.add_argument("--z-inflation", type=float, default=0.125,
                        help="机体高度带相对轨迹 z 的上下容差（与 SCAN 的 z 膨胀一致）")
    parser.add_argument("--name", default="terrain route")
    args = parser.parse_args()

    ground = Ground(args.ground_grid)
    sx, sy = args.start
    gx, gy = args.goal
    length = math.hypot(gx - sx, gy - sy)
    n = max(2, int(math.ceil(length / args.step)))

    pts = []
    for i in range(n + 1):
        t = i / n
        x = sx + t * (gx - sx)
        y = sy + t * (gy - sy)
        gz = ground.at(x, y)
        z = gz + (args.body_height if args.mode == 2 else 0.0)
        pts.append((x, y, z, gz))

    zs = [p[2] for p in pts]
    gzs = [p[3] for p in pts]
    print("%s：%d 个采样点，路径长 %.2f m" % (args.name, len(pts), length))
    print("地面高度沿路径: %.3f -> %.3f m（变化 %.3f m）"
          % (gzs[0], gzs[-1], max(gzs) - min(gzs)))
    print("输出 z（%s）: %.3f -> %.3f m" % (
        "ground + body_height" if args.mode == 2 else "ground only，SCAN 再加 body_height",
        zs[0], zs[-1]))

    if args.obstacles:
        obs = load_obstacles_xyz(args.obstacles)
        worst = None
        for (x, y, z, _) in pts:
            # 机体高度带由**地面网格 + 机体中心高度**推导，与 SCAN 的 z 膨胀一致
            z_center = ground.at(x, y) + args.body_height
            c = min_clearance(obs, x, y, z_center - args.z_inflation,
                              z_center + args.z_inflation)
            if worst is None or c < worst[0]:
                worst = (c, x, y)
        print("沿线最小净空: %.3f m（要求 >= %.2f，按机体高度带 z=地面+%.3f±%.3f），"
              "出现在 (%.2f, %.2f)"
              % (worst[0], args.radius, args.body_height, args.z_inflation, worst[1], worst[2]))
        if worst[0] < args.radius:
            print("警告：该路线会撞进障碍，请换起点/终点或改用避障规划")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    flat = []
    for (x, y, z, _) in pts:
        flat += ["%.3f" % x, "%.3f" % y, "%.3f" % z]

    with open(args.out, "w") as handle:
        handle.write("# 由 scripts/make_terrain_route.py 从地面网格生成，请勿手改 z\n")
        handle.write("# 地面高度沿路径变化 %.3f m\n" % (max(gzs) - min(gzs)))
        if args.mode == 2:
            handle.write("/sentry_sim/scan_planner_node:\n  ros__parameters:\n"
                         "    fsm.navi_mode: 2\n")
            handle.write("    fsm.waypoints:\n      [")
        else:
            handle.write("/sentry_sim/reference_path_publisher:\n  ros__parameters:\n")
            handle.write("    frame_id: world\n    publish_delay_sec: 0.5\n    waypoints:\n      [")
        for i in range(0, len(flat), 6):
            handle.write("\n       " + ", ".join(flat[i:i + 6]) + ",")
        handle.write("\n      ]\n")
    print("已写入 %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
