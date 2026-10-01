#!/usr/bin/env python3
"""生成尺寸明确的合成测试地图（二进制 PCD）。

用途：用可控几何验证碰撞包络逻辑本身。这些是**示例地图**，尺寸是人为设定的
测试尺寸，不是实车标定。

几何设计（关键）：每个场景都是一个**封闭房间**，房间被一道带缺口的隔墙分成两半，
起点在一侧、目标在另一侧。这样"绕过墙"不可行，机器人只能穿过缺口，
判据才有唯一解释。早期版本用两端开放的两片墙，机器人直接从墙外侧绕过去了，
测试因此失效——这是本文件改成封闭房间的原因。

约定：
- z=0 是可行驶地面参考面；机体系高度带取 [0, 0.25] m，轨迹 z=0.125。
- **合成地图不含地面点云**：SCAN 的碰撞查询把地面本身视为障碍，因此这里只隔离
  "碰撞包络"逻辑；地面处理是另一项已知缺口。
- 墙：z ∈ [0, 1.0]。矮障碍：z ∈ [0, 0.15]（低于演示地图 0.30 m 的删点阈值）。

用法: make_test_maps.py --out-dir docs/testing/maps
"""

import argparse
import os
import struct

ROOM_MIN_X, ROOM_MAX_X = -2.0, 2.0
ROOM_MIN_Y, ROOM_MAX_Y = -2.0, 3.5
WALL_Z0, WALL_Z1 = 0.0, 1.0
LOW_Z1 = 0.15
STEP = 0.03


def wall_segment(p0, p1, z0=WALL_Z0, z1=WALL_Z1, step=STEP):
    """在 p0->p1 的竖直平面上撒点（沿水平线段展开，再按高度复制）。"""
    pts = []
    length = ((p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2) ** 0.5
    n = max(1, int(length / step))
    for i in range(n + 1):
        t = i / n
        x = p0[0] + t * (p1[0] - p0[0])
        y = p0[1] + t * (p1[1] - p0[1])
        z = z0
        while z <= z1 + 1e-9:
            pts.append((round(x, 4), round(y, 4), round(z, 4)))
            z += step
    return pts


def _outer_walls():
    pts = []
    pts += wall_segment((ROOM_MIN_X, ROOM_MIN_Y), (ROOM_MAX_X, ROOM_MIN_Y))
    pts += wall_segment((ROOM_MIN_X, ROOM_MAX_Y), (ROOM_MAX_X, ROOM_MAX_Y))
    pts += wall_segment((ROOM_MIN_X, ROOM_MIN_Y), (ROOM_MIN_X, ROOM_MAX_Y))
    pts += wall_segment((ROOM_MAX_X, ROOM_MIN_Y), (ROOM_MAX_X, ROOM_MAX_Y))
    return pts


def room_with_gap(gap, div_y=0.5):
    """封闭房间 + 一道带 gap 缺口的隔墙（沿 x 方向，位于 y=div_y）。"""
    half = gap / 2.0
    pts = _outer_walls()
    pts += wall_segment((ROOM_MIN_X, div_y), (-half, div_y))
    pts += wall_segment((half, div_y), (ROOM_MAX_X, div_y))
    return pts


def room_with_low_wall(div_y=0.5, low_z1=LOW_Z1):
    """封闭房间 + 一道横贯整个房间的矮墙：不能绕行，只能被碰撞检查拦下。"""
    pts = _outer_walls()
    pts += wall_segment((ROOM_MIN_X, div_y), (ROOM_MAX_X, div_y), z0=0.0, z1=low_z1)
    return pts


START = (0.0, -1.0, 0.125)
GOAL = (0.0, 2.5)

MAPS = {
    "gap_0.80": dict(build=lambda: room_with_gap(0.80), gap=0.80, expect="pass",
                     desc="缺口 0.80 m，宽于车体 0.52 m -> 应通过"),
    "gap_0.30": dict(build=lambda: room_with_gap(0.30), gap=0.30, expect="refuse",
                     desc="缺口 0.30 m，远窄于车体 -> 应拒绝"),
    "gap_0.44": dict(build=lambda: room_with_gap(0.44), gap=0.44, expect="refuse",
                     desc="缺口 0.44 m：中心线离墙 0.22 m 不碰，车体半径 0.26 m 会碰 -> 应拒绝"),
    "low_obstacle": dict(build=lambda: room_with_low_wall(), gap=None, expect="refuse",
                         desc="横贯房间的 0.15 m 矮墙（不平移、不删点）-> 矮障碍必须仍然拦停"),
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    index = [
        "# 合成测试地图（示例尺寸，非实车标定）",
        "# 封闭房间 x[%.1f, %.1f] y[%.1f, %.1f]；墙高 %.2f m" % (
            ROOM_MIN_X, ROOM_MAX_X, ROOM_MIN_Y, ROOM_MAX_Y, WALL_Z1),
        "# 机器人半径 0.26 m -> 需要净宽 >= 0.52 m；轨迹 z=0.125，碰撞高度带 [0, 0.25] m",
        "# 起点 (%.2f, %.2f) 目标 (%.2f, %.2f)" % (START[0], START[1], GOAL[0], GOAL[1]),
        "name\tpoints\texpect\tdescription",
    ]
    for name, spec in MAPS.items():
        pts = spec["build"]()
        path = os.path.join(args.out_dir, name + ".pcd")
        with open(path, "wb") as handle:
            header = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
                      "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                      "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
                      % (len(pts), len(pts)))
            handle.write(header.encode("ascii"))
            handle.write(struct.pack("<%df" % (3 * len(pts)),
                                     *[c for p in pts for c in p]))
        print("%-14s %6d 点  %s" % (name, len(pts), spec["desc"]))
        index.append("%s\t%d\t%s\t%s" % (name, len(pts), spec["expect"], spec["desc"]))

    with open(os.path.join(args.out_dir, "INDEX.txt"), "w") as handle:
        handle.write("\n".join(index) + "\n")
    print("\n索引写入 %s/INDEX.txt" % args.out_dir)


if __name__ == "__main__":
    main()
