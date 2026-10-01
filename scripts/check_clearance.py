#!/usr/bin/env python3
"""独立的几何净空检查（不依赖 SCAN 自身的碰撞判断）。

对场景 CSV 中的每个位姿，在原始 PCD 中查找机体高度带内的最近点，
报告整段轨迹的最小水平净空。用于验证"规划器认为安全"与
"机身包络确实没有贴到障碍"一致。

高度带用原始 PCD 坐标给出（默认 [0.30, 0.55] m），因此不依赖
map_pub 的 map_offset_z 归一化，是独立检查。

用法:
    check_clearance.py --csv log/scenarios/mode1_lateral.csv \
        --pcd ~/pcd_map/rmuc2026_field.pcd --radius 0.26
"""

import argparse
import array
import csv
import math
import os
import sys


def load_pcd_xy(path, z_min, z_max):
    with open(path, "rb") as handle:
        header = {}
        while True:
            line = handle.readline()
            if not line:
                raise RuntimeError("PCD 头部不完整")
            text = line.decode("ascii", "replace").strip()
            if text.startswith("#") or not text:
                continue
            key, _, value = text.partition(" ")
            header[key.upper()] = value
            if key.upper() == "DATA":
                break
        if header.get("DATA", "").strip() != "binary":
            raise RuntimeError("仅支持 binary PCD")
        fields = header["FIELDS"].split()
        if fields[:3] != ["x", "y", "z"]:
            raise RuntimeError("PCD 前三个字段必须是 x y z")
        count = int(header["POINTS"])
        raw = handle.read(count * 12)
    values = array.array("f")
    values.frombytes(raw[: (len(raw) // 12) * 12])
    points = []
    for i in range(0, len(values), 3):
        z = values[i + 2]
        if z_min <= z <= z_max:
            points.append((values[i], values[i + 1]))
    return points


class GridHash:
    """均匀网格哈希，用于最近邻查询（避免引入 scipy 依赖）。"""

    def __init__(self, points, cell=0.5):
        self.cell = cell
        self.buckets = {}
        for x, y in points:
            key = (int(math.floor(x / cell)), int(math.floor(y / cell)))
            self.buckets.setdefault(key, []).append((x, y))

    def min_distance(self, x, y, search_radius):
        cell = self.cell
        cx = int(math.floor(x / cell))
        cy = int(math.floor(y / cell))
        span = int(math.ceil(search_radius / cell))
        best = search_radius
        for i in range(cx - span, cx + span + 1):
            for j in range(cy - span, cy + span + 1):
                bucket = self.buckets.get((i, j))
                if not bucket:
                    continue
                for px, py in bucket:
                    d = math.hypot(px - x, py - y)
                    if d < best:
                        best = d
        return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--pcd", default=os.path.expanduser("~/pcd_map/rmuc2026_field.pcd"))
    parser.add_argument("--radius", type=float, default=0.26,
                        help="机体包络半径（不含安全余量）")
    parser.add_argument("--margin", type=float, default=0.0, help="安全余量")
    parser.add_argument("--z-min", type=float, default=0.30,
                        help="机体高度带下沿（原始 PCD 坐标）")
    parser.add_argument("--z-max", type=float, default=0.55,
                        help="机体高度带上沿（原始 PCD 坐标）")
    parser.add_argument("--search", type=float, default=1.5)
    args = parser.parse_args()

    points = load_pcd_xy(args.pcd, args.z_min, args.z_max)
    print("高度带 [%.2f, %.2f] 内的地图点数: %d" % (args.z_min, args.z_max, len(points)))
    index = GridHash(points)

    required = args.radius + args.margin
    worst = None
    n = 0
    with open(args.csv) as handle:
        for row in csv.DictReader(handle):
            x = float(row["x"])
            y = float(row["y"])
            n += 1
            d = index.min_distance(x, y, args.search)
            if worst is None or d < worst[0]:
                worst = (d, x, y, float(row["t"]))

    if worst is None:
        print("CSV 没有样本")
        return 1

    d, x, y, t = worst
    print("轨迹样本数: %d" % n)
    print("最小水平净空: %.4f m，出现在 t=%.2fs 位置 (%.3f, %.3f)" % (d, t, x, y))
    print("机体包络要求: %.4f m (半径 %.3f + 余量 %.3f)" % (required, args.radius, args.margin))
    ok = d >= required - 1e-3
    print("结论: %s" % ("通过 —— 包络全程未侵入障碍" if ok else
                        "失败 —— 包络侵入障碍 %.4f m" % (required - d)))
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
