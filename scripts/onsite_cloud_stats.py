#!/usr/bin/env python3
"""只读采样点云特征，用于确定 min_valid_points、包络与"是否含自身"。

用法：
    python3 scripts/onsite_cloud_stats.py --topic /cloud_registered --duration 6

输出（每个话题一段）：
  * 消息数、实测频率；
  * width/height/point_step/row_step/is_dense/is_bigendian/fields（判断布局与是否需要重排）；
  * frame_id 与 header.stamp 相对本机时钟的年龄（判断时间戳尺度）；
  * 有限点的数量、x/y/z 的 min/max/均值（判断点云尺度与是否含机体自身）；
  * 距原点 0.6 m 内的点数（近似"是否把车体自己扫进来"）。
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

FLOAT32 = PointField.FLOAT32
FLOAT64 = PointField.FLOAT64


def parse_xyz(msg):
    """按消息布局解析 xyz（行填充按行首址；非小端 FLOAT32 返回 None）。"""
    if msg.is_bigendian:
        return None, "is_bigendian=true（适配器会拒绝这种布局）"
    fields = {f.name: f for f in msg.fields}
    for axis in ("x", "y", "z"):
        if axis not in fields:
            return None, "缺少字段 %s" % axis
        if fields[axis].datatype != FLOAT32:
            return None, "字段 %s 不是 FLOAT32（datatype=%d）" % (axis, fields[axis].datatype)
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.height * msg.row_step:
        return None, "row_step/data 长度与 width/height 不一致"

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    rows = np.arange(msg.height, dtype=np.int64) * msg.row_step
    cols = np.arange(msg.width, dtype=np.int64) * msg.point_step
    starts = (rows[:, None] + cols[None, :]).reshape(-1)
    out = np.empty((starts.size, 3), dtype=np.float64)
    for index, axis in enumerate(("x", "y", "z")):
        off = fields[axis].offset
        if off + 4 > msg.point_step:
            return None, "字段 %s 的 offset 超出 point_step" % axis
        # 每个点取该字段的 4 个字节（行填充已由 starts 处理）
        byte_index = starts[:, None] + np.arange(4, dtype=np.int64)[None, :] + off
        out[:, index] = raw[byte_index].reshape(-1).copy().view("<f4").astype(np.float64)
    return out, ""


class CloudStats(Node):
    def __init__(self, topic):
        super().__init__("onsite_cloud_stats")
        self.topic = topic
        self.count = 0
        self.first = None
        self.last = None
        self.frames = {}
        self.ages = []
        self.stats = []
        self.layout = None
        self.note = ""
        self.create_subscription(PointCloud2, topic, self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, msg):
        now = time.time()
        self.count += 1
        if self.first is None:
            self.first = now
        self.last = now
        self.frames[msg.header.frame_id] = self.frames.get(msg.header.frame_id, 0) + 1
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.ages.append(now - stamp)
        self.layout = (msg.width, msg.height, msg.point_step, msg.row_step,
                       msg.is_dense, msg.is_bigendian,
                       [(f.name, f.offset, f.datatype, f.count) for f in msg.fields])
        if len(self.stats) < 30:
            points, note = parse_xyz(msg)
            self.note = note
            if points is not None:
                finite = points[np.isfinite(points).all(axis=1)]
                if finite.size:
                    near = np.linalg.norm(finite, axis=1) < 0.6
                    self.stats.append((
                        int(finite.shape[0]),
                        finite[:, 0].min(), finite[:, 0].max(), finite[:, 0].mean(),
                        finite[:, 1].min(), finite[:, 1].max(), finite[:, 1].mean(),
                        finite[:, 2].min(), finite[:, 2].max(), finite[:, 2].mean(),
                        int(near.sum())))

    def report(self):
        print("话题: %s" % self.topic)
        print("  消息数=%d  频率=%.2f Hz  接收窗口=%.2fs"
              % (self.count, self.rate(), 0.0 if self.first is None else self.last - self.first))
        print("  frame_id: %s" % (", ".join("%s(%d)" % kv for kv in self.frames.items()) or "-"))
        if self.ages:
            print("  header.stamp 相对本机时钟: min=%.3fs max=%.3fs（负值=未来）"
                  % (min(self.ages), max(self.ages)))
        if self.layout:
            width, height, point_step, row_step, dense, big, fields = self.layout
            print("  布局: width=%d height=%d point_step=%d row_step=%d is_dense=%s is_bigendian=%s"
                  % (width, height, point_step, row_step, dense, big))
            print("        行填充=%s（适配器会先重排为密集布局）"
                  % ("有" if row_step > width * point_step else "无"))
            print("  fields: %s" % ", ".join("%s@%d(type=%d)" % f[:3] for f in fields))
        if self.note:
            print("  解析提示: %s" % self.note)
        if not self.stats:
            print("  （没有可统计的有限点）")
            return
        finite_n = [s[0] for s in self.stats]
        print("  有限点数: min=%d max=%d 均值=%.0f" % (min(finite_n), max(finite_n),
                                                    sum(finite_n) / len(finite_n)))
        print("  x: [%.2f, %.2f] 均值 %.2f" % (min(s[1] for s in self.stats),
                                              max(s[2] for s in self.stats),
                                              sum(s[3] for s in self.stats) / len(self.stats)))
        print("  y: [%.2f, %.2f] 均值 %.2f" % (min(s[4] for s in self.stats),
                                              max(s[5] for s in self.stats),
                                              sum(s[6] for s in self.stats) / len(self.stats)))
        print("  z: [%.2f, %.2f] 均值 %.2f" % (min(s[7] for s in self.stats),
                                              max(s[8] for s in self.stats),
                                              sum(s[9] for s in self.stats) / len(self.stats)))
        print("  距原点<0.6m 的点数: 均值 %.0f（明显>0 说明可能把车体自身扫进来了）"
              % (sum(s[10] for s in self.stats) / len(self.stats)))
        print("  建议 min_valid_points: 取有限点数最小值的 1/10 量级（当前配置默认 10）")

    def rate(self):
        if self.count < 2:
            return 0.0
        span = self.last - self.first
        return 0.0 if span <= 0 else (self.count - 1) / span


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", action="append", default=[])
    parser.add_argument("--duration", type=float, default=6.0)
    args, ros_args = parser.parse_known_args(argv)
    topics = args.topic or ["/cloud_registered", "/pointcloud"]

    rclpy.init(args=ros_args)
    nodes = [CloudStats(t) for t in topics]
    end = time.time() + args.duration
    # 多话题采样：单线程执行器轮流 spin
    executor = rclpy.executors.SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    while time.time() < end and rclpy.ok():
        executor.spin_once(timeout_sec=0.1)
    for node in nodes:
        node.report()
        print()
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
