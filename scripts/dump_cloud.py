#!/usr/bin/env python3
"""抓取一个 PointCloud2 话题的首帧，写出 x y z 文本（用于离线分析 SCAN 看到的地图）。

用法: dump_cloud.py --topic /sentry_sim/grid_map/occupancy_inflate --out /tmp/a.txt [--timeout 30]
"""

import argparse
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class Dumper(Node):
    def __init__(self, topic, out, timeout):
        super().__init__("cloud_dumper")
        self.out = out
        self.done = False
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, topic, self.cb, qos)
        self.create_subscription(PointCloud2, topic, self.cb, 5)
        self.deadline = time.time() + timeout

    def cb(self, msg: PointCloud2):
        if self.done or msg.width == 0:
            return
        offsets = {f.name: f.offset for f in msg.fields}
        step = msg.point_step
        rows = []
        for i in range(msg.width):
            base = i * step
            x = struct.unpack_from("<f", msg.data, base + offsets["x"])[0]
            y = struct.unpack_from("<f", msg.data, base + offsets["y"])[0]
            z = struct.unpack_from("<f", msg.data, base + offsets["z"])[0]
            rows.append((x, y, z))
        with open(self.out, "w") as handle:
            handle.write("# topic=%s frame=%s stamp=%d.%09d points=%d\n"
                         % (msg.header.frame_id, msg.header.frame_id,
                            msg.header.stamp.sec, msg.header.stamp.nanosec, len(rows)))
            for x, y, z in rows:
                handle.write("%.4f %.4f %.4f\n" % (x, y, z))
        self.done = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = Dumper(args.topic, args.out, args.timeout)
    while not node.done and time.time() < node.deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not node.done:
        print("未在 %.0fs 内收到 %s" % (args.timeout, args.topic))
        return 1
    print("已写出 %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
