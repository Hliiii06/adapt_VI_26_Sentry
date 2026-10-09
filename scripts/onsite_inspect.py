#!/usr/bin/env python3
"""实车在线只读采集：把"实际在跑什么"写成可回传的报告（不发布任何话题）。

用途（现场第一步，机器人静止即可跑）：

    source /opt/ros/humble/setup.bash && source <本仓库>/install/setup.bash
    python3 scripts/onsite_inspect.py --duration 20

产出 `log/onsite/<时间戳>/report.txt` 与 `report.json`，包含：

  * 运行中的节点（名字/命名空间）；
  * 关键话题的类型、发布者/订阅者、QoS（reliability/durability/depth）、实测频率；
  * 每条消息的 `header.frame_id` 与本机接收时刻的年龄（判断 frame 与时间戳尺度）；
  * `/tf`、`/tf_static` 里出现的所有 parent→child 及其最新时间戳（判断坐标系链）；
  * **控制话题审计**：`/cmd_vel`、`/cmd_vel_remap`、`/cmd_vel_nav` 的发布者名单，
    用于回答"当前底盘速度由谁发布"以及"影子是否碰了真实控制入口"。

只读实现：本脚本只创建订阅者与 TF 订阅，不创建任何发布者，不调用任何服务/action。
判据：缺少 odom/点云、或控制话题上出现影子命名空间的发布者 → 返回非零。
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)

import tf2_msgs.msg
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

# 关键输入（RM main 7dfe71a 的候选来源；现场以实测为准）
KEY_TOPICS = [
    "/Odometry_transformed",
    "/Odometry",
    "/LIVO2/imu_propagate",
    "/cloud_registered",
    "/pointcloud",
    "/segmentation/obstacle",
    "/map",
]
CONTROL_TOPICS = ["/cmd_vel", "/cmd_vel_remap", "/cmd_vel_nav", "/cmd_vel_remap_unused"]
SHADOW_PREFIX = "/sentry_scan"
# 这些话题的发布者常带 transient_local（map_server 的 /map、静态外参 /tf_static）。
# 采集器通常**后**加入，用 volatile 订阅会收不到已经发过的内容，因此显式请求 TL。
TRANSIENT_LOCAL_TOPICS = ("/map", "/tf_static")

TYPE_REGISTRY = {
    "nav_msgs/msg/Odometry": Odometry,
    "nav_msgs/msg/Path": Path,
    "sensor_msgs/msg/PointCloud2": PointCloud2,
    "sensor_msgs/msg/LaserScan": LaserScan,
    "sensor_msgs/msg/Imu": Imu,
    "sensor_msgs/msg/Image": Image,
    "geometry_msgs/msg/Twist": Twist,
    "geometry_msgs/msg/TwistStamped": TwistStamped,
    "geometry_msgs/msg/PoseStamped": PoseStamped,
    "geometry_msgs/msg/PoseWithCovarianceStamped": PoseWithCovarianceStamped,
    "std_msgs/msg/Bool": Bool,
    "std_msgs/msg/String": String,
    "visualization_msgs/msg/Marker": Marker,
    "visualization_msgs/msg/MarkerArray": MarkerArray,
    "tf2_msgs/msg/TFMessage": tf2_msgs.msg.TFMessage,
}


def _stamp_s(msg):
    header = getattr(msg, "header", None)
    if header is None:
        return None
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


def _qos_for(topic):
    if topic in TRANSIENT_LOCAL_TOPICS:
        return QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # SensorDataQoS（best effort）与 reliable 发布者兼容；对 volatile 的流式数据足够。
    return qos_profile_sensor_data


class Channel:
    def __init__(self, topic, type_name):
        self.topic = topic
        self.type_name = type_name
        self.subscribed = False
        self.count = 0
        self.first = None
        self.last = None
        self.last_stamp = None
        self.frames = {}
        self.notes = []

    def note(self, msg):
        now = time.time()
        self.count += 1
        if self.first is None:
            self.first = now
        self.last = now
        stamp = _stamp_s(msg)
        if stamp is not None:
            self.last_stamp = stamp
        header = getattr(msg, "header", None)
        frame = getattr(header, "frame_id", "") if header is not None else ""
        if frame:
            self.frames[frame] = self.frames.get(frame, 0) + 1

    def rate(self):
        if self.count < 2 or self.first is None or self.last is None:
            return 0.0
        span = self.last - self.first
        return 0.0 if span <= 0 else (self.count - 1) / span

    def receive_age(self, now):
        return None if self.last is None else now - self.last

    def stamp_age(self, now):
        return None if self.last_stamp is None else now - self.last_stamp


class OnsiteInspector(Node):
    def __init__(self, topics):
        super().__init__("onsite_inspect")
        self.topics = list(topics)
        self.channels = {topic: Channel(topic, "") for topic in self.topics}
        self.tf_pairs = {}
        self.tf_static_pairs = {}
        self._tf_subscribed = False
        self._tf_static_subscribed = False
        # 首轮 + 观察期内持续发现：实车话题可能在采集器之后才出现（DDS 发现/启动顺序）。
        self.refresh_subscriptions()

    def refresh_subscriptions(self):
        """把"现在才出现"的话题补上订阅。已订阅的不重复创建。"""
        available = dict(self.get_topic_names_and_types())
        for topic in self.topics:
            channel = self.channels[topic]
            if channel.subscribed:
                continue
            types = available.get(topic)
            if not types:
                if "topic not present" not in channel.notes:
                    channel.notes.append("topic not present")
                continue
            type_name = types[0]
            channel.type_name = type_name
            msg_class = TYPE_REGISTRY.get(type_name)
            if msg_class is None:
                if "type not auto-subscribed (not in registry)" not in channel.notes:
                    channel.notes.append("type not auto-subscribed (not in registry)")
                continue
            self.create_subscription(msg_class, topic, channel.note, _qos_for(topic))
            channel.subscribed = True
            channel.notes = [n for n in channel.notes if n != "topic not present"]
        if not self._tf_subscribed and "/tf" in available:
            self.create_subscription(tf2_msgs.msg.TFMessage, "/tf", self._on_tf, _qos_for("/tf"))
            self._tf_subscribed = True
        if not self._tf_static_subscribed and "/tf_static" in available:
            # 一次性 latch 的话题：必须 transient_local，否则先启动实车再采集会收不到
            self.create_subscription(tf2_msgs.msg.TFMessage, "/tf_static", self._on_tf_static,
                                     _qos_for("/tf_static"))
            self._tf_static_subscribed = True

    @staticmethod
    def _merge(pairs, msg):
        for transform in msg.transforms:
            key = "%s -> %s" % (transform.header.frame_id, transform.child_frame_id)
            stamp = float(transform.header.stamp.sec) + float(transform.header.stamp.nanosec) * 1e-9
            pairs[key] = stamp

    def _on_tf(self, msg):
        self._merge(self.tf_pairs, msg)

    def _on_tf_static(self, msg):
        self._merge(self.tf_static_pairs, msg)

    # ------------------------------------------------------------------ 报告
    def endpoint_lines(self, topic):
        lines = []
        for kind, infos in (("pub", self.get_publishers_info_by_topic(topic)),
                            ("sub", self.get_subscriptions_info_by_topic(topic))):
            for info in infos:
                qos = info.qos_profile
                namespace = (info.node_namespace or "/").rstrip("/")
                lines.append("      %s: %s/%s  [%s/%s/%s]" % (
                    kind, namespace or "", info.node_name,
                    qos.reliability.name, qos.durability.name, qos.depth))
        return lines

    def collect(self):
        now = time.time()
        report = {
            "nodes": sorted(self.get_node_names_and_namespaces()),
            "channels": [],
            "tf": self.tf_pairs,
            "tf_static": self.tf_static_pairs,
            "control_topics": {},
        }
        for topic, channel in self.channels.items():
            report["channels"].append({
                "topic": topic,
                "type": channel.type_name,
                "count": channel.count,
                "rate_hz": round(channel.rate(), 2),
                "receive_age_s": None if channel.receive_age(now) is None
                else round(channel.receive_age(now), 3),
                "stamp_age_s": None if channel.stamp_age(now) is None
                else round(channel.stamp_age(now), 3),
                "frames": channel.frames,
                "notes": channel.notes,
                "endpoints": self.endpoint_lines(topic),
            })
        for topic in CONTROL_TOPICS:
            infos = self.get_publishers_info_by_topic(topic)
            report["control_topics"][topic] = [
                {"node": info.node_name, "namespace": info.node_namespace,
                 "type": ",".join(info.topic_type.split("/")[1:]) if info.topic_type else ""}
                for info in infos]
        return report

    def verdicts(self, report):
        failures, warnings = [], []
        for channel in report["channels"]:
            if channel["topic"] in ("/Odometry_transformed", "/LIVO2/imu_propagate",
                                    "/cloud_registered") and channel["count"] == 0:
                failures.append("%s: 没有收到消息（现场核对话题名/命名空间是否与文档一致）"
                                % channel["topic"])
            if channel["stamp_age_s"] is not None and channel["stamp_age_s"] > 0.5:
                warnings.append("%s: header.stamp 比本机时钟旧 %.3fs（时间戳尺度可能不同）"
                                % (channel["topic"], channel["stamp_age_s"]))
            if channel["count"] and not channel["frames"]:
                warnings.append("%s: 消息没有 header.frame_id" % channel["topic"])
        control_publishers = []
        for topic, entries in report["control_topics"].items():
            for entry in entries:
                control_publishers.append((topic, entry["namespace"], entry["node"]))
                if (entry["namespace"] or "").startswith(SHADOW_PREFIX):
                    failures.append("%s 被影子命名空间节点 %s%s 发布（绝不允许）"
                                    % (topic, entry["namespace"], entry["node"]))
        if not control_publishers:
            warnings.append("没有任何 /cmd_vel* 发布者：确认底盘是否由其它话题驱动")
        return failures, warnings, control_publishers


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--out-dir", default="log/onsite")
    parser.add_argument("--topics", default="",
                        help="逗号分隔，覆盖默认关键话题列表")
    args, ros_args = parser.parse_known_args(argv)

    topics = [t.strip() for t in args.topics.split(",") if t.strip()] or KEY_TOPICS
    rclpy.init(args=ros_args)
    node = OnsiteInspector(topics)
    end = time.time() + args.duration
    next_refresh = 0.0
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() >= next_refresh:
            node.refresh_subscriptions()
            next_refresh = time.time() + 0.5
    report = node.collect()
    failures, warnings, control_publishers = node.verdicts(report)
    node.destroy_node()
    rclpy.shutdown()

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(args.out_dir, stamp)
    os.makedirs(out_dir, exist_ok=True)
    report_path = os.path.join(out_dir, "report.json")
    with open(report_path, "w") as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, default=str)

    lines = []
    lines.append("实车在线只读采集报告  %s" % stamp)
    lines.append("观测窗口: %.1fs" % args.duration)
    lines.append("")
    lines.append("== 节点 ==")
    # rclpy 的 get_node_names_and_namespaces() 返回 (name, namespace)
    for name, namespace in report["nodes"]:
        prefix = (namespace or "/").rstrip("/")
        lines.append("  %s/%s" % (prefix, name))
    lines.append("")
    lines.append("== 关键话题 ==")
    lines.append("%-28s %-34s %8s %7s %9s %9s  %s"
                 % ("topic", "type", "count", "rate", "recv_age", "stamp_age", "frames"))
    for channel in report["channels"]:
        lines.append("%-28s %-34s %8d %7.2f %9s %9s  %s" % (
            channel["topic"], channel["type"] or "-", channel["count"], channel["rate_hz"],
            "-" if channel["receive_age_s"] is None else "%.3f" % channel["receive_age_s"],
            "-" if channel["stamp_age_s"] is None else "%.3f" % channel["stamp_age_s"],
            ",".join("%s(%d)" % (f, n) for f, n in channel["frames"].items()) or "-"))
        for note in channel["notes"]:
            lines.append("      note: %s" % note)
        for endpoint in channel["endpoints"]:
            lines.append(endpoint)
    lines.append("")
    lines.append("== TF 动态 (/tf) ==")
    for key, stamp_value in sorted(report["tf"].items()):
        lines.append("  %-45s %.3f" % (key, stamp_value))
    lines.append("== TF 静态 (/tf_static) ==")
    for key, stamp_value in sorted(report["tf_static"].items()):
        lines.append("  %-45s %.3f" % (key, stamp_value))
    lines.append("")
    lines.append("== 控制话题发布者（谁在控制底盘）==")
    for topic, entries in report["control_topics"].items():
        if not entries:
            lines.append("  %-20s 无发布者" % topic)
        for entry in entries:
            lines.append("  %-20s %s%s" % (topic, entry["namespace"], entry["node"]))
    lines.append("")
    lines.append("== 人工复核命令（可直接复制，用于交叉验证）==")
    lines.append("  ros2 node list")
    lines.append("  ros2 topic list -t")
    for topic in topics + CONTROL_TOPICS:
        lines.append("  ros2 topic info -v %s" % topic)
    lines.append("  timeout 5 ros2 topic hz /cloud_registered")
    lines.append("  timeout 5 ros2 topic hz /Odometry_transformed")
    lines.append("  timeout 3 ros2 topic echo --once /cloud_registered --field header")
    lines.append("  timeout 3 ros2 topic echo --once /Odometry_transformed --field header")
    lines.append("  timeout 3 ros2 topic echo --once /LIVO2/imu_propagate --field twist")
    lines.append("  timeout 5 ros2 run tf2_tools view_frames  # 生成 frames.pdf")
    lines.append("  ros2 run tf2_ros tf2_echo odom base_link")
    lines.append("  ros2 run tf2_ros tf2_echo odom lidar_link")
    if failures:
        lines.append("")
        lines.append("== FAIL ==")
        lines.extend("  - %s" % item for item in failures)
    if warnings:
        lines.append("")
        lines.append("== WARN（需要现场确认）==")
        lines.extend("  - %s" % item for item in warnings)
    if not failures:
        lines.append("")
        lines.append("== PASS：关键输入存在，且控制话题上没有影子命名空间的发布者 ==")

    text = "\n".join(lines) + "\n"
    with open(os.path.join(out_dir, "report.txt"), "w") as handle:
        handle.write(text)
    print(text)
    print("报告已写入: %s" % out_dir)
    sys.exit(2 if failures else 0)


if __name__ == "__main__":
    main()
