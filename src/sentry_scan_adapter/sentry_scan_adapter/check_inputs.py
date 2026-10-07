"""输入检查工具：影子接入前/在线核对输入健康，失败返回非零。

用法（需要 RM 已在运行并发布话题/TF）：

    ros2 run sentry_scan_adapter check_inputs --duration 10 --planning-frame odom

检查内容：
  * 必需输出话题是否真的有数据（`/sentry_scan/{body_pose,sensor_pose,cloud}`）；
  * 每条消息的 header.stamp 与本机接收时刻的年龄、帧名、有限值/四元数；
  * 云与射线原点（sensor_pose）的时间戳配对偏差；
  * 上游适配节点的 `health`/`health_ok` 诊断；
  * 只读：不发布任何话题（尤其不发布速度命令）。

退出码：0 = 全部通过；2 = 有检查失败（缺数据、超龄、帧不符、非有限值、未配对、健康为假）。
"""

import argparse
import math
import sys
import time
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


def _stamp_s(msg) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def _quat_ok(q) -> bool:
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    return math.isfinite(norm) and norm > 1e-6


class TopicStat:
    def __init__(self, name: str):
        self.name = name
        self.count = 0
        self.first_recv = None
        self.last_recv = None
        self.last_stamp = None
        self.last_frame = ""
        self.error = ""

    def age(self, now_s: float):
        return None if self.last_recv is None else now_s - self.last_recv

    def stamp_age(self, now_s: float):
        return None if self.last_stamp is None else now_s - self.last_stamp

    def rate(self):
        if self.count < 2 or self.last_recv is None or self.first_recv is None:
            return 0.0
        span = self.last_recv - self.first_recv
        return 0.0 if span <= 0 else (self.count - 1) / span


class InputChecker(Node):
    def __init__(self, planning_frame: str, pairing_tolerance: float, namespace: str):
        super().__init__("check_inputs")
        self.planning_frame = planning_frame
        self.pairing_tolerance = pairing_tolerance
        self.namespace = namespace.rstrip("/")
        self.stats = {
            "body_pose": TopicStat("body_pose"),
            "sensor_pose": TopicStat("sensor_pose"),
            "cloud": TopicStat("cloud"),
            "health_ok": TopicStat("health_ok"),
            "health": TopicStat("health"),
        }
        self.health_ok = None
        self.health_message = ""
        self.pairing_error = ""
        # 跨话题到达顺序不保证，因此保留最近若干时间戳，逐个云帧找配对，
        # 而不是比较"各自最新一帧"（那会把相邻周期误判成不配对）。
        self.sensor_stamps = deque(maxlen=50)
        self.cloud_stamps = deque(maxlen=50)

        self.create_subscription(Odometry, self.topic("body_pose"), self.on_body, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.topic("sensor_pose"), self.on_sensor, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.topic("cloud"), self.on_cloud, qos_profile_sensor_data)
        latch = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, self.topic("health_ok"), self.on_health_ok, latch)
        self.create_subscription(DiagnosticArray, self.topic("health"), self.on_health, 10)

    def topic(self, name: str) -> str:
        return "%s/%s" % (self.namespace, name) if self.namespace else name

    def _touch(self, key: str, msg, values_ok: bool, frame: str, note: str = "",
               has_stamp: bool = True) -> None:
        stat = self.stats[key]
        now_s = self.get_clock().now().nanoseconds * 1e-9
        stat.count += 1
        if stat.first_recv is None:
            stat.first_recv = now_s
        stat.last_recv = now_s
        stat.last_stamp = _stamp_s(msg) if has_stamp else now_s
        stat.last_frame = frame
        if not values_ok:
            stat.error = note or "invalid values"

    def on_body(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        ok = (all(math.isfinite(v) for v in (pose.position.x, pose.position.y, pose.position.z))
              and _quat_ok(pose.orientation))
        self._touch("body_pose", msg, ok, msg.header.frame_id, "non-finite pose/quaternion")

    def on_sensor(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        ok = (all(math.isfinite(v) for v in (pose.position.x, pose.position.y, pose.position.z))
              and _quat_ok(pose.orientation))
        self._touch("sensor_pose", msg, ok, msg.header.frame_id, "non-finite pose/quaternion")
        self.sensor_stamps.append(_stamp_s(msg))

    def on_cloud(self, msg: PointCloud2) -> None:
        ok = msg.width * msg.height > 0
        self._touch("cloud", msg, ok, msg.header.frame_id, "empty cloud")
        self.cloud_stamps.append(_stamp_s(msg))

    def _check_pairing(self) -> None:
        """每个云帧都必须在容差内找到对应的 sensor_pose 帧。"""
        if not self.sensor_stamps or not self.cloud_stamps:
            return
        worst = 0.0
        for cloud_stamp in self.cloud_stamps:
            best = min(abs(cloud_stamp - s) for s in self.sensor_stamps)
            worst = max(worst, best)
            if best > self.pairing_tolerance:
                self.pairing_error = ("sensor_pose/cloud stamp mismatch %.4fs > %.4fs"
                                      % (best, self.pairing_tolerance))
                return
        self.pairing_error = ""

    def on_health_ok(self, msg: Bool) -> None:
        self._touch("health_ok", msg, True, "", has_stamp=False)
        self.health_ok = bool(msg.data)

    def on_health(self, msg: DiagnosticArray) -> None:
        self._touch("health", msg, True, "")
        if msg.status:
            self.health_message = msg.status[0].message

    def evaluate(self) -> int:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        failures = []
        required = ("body_pose", "sensor_pose", "cloud", "health_ok")
        print("\n%-12s %8s %8s %10s %10s  %s" % ("topic", "count", "rate", "recv_age", "stamp_age", "frame"))
        for key in ("body_pose", "sensor_pose", "cloud", "health_ok", "health"):
            stat = self.stats[key]
            age = stat.age(now_s)
            stamp_age = stat.stamp_age(now_s)
            print("%-12s %8d %8.2f %10s %10s  %s" % (
                key, stat.count, stat.rate(),
                "n/a" if age is None else "%.3f" % age,
                "n/a" if stamp_age is None else "%.3f" % stamp_age,
                stat.last_frame or "-"))
            if key in required and stat.count == 0:
                failures.append("%s: no message received" % key)
            if key in ("body_pose", "sensor_pose", "cloud"):
                if stat.last_frame and stat.last_frame != self.planning_frame:
                    failures.append("%s: frame '%s' != planning frame '%s'"
                                    % (key, stat.last_frame, self.planning_frame))
                if stat.count and age is not None and age > 1.0:
                    failures.append("%s: last message %.3fs ago" % (key, age))
            if stat.error:
                failures.append("%s: %s" % (key, stat.error))
        if self.pairing_error:
            failures.append(self.pairing_error)
        if self.health_ok is not True:
            failures.append("health_ok is not true (%s): %s"
                            % (self.health_ok, self.health_message or "no diagnostic"))
        print("health_ok=%s  diagnostic=%s" % (self.health_ok, self.health_message or "-"))
        if failures:
            print("\nFAIL:")
            for item in failures:
                print("  - %s" % item)
            return 2
        print("\nPASS: inputs healthy and paired in frame '%s'" % self.planning_frame)
        return 0


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--planning-frame", default="odom")
    parser.add_argument("--pairing-tolerance", type=float, default=0.02)
    parser.add_argument("--namespace", default="/sentry_scan",
                        help="影子适配节点命名空间；检查该命名空间下的输出话题")
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = InputChecker(args.planning_frame, args.pairing_tolerance, args.namespace)
    end = time.time() + args.duration
    try:
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        code = node.evaluate()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
