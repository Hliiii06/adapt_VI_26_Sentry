"""影子保护层：门控候选速度并记录，绝不下发底盘命令。

数据流：

    closed_loop_controller ──/sentry_scan/cmd_vel_candidate──> 本节点
    rm_input_adapter ──/sentry_scan/health_ok───────────────> 本节点
    rm_input_adapter ──/sentry_scan/body_pose───────────────> 本节点
                                                             │
                                                             └──> /sentry_scan/cmd_vel_shadow（仅记录/显示）

安全边界（代码层强制）：
  * 本节点只创建 `cmd_vel_shadow` 一个速度发布者；不存在 `/cmd_vel`、`/cmd_vel_remap` 发布者。
  * 启动后周期检查 ROS 图中 `/cmd_vel`、`/cmd_vel_remap` 是否出现发布者；一旦出现，
    影子输出立即归零并在 CSV 中标记 `graph_violation`（失效关闭，而不是继续跟随）。
  * 健康状态不新鲜、候选值非有限、超过限幅时输出零。
"""

import csv
import json
import math
import os
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from std_msgs.msg import Bool

FORBIDDEN_TOPICS = ("/cmd_vel", "/cmd_vel_remap")


class ShadowGuard(Node):
    def __init__(self) -> None:
        super().__init__("shadow_guard")

        self.health_topic = self.declare_parameter("health_topic", "health_ok").value
        self.candidate_topic = self.declare_parameter("candidate_topic", "cmd_vel_candidate").value
        self.shadow_topic = self.declare_parameter("shadow_topic", "cmd_vel_shadow").value
        self.body_pose_topic = self.declare_parameter("body_pose_topic", "body_pose").value
        self.max_health_age = float(self.declare_parameter("max_health_age", 0.5).value)
        self.max_body_age = float(self.declare_parameter("max_body_age", 0.5).value)
        self.max_vx = float(self.declare_parameter("max_vx", 1.0).value)
        self.max_vy = float(self.declare_parameter("max_vy", 1.0).value)
        self.max_wz = float(self.declare_parameter("max_wz", 1.5).value)
        self.zero_yaw_candidate = bool(self.declare_parameter("zero_yaw_candidate", True).value)
        self.output_rate = float(self.declare_parameter("output_rate", 20.0).value)
        self.record = bool(self.declare_parameter("record", True).value)
        self.log_dir = os.path.expanduser(self.declare_parameter("log_dir", "log/shadow").value)
        self.graph_check_period = float(self.declare_parameter("graph_check_period", 1.0).value)

        self.health = False
        self.health_recv: Optional[float] = None
        self.body_recv: Optional[float] = None
        self.candidate = Twist()
        self.candidate_recv: Optional[float] = None
        self.graph_violation = ""
        self.gate_reason = "startup"

        latch_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, self.health_topic, self.on_health, latch_qos)
        self.create_subscription(Twist, self.candidate_topic, self.on_candidate,
                                 QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Odometry, self.body_pose_topic, self.on_body, qos_profile_sensor_data)
        self.shadow_pub = self.create_publisher(
            Twist, self.shadow_topic,
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE))

        self.create_timer(1.0 / max(1.0, self.output_rate), self.on_timer)
        self.create_timer(self.graph_check_period, self.check_forbidden_publishers)

        self._writer = None
        self._csv = None
        self._start_wall = time.time()
        self._samples = 0
        # 记录参数快照，便于证据审查（含"影子层没有底盘输出"这一点）。
        if self.record:
            os.makedirs(self.log_dir, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self._path = os.path.join(self.log_dir, "shadow_%s.csv" % stamp)
            self._csv = open(self._path, "w", newline="")
            self._writer = csv.writer(self._csv)
            self._writer.writerow(["wall_time", "ros_time", "health", "gate", "cand_vx", "cand_vy",
                                   "cand_wz", "out_vx", "out_vy", "out_wz", "graph_violation"])
            with open(os.path.join(self.log_dir, "shadow_%s_meta.json" % stamp), "w") as handle:
                json.dump({
                    "note": "shadow only: this node publishes cmd_vel_shadow and never /cmd_vel",
                    "candidate_topic": self.candidate_topic,
                    "shadow_topic": self.shadow_topic,
                    "health_topic": self.health_topic,
                    "max_health_age": self.max_health_age,
                    "zero_yaw_candidate": self.zero_yaw_candidate,
                    "limits": {"vx": self.max_vx, "vy": self.max_vy, "wz": self.max_wz},
                }, handle, ensure_ascii=False, indent=2)

        self.get_logger().warn(
            "Shadow guard ready: candidate='%s' -> shadow='%s'. This node creates NO publisher on "
            "%s; it only records/tees the gated candidate."
            % (self.candidate_topic, self.shadow_topic, " or ".join(FORBIDDEN_TOPICS)))
        self.check_forbidden_publishers()

    # ------------------------------------------------------------------ 回调
    def on_health(self, msg: Bool) -> None:
        self.health = bool(msg.data)
        self.health_recv = self._now_s()

    def on_body(self, msg: Odometry) -> None:
        self.body_recv = self._now_s()

    def on_candidate(self, msg: Twist) -> None:
        self.candidate = msg
        self.candidate_recv = self._now_s()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def check_forbidden_publishers(self) -> None:
        violation = ""
        for topic in FORBIDDEN_TOPICS:
            try:
                infos = self.get_publishers_info_by_topic(topic)
            except Exception as exc:  # graph API failures must not silently pass
                violation = "graph_query_failed(%s:%s)" % (topic, exc)
                break
            if infos:
                violation = "%s has %d publisher(s)" % (topic, len(infos))
                break
        if violation and violation != self.graph_violation:
            self.get_logger().error(
                "GRAPH VIOLATION: %s. Shadow output is forced to zero; this build must never drive "
                "the chassis." % violation)
        self.graph_violation = violation

    # ------------------------------------------------------------------ 门控
    def decide(self) -> tuple[Twist, str]:
        now_s = self._now_s()
        if self.graph_violation:
            return Twist(), "graph_violation"
        if not self.health:
            return Twist(), "inputs_unhealthy"
        if self.health_recv is None or (now_s - self.health_recv) > self.max_health_age:
            return Twist(), "health_stale"
        if self.body_recv is None or (now_s - self.body_recv) > self.max_body_age:
            return Twist(), "body_pose_stale"
        if self.candidate_recv is None or (now_s - self.candidate_recv) > self.max_health_age:
            return Twist(), "candidate_stale"

        values = (self.candidate.linear.x, self.candidate.linear.y, self.candidate.linear.z,
                  self.candidate.angular.x, self.candidate.angular.y, self.candidate.angular.z)
        if not all(math.isfinite(float(v)) for v in values):
            return Twist(), "candidate_non_finite"
        if abs(self.candidate.linear.z) > 1e-9 or abs(self.candidate.angular.x) > 1e-9 \
                or abs(self.candidate.angular.y) > 1e-9:
            # linear.z / angular.x,y 属于 chassis 协议的自定义语义或未定义分量，影子层不透传。
            return Twist(), "candidate_unsupported_axes"

        out = Twist()
        out.linear.x = max(-self.max_vx, min(self.max_vx, self.candidate.linear.x))
        out.linear.y = max(-self.max_vy, min(self.max_vy, self.candidate.linear.y))
        out.angular.z = 0.0 if self.zero_yaw_candidate else max(
            -self.max_wz, min(self.max_wz, self.candidate.angular.z))
        reason = "pass" + ("_yaw_zeroed" if self.zero_yaw_candidate and abs(self.candidate.angular.z) > 1e-9 else "")
        return out, reason

    def on_timer(self) -> None:
        out, reason = self.decide()
        self.gate_reason = reason
        self.shadow_pub.publish(out)
        self._samples += 1
        if self._writer is not None and (self._samples % 5 == 0 or reason != "pass"):
            self._writer.writerow([
                "%.3f" % time.time(), "%.6f" % self._now_s(), int(self.health), reason,
                "%.4f" % self.candidate.linear.x, "%.4f" % self.candidate.linear.y,
                "%.4f" % self.candidate.angular.z,
                "%.4f" % out.linear.x, "%.4f" % out.linear.y, "%.4f" % out.angular.z,
                self.graph_violation])

    def destroy_node(self):
        if self._csv is not None:
            self._csv.flush()
            self._csv.close()
        return super().destroy_node()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = ShadowGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
