"""影子保护层：门控候选速度并记录，绝不下发底盘命令。

数据流：

    closed_loop_controller ──/sentry_scan/cmd_vel_candidate──> 本节点
    rm_input_adapter ──/sentry_scan/health_ok───────────────> 本节点
    rm_input_adapter ──/sentry_scan/body_pose───────────────> 本节点
    scan_planner_node ──/sentry_scan/grid_map/cloud_update──> 本节点（地图真的更新了吗）
                                                             │
                                                             └──> /sentry_scan/cmd_vel_shadow（仅记录/显示）

安全边界（代码层强制）：
  * 本节点只创建 `cmd_vel_shadow` 一个速度发布者；不存在 `/cmd_vel`、`/cmd_vel_remap` 发布者。
  * 周期检查 `/cmd_vel`、`/cmd_vel_remap` 是否被**影子命名空间内**的节点发布；若是则影子输出
    立即归零并标记 `graph_violation`。**外部**发布者（例如与影子并存的 Nav2 controller_server）
    是允许的：在线影子观察本来就要与旧导航共存，只记录数量，不阻断影子。
  * 健康状态不新鲜、候选值非有限、超过限幅、或**规划地图长时间没有接受新点云**时输出零。
  * **地图停更等于一次任务失效**：不止临时归零，还会发布 `planning/reset` 撤销任务并**锁止**，
    只有出现编号更大的**新任务**授权（且地图已恢复）才解除；否则心跳恢复后旧速度会被重新放行。
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
from scan_planner_msgs.msg import TaskAuthorization
from std_msgs.msg import Bool, Header

FORBIDDEN_TOPICS = ("/cmd_vel", "/cmd_vel_remap")


class ShadowGuard(Node):
    def __init__(self) -> None:
        super().__init__("shadow_guard")

        self.health_topic = self.declare_parameter("health_topic", "health_ok").value
        self.candidate_topic = self.declare_parameter("candidate_topic", "cmd_vel_candidate").value
        self.shadow_topic = self.declare_parameter("shadow_topic", "cmd_vel_shadow").value
        self.body_pose_topic = self.declare_parameter("body_pose_topic", "body_pose").value
        # 地图心跳：grid_map 只在"配对通过 + 非空 + 有有效点"时发布，
        # 因此该话题静默 == 规划地图没有新数据（例如云全是 NaN 或配对一直失败）。
        self.cloud_update_topic = self.declare_parameter("cloud_update_topic",
                                                         "grid_map/cloud_update").value
        self.max_map_age = float(self.declare_parameter("max_map_age", 0.5).value)
        # 地图停更时的任务语义：撤销 + 锁止，直到新任务（task_id 更大）到来。
        self.task_active_topic = self.declare_parameter("task_active_topic",
                                                        "planning/task_active").value
        # 地图过期**始终**输出零；撤销+锁止也始终执行（不提供关闭开关：
        # 曾经用同一个开关控制两件事，关掉撤销会连"过期地图仍可输出"一起放行）。
        self.revoke_repeat_period = float(self.declare_parameter("revoke_repeat_period", 1.0).value)
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
        self.map_recv: Optional[float] = None
        self.map_stamp: Optional[float] = None
        self.map_ever_fresh = False
        self.map_latched = False
        self.latched_task_id = 0
        self.last_task_id = 0
        # 当前是否已授权执行。锁止的前提是"有任务可撤销"：启动阶段（还没任务）地图过期
        # 只需输出零，不必锁止——否则会在第一个任务到来时被误当成"解锁"，造成语义混乱。
        self.task_active = False
        self._last_revoke = -1.0
        self.external_publishers = 0
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
        self.create_subscription(Header, self.cloud_update_topic, self.on_map_update,
                                 QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(TaskAuthorization, self.task_active_topic, self.on_task_active,
                                 latch_qos)
        self.reset_pub = self.create_publisher(Bool, "planning/reset", 10)
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
                                   "cand_wz", "out_vx", "out_vy", "out_wz", "map_age", "map_latched",
                                   "external_cmdvel_pubs", "graph_violation"])
            with open(os.path.join(self.log_dir, "shadow_%s_meta.json" % stamp), "w") as handle:
                json.dump({
                    "note": "shadow only: this node publishes cmd_vel_shadow and never /cmd_vel",
                    "candidate_topic": self.candidate_topic,
                    "shadow_topic": self.shadow_topic,
                    "health_topic": self.health_topic,
                    "max_health_age": self.max_health_age,
                    "max_map_age": self.max_map_age,
                    "cloud_update_topic": self.cloud_update_topic,
                    "zero_yaw_candidate": self.zero_yaw_candidate,
                    "limits": {"vx": self.max_vx, "vy": self.max_vy, "wz": self.max_wz},
                }, handle, ensure_ascii=False, indent=2)

        self.get_logger().warn(
            "Shadow guard ready: candidate='%s' -> shadow='%s'. This node creates NO publisher on "
            "%s; it only records/tees the gated candidate. Map heartbeat='%s' (max age %.2fs; "
            "stale map always stops and is latched until a new task); external publishers on those "
            "topics are tolerated."
            % (self.candidate_topic, self.shadow_topic, " or ".join(FORBIDDEN_TOPICS),
               self.cloud_update_topic, self.max_map_age))
        self.check_forbidden_publishers()

    # ------------------------------------------------------------------ 回调
    def on_health(self, msg: Bool) -> None:
        self.health = bool(msg.data)
        self.health_recv = self._now_s()

    def on_body(self, msg: Odometry) -> None:
        self.body_recv = self._now_s()

    def on_map_update(self, msg: Header) -> None:
        self.map_recv = self._now_s()
        self.map_stamp = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
        self.map_ever_fresh = True

    def on_task_active(self, msg: TaskAuthorization) -> None:
        if msg.task_id > self.last_task_id:
            self.last_task_id = msg.task_id
        if not msg.active:
            self.task_active = False
            return
        self.task_active = True
        if not self.map_latched:
            return
        if msg.task_id <= self.latched_task_id:
            self.get_logger().warn(
                "Authorization task_id=%u is not newer than the latched id=%u; map latch kept"
                % (msg.task_id, self.latched_task_id))
            return
        reason = self._map_stale_reason(self._now_s())
        if reason:
            self.get_logger().warn(
                "New task %u authorized but the map is still stale (%s); latch kept"
                % (msg.task_id, reason))
            return
        self.map_latched = False
        self.get_logger().warn(
            "New task %u authorized after the map recovered; map latch cleared (old task was revoked)"
            % msg.task_id)

    def _map_stale_reason(self, now_s: float) -> str:
        if self.map_recv is None or (now_s - self.map_recv) > self.max_map_age:
            return "map_update_stale"
        if self.map_stamp is None or (now_s - self.map_stamp) > self.max_map_age:
            return "map_update_stamp_stale"
        return ""

    def _publish_reset(self, reason: str, now_s: float) -> None:
        msg = Bool()
        msg.data = True
        self.reset_pub.publish(msg)
        self._last_revoke = now_s
        self.get_logger().error(
            "Map update NOT healthy (%s): publishing planning/reset and LATCHING the shadow output; "
            "only a new task (task_id > %u) with a recovered map will resume" % (reason, self.latched_task_id))

    def on_candidate(self, msg: Twist) -> None:
        self.candidate = msg
        self.candidate_recv = self._now_s()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def check_forbidden_publishers(self) -> None:
        """只禁止**影子命名空间内**的节点发布真实控制话题。

        与 Nav2 并存是在线影子观察的前提：`/cmd_vel` 上出现 `controller_server` 等外部发布者
        属于正常情况，不阻断影子（只记录数量）。若影子自己的节点（同命名空间）发布了
        `/cmd_vel`/`/cmd_vel_remap`，那才是配置错误，必须失效关闭。
        """
        own_ns = self.get_namespace().rstrip("/") or "/"
        violation = ""
        external = 0
        for topic in FORBIDDEN_TOPICS:
            try:
                infos = self.get_publishers_info_by_topic(topic)
            except Exception as exc:  # graph API failures must not silently pass
                violation = "graph_query_failed(%s:%s)" % (topic, exc)
                break
            for info in infos:
                namespace = (info.node_namespace or "/").rstrip("/") or "/"
                if namespace == own_ns:
                    violation = "%s is published by shadow node %s%s" % (
                        topic, namespace, info.node_name)
                    break
                external += 1
            if violation:
                break
        if external != self.external_publishers:
            self.get_logger().warn(
                "External publisher(s) on %s observed: %d (coexistence with the existing stack; "
                "shadow output is NOT blocked by them)"
                % ("/".join(FORBIDDEN_TOPICS), external))
        self.external_publishers = external
        if violation and violation != self.graph_violation:
            self.get_logger().error(
                "GRAPH VIOLATION: %s. Shadow output is forced to zero; shadow nodes must never "
                "publish real control topics." % violation)
        self.graph_violation = violation

    # ------------------------------------------------------------------ 门控
    def decide(self) -> tuple[Twist, str]:
        now_s = self._now_s()
        if self.graph_violation:
            return Twist(), "graph_violation"
        # 地图停更 = 任务失效：先锁止并撤销，而不是"临时输出零、恢复后继续放行"。
        stale_reason = self._map_stale_reason(now_s)
        if stale_reason and self.map_ever_fresh and self.task_active and not self.map_latched:
            self.map_latched = True
            self.latched_task_id = self.last_task_id
            self._publish_reset(stale_reason, now_s)
        if self.map_latched:
            if (now_s - self._last_revoke) >= self.revoke_repeat_period:
                self._publish_reset(stale_reason or "map_latched", now_s)
            return Twist(), "map_latched"
        if stale_reason:
            # 没有任务可撤销（启动阶段或任务已结束）：直接零，不锁止。
            return Twist(), stale_reason
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
                "%.3f" % ((self._now_s() - self.map_recv) if self.map_recv else -1.0),
                int(self.map_latched), self.external_publishers, self.graph_violation])

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
