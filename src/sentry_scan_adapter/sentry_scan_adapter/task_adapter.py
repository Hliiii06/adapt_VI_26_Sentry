"""任务/路线坐标适配：把 Mode 1 目标与 Mode 3 参考路线转换到规划系。

背景（审查 P1）：SCAN 的 FSM **不按 `header.frame_id` 做变换**，把收到的坐标直接当规划系数值。
影子入口因此不能把 RViz/上层的目标或参考路线直接接进 FSM，否则 `map` 下的坐标会被当成 `odom`。

本节点：
  * 订阅 `task/goal_in`(PoseStamped) 与 `task/path_in`(Path)；
  * 按**消息时间戳**查询 TF `planning_frame <- header.frame_id`（允许小范围未来容差，
    与 rm_input_adapter 同一策略；超限即拒绝）；
  * 转换后以 `planning_frame` 重新发布到 `goal` 与 `initial_path`（FSM 的输入）；
  * 空 frame / 未声明 frame / 查不到 TF / 时间戳异常：**明确拒绝并告警，不发布**。

不做的事：不发布任何速度命令；不改变 FSM 侧订阅名以外的接口；不静默假设同名坐标系。
"""

import copy
import math
from typing import Optional

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose_stamped


class TaskAdapter(Node):
    def __init__(self) -> None:
        super().__init__("task_adapter")
        self.planning_frame = self.declare_parameter("planning_frame", "odom").value
        self.goal_in_topic = self.declare_parameter("goal_in_topic", "task/goal_in").value
        self.goal_out_topic = self.declare_parameter("goal_out_topic", "goal").value
        self.path_in_topic = self.declare_parameter("path_in_topic", "task/path_in").value
        self.path_out_topic = self.declare_parameter("path_out_topic", "initial_path").value
        self.tf_lookup_timeout = float(self.declare_parameter("tf_lookup_timeout", 0.0).value)
        self.tf_future_tolerance = float(self.declare_parameter("tf_future_tolerance", 0.05).value)
        self.max_source_age = float(self.declare_parameter("max_source_age", 1.0).value)
        self.max_future_stamp = float(self.declare_parameter("max_future_stamp", 0.05).value)
        self.empty_frame_is_planning = bool(
            self.declare_parameter("empty_frame_is_planning", False).value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        goal_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        # reference_path_publisher 用 transient_local 发布，订阅端必须同样 TL 才拿得到。
        path_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, self.goal_in_topic, self.on_goal, goal_qos)
        self.create_subscription(Path, self.path_in_topic, self.on_path, path_qos)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_out_topic, goal_qos)
        # 路线用 transient_local 发布：与订阅端（FSM volatile、检查器 TL）都兼容，
        # 也避免"发布时订阅者尚未建立"导致的静默丢失。
        self.path_pub = self.create_publisher(Path, self.path_out_topic, path_qos)

        self.goal_accepted = 0
        self.goal_rejected = 0
        self.path_accepted = 0
        self.path_rejected = 0

        self.get_logger().warn(
            "Task adapter ready: '%s' -> '%s', '%s' -> '%s'; all inputs are transformed into '%s' "
            "at their message stamp. Unknown/empty frames are rejected, never passed through."
            % (self.goal_in_topic, self.goal_out_topic, self.path_in_topic, self.path_out_topic,
               self.planning_frame))

    # ------------------------------------------------------------------ 工具
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_s(msg) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _stamp_ok(self, stamp_s: float, what: str) -> bool:
        now_s = self._now_s()
        if not math.isfinite(stamp_s) or stamp_s <= 1e-5:
            self.get_logger().warn("Rejecting %s with missing/invalid stamp (%.6f)" % (what, stamp_s),
                                   throttle_duration_sec=2.0)
            return False
        if (stamp_s - now_s) > self.max_future_stamp:
            self.get_logger().warn(
                "Rejecting %s with a stamp %.3fs in the future" % (what, stamp_s - now_s),
                throttle_duration_sec=2.0)
            return False
        if (now_s - stamp_s) > self.max_source_age:
            self.get_logger().warn(
                "Rejecting %s whose stamp is %.3fs old (limit %.3fs)"
                % (what, now_s - stamp_s, self.max_source_age), throttle_duration_sec=2.0)
            return False
        return True

    def _resolve_frame(self, frame_id: str, what: str) -> Optional[str]:
        frame = (frame_id or "").strip()
        if not frame:
            if self.empty_frame_is_planning:
                return self.planning_frame
            self.get_logger().warn(
                "Rejecting %s with an empty header.frame_id; set empty_frame_is_planning only after "
                "verifying the publisher really means the planning frame" % what,
                throttle_duration_sec=2.0)
            return None
        return frame

    def _transform(self, target_frame: str, stamp_s: float, what: str):
        if target_frame == self.planning_frame:
            return None, True  # 已经在规划系，无需变换
        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame, target_frame, Time(seconds=stamp_s),
                timeout=Duration(seconds=self.tf_lookup_timeout))
            return transform, True
        except Exception as exc:
            message = str(exc)
            is_extrapolation = (isinstance(exc, tf2_ros.ExtrapolationException)
                                or "extrapolation" in message.lower()
                                or "future" in message.lower())
            if is_extrapolation:
                try:
                    latest = self.tf_buffer.lookup_transform(
                        self.planning_frame, target_frame, Time(),
                        timeout=Duration(seconds=self.tf_lookup_timeout))
                    latest_s = (float(latest.header.stamp.sec)
                                + float(latest.header.stamp.nanosec) * 1e-9)
                    delay = stamp_s - latest_s
                    if 0.0 <= delay <= self.tf_future_tolerance:
                        return latest, True
                except Exception:
                    pass
            self.get_logger().warn(
                "Rejecting %s: cannot transform '%s' -> '%s' at %.3f (%s)"
                % (what, target_frame, self.planning_frame, stamp_s, exc),
                throttle_duration_sec=2.0)
            return None, False

    # ------------------------------------------------------------------ 回调
    def on_goal(self, msg: PoseStamped) -> None:
        stamp_s = self._stamp_s(msg)
        if not self._stamp_ok(stamp_s, "goal"):
            self.goal_rejected += 1
            return
        source = self._resolve_frame(msg.header.frame_id, "goal")
        if source is None:
            self.goal_rejected += 1
            return
        transform, ok = self._transform(source, stamp_s, "goal")
        if not ok:
            self.goal_rejected += 1
            return
        out = do_transform_pose_stamped(msg, transform) if transform is not None else msg
        out.header.frame_id = self.planning_frame
        out.header.stamp = msg.header.stamp
        self.goal_pub.publish(out)
        self.goal_accepted += 1
        self.get_logger().info(
            "goal %s (%.3f, %.3f, %.3f) -> %s (%.3f, %.3f, %.3f)"
            % (source, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
               self.planning_frame, out.pose.position.x, out.pose.position.y, out.pose.position.z))

    def on_path(self, msg: Path) -> None:
        if not msg.poses:
            self.path_rejected += 1
            self.get_logger().warn("Rejecting empty reference path", throttle_duration_sec=2.0)
            return
        stamp_s = self._stamp_s(msg)
        if not self._stamp_ok(stamp_s, "reference path"):
            self.path_rejected += 1
            return
        source = self._resolve_frame(msg.header.frame_id, "reference path")
        if source is None:
            self.path_rejected += 1
            return
        transform, ok = self._transform(source, stamp_s, "reference path")
        if not ok:
            self.path_rejected += 1
            return
        # 这份 tf2_geometry_msgs 没有 do_transform_path，逐点用同一时刻的变换（契约允许：
        # 整条路线按消息时间戳的单一 TF 转换，不做逐点时间插值）。
        out = Path()
        out.header.frame_id = self.planning_frame
        out.header.stamp = msg.header.stamp
        for pose in msg.poses:
            converted = (do_transform_pose_stamped(copy.deepcopy(pose), transform)
                         if transform is not None else copy.deepcopy(pose))
            converted.header.frame_id = self.planning_frame
            converted.header.stamp = pose.header.stamp
            out.poses.append(converted)
        self.path_pub.publish(out)
        self.path_accepted += 1
        first = out.poses[0].pose.position
        self.get_logger().info(
            "reference path %s -> %s, %d poses, first (%.3f, %.3f)"
            % (source, self.planning_frame, len(out.poses), first.x, first.y))


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = TaskAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
