"""RM → SCAN 输入适配节点（影子运行，只读输入）。

职责（契约见 docs/interfaces/shadow_input_contract.md）：

    RM TF/odom/云 ──> 本节点 ──> /sentry_scan/{body_pose,sensor_pose,cloud}
                              ──> /sentry_scan/health, /sentry_scan/health_ok
                              ──> /sentry_scan/planning/reset（失效时撤销任务）

硬性边界：
  * 不创建任何速度发布者，**不会**发布 /cmd_vel、/cmd_vel_remap 或 /sentry_scan/cmd_vel*；
  * 不启动 / 不依赖 UART、Nav2、LIO、registration、机器人驱动，只订阅它们已有的话题与 TF；
  * 输出云与射线原点按**同一消息时间戳**配对；时间/有限值/TF/定位跳变不合格即拒绝该帧，
    并使整组输入进入不健康状态（停止输出 + 发布 reset），恢复后不自动恢复旧任务。

已知限制（必须随交付一起说明，不得当成已解决）：
  * 速度候选来自 `/LIVO2/imu_propagate`，是 **IMU 点的世界系线速度**。未配置
    `imu_to_body_offset_xyz` 时不做杆臂补偿，诊断里标 `lever_arm_uncompensated`；
    配置 `require_center_velocity=true` 时直接拒绝该速度（twist 置零）。
  * 角速度只有在来源消息 `twist.angular` 非零且通过有限值检查时才使用；RM 当前
    是否填充分量需实机核对。
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


def _parse_vector(text: str, default: Sequence[float]) -> List[float]:
    text = (text or "").strip()
    if not text:
        return list(default)
    parts = [p for p in text.replace(";", ",").split(",") if p.strip() != ""]
    if len(parts) != 3:
        raise ValueError("expected 3 comma-separated numbers, got %r" % text)
    return [float(p) for p in parts]


def _finite(*values: float) -> bool:
    return all(math.isfinite(float(v)) for v in values)


def _quat_valid(q) -> bool:
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    return math.isfinite(norm) and norm > 1e-6


def _rotate(q, v: Sequence[float]) -> Tuple[float, float, float]:
    """用四元数 q（需已归一化）旋转向量 v。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _count_finite_points(msg) -> tuple:
    """按 PointCloud2 的**消息布局**统计有限 xyz 点数，返回 (有效点数, 原因)。

    只支持一种布局，其余显式拒绝（不猜、不误读）：

      * `is_bigendian == false`；
      * x/y/z 三个字段都存在且 `datatype == FLOAT32`；
      * `row_step >= width * point_step`、`len(data) >= height * row_step`；
      * 字段偏移在 `point_step` 内。

    行填充（`row_step > width * point_step`）按行起始地址逐点寻址，不会把填充字节当成点。
    """
    if msg.is_bigendian:
        return 0, "unsupported layout: is_bigendian=true"
    if msg.width <= 0 or msg.height <= 0:
        return 0, "empty cloud (%dx%d)" % (msg.width, msg.height)
    if msg.point_step < 12:
        return 0, "point_step=%d < 12" % msg.point_step
    if msg.row_step < msg.width * msg.point_step:
        return 0, "row_step=%d < width*point_step=%d" % (msg.row_step, msg.width * msg.point_step)
    if msg.height * msg.row_step > len(msg.data):
        return 0, "data=%d bytes < height*row_step=%d" % (len(msg.data), msg.height * msg.row_step)

    offsets = {}
    for field in msg.fields:
        if field.name not in ("x", "y", "z"):
            continue
        if field.datatype != PointField.FLOAT32:
            return 0, ("field %s datatype=%d is not FLOAT32(%d); layout unsupported"
                       % (field.name, field.datatype, PointField.FLOAT32))
        if field.offset + 4 > msg.point_step:
            return 0, "%s offset %d exceeds point_step %d" % (field.name, field.offset, msg.point_step)
        offsets[field.name] = field.offset
    if len(offsets) != 3:
        return 0, "missing x/y/z fields (got %s)" % sorted(offsets)

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    # 行填充安全：每行按 row_step 起址，行内按 point_step 起址。
    rows = np.arange(msg.height, dtype=np.int64) * msg.row_step
    cols = np.arange(msg.width, dtype=np.int64) * msg.point_step
    starts = (rows[:, None] + cols[None, :]).reshape(-1)
    axes = []
    for name in ("x", "y", "z"):
        off = offsets[name]
        four_bytes = np.stack([raw[starts + off + i] for i in range(4)], axis=1)
        axes.append(four_bytes.copy().view("<f4").reshape(-1))
    valid = np.isfinite(axes[0]) & np.isfinite(axes[1]) & np.isfinite(axes[2])
    return int(valid.sum()), ""


def _pose_from_transform(transform) -> Pose:
    """TransformStamped.transform (Transform) -> geometry_msgs/Pose。"""
    pose = Pose()
    pose.position.x = transform.translation.x
    pose.position.y = transform.translation.y
    pose.position.z = transform.translation.z
    pose.orientation = transform.rotation
    return pose


def _stamp_to_msg(stamp_s: float):
    from builtin_interfaces.msg import Time as TimeMsg
    msg = TimeMsg()
    msg.sec = int(math.floor(stamp_s))
    msg.nanosec = int(round((stamp_s - math.floor(stamp_s)) * 1e9))
    if msg.nanosec >= 1000000000:
        msg.sec += 1
        msg.nanosec -= 1000000000
    return msg


@dataclass
class Channel:
    """单个输入通道的运行时状态，用于健康判定与诊断输出。"""

    name: str
    last_receive: Optional[float] = None
    last_valid: Optional[float] = None
    last_stamp: Optional[float] = None
    last_frame: str = ""
    count: int = 0
    rejected: int = 0
    error: str = ""
    extra: Dict[str, str] = field(default_factory=dict)

    def fresh(self, now_s: float, max_receive_age: float) -> bool:
        return self.last_valid is not None and (now_s - self.last_valid) <= max_receive_age


class RmInputAdapter(Node):
    def __init__(self) -> None:
        super().__init__("rm_input_adapter")

        # ---- 坐标系与参考点 ----
        self.planning_frame = self.declare_parameter("planning_frame", "odom").value
        self.task_frame = self.declare_parameter("task_frame", "map").value
        self.body_frame = self.declare_parameter("body_frame", "base_link").value
        self.sensor_frame = self.declare_parameter("sensor_frame", "lidar_link").value
        self.body_center_offset = _parse_vector(
            self.declare_parameter("body_center_offset_xyz", "").value, (0.0, 0.0, 0.0))
        raw_imu_offset = self.declare_parameter("imu_to_body_offset_xyz", "").value
        self.imu_to_body_offset: Optional[List[float]] = (
            _parse_vector(raw_imu_offset, (0.0, 0.0, 0.0)) if raw_imu_offset.strip() else None)

        # ---- 输入话题 ----
        self.odom_topic = self.declare_parameter("odom_topic", "/Odometry_transformed").value
        self.velocity_topic = self.declare_parameter("velocity_topic", "/LIVO2/imu_propagate").value
        self.cloud_topic = self.declare_parameter("cloud_topic", "/cloud_registered").value
        # 点云结构/数值检查：点数 ≠ 有效点。全 NaN 或结构不一致的云必须拒绝，
        # 否则"有消息"会被当成"地图有更新"。
        self.min_valid_points = int(self.declare_parameter("min_valid_points", 10).value)
        self.velocity_frame = self.declare_parameter("velocity_frame", "world").value
        # RM 的 /LIVO2/imu_propagate header 写 world，但 world 通常不是 TF 里的 frame。
        # `velocity_frame_alias` 用于显式声明"该名字在数值上等同规划系"（I1 必须核对后填写）；
        # 不填且 TF 查不到时，速度显式降级为零，而不是把它当成机体中心速度。
        self.velocity_frame_alias = self.declare_parameter("velocity_frame_alias", "").value

        # ---- 时间 / TF 门控 ----
        self.max_source_age = float(self.declare_parameter("max_source_age", 0.5).value)
        self.max_receive_age = float(self.declare_parameter("max_receive_age", 0.5).value)
        self.max_stamp_regression = float(self.declare_parameter("max_stamp_regression", 0.0).value)
        # 未来时间戳容差：超过它的 stamp 直接拒绝，且**不更新** last_stamp
        # （否则一个 3600 s 之后的坏 stamp 会让之后所有正常消息都被判成"时间倒退"）。
        self.max_future_stamp = float(self.declare_parameter("max_future_stamp", 0.05).value)
        # 0 = 非阻塞查询。带 timeout 的查询会在单线程 executor 里阻塞 TF 订阅，
        # 使 tf buffer 越来越旧（实测滞后可达 1 s 以上），因此默认 0 并靠容差回退。
        self.tf_lookup_timeout = float(self.declare_parameter("tf_lookup_timeout", 0.0).value)
        # 同一节点在同一回调里先广播 TF 再发布消息时，tf listener 可能还没收到该时刻的 TF；
        # 允许用"最新可用 TF"代替，但偏差必须在该容差内（配对语义仍然按消息时刻约束）。
        self.tf_future_tolerance = float(self.declare_parameter("tf_future_tolerance", 0.05).value)
        self.require_tf = bool(self.declare_parameter("require_tf", True).value)
        self.cloud_assume_planning_frame = bool(
            self.declare_parameter("cloud_assume_planning_frame", False).value)

        # ---- 定位跳变 / 失效锁止 ----
        self.monitor_task_frame = bool(self.declare_parameter("monitor_task_frame", True).value)
        self.jump_translation = float(self.declare_parameter("localization_jump_translation", 0.5).value)
        self.jump_rotation = float(self.declare_parameter("localization_jump_rotation", 0.35).value)
        self.jump_latch_duration = float(self.declare_parameter("jump_latch_duration", 0.0).value)
        self.reset_on_unhealthy = bool(self.declare_parameter("reset_on_unhealthy", True).value)
        self.reset_repeat_period = float(self.declare_parameter("reset_repeat_period", 1.0).value)

        # ---- 速度语义 ----
        self.require_center_velocity = bool(self.declare_parameter("require_center_velocity", False).value)
        self.require_angular_velocity = bool(
            self.declare_parameter("require_angular_velocity", False).value)

        self.health_period = float(self.declare_parameter("health_period", 0.5).value)

        self.channels: Dict[str, Channel] = {
            "odom": Channel("odom"),
            "velocity": Channel("velocity"),
            "cloud": Channel("cloud"),
            "tf": Channel("tf"),
        }
        self.health_ok = False
        # 只有"曾经健康过"之后的失效才需要撤销任务：启动阶段输入还没到，
        # 此时发 planning/reset 会无谓地取消不存在的任务，并会永久关掉 Mode 2 的自动起步。
        self._ever_healthy = False
        self.active_reasons: List[str] = []
        self.jump_reason = ""
        self._last_reset_time = -1.0
        self._jump_latched_until = 0.0
        self._warned_lever_arm = False
        self._last_map_odom: Optional[Tuple[float, ...]] = None

        # 最新速度样本：(recv_s, stamp_s, frame, linear, angular)
        self._velocity: Optional[Tuple[float, float, str, Tuple[float, float, float],
                                       Tuple[float, float, float]]] = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.velocity_topic, self.on_velocity, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, qos_profile_sensor_data)

        self.body_pose_pub = self.create_publisher(Odometry, "body_pose", qos_profile_sensor_data)
        self.sensor_pose_pub = self.create_publisher(Odometry, "sensor_pose", qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, "cloud", qos_profile_sensor_data)
        self.reset_pub = self.create_publisher(Bool, "planning/reset", 10)
        self.health_pub = self.create_publisher(DiagnosticArray, "health", 10)
        latch_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.health_ok_pub = self.create_publisher(Bool, "health_ok", latch_qos)

        self.create_timer(self.health_period, self.publish_health)
        if self.monitor_task_frame:
            self.create_timer(max(0.05, self.health_period), self.check_localization_jump)

        self.get_logger().warn(
            "RM input adapter (SHADOW, read-only inputs). planning=%s task=%s body=%s sensor=%s "
            "cloud_assume_planning_frame=%s require_tf=%s source_age<=%.2fs receive_age<=%.2fs "
            "max_future_stamp=%.2fs min_valid_points=%d"
            % (self.planning_frame, self.task_frame, self.body_frame, self.sensor_frame,
               self.cloud_assume_planning_frame, self.require_tf,
               self.max_source_age, self.max_receive_age,
               self.max_future_stamp, self.min_valid_points))
        self.get_logger().warn(
            "This node publishes NO velocity command. Velocity is the IMU-point velocity unless "
            "imu_to_body_offset_xyz is set (require_center_velocity=%s)."
            % self.require_center_velocity)

    # ------------------------------------------------------------------ 基础
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _stamp_seconds(self, msg) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _reject(self, channel: Channel, reason: str) -> None:
        channel.rejected += 1
        channel.error = reason
        self.get_logger().warn("%s rejected: %s" % (channel.name, reason), throttle_duration_sec=2.0)

    def _accept_stamp(self, channel: Channel, stamp_s: float, recv_s: float) -> bool:
        if not math.isfinite(stamp_s) or stamp_s <= 1e-5:
            self._reject(channel, "missing/invalid header.stamp (%.6f)" % stamp_s)
            return False
        future = stamp_s - recv_s
        if future > self.max_future_stamp:
            self._reject(channel, "timestamp %.3f is %.3fs in the future (limit %.3fs); "
                                  "not recorded as history"
                         % (stamp_s, future, self.max_future_stamp))
            return False
        if channel.last_stamp is not None and stamp_s < channel.last_stamp - self.max_stamp_regression:
            self._reject(channel, "timestamp went backwards: %.3f -> %.3f"
                         % (channel.last_stamp, stamp_s))
            return False
        source_age = recv_s - stamp_s
        if source_age > self.max_source_age:
            self._reject(channel, "source stamp is %.3fs old (limit %.3fs)"
                         % (source_age, self.max_source_age))
            return False
        channel.last_stamp = stamp_s
        return True

    def _lookup(self, target: str, source: str, stamp_s: float, allow_future: bool = False,
                required: bool = True):
        delay = 0.0
        try:
            transform = self.tf_buffer.lookup_transform(
                target, source, Time(seconds=stamp_s),
                timeout=Duration(seconds=self.tf_lookup_timeout))
        except Exception as exc:  # tf2 can raise several exception types
            message = str(exc)
            is_extrapolation = (isinstance(exc, tf2_ros.ExtrapolationException)
                                or "extrapolation" in message.lower()
                                or "future" in message.lower())
            if allow_future and is_extrapolation:
                # 回退到最新可用 TF，但只在该 TF 与消息时刻的偏差在容差内时接受。
                try:
                    transform = self.tf_buffer.lookup_transform(
                        target, source, Time(),
                        timeout=Duration(seconds=self.tf_lookup_timeout))
                    latest = (float(transform.header.stamp.sec)
                              + float(transform.header.stamp.nanosec) * 1e-9)
                    # 只接受"请求时刻略微领先于 buffer"的情形；请求早于 buffer 起点
                    # （past extrapolation）说明 TF 数据缺失，直接拒绝。
                    delay = stamp_s - latest
                    if delay < 0.0 or delay > self.tf_future_tolerance:
                        if required and self.require_tf:
                            self._reject(self.channels["tf"],
                                         "lookup %s<-%s at %.3f needs future data (nearest %.3f, "
                                         "delta %.4fs > %.4fs)"
                                         % (target, source, stamp_s, latest, delay,
                                            self.tf_future_tolerance))
                        return None
                except Exception as exc2:
                    if required and self.require_tf:
                        self._reject(self.channels["tf"],
                                     "lookup %s<-%s at %.3f failed: %s / fallback: %s"
                                     % (target, source, stamp_s, exc, exc2))
                    return None
            else:
                if required and self.require_tf:
                    self._reject(self.channels["tf"],
                                 "lookup %s<-%s at %.3f failed: %s" % (target, source, stamp_s, exc))
                return None
        tf_channel = self.channels["tf"]
        tf_channel.count += 1
        tf_channel.last_valid = self._now_s()
        tf_channel.last_receive = tf_channel.last_valid
        tf_channel.last_frame = "%s<-%s" % (target, source)
        tf_channel.extra["last_lookup_delay_s"] = "%.4f" % delay
        if not self.jump_reason:
            tf_channel.error = ""
        return transform

    # --------------------------------------------------------------- 输入回调
    def on_odom(self, msg: Odometry) -> None:
        channel = self.channels["odom"]
        channel.count += 1
        recv_s = self._now_s()
        stamp_s = self._stamp_seconds(msg)
        if not self._accept_stamp(channel, stamp_s, recv_s):
            return
        pose = msg.pose.pose
        if not _finite(pose.position.x, pose.position.y, pose.position.z) or not _quat_valid(pose.orientation):
            self._reject(channel, "non-finite position or invalid quaternion")
            return
        tf = self._lookup(self.planning_frame, self.body_frame, stamp_s, allow_future=True)
        if tf is None:
            return
        channel.last_receive = recv_s
        channel.last_valid = recv_s
        channel.last_frame = self.planning_frame
        channel.error = ""
        self.publish_body_pose(tf, stamp_s)

    def on_velocity(self, msg: Odometry) -> None:
        channel = self.channels["velocity"]
        channel.count += 1
        recv_s = self._now_s()
        stamp_s = self._stamp_seconds(msg)
        if not self._accept_stamp(channel, stamp_s, recv_s):
            return
        linear = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
        angular = (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z)
        if not _finite(*linear):
            self._reject(channel, "non-finite linear velocity")
            return
        if not _finite(*angular):
            self._reject(channel, "non-finite angular velocity")
            return
        frame = msg.header.frame_id or self.velocity_frame
        channel.last_receive = recv_s
        channel.last_valid = recv_s
        channel.last_frame = frame
        channel.error = ""
        self._velocity = (recv_s, stamp_s, frame, linear, angular)

    def on_cloud(self, msg: PointCloud2) -> None:
        channel = self.channels["cloud"]
        channel.count += 1
        recv_s = self._now_s()
        stamp_s = self._stamp_seconds(msg)
        if not self._accept_stamp(channel, stamp_s, recv_s):
            return
        if msg.width * msg.height == 0:
            self._reject(channel, "empty point cloud (%dx%d)" % (msg.width, msg.height))
            return
        valid_points, why = _count_finite_points(msg)
        if valid_points < self.min_valid_points:
            self._reject(channel, "point cloud has %d finite xyz points (< %d)%s"
                         % (valid_points, self.min_valid_points,
                            (": " + why) if why else ""))
            return
        channel.extra["valid_points"] = str(valid_points)

        source_frame = msg.header.frame_id
        if source_frame == self.planning_frame or (
                self.cloud_assume_planning_frame and source_frame == ""):
            transformed = msg
            transformed.header.frame_id = self.planning_frame
        else:
            tf = self._lookup(self.planning_frame, source_frame, stamp_s, allow_future=True)
            if tf is None:
                return
            try:
                transformed = do_transform_cloud(msg, tf)
            except Exception as exc:
                self._reject(channel, "cloud transform failed: %s" % exc)
                return

        # 射线原点按**同一时间戳**取，保证与云配对；取不到就不发布云。
        sensor_tf = self._lookup(self.planning_frame, self.sensor_frame, stamp_s, allow_future=True)
        if sensor_tf is None:
            return

        sensor_pose = Odometry()
        sensor_pose.header.stamp = msg.header.stamp
        sensor_pose.header.frame_id = self.planning_frame
        sensor_pose.child_frame_id = self.sensor_frame
        sensor_pose.pose.pose = _pose_from_transform(sensor_tf.transform)
        self.sensor_pose_pub.publish(sensor_pose)

        channel.last_receive = recv_s
        channel.last_valid = recv_s
        channel.last_frame = source_frame
        channel.error = ""
        channel.extra["output_points"] = str(msg.width * msg.height)
        self.cloud_pub.publish(transformed)

    # --------------------------------------------------------------- 输出组装
    def publish_body_pose(self, tf, stamp_s: float) -> None:
        pose = _pose_from_transform(tf.transform)
        if any(abs(v) > 0.0 for v in self.body_center_offset):
            ox, oy, oz = _rotate(pose.orientation, self.body_center_offset)
            pose.position.x += ox
            pose.position.y += oy
            pose.position.z += oz

        linear, angular, reason = self.build_velocity(stamp_s)
        msg = Odometry()
        msg.header.stamp = _stamp_to_msg(stamp_s)
        msg.header.frame_id = self.planning_frame
        msg.child_frame_id = self.body_frame
        msg.pose.pose = pose
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = linear
        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = angular
        self.channels["odom"].extra["velocity"] = reason
        self.body_pose_pub.publish(msg)

    def build_velocity(self, stamp_s: float) -> Tuple[Tuple[float, float, float],
                                                      Tuple[float, float, float], str]:
        """把速度源表达成规划系下机体参考中心的线/角速度。

        返回 (linear, angular, reason)。任何不确定的情况都显式降级为零速度并写明 reason，
        不把 IMU 点速度悄悄当成机体中心速度。
        """
        zero = (0.0, 0.0, 0.0)
        if self._velocity is None:
            return zero, zero, "velocity_missing"
        recv_s, v_stamp, frame, linear, angular = self._velocity
        now_s = self._now_s()
        if (now_s - recv_s) > self.max_receive_age:
            return zero, zero, "velocity_stale_receive"
        if (now_s - v_stamp) > self.max_source_age:
            return zero, zero, "velocity_stale_stamp"

        frame_is_planning = (frame == self.planning_frame
                             or (self.velocity_frame_alias and frame == self.velocity_frame_alias))
        if not frame_is_planning:
            # 非必需查找：查不到就降级为零速度，不把输入判成不健康（位姿/云才是必需输入）。
            tf_frame = self._lookup(self.planning_frame, frame, v_stamp, allow_future=True,
                                    required=False)
            if tf_frame is None:
                self.get_logger().warn(
                    "velocity frame '%s' has no TF to '%s'; velocity degraded to zero. "
                    "Set velocity_frame_alias only after verifying numeric equivalence."
                    % (frame, self.planning_frame), throttle_duration_sec=5.0)
                return zero, zero, "velocity_frame_unresolved(%s)" % frame
            linear = _rotate(tf_frame.transform.rotation, linear)
            angular = _rotate(tf_frame.transform.rotation, angular)

        if self.require_angular_velocity and all(abs(a) < 1e-9 for a in angular):
            return zero, zero, "angular_velocity_missing"

        if self.imu_to_body_offset is None:
            if self.require_center_velocity:
                return zero, zero, "lever_arm_uncompensated"
            if not self._warned_lever_arm:
                self._warned_lever_arm = True
                self.get_logger().warn(
                    "Publishing IMU-point velocity rotated to %s WITHOUT lever-arm compensation; "
                    "set imu_to_body_offset_xyz or require_center_velocity=true to change this."
                    % self.planning_frame)
            return linear, angular, "lever_arm_uncompensated"

        # v_center = v_point + omega x r；r 是 point->center 的规划系向量，
        # 由机体姿态在速度时刻把 body 系偏移旋到规划系。
        tf_body = self._lookup(self.planning_frame, self.body_frame, v_stamp, allow_future=True)
        if tf_body is None:
            return zero, zero, "lever_arm_body_tf_missing"
        r = _rotate(tf_body.transform.rotation, self.imu_to_body_offset)
        vx = linear[0] + (angular[1] * r[2] - angular[2] * r[1])
        vy = linear[1] + (angular[2] * r[0] - angular[0] * r[2])
        vz = linear[2] + (angular[0] * r[1] - angular[1] * r[0])
        if not _finite(vx, vy, vz):
            return zero, zero, "lever_arm_compensation_non_finite"
        return (vx, vy, vz), angular, "center_velocity_compensated"

    # --------------------------------------------------------------- 健康状态
    def check_localization_jump(self) -> None:
        now_s = self._now_s()
        if now_s < self._jump_latched_until:
            return
        tf = self._lookup(self.planning_frame, self.task_frame, now_s, allow_future=True,
                          required=False)
        if tf is None:
            return
        translation = tf.transform.translation
        rotation = tf.transform.rotation
        key = (translation.x, translation.y, translation.z,
               rotation.x, rotation.y, rotation.z, rotation.w)
        previous = self._last_map_odom
        self._last_map_odom = key
        if previous is None:
            return
        dt = math.dist(key[:3], previous[:3])
        dq = abs(rotation.w * previous[6] + rotation.x * previous[3]
                 + rotation.y * previous[4] + rotation.z * previous[5])
        dr = 2.0 * math.acos(min(1.0, dq))
        if dt > self.jump_translation or dr > self.jump_rotation:
            self._jump_latched_until = (now_s + self.jump_latch_duration
                                        if self.jump_latch_duration > 0 else float("inf"))
            self.jump_reason = ("localization jump %s<-%s: d=%.3fm, r=%.3frad"
                                % (self.planning_frame, self.task_frame, dt, dr))
            self.get_logger().error(
                "Localization jump detected (%s<-%s): d=%.3fm r=%.3frad; old task/trajectory must be "
                "re-issued. Latch=%s"
                % (self.planning_frame, self.task_frame, dt, dr,
                   "restart required" if self.jump_latch_duration <= 0
                   else "%.1fs" % self.jump_latch_duration))

    def collect_reasons(self) -> List[str]:
        now_s = self._now_s()
        reasons: List[str] = []
        if self.jump_reason and now_s < self._jump_latched_until:
            reasons.append(self.jump_reason)
        for name, channel in self.channels.items():
            if channel.error:
                reasons.append("%s: %s" % (name, channel.error))
            if not channel.fresh(now_s, self.max_receive_age):
                reasons.append("%s: no valid sample within %.2fs (count=%d rejected=%d)"
                               % (name, self.max_receive_age, channel.count, channel.rejected))
        return reasons

    def publish_health(self) -> None:
        now_s = self._now_s()
        reasons = self.collect_reasons()
        healthy = not reasons

        status = DiagnosticStatus()
        status.name = "sentry_scan/inputs"
        status.hardware_id = "rm_shadow"
        status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
        status.message = "OK" if healthy else "; ".join(reasons[:4])
        for name, channel in self.channels.items():
            status.values.append(KeyValue(key="%s.count" % name, value=str(channel.count)))
            status.values.append(KeyValue(key="%s.rejected" % name, value=str(channel.rejected)))
            status.values.append(KeyValue(key="%s.frame" % name, value=channel.last_frame))
            status.values.append(KeyValue(
                key="%s.age_s" % name,
                value=("%.3f" % (now_s - channel.last_valid)) if channel.last_valid else "n/a"))
        for key, value in self.channels["odom"].extra.items():
            status.values.append(KeyValue(key="odom.%s" % key, value=value))
        for key, value in self.channels["tf"].extra.items():
            status.values.append(KeyValue(key="tf.%s" % key, value=value))
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self.health_pub.publish(array)

        latch = Bool()
        latch.data = healthy
        self.health_ok_pub.publish(latch)

        if healthy:
            if not self.health_ok:
                self.get_logger().warn("Inputs healthy again; the previous task is NOT resumed. "
                                       "Send a new goal/path to start a new task.")
            self.health_ok = True
            self._ever_healthy = True
            return

        self.health_ok = False
        self.active_reasons = reasons
        if (self.reset_on_unhealthy and self._ever_healthy
                and (now_s - self._last_reset_time) >= self.reset_repeat_period):
            self._last_reset_time = now_s
            reset = Bool()
            reset.data = True
            self.reset_pub.publish(reset)
            self.get_logger().error(
                "Inputs unhealthy -> publishing planning/reset (tasks revoked): %s" % status.message)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = RmInputAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
