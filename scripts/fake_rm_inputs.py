#!/usr/bin/env python3
"""合成 RM 输入源（测试用，不属于实车部署链）。

只发布 RM 侧已有的话题与 TF，用来在没有录包/没有实车时验证影子接入契约：

    TF: odom→base_link(动态), map→odom(动态), base_link→base_footprint,
        base_footprint→lidar_link, odom→camera_init（单位，当前 RM 默认行为）
    话题: /Odometry_transformed, /LIVO2/imu_propagate, /cloud_registered
    任务: /sentry_scan/move_base_simple/goal, /sentry_scan/planning/reset（仅测试场景）

故意**不**发布任何速度命令；它不是车辆接口，也不代表真实传感器噪声模型。

故障注入（B3 测试矩阵）：
    --repeat-old-stamp       header.stamp 立刻固定不前进（旧消息重发）
    --freeze-stamp-after N   N 秒后把 header.stamp 固定（先正常运行再模拟旧 stamp 重发）
    --second-goal-after T   T 秒后再发一个目标（用于验证"新任务才恢复"）
    --tf-after-odom          复现实车 TfTransformer 的顺序：先发 /Odometry_transformed，
                             再广播**同 stamp** 的 odom→base_link（动态 TF 晚于消息到达）
    --tf-delay-after-odom N  配合 --tf-after-odom：把动态 TF 延后 N 秒发布（默认 0.0），
                             用来稳定复现"查询时 TF 还没进 buffer"（实车受调度/DDS 影响）
    --padded-cloud           发布带行填充的点云（point_step=16、row_step>width*point_step，
                             填充区写 (90,90,90)），用于验证适配层会先重排为密集布局
    --fake-map-heartbeat-until / --fake-map-heartbeat-resume
                            测试心跳在 N 秒停、M 秒恢复（验证地图锁止）
    注入故障时会在 /sentry_scan/test/fault_marker 发布一条标记（frame_id=原因），
    供判据脚本按**实际事件时刻**计算停车延迟。
    --stamp-backwards-after  N 秒后周期性让时间戳倒退且不再追上
    --stop-cloud-after       N 秒后停止发布点云
    --jump-after             N 秒后 map→odom 平移跳变 1.0 m
    --cancel-after           N 秒后发布 planning/reset
    --send-goal              发布一个 Mode 1 目标（默认发给 task/goal_in，frame 见 --goal-frame）
    --goal-frame            目标/路线的 frame（默认 odom；设成 map 可测坐标变换）
    --map-odom-offset       非单位 map->odom："dx,dy,yaw"（默认 0,0,0）
    --nan-cloud-after       N 秒后把云换成全 NaN（测"有消息但无有效点"）
    --stop-cloud-after      N 秒后停止发布云；--resume-cloud-after M 秒后恢复
    --spoof-nav2-cmdvel     由另一个名为 controller_server 的节点发布 /cmd_vel（测与 Nav2 并存）
    --flood-stale-sensor-pose-after
                            N 秒后以 50 Hz 向 /sentry_scan/sensor_pose 注入过期时间戳
                            （让 GridMap 严格配对持续拒绝适配器的云，但适配器本身仍然健康）
"""

import argparse
import math
import struct
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Bool, Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def _quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def _tf(parent, child, translation, rotation, stamp):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    (msg.transform.translation.x, msg.transform.translation.y,
     msg.transform.translation.z) = translation
    (msg.transform.rotation.x, msg.transform.rotation.y,
     msg.transform.rotation.z, msg.transform.rotation.w) = rotation
    return msg


class FakeRmInputs(Node):
    def __init__(self, args):
        super().__init__("fake_rm_inputs")
        self.args = args
        self.t0 = time.time()
        self.old_stamp = None
        self.backwards_offset = 0.0
        self.jump_offset = (0.0, 0.0, 0.0)
        self.map_odom_offset = self._parse_offset(args.map_odom_offset)
        self._goal_sent = False
        self._second_goal_sent = False
        self._cancelled = False
        self._marked = set()
        self._cloud_stopped = False
        self._nan_started = False
        self._heartbeat_cut = False
        self._pending_tf = []
        self._flood_pub = None

        self.dyn_tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()
        lidar_to_base = (-0.03, 0.077, 0.0601)
        self.static_tf.sendTransform([
            _tf("base_link", "base_footprint", tuple(-v for v in lidar_to_base),
                (0.0, 0.0, 0.0, 1.0), now),
            _tf("base_footprint", "lidar_link", lidar_to_base, (0.0, 0.0, 0.0, 1.0), now),
            _tf("odom", "camera_init", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), now),
        ])

        self.odom_pub = self.create_publisher(Odometry, "/Odometry_transformed", qos_profile_sensor_data)
        self.vel_pub = self.create_publisher(Odometry, "/LIVO2/imu_propagate", qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, "/cloud_registered", qos_profile_sensor_data)
        # 目标发给 task_adapter 的输入；FSM 只接受已转换到规划系的 `goal`。
        self.goal_pub = self.create_publisher(PoseStamped, "/sentry_scan/task/goal_in", 1)
        self.reset_pub = self.create_publisher(Bool, "/sentry_scan/planning/reset", 10)
        self.marker_pub = self.create_publisher(Header, "/sentry_scan/test/fault_marker", 10)
        # 仅用于 pairing_mismatch 场景：直接向适配器输出话题注入时间戳不一致的
        # sensor_pose / cloud，用来验证 GridMap 的严格配对拒绝（不经过适配器）。
        self.mismatch_sensor_pub = None
        self.mismatch_cloud_pub = None
        if args.mismatch_pairing:
            self.mismatch_sensor_pub = self.create_publisher(
                Odometry, "/sentry_scan/sensor_pose", qos_profile_sensor_data)
            self.mismatch_cloud_pub = self.create_publisher(
                PointCloud2, "/sentry_scan/cloud", qos_profile_sensor_data)
            self.create_timer(0.2, self.on_mismatch_pair)

        # 与 Nav2 并存的场景由独立进程 scripts/spoof_nav2_cmdvel.py 提供（节点名 controller_server），
        # 避免在同一进程里创建第二个节点导致定时器不执行。
        if args.flood_stale_sensor_pose_after:
            self._flood_pub = self.create_publisher(
                Odometry, "/sentry_scan/sensor_pose", qos_profile_sensor_data)
            self.create_timer(0.02, self.on_flood_stale_sensor_pose)
        # 测试用"地图心跳"替身：只用于验证 shadow_guard 的心跳门控行为。
        self._heartbeat_pub = None
        if args.fake_map_heartbeat_until:
            self._heartbeat_pub = self.create_publisher(Header, "/sentry_scan/test/map_heartbeat", 10)
            self.create_timer(0.1, self.on_heartbeat)

        self.cloud_points = self.build_room_cloud()
        self.create_timer(0.02, self.on_tf)
        self.create_timer(0.02, self.on_odom)
        self.create_timer(0.02, self.on_velocity)
        self.create_timer(0.1, self.on_cloud)
        self.create_timer(0.5, self.on_events)
        if args.tf_after_odom and args.tf_delay_after_odom > 0.0:
            self.create_timer(0.01, self.flush_pending_tf)
        if args.send_goal:
            # 只发一次（重复发目标会在取消后重新授权新任务，使"取消后不动"的判据失真），
            # 但必须等订阅端出现再发：DDS 发现未完成时单次发布会被丢掉，导致场景偶发"无运动"。
            self.create_timer(0.5, self.send_goal_once)

    def mark(self, reason: str) -> None:
        """发布一次故障标记（每种原因只发一次），供判据脚本对齐事件时间。"""
        if reason in self._marked:
            return
        self._marked.add(reason)
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = reason
        self.marker_pub.publish(msg)
        self.get_logger().warn("fake RM inputs: fault marker '%s' published" % reason)

    @staticmethod
    def _parse_offset(text):
        parts = [p for p in (text or "").replace(";", ",").split(",") if p.strip()]
        if len(parts) != 3:
            raise ValueError("--map-odom-offset expects dx,dy,yaw")
        return tuple(float(p) for p in parts)

    def on_heartbeat(self):
        elapsed = self.elapsed()
        if elapsed > self.args.fake_map_heartbeat_until:
            resuming = (self.args.fake_map_heartbeat_resume
                        and elapsed > self.args.fake_map_heartbeat_resume)
            if not resuming:
                self.mark("map_heartbeat_cut")
                return
            self.mark("map_heartbeat_resume")
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = "odom"
        self._heartbeat_pub.publish(msg)

    def on_flood_stale_sensor_pose(self):
        if not self.args.flood_stale_sensor_pose_after:
            return
        if self.elapsed() < self.args.flood_stale_sensor_pose_after:
            return
        msg = Odometry()
        base = self.get_clock().now()
        msg.header.stamp = (base - rclpy.duration.Duration(seconds=1.0)).to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "lidar_link"
        msg.pose.pose.position.z = 0.1
        msg.pose.pose.orientation.w = 1.0
        self._flood_pub.publish(msg)

    def build_padded_cloud(self, header):
        """把房间点云重新打包成**带行填充**的 PointCloud2（填充区写 90.0）。

        point_step=16（xyz + 4 字节填充），row_step = width*16 + 8（每行尾部再填 8 字节）。
        当前环境安装的 do_transform_cloud 会按自身布局重排输出，但**读端若沿用输入布局**
        就会把 90 当成点；适配层因此先按行首址重排为密集布局再变换。
        """
        width = 8
        height = len(self.cloud_points) // width
        point_step, pad_tail = 16, 8
        row_step = width * point_step + pad_tail
        data = bytearray()
        for row in range(height):
            body = bytearray()
            for col in range(width):
                x, y, z = self.cloud_points[row * width + col]
                body += struct.pack("<fff", x, y, z) + struct.pack("<f", 90.0)
            data += body + struct.pack("<ff", 90.0, 90.0)

        msg = PointCloud2()
        msg.header = header
        msg.height, msg.width = height, width
        msg.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                      for name, offset in (("x", 0), ("y", 4), ("z", 8))]
        msg.is_bigendian = False
        msg.point_step, msg.row_step = point_step, row_step
        msg.data = bytes(data)
        msg.is_dense = False
        return msg

    @staticmethod
    def build_room_cloud():
        """四面墙（不含地面），给 SCAN 划出自由空间。"""
        points = []
        for z in (0.2, 0.5, 0.8, 1.1):
            for i in range(-20, 21):
                v = i * 0.2
                points.append((4.0, v, z))
                points.append((-4.0, v, z))
                points.append((v, 4.0, z))
                points.append((v, -4.0, z))
        return points

    def elapsed(self):
        return time.time() - self.t0

    def stamp(self):
        freeze = self.args.repeat_old_stamp or (
            self.args.freeze_stamp_after and self.elapsed() > self.args.freeze_stamp_after)
        if freeze:
            if self.old_stamp is None:
                self.old_stamp = self.get_clock().now().to_msg()
                self.mark("stamp_freeze")
            return self.old_stamp
        now = self.get_clock().now()
        if self.backwards_offset:
            now = now - rclpy.duration.Duration(seconds=self.backwards_offset)
        return now.to_msg()

    # ---------------------------------------------------------------- 发布
    def on_tf(self):
        if self.args.stop_tf_after and self.elapsed() > self.args.stop_tf_after:
            self.mark("tf_stop")
            return
        stamp = self.stamp()
        if not self.args.tf_after_odom:
            self.dyn_tf.sendTransform(
                _tf("odom", "base_link", (0.0, 0.0, 0.1), _quat_from_yaw(0.0), stamp))
        dx, dy, dyaw = self.map_odom_offset
        offset = (dx + self.jump_offset[0], dy + self.jump_offset[1], self.jump_offset[2])
        self.dyn_tf.sendTransform(
            _tf("map", "odom", offset, _quat_from_yaw(dyaw), stamp))

    def on_odom(self):
        if self.args.stop_odom_after and self.elapsed() > self.args.stop_odom_after:
            self.mark("odom_stop")
            return
        msg = Odometry()
        msg.header.stamp = self.stamp()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.z = 0.1
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)
        if self.args.tf_after_odom:
            # 与实车一致：消息先发，动态 TF 晚于消息（可加延迟以稳定复现调度/DDS 差异）
            if self.args.tf_delay_after_odom > 0.0:
                due = self.get_clock().now().nanoseconds * 1e-9 + self.args.tf_delay_after_odom
                self._pending_tf.append((due, msg.header.stamp))
            else:
                self.dyn_tf.sendTransform(
                    _tf("odom", "base_link", (0.0, 0.0, 0.1), _quat_from_yaw(0.0),
                        msg.header.stamp))

    def flush_pending_tf(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        still = []
        for when, stamp in self._pending_tf:
            if now >= when:
                self.dyn_tf.sendTransform(
                    _tf("odom", "base_link", (0.0, 0.0, 0.1), _quat_from_yaw(0.0), stamp))
            else:
                still.append((when, stamp))
        self._pending_tf = still

    def on_velocity(self):
        msg = Odometry()
        msg.header.stamp = self.stamp()
        msg.header.frame_id = "world"
        self.vel_pub.publish(msg)

    def on_cloud(self):
        elapsed = self.elapsed()
        if self.args.stop_cloud_after and elapsed > self.args.stop_cloud_after:
            if not (self.args.resume_cloud_after and elapsed > self.args.resume_cloud_after):
                self.mark("cloud_stop")
                return
            self.mark("cloud_resume")
        header = Header()
        header.stamp = self.stamp()
        header.frame_id = "camera_init"
        points = self.cloud_points
        if self.args.nan_cloud_after and elapsed > self.args.nan_cloud_after:
            self.mark("nan_cloud")
            points = [(float("nan"), float("nan"), float("nan"))] * len(self.cloud_points)
        if self.args.padded_cloud and not (self.args.nan_cloud_after
                                           and elapsed > self.args.nan_cloud_after):
            self.mark("padded_cloud")
            self.cloud_pub.publish(self.build_padded_cloud(header))
        else:
            self.cloud_pub.publish(create_cloud_xyz32(header, points))

    def on_mismatch_pair(self):
        """故意让 sensor_pose 与 cloud 的时间戳差 0.30 s。"""
        now = self.get_clock().now()
        sensor = Odometry()
        sensor.header.stamp = (now - rclpy.duration.Duration(seconds=0.30)).to_msg()
        sensor.header.frame_id = "odom"
        sensor.child_frame_id = "lidar_link"
        sensor.pose.pose.position.z = 0.1
        sensor.pose.pose.orientation.w = 1.0
        self.mismatch_sensor_pub.publish(sensor)
        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = "odom"
        self.mismatch_cloud_pub.publish(create_cloud_xyz32(header, self.cloud_points))

    def send_goal_once(self, x=None, y=None, second=False):
        if not second and self._goal_sent:
            return
        if self.elapsed() < self.args.goal_delay:
            # 太早发目标会撞上"FSM 还没收到第一帧 body_pose"，会被直接忽略
            # （日志 `Ignore RViz goal before receiving initial body pose`）。
            return
        if self.goal_pub.get_subscription_count() == 0:
            # 订阅端尚未匹配（DDS 发现中）：本次不发，等下一次 timer 重试。
            return
        if not second:
            self._goal_sent = True
        msg = PoseStamped()
        # 任务消息用真实当前时间：时间戳故障注入只针对 RM 传感器输入。
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.goal_frame
        msg.pose.position.x = float(self.args.goal_x if x is None else x)
        msg.pose.position.y = float(self.args.goal_y if y is None else y)
        msg.pose.position.z = 0.1
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.get_logger().info(
            "fake RM inputs: sent Mode 1 goal (%.2f, %.2f) in frame '%s' to %s"
            % (msg.pose.position.x, msg.pose.position.y, self.args.goal_frame,
               self.goal_pub.topic_name))

    def on_events(self):
        t = self.elapsed()
        if self.args.stamp_backwards_after and t > self.args.stamp_backwards_after:
            self.mark("stamp_backwards")
            # 每 0.5 s 倒退 2 s：时间戳再也不追上先前值，且来源年龄持续增大。
            self.backwards_offset += 2.0
        if self.args.jump_after and t > self.args.jump_after and self.jump_offset == (0.0, 0.0, 0.0):
            self.mark("localization_jump")
            self.jump_offset = (1.0, 0.0, 0.0)
            self.get_logger().warn("fake RM inputs: injected map->odom jump of 1.0 m")
        if (self.args.second_goal_after and t > self.args.second_goal_after
                and not self._second_goal_sent
                and self.goal_pub.get_subscription_count() > 0):
            self._second_goal_sent = True
            self.send_goal_once(x=self.args.second_goal_x, y=self.args.second_goal_y, second=True)
        if self.args.cancel_after and t > self.args.cancel_after and not self._cancelled:
            self._cancelled = True
            self.mark("cancel")
            msg = Bool()
            msg.data = True
            self.reset_pub.publish(msg)
            self.get_logger().warn("fake RM inputs: published planning/reset (cancel)")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--send-goal", action="store_true")
    parser.add_argument("--repeat-old-stamp", action="store_true")
    parser.add_argument("--stamp-backwards-after", type=float, default=0.0)
    parser.add_argument("--freeze-stamp-after", type=float, default=0.0)
    parser.add_argument("--fake-map-heartbeat-until", type=float, default=0.0)
    parser.add_argument("--fake-map-heartbeat-resume", type=float, default=0.0)
    parser.add_argument("--second-goal-after", type=float, default=0.0)
    parser.add_argument("--second-goal-x", type=float, default=-1.5)
    parser.add_argument("--second-goal-y", type=float, default=1.5)
    parser.add_argument("--stop-cloud-after", type=float, default=0.0)
    parser.add_argument("--stop-odom-after", type=float, default=0.0)
    parser.add_argument("--stop-tf-after", type=float, default=0.0)
    parser.add_argument("--mismatch-pairing", action="store_true")
    parser.add_argument("--padded-cloud", action="store_true")
    parser.add_argument("--tf-after-odom", action="store_true")
    parser.add_argument("--tf-delay-after-odom", type=float, default=0.0)
    parser.add_argument("--resume-cloud-after", type=float, default=0.0)
    parser.add_argument("--nan-cloud-after", type=float, default=0.0)
    parser.add_argument("--flood-stale-sensor-pose-after", type=float, default=0.0)
    parser.add_argument("--goal-delay", type=float, default=4.0,
                        help="启动后至少等这么久再发目标（等 FSM 收到第一帧 body_pose）")
    parser.add_argument("--goal-x", type=float, default=2.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-frame", default="odom")
    parser.add_argument("--map-odom-offset", default="0,0,0")
    parser.add_argument("--jump-after", type=float, default=0.0)
    parser.add_argument("--cancel-after", type=float, default=0.0)
    parser.add_argument("--domain-note", default="")
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = FakeRmInputs(args)
    node.get_logger().warn("fake RM inputs started (scenario=%s, duration=%.1fs)"
                           % (args.domain_note, args.duration))
    end = time.time() + args.duration
    try:
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
