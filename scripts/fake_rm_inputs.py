#!/usr/bin/env python3
"""合成 RM 输入源（测试用，不属于实车部署链）。

只发布 RM 侧已有的话题与 TF，用来在没有录包/没有实车时验证影子接入契约：

    TF: odom→base_link(动态), map→odom(动态), base_link→base_footprint,
        base_footprint→lidar_link, odom→camera_init（单位，当前 RM 默认行为）
    话题: /Odometry_transformed, /LIVO2/imu_propagate, /cloud_registered
    任务: /sentry_scan/move_base_simple/goal, /sentry_scan/planning/reset（仅测试场景）

故意**不**发布任何速度命令；它不是车辆接口，也不代表真实传感器噪声模型。

故障注入（B3 测试矩阵）：
    --repeat-old-stamp       header.stamp 固定不前进（旧消息重发）
    --stamp-backwards-after  N 秒后周期性让时间戳倒退且不再追上
    --stop-cloud-after       N 秒后停止发布点云
    --jump-after             N 秒后 map→odom 平移跳变 1.0 m
    --cancel-after           N 秒后发布 planning/reset
    --send-goal              发布一个 Mode 1 目标
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
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
        self._goal_sent = False
        self._cancelled = False

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
        self.goal_pub = self.create_publisher(PoseStamped, "/sentry_scan/move_base_simple/goal", 1)
        self.reset_pub = self.create_publisher(Bool, "/sentry_scan/planning/reset", 10)

        self.cloud_points = self.build_room_cloud()
        self.create_timer(0.02, self.on_tf)
        self.create_timer(0.02, self.on_odom)
        self.create_timer(0.02, self.on_velocity)
        self.create_timer(0.1, self.on_cloud)
        self.create_timer(0.5, self.on_events)
        if args.send_goal:
            # 只发一次：重复发目标会在取消后重新授权新任务，使"取消后不动"的判据失真。
            self.create_timer(2.5, self.send_goal_once)

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
        if self.args.repeat_old_stamp:
            if self.old_stamp is None:
                self.old_stamp = self.get_clock().now().to_msg()
            return self.old_stamp
        now = self.get_clock().now()
        if self.backwards_offset:
            now = now - rclpy.duration.Duration(seconds=self.backwards_offset)
        return now.to_msg()

    # ---------------------------------------------------------------- 发布
    def on_tf(self):
        stamp = self.stamp()
        self.dyn_tf.sendTransform(
            _tf("odom", "base_link", (0.0, 0.0, 0.1), _quat_from_yaw(0.0), stamp))
        self.dyn_tf.sendTransform(
            _tf("map", "odom", self.jump_offset, (0.0, 0.0, 0.0, 1.0), stamp))

    def on_odom(self):
        msg = Odometry()
        msg.header.stamp = self.stamp()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.z = 0.1
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)

    def on_velocity(self):
        msg = Odometry()
        msg.header.stamp = self.stamp()
        msg.header.frame_id = "world"
        self.vel_pub.publish(msg)

    def on_cloud(self):
        if self.args.stop_cloud_after and self.elapsed() > self.args.stop_cloud_after:
            return
        header = Header()
        header.stamp = self.stamp()
        header.frame_id = "camera_init"
        self.cloud_pub.publish(create_cloud_xyz32(header, self.cloud_points))

    def send_goal_once(self):
        if self._goal_sent:
            return
        self._goal_sent = True
        msg = PoseStamped()
        msg.header.stamp = self.stamp()
        msg.header.frame_id = "odom"
        msg.pose.position.x = 2.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.1
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.get_logger().info("fake RM inputs: sent Mode 1 goal (2.0, 0.0) in odom")

    def on_events(self):
        t = self.elapsed()
        if self.args.stamp_backwards_after and t > self.args.stamp_backwards_after:
            # 每 0.5 s 倒退 2 s：时间戳再也不追上先前值，且来源年龄持续增大。
            self.backwards_offset += 2.0
        if self.args.jump_after and t > self.args.jump_after and self.jump_offset == (0.0, 0.0, 0.0):
            self.jump_offset = (1.0, 0.0, 0.0)
            self.get_logger().warn("fake RM inputs: injected map->odom jump of 1.0 m")
        if self.args.cancel_after and t > self.args.cancel_after and not self._cancelled:
            self._cancelled = True
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
    parser.add_argument("--stop-cloud-after", type=float, default=0.0)
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
