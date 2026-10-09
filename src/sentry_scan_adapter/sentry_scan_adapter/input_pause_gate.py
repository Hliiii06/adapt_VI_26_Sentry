"""影子专用输入暂停闸门（现场断流/恢复验证用，只读实车话题、不改实车节点）。

问题（复审 P2）：手册原来让操作者"把输入话题改成不存在再重启影子"，这既会被预检挡下，
也**清空了旧任务**，无法证明"运行中的断流锁止"与"恢复后不自动续跑"。

本节点的作用：插在**实车话题**与 `rm_input_adapter` 之间，由操作者显式暂停/恢复转发：

    实车 /Odometry_transformed ─┐
    实车 /LIVO2/imu_propagate  ─┼─> input_pause_gate ─> /sentry_scan/test/{odom,velocity,cloud}
    实车 /cloud_registered     ─┘        ▲
                                        └─ Bool: /sentry_scan/test/pause_inputs （true=暂停）

暂停时**只停止转发**：规划器、跟踪器、门控继续运行，因此可以观察
"已有非零候选 → 停车撤权 → 恢复转发 → 不发新目标仍保持零"。

边界：
  * 只订阅实车话题，不发布任何实车话题、不发速度、不改 TF；
  * 暂停/恢复状态变化时在 `/sentry_scan/test/fault_marker` 发一条标记，
    供判据脚本按**实际事件时刻**计算停车延迟；
  * 可用 `pause_after` / `resume_after` 自动执行（测试用）。
"""

import argparse
import time
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header


class InputPauseGate(Node):
    def __init__(self, args) -> None:
        super().__init__("input_pause_gate")
        self.args = args
        self.odom_in = self.declare_parameter("odom_in", args.odom_in).value
        self.velocity_in = self.declare_parameter("velocity_in", args.velocity_in).value
        self.cloud_in = self.declare_parameter("cloud_in", args.cloud_in).value
        self.odom_out = self.declare_parameter("odom_out", args.odom_out).value
        self.velocity_out = self.declare_parameter("velocity_out", args.velocity_out).value
        self.cloud_out = self.declare_parameter("cloud_out", args.cloud_out).value
        self.pause_topic = self.declare_parameter("pause_topic", args.pause_topic).value
        self.marker_topic = self.declare_parameter("marker_topic", args.marker_topic).value
        self.pause_after = float(self.declare_parameter("pause_after", args.pause_after).value)
        self.resume_after = float(self.declare_parameter("resume_after", args.resume_after).value)

        self.paused = False
        qos = qos_profile_sensor_data
        self.create_subscription(Odometry, self.odom_in, self.on_odom, qos)
        self.create_subscription(Odometry, self.velocity_in, self.on_velocity, qos)
        self.create_subscription(PointCloud2, self.cloud_in, self.on_cloud, qos)
        self.create_subscription(Bool, self.pause_topic, self.on_pause_command, 10)

        self.odom_pub = self.create_publisher(Odometry, self.odom_out, qos)
        self.velocity_pub = self.create_publisher(Odometry, self.velocity_out, qos)
        self.cloud_pub = self.create_publisher(PointCloud2, self.cloud_out, qos)
        self.marker_pub = self.create_publisher(Header, self.marker_topic, 10)
        latch = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.paused_pub = self.create_publisher(Bool, "test/input_paused", latch)
        self.forwarded = 0
        self.dropped = 0
        self.started = time.time()
        self._automation_done = False

        self.get_logger().warn(
            "Input pause gate: %s -> %s, %s -> %s, %s -> %s; pause via '%s' (true=pause). "
            "Pausing only stops forwarding to /sentry_scan — no real node is touched."
            % (self.odom_in, self.odom_out, self.velocity_in, self.velocity_out,
               self.cloud_in, self.cloud_out, self.pause_topic))
        self.publish_state("ready")

    # ------------------------------------------------------------------ 控制
    def publish_state(self, marker: str) -> None:
        msg = Bool()
        msg.data = self.paused
        self.paused_pub.publish(msg)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = marker
        self.marker_pub.publish(header)

    def set_paused(self, paused: bool, reason: str) -> None:
        if paused == self.paused:
            return
        self.paused = paused
        self.get_logger().warn("%s inputs (%s); forwarded=%d dropped=%d"
                               % ("PAUSED" if paused else "RESUMED", reason,
                                  self.forwarded, self.dropped))
        self.publish_state("input_paused" if paused else "input_resumed")

    def on_pause_command(self, msg: Bool) -> None:
        self.set_paused(bool(msg.data), "operator command")

    def maybe_automate(self) -> None:
        if self._automation_done or not (self.pause_after or self.resume_after):
            return
        elapsed = time.time() - self.started
        if self.pause_after and not self.paused and elapsed >= self.pause_after:
            self.set_paused(True, "pause_after=%.1fs" % self.pause_after)
        if self.resume_after and self.paused and elapsed >= self.resume_after:
            self.set_paused(False, "resume_after=%.1fs" % self.resume_after)
        if self.resume_after and not self.paused and elapsed >= self.resume_after:
            self._automation_done = True

    # ------------------------------------------------------------------ 转发
    def on_odom(self, msg: Odometry) -> None:
        if self.paused:
            self.dropped += 1
            return
        self.forwarded += 1
        self.odom_pub.publish(msg)

    def on_velocity(self, msg: Odometry) -> None:
        if self.paused:
            self.dropped += 1
            return
        self.forwarded += 1
        self.velocity_pub.publish(msg)

    def on_cloud(self, msg: PointCloud2) -> None:
        if self.paused:
            self.dropped += 1
            return
        self.forwarded += 1
        self.cloud_pub.publish(msg)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--odom-in", default="/Odometry_transformed")
    parser.add_argument("--velocity-in", default="/LIVO2/imu_propagate")
    parser.add_argument("--cloud-in", default="/cloud_registered")
    parser.add_argument("--odom-out", default="test/odom")
    parser.add_argument("--velocity-out", default="test/velocity")
    parser.add_argument("--cloud-out", default="test/cloud")
    parser.add_argument("--pause-topic", default="test/pause_inputs")
    parser.add_argument("--marker-topic", default="test/fault_marker")
    parser.add_argument("--pause-after", type=float, default=0.0,
                        help="启动后自动暂停（秒，0=不自动）")
    parser.add_argument("--resume-after", type=float, default=0.0,
                        help="启动后自动恢复（秒，0=不自动）")
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = InputPauseGate(args)
    node.create_timer(0.1, node.maybe_automate)
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
