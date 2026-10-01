#!/usr/bin/env python3
"""S3 场景记录与指标计算。

在仿真运行期间订阅 body_pose / cmd_vel，按需发送 RViz 目标或取消指令，
结束后输出 CSV 与关键指标（位移方向、朝向保持、跟踪误差、速度峰值、停车时间）。

用法（由 scripts/scenario.sh 调用，也可单独使用）：
    scenario_test.py --scenario mode1_lateral --duration 25 --out log/scenarios/mode1
"""

import argparse
import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

NS = "/sentry_sim"

# 需要外部 RViz 目标驱动的场景（其余由 FSM 自行起步）
GOAL_SCENARIOS = {
    "mode1_lateral", "mode1_align", "mode1_far",
    "collision_narrow", "collision_block", "tight_pass", "cancel",
}




def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class Recorder(Node):
    def __init__(self, args):
        super().__init__("sentry_scenario_recorder")
        self.args = args
        self.samples = []
        self.cmd_samples = []
        self.cmd_peak = [0.0, 0.0, 0.0]
        self.t0 = None
        self.cancel_time = None
        self.stop_after_cancel = None
        self.last_moving_time = None

        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, f"{NS}/body_pose", self.on_odom, qos)
        self.create_subscription(Twist, f"{NS}/cmd_vel", self.on_cmd, qos)
        self.goal_pub = self.create_publisher(PoseStamped, f"{NS}/move_base_simple/goal", 1)
        self.reset_pub = self.create_publisher(Bool, f"{NS}/planning/reset", 10)

    def on_cmd(self, msg: Twist):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        self.cmd_peak[0] = max(self.cmd_peak[0], abs(msg.linear.x))
        self.cmd_peak[1] = max(self.cmd_peak[1], abs(msg.linear.y))
        self.cmd_peak[2] = max(self.cmd_peak[2], abs(msg.angular.z))
        self.cmd_samples.append((now - self.t0, msg.linear.x, msg.linear.y,
                                 msg.angular.z))

    def on_odom(self, msg: Odometry):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.samples.append((now - self.t0, p.x, p.y, p.z, yaw,
                             msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                             msg.twist.twist.angular.z))
        if speed > 0.02:
            self.last_moving_time = now
        if self.cancel_time is not None and self.stop_after_cancel is None:
            if speed <= 0.02 and (now - self.cancel_time) > 0.05:
                self.stop_after_cancel = now - self.cancel_time

    def send_goal(self, x, y, yaw=0.0):
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(0.5 * yaw)
        msg.pose.orientation.w = math.cos(0.5 * yaw)
        self.goal_pub.publish(msg)

    def send_reset(self):
        msg = Bool()
        msg.data = True
        self.reset_pub.publish(msg)


def spin_for(node, seconds):
    end = time.time() + seconds
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--duration", type=float, default=25.0)
    parser.add_argument("--goal-x", type=float, default=-6.0)
    parser.add_argument("--goal-y", type=float, default=7.5)
    parser.add_argument("--out", required=True)
    parser.add_argument("--cancel-after", type=float, default=4.0,
                        help="cancel 场景中发出取消指令前的等待秒数")
    args = parser.parse_args()

    rclpy.init()
    node = Recorder(args)

    # Mode 2/3 在收到第一帧 odom 后立即起步，必须先等 body_pose 上线再开始计时，
    # 否则会在等待节点就绪期间漏掉整段运动。
    print("[recorder] 等待 body_pose ...")
    ready_deadline = time.time() + 60.0
    while time.time() < ready_deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if len(node.samples) > 0:
            break
    if not node.samples:
        print("[recorder] 超时：未收到 body_pose")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print("[recorder] body_pose 已上线，样本数=%d" % len(node.samples))

    # 等待订阅者连接后再发目标，避免消息丢失
    deadline = time.time() + 20.0
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.goal_pub.get_subscription_count() > 0:
            break
    subs = node.goal_pub.get_subscription_count()
    print(f"[recorder] goal 订阅者数量 = {subs}")

    if args.scenario in GOAL_SCENARIOS:
        print("[recorder] 先静止观察 3 s，再发送目标")
        spin_for(node, 3.0)
    start_idx = len(node.samples) - 1

    if args.scenario in GOAL_SCENARIOS:
        for _ in range(3):
            node.send_goal(args.goal_x, args.goal_y)
            spin_for(node, 0.4)
        print(f"[recorder] 已发送目标 ({args.goal_x}, {args.goal_y})")
    elif args.scenario == "cancel":
        pass
    # mode2 / mode3 由 FSM 自行起步，无需外部目标

    remaining = args.duration
    if args.scenario == "cancel":
        # 在目标尚未到达时中途取消（此前的等待时间足以跑完全程）。
        spin_for(node, args.cancel_after)
        node.cancel_time = node.get_clock().now().nanoseconds * 1e-9
        for _ in range(5):
            node.send_reset()
            spin_for(node, 0.2)
        print("[recorder] 已发送 planning/reset 取消指令")
        remaining = 8.0
    else:
        spin_for(node, args.duration)

    # 结束前再观察一小段
    spin_for(node, 2.0)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out + ".csv", "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["t", "x", "y", "z", "yaw", "vx_body", "vy_body", "wz"])
        w.writerows(node.samples)

    with open(args.out + "_cmdvel.csv", "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["t", "vx", "vy", "wz"])
        w.writerows(node.cmd_samples)

    s = node.samples
    report = [f"场景: {args.scenario}", f"样本数: {len(s)}"]
    if len(s) > 2:
        xs = [r[1] for r in s]
        ys = [r[2] for r in s]
        yaws = [r[4] for r in s]
        yaw0 = yaws[0]
        dyaw = [abs(normalize(a - yaw0)) for a in yaws]
        path = sum(math.dist(s[i][1:3], s[i + 1][1:3]) for i in range(len(s) - 1))
        report += [
            f"起点: ({xs[0]:.3f}, {ys[0]:.3f}) yaw={yaws[0]:.4f}",
            f"终点: ({xs[-1]:.3f}, {ys[-1]:.3f}) yaw={yaws[-1]:.4f}",
            f"位移: dx={xs[-1]-xs[0]:+.3f} m, dy={ys[-1]-ys[0]:+.3f} m",
            f"路径长度: {path:.3f} m",
            f"朝向最大偏离初始值: {math.degrees(max(dyaw)):.2f} deg",
            f"高度 z 范围: [{min(r[3] for r in s):.4f}, {max(r[3] for r in s):.4f}]",
            f"命令峰值 |vx|={node.cmd_peak[0]:.3f} |vy|={node.cmd_peak[1]:.3f} "
            f"|wz|={node.cmd_peak[2]:.3f}",
        ]
        if args.scenario in GOAL_SCENARIOS and args.scenario != "cancel":
            d = math.dist((xs[-1], ys[-1]), (args.goal_x, args.goal_y))
            report.append(f"终点到目标距离: {d:.3f} m")
    if node.stop_after_cancel is not None:
        report.append(f"取消后停止时间: {node.stop_after_cancel:.3f} s")
    elif args.scenario == "cancel":
        report.append("取消后停止时间: 未在观察窗内检测到停车")

    text = "\n".join(report)
    with open(args.out + ".txt", "w") as f:
        f.write(text + "\n")
    print("\n===== 场景报告 =====\n" + text + "\n====================")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
