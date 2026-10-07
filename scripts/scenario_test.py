#!/usr/bin/env python3
"""S3 场景记录器。

职责仅限于"记录 + 触发"：订阅 body_pose / cmd_vel，按场景发送 RViz 目标或取消指令，
把轨迹与命令写盘，并写出元数据（t0 的绝对时刻、取消时刻）。
停车判据由 scripts/check_stop.py 独立判定，避免记录器自己给自己打分。

产出：
    <out>.csv          轨迹（t, x, y, z, yaw, vx_body, vy_body, wz）
    <out>_cmdvel.csv   命令（t, vx, vy, wz）
    <out>_planned.csv  规划器发布的 B 样条采样（traj_id, start_time, t, x, y, z）
    <out>_meta.txt     key=value：t0_epoch、cancel_t（若有）
    <out>.txt          简要统计
"""

import argparse
import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
from scan_planner_msgs.msg import Bspline, TaskAuthorization
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

NS = "/sentry_sim"


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
        self.t0_epoch = None
        self.cancel_t = None

        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, f"{NS}/body_pose", self.on_odom, qos)
        self.create_subscription(Twist, f"{NS}/cmd_vel", self.on_cmd, qos)
        # 记录**规划器发布**的轨迹：用于比较"规划检查的高度"与"实际执行的高度"。
        self.planned = []
        self.create_subscription(Bspline, f"{NS}/planning/bspline", self.on_bspline, qos)
        self.goal_pub = self.create_publisher(PoseStamped, f"{NS}/move_base_simple/goal", 1)
        self.reset_pub = self.create_publisher(Bool, f"{NS}/planning/reset", 10)
        # 必须与执行端订阅(QoS reliable + transient_local)兼容，否则注入的消息根本收不到，
        # 测试会"通过"但什么都没验证。
        from rclpy.qos import DurabilityPolicy, HistoryPolicy
        auth_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              history=HistoryPolicy.KEEP_LAST)
        self.auth_pub = self.create_publisher(
            TaskAuthorization, f"{NS}/planning/task_active", auth_qos)
        self.bspline_pub = self.create_publisher(Bspline, f"{NS}/planning/bspline", 10)

    def rel(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
            self.t0_epoch = now
        return now - self.t0

    def on_bspline(self, msg: Bspline):
        """把收到的 B 样条按 de Boor 求值采样，存下 (traj_id, start_time, t, x, y, z)。"""
        knots = list(msg.knots)
        pts = [(p.x, p.y, p.z) for p in msg.pos_pts]
        p = msg.order - 1
        n = len(pts) - 1
        if n < p or len(knots) < n + p + 2:
            return
        t0 = msg.start_time.sec + msg.start_time.nanosec * 1e-9
        lo, hi = knots[p], knots[n + 1]
        steps = 40
        for i in range(steps + 1):
            tt = lo + (hi - lo) * i / steps
            k = p
            while k < n and knots[k + 1] <= tt:
                k += 1
            k = min(max(k, p), n)
            d = [list(pts[j + k - p]) for j in range(p + 1)]
            for r in range(1, p + 1):
                for j in range(p, r - 1, -1):
                    denom = knots[j + 1 + k - r] - knots[j + k - p]
                    alpha = 0.0 if abs(denom) < 1e-12 else (tt - knots[j + k - p]) / denom
                    d[j] = [d[j - 1][c] * (1 - alpha) + d[j][c] * alpha for c in range(3)]
            x, y, z = d[p]
            self.planned.append((msg.traj_id, t0, tt, x, y, z))

    def on_cmd(self, msg: Twist):
        t = self.rel()
        self.cmd_peak[0] = max(self.cmd_peak[0], abs(msg.linear.x))
        self.cmd_peak[1] = max(self.cmd_peak[1], abs(msg.linear.y))
        self.cmd_peak[2] = max(self.cmd_peak[2], abs(msg.angular.z))
        self.cmd_samples.append((t, msg.linear.x, msg.linear.y, msg.angular.z))

    def send_stale_authorization(self):
        msg = TaskAuthorization()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.active = True
        msg.task_id = 1          # 远小于规划端已撤销到的编号
        self.auth_pub.publish(msg)

    def send_late_trajectory(self):
        """构造一条 start_time = 现在（晚于取消时刻）的直线 B 样条并发布。"""
        if not self.samples:
            return
        _, x0, y0, z0, _, _, _, _ = self.samples[-1]
        msg = Bspline()
        msg.order = 4
        msg.traj_id = 999001
        msg.start_time = self.get_clock().now().to_msg()
        msg.knots = [float(i) for i in range(12)]
        pts = []
        for i in range(8):
            p = Point()
            p.x = x0
            p.y = y0 + 0.30 * i      # 以 0.6 m/s 向 +y 走
            p.z = z0
            pts.append(p)
        msg.pos_pts = pts
        msg.yaw_pts = [0.0, 0.0]
        msg.yaw_dt = 1.0
        self.bspline_pub.publish(msg)

    def on_odom(self, msg: Odometry):
        t = self.rel()
        p = msg.pose.pose.position
        self.samples.append((t, p.x, p.y, p.z, yaw_from_quaternion(msg.pose.pose.orientation),
                             msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                             msg.twist.twist.angular.z))

    def send_goal(self, x, y, yaw=0.0):
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
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
    parser.add_argument("--odom-only", action="store_true",
                        help="Explicit trajectory-preview recording: require odom, not cmd_vel; NOT a tracking test")
    parser.add_argument("--goal-x", type=float, default=-6.0)
    parser.add_argument("--goal-y", type=float, default=7.5)
    parser.add_argument("--out", required=True)
    parser.add_argument("--send-goal", default="true",
                        help="true=发送 RViz 目标；false=由 FSM 自行起步（Mode 2/3）")
    parser.add_argument("--cancel-after", type=float, default=-1.0,
                        help=">=0 时：发送目标后等待该秒数再发 planning/reset 取消")
    parser.add_argument("--inject-stale", action="store_true",
                        help="取消后注入**延迟到达**的旧授权与一条 start_time 很新的轨迹，"
                             "用于验证执行端本地锁止后不会被旧任务重新放行")
    parser.add_argument("--post-cancel", type=float, default=8.0,
                        help="取消后的观察时长（必须真正等待，否则会漏检取消后再次运动）")
    args = parser.parse_args()

    rclpy.init()
    node = Recorder(args)

    print("[recorder] 等待 body_pose ..." if args.odom_only
          else "[recorder] 等待 cmd_vel 与 body_pose ...")
    deadline = time.time() + 60.0
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.samples and (args.odom_only or node.cmd_samples):
            break
    if not node.samples or (not args.odom_only and not node.cmd_samples):
        print("[recorder] 超时：未收到 body_pose / cmd_vel")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print("[recorder] 已上线，cmd_vel 样本=%d" % len(node.cmd_samples))

    deadline = time.time() + (0.0 if args.odom_only else 20.0)
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.goal_pub.get_subscription_count() > 0:
            break
    print("[recorder] goal 订阅者数量 = %d" % node.goal_pub.get_subscription_count())

    if args.send_goal.lower() in ("1", "true", "yes", "on"):
        print("[recorder] 先静止观察 3 s，再发送目标")
        spin_for(node, 3.0)
        for _ in range(3):
            node.send_goal(args.goal_x, args.goal_y)
            spin_for(node, 0.4)
        print("[recorder] 已发送目标 (%.2f, %.2f)" % (args.goal_x, args.goal_y))

    if args.cancel_after >= 0.0:
        # 目标尚未到达时中途取消，然后完整等待 post_cancel 秒
        # （旧版本把观察时长算出来却没有真正等待，导致"取消后又动起来"没被发现）。
        spin_for(node, args.cancel_after)
        node.cancel_t = node.rel()
        for _ in range(5):
            node.send_reset()
            spin_for(node, 0.2)
        print("[recorder] 已发送 planning/reset，取消时刻 t=%.3fs" % node.cancel_t)
        if args.inject_stale:
            # 模拟"取消时规划端还在算"：先注入一条编号很旧、但此时才到达的授权，
            # 再注入一条 start_time 很新（晚于取消时刻）的轨迹。
            # 正确的实现必须把两者都拒掉，cmd_vel 保持为零。
            node.send_stale_authorization()
            spin_for(node, 0.3)
            node.send_late_trajectory()
            spin_for(node, 0.3)
            print("[recorder] 已注入延迟到达的旧授权 + 晚于取消时刻的轨迹，t=%.3fs" % node.rel())
        spin_for(node, args.post_cancel)
    else:
        spin_for(node, args.duration)
        spin_for(node, 2.0)

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out + ".csv", "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["t", "x", "y", "z", "yaw", "vx_body", "vy_body", "wz"])
        w.writerows(node.samples)
    with open(args.out + "_planned.csv", "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["traj_id", "start_time", "t", "x", "y", "z"])
        w.writerows(node.planned)
    with open(args.out + "_cmdvel.csv", "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["t", "vx", "vy", "wz"])
        w.writerows(node.cmd_samples)
    with open(args.out + "_meta.txt", "w") as f:
        f.write("scenario=%s\n" % args.scenario)
        f.write("odom_only=%s\n" % args.odom_only)
        f.write("t0_epoch=%.6f\n" % (node.t0_epoch or 0.0))
        if node.cancel_t is not None:
            f.write("cancel_t=%.6f\n" % node.cancel_t)

    s = node.samples
    report = ["场景: %s" % args.scenario, "轨迹样本数: %d" % len(s)]
    if len(s) > 2:
        xs = [r[1] for r in s]
        ys = [r[2] for r in s]
        yaws = [r[4] for r in s]
        yaw0 = yaws[0]
        path = sum(math.dist(s[i][1:3], s[i + 1][1:3]) for i in range(len(s) - 1))
        report += [
            "起点: (%.3f, %.3f) yaw=%.4f" % (xs[0], ys[0], yaws[0]),
            "终点: (%.3f, %.3f) yaw=%.4f" % (xs[-1], ys[-1], yaws[-1]),
            "位移: dx=%+.3f m, dy=%+.3f m" % (xs[-1] - xs[0], ys[-1] - ys[0]),
            "路径长度: %.3f m" % path,
            "朝向最大偏离初始值: %.2f deg" % math.degrees(
                max(abs(normalize(a - yaw0)) for a in yaws)),
            "高度 z 范围: [%.4f, %.4f]" % (min(r[3] for r in s), max(r[3] for r in s)),
            "命令峰值 |vx|=%.3f |vy|=%.3f |wz|=%.3f" % tuple(node.cmd_peak),
        ]
        if args.send_goal.lower() in ("1", "true", "yes", "on") and args.cancel_after < 0.0:
            report.append("终点到目标距离: %.3f m"
                          % math.dist((xs[-1], ys[-1]), (args.goal_x, args.goal_y)))
    if node.cancel_t is not None:
        report.append("取消时刻: t=%.3fs" % node.cancel_t)

    text = "\n".join(report)
    with open(args.out + ".txt", "w") as f:
        f.write(text + "\n")
    print("\n===== 场景记录 =====\n" + text + "\n====================")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
