#!/usr/bin/env python3
"""校验任务/路线坐标适配（判据脚本，失败返回非零）。

前提：影子入口已在运行，且有非单位 `map→odom`（由 `fake_rm_inputs.py --map-odom-offset` 提供）。

检查：
  1. `map` 下的 Mode 1 目标被转换到规划系，数值等于 `T_odom<-map · p`；
  2. `map` 下的 Mode 3 参考路线逐点转换；
  3. 未知 frame 与空 frame 的输入被**拒绝**（不产生输出）。

退出码：0 = 通过；2 = 任一判据失败。
"""

import argparse
import copy
import math
import sys
import time

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

GOAL_IN = "/sentry_scan/task/goal_in"
PATH_IN = "/sentry_scan/task/path_in"
GOAL_OUT = "/sentry_scan/goal"
PATH_OUT = "/sentry_scan/initial_path"


class TaskAdapterChecker(Node):
    def __init__(self) -> None:
        super().__init__("check_task_adapter")
        self.goals = []
        self.paths = []
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        goal_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        path_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.create_publisher(PoseStamped, GOAL_IN, goal_qos)
        self.path_pub = self.create_publisher(Path, PATH_IN, path_qos)
        self.create_subscription(PoseStamped, GOAL_OUT, lambda m: self.goals.append(m), goal_qos)
        self.create_subscription(Path, PATH_OUT, lambda m: self.paths.append(m), path_qos)

    def spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _lookup(self, frame: str, stamp_s: float):
        """与 task_adapter 相同的时间策略：先按消息时刻查，允许用最新 TF 顶替（小容差）。"""
        try:
            return self.tf_buffer.lookup_transform(
                "odom", frame, Time(seconds=stamp_s), timeout=Duration(seconds=0.0))
        except Exception:
            latest = self.tf_buffer.lookup_transform(
                "odom", frame, Time(), timeout=Duration(seconds=0.0))
            latest_s = (float(latest.header.stamp.sec)
                        + float(latest.header.stamp.nanosec) * 1e-9)
            if 0.0 <= stamp_s - latest_s <= 0.05:
                return latest
            raise

    def expected(self, frame: str, x: float, y: float, z: float, stamp):
        transform = self._lookup(frame, float(stamp.sec) + float(stamp.nanosec) * 1e-9)
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.header.stamp = stamp
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        msg.pose.orientation.w = 1.0
        return do_transform_pose_stamped(msg, transform).pose.position


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tolerance", type=float, default=2e-3)
    parser.add_argument("--wait", type=float, default=1.5)
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = TaskAdapterChecker()
    failures = []
    node.spin_for(2.0)  # 等 TF/订阅建立

    # ---- 1) Mode 1 目标：map -> odom ----
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = 2.0, 1.0, 0.1
    goal.pose.orientation.w = 1.0
    try:
        want = node.expected("map", 2.0, 1.0, 0.1, goal.header.stamp)
    except Exception as exc:
        print("FAIL: cannot compute expected odom position: %s" % exc)
        node.destroy_node(); rclpy.shutdown(); sys.exit(2)
    before = len(node.goals)
    node.goal_pub.publish(goal)
    node.spin_for(args.wait)
    if len(node.goals) <= before:
        failures.append("goal in 'map' was not republished (transform layer missing?)")
    else:
        got = node.goals[-1]
        if got.header.frame_id != "odom":
            failures.append("goal output frame is '%s', expected 'odom'" % got.header.frame_id)
        delta = math.dist([got.pose.position.x, got.pose.position.y, got.pose.position.z],
                          [want.x, want.y, want.z])
        if delta > args.tolerance:
            failures.append("goal transform wrong: got (%.3f, %.3f, %.3f) want (%.3f, %.3f, %.3f)"
                            % (got.pose.position.x, got.pose.position.y, got.pose.position.z,
                               want.x, want.y, want.z))
        else:
            print("goal map(2.000, 1.000, 0.100) -> odom(%.3f, %.3f, %.3f) OK"
                  % (got.pose.position.x, got.pose.position.y, got.pose.position.z))

    # ---- 2) Mode 3 参考路线：map -> odom ----
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = node.get_clock().now().to_msg()
    for x, y in ((2.0, 1.0), (3.0, 1.0)):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, 0.0
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    try:
        want_pts = [node.expected("map", p.pose.position.x, p.pose.position.y, 0.0,
                                  path.header.stamp) for p in path.poses]
    except Exception as exc:
        print("FAIL: cannot compute expected path: %s" % exc)
        node.destroy_node(); rclpy.shutdown(); sys.exit(2)
    before_paths = len(node.paths)
    node.path_pub.publish(path)
    node.spin_for(args.wait)
    if len(node.paths) <= before_paths:
        failures.append("reference path in 'map' was not republished")
    else:
        got_path = node.paths[-1]
        if got_path.header.frame_id != "odom":
            failures.append("path output frame is '%s', expected 'odom'" % got_path.header.frame_id)
        elif len(got_path.poses) != 2:
            failures.append("path pose count changed: %d" % len(got_path.poses))
        else:
            for index, (pose, want_pt) in enumerate(zip(got_path.poses, want_pts)):
                delta = math.dist([pose.pose.position.x, pose.pose.position.y],
                                  [want_pt.x, want_pt.y])
                if delta > args.tolerance:
                    failures.append("path pose %d wrong (delta %.4f)" % (index, delta))
            if not failures:
                print("reference path map -> odom, 2 poses OK")

    # ---- 3) 未知 frame 必须被拒绝 ----
    bad = copy.deepcopy(goal)
    bad.header.frame_id = "no_such_frame"
    # 刷新时间戳，保证被拒绝的原因是 frame 而不是"消息太旧"。
    bad.header.stamp = node.get_clock().now().to_msg()
    before_bad = len(node.goals)
    node.goal_pub.publish(bad)
    node.spin_for(args.wait)
    if len(node.goals) != before_bad:
        failures.append("goal with unknown frame 'no_such_frame' was forwarded (must be rejected)")
    else:
        print("goal with unknown frame rejected OK")

    # ---- 4) 空 frame 必须被拒绝 ----
    empty = copy.deepcopy(goal)
    empty.header.frame_id = ""
    empty.header.stamp = node.get_clock().now().to_msg()
    before_empty = len(node.goals)
    node.goal_pub.publish(empty)
    node.spin_for(args.wait)
    if len(node.goals) != before_empty:
        failures.append("goal with empty frame was forwarded (must be rejected)")
    else:
        print("goal with empty frame rejected OK")

    node.destroy_node()
    rclpy.shutdown()
    if failures:
        print("\nFAIL:")
        for item in failures:
            print("  - %s" % item)
        sys.exit(2)
    print("\nPASS: task/goal and reference path are transformed into the planning frame; "
          "unknown/empty frames are rejected")


if __name__ == "__main__":
    main()
