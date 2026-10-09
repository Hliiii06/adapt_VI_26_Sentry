#!/usr/bin/env python3
"""向 SCAN 影子入口单独发送目标/参考路线（绝不发给 Nav2 或底盘）。

只发布到影子任务输入：

    Mode 1 目标   -> /sentry_scan/task/goal_in   (geometry_msgs/PoseStamped)
    Mode 3 路线   -> /sentry_scan/task/path_in   (nav_msgs/Path)

安全约束（代码层强制）：
  * 话题必须以 `/sentry_scan/` 开头，否则直接拒绝——避免把目标误发到
    `/move_base_simple/goal`、`/goal_pose` 等原有 Nav2 入口；
  * 等订阅端匹配后再发（DDS 发现未完成时单次发布会被丢掉）；
  * 不发布任何速度/控制话题。

用法：

    # 静止检查：在 odom 系下发一个 2 m 前的目标（默认只到影子）
    python3 scripts/onsite_send_goal.py --frame odom --x 2.0 --y 0.0

    # 若 RViz 目标在 map 系，显式声明 frame，由 task_adapter 转换
    python3 scripts/onsite_send_goal.py --frame map --x 3.0 --y 1.0

    # Mode 3 参考路线（地面 z；body_height 由 SCAN 加一次）
    python3 scripts/onsite_send_goal.py --path "0,0,0.0;2,0,0.0;2,2,0.0" --frame odom
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

GOAL_TOPIC = "/sentry_scan/task/goal_in"
PATH_TOPIC = "/sentry_scan/task/path_in"
ALLOWED_PREFIX = "/sentry_scan/"


def _quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frame", default="odom",
                        help="消息所在的坐标系（由 task_adapter 转换到规划系）")
    parser.add_argument("--x", type=float, default=2.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--path", default="",
                        help="Mode 3 路线：'x,y,z;x,y,z;...'（z 为地面高度，可为 0）")
    parser.add_argument("--topic", default="", help="覆盖输出话题（必须仍在 /sentry_scan/ 下）")
    parser.add_argument("--wait", type=float, default=10.0,
                        help="等待订阅端匹配的最长时间（秒）")
    args, ros_args = parser.parse_known_args(argv)

    topic = args.topic or (PATH_TOPIC if args.path else GOAL_TOPIC)
    if not topic.startswith(ALLOWED_PREFIX):
        print("拒绝：话题 '%s' 不在 %s 下；本脚本只允许操作影子任务入口。" % (topic, ALLOWED_PREFIX))
        return 2

    rclpy.init(args=ros_args)
    node = Node("onsite_send_goal")
    if args.path:
        publisher = node.create_publisher(Path, topic, 1)
    else:
        publisher = node.create_publisher(PoseStamped, topic, 1)

    deadline = time.time() + args.wait
    while time.time() < deadline and publisher.get_subscription_count() == 0 and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        print("失败：%.1fs 内没有订阅者出现在 %s；影子入口是否已启动？" % (args.wait, topic))
        node.destroy_node()
        rclpy.shutdown()
        return 2

    stamp = node.get_clock().now().to_msg()
    if args.path:
        msg = Path()
        msg.header.stamp = stamp
        msg.header.frame_id = args.frame
        for chunk in args.path.split(";"):
            values = [v for v in chunk.replace(" ", "").split(",") if v != ""]
            if len(values) != 3:
                print("拒绝：路线点 '%s' 不是 x,y,z" % chunk)
                node.destroy_node()
                rclpy.shutdown()
                return 2
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
                float(values[0]), float(values[1]), float(values[2]))
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        publisher.publish(msg)
        print("已发送参考路线到 %s：frame=%s，%d 个点（地面 z；body_height 由 SCAN 加一次）"
              % (topic, args.frame, len(msg.poses)))
    else:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = args.frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = args.x, args.y, args.z
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = _quat_from_yaw(args.yaw)
        publisher.publish(msg)
        print("已发送 Mode 1 目标到 %s：frame=%s，位置 (%.2f, %.2f, %.2f)，yaw=%.2f rad"
              % (topic, args.frame, args.x, args.y, args.z, args.yaw))
    print("注意：这是影子任务输入；速度只会出现在 /sentry_scan/cmd_vel_shadow。")

    # 给 DDS 一点时间把消息发出去
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
