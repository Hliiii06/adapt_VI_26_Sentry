#!/usr/bin/env python3
"""影子接入图与门控检查（判据脚本；失败返回非零）。

用法（同一 ROS_DOMAIN_ID 下，影子 launch 已在运行）：

    ros2 run sentry_scan_adapter_cpp check_inputs ...   # 输入侧健康
    python3 scripts/check_shadow_graph.py --duration 6 --expect-unhealthy

检查项：
  1. 影子命名空间内必需节点存在（rm_input_adapter / task_adapter / shadow_guard /
     scan_planner_node / closed_loop_controller）；
  2. **影子命名空间内**不存在禁止节点（map_pub / pcl_render_node / go2_kinematic_sim /
     open_loop_controller / go2_gait_publisher / uart_node）。实车原有 `/uart_node` 等
     外部节点是正常系统的一部分，只统计、不判失败；
  3. `/cmd_vel`、`/cmd_vel_remap`、`/sentry_scan/cmd_vel` **没有被影子命名空间内的节点发布**；
     外部发布者（例如并存的 Nav2）只统计、不判失败；
  4. `health_ok` 与 `cmd_vel_shadow` 的实际取值符合预期；
  5. 影子速度的 yaw 候选恒为 0、linear.z 恒为 0（不透传未定义分量）。

退出码：0 = 全部通过；2 = 任一判据失败。
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

REQUIRED_NODES = {"rm_input_adapter", "task_adapter", "shadow_guard", "scan_planner_node",
                   "closed_loop_controller"}
FORBIDDEN_NODES = {"map_pub", "pcl_render_node", "go2_kinematic_sim", "go2_gait_publisher",
                   "open_loop_controller", "uart_node"}
FORBIDDEN_TOPICS = ("/cmd_vel", "/cmd_vel_remap", "/sentry_scan/cmd_vel")


class ShadowGraphChecker(Node):
    def __init__(self) -> None:
        super().__init__("check_shadow_graph")
        self.health = None
        self.last_health_recv = None
        self.shadow_samples = []
        self.shadow_count = 0
        # DDS 发现是异步的：单次查询可能少看到已经存活的节点/发布者，整个观察窗内取并集。
        # 注意 `get_node_names()` 在本环境返回**不带命名空间**的名字，无法区分"影子节点"与
        # "实车原有节点"（例如 /uart_node），因此这里保存 (namespace, name) 对。
        self.seen_nodes = set()
        self.seen_publishers = {}
        latch = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/sentry_scan/health_ok", self.on_health, latch)
        self.create_subscription(Twist, "/sentry_scan/cmd_vel_shadow", self.on_shadow, 10)

    def on_health(self, msg: Bool) -> None:
        self.health = bool(msg.data)
        self.last_health_recv = self.get_clock().now().nanoseconds * 1e-9

    def on_shadow(self, msg: Twist) -> None:
        self.shadow_count += 1
        self.shadow_samples.append(
            (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z))
        if len(self.shadow_samples) > 4000:
            del self.shadow_samples[:2000]


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--shadow-namespace", default="/sentry_scan")
    parser.add_argument("--expect-healthy", action="store_true")
    parser.add_argument("--expect-unhealthy", action="store_true")
    parser.add_argument("--expect-motion", action="store_true",
                        help="要求影子速度出现过 >0.05 m/s 的平移分量（候选速度确实流通）")
    parser.add_argument("--expect-zero", action="store_true",
                        help="要求影子速度全程为 0")
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = ShadowGraphChecker()
    end = time.time() + args.duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        node.seen_nodes |= set(node.get_node_names_and_namespaces())
        for topic in FORBIDDEN_TOPICS:
            try:
                for info in node.get_publishers_info_by_topic(topic):
                    namespace = (info.node_namespace or "/").rstrip("/") or "/"
                    node.seen_publishers[(topic, namespace, info.node_name)] = True
            except Exception:
                pass

    failures = []
    own_ns = args.shadow_namespace.rstrip("/") or "/"

    shadow_nodes = {}
    external_forbidden = []
    # rclpy 的 get_node_names_and_namespaces() 返回 (name, namespace)
    for name, namespace in sorted(node.seen_nodes):
        full = "%s%s" % (((namespace or "/").rstrip("/") or "") + "/", name)
        if (namespace or "/").rstrip("/") == own_ns:
            shadow_nodes[name] = full
        elif name in FORBIDDEN_NODES:
            external_forbidden.append(full)

    missing = REQUIRED_NODES - set(shadow_nodes)
    if missing:
        failures.append("missing required nodes in %s: %s" % (own_ns, sorted(missing)))
    present_forbidden = sorted(set(shadow_nodes) & FORBIDDEN_NODES)
    if present_forbidden:
        failures.append("forbidden nodes running inside %s: %s" % (own_ns, present_forbidden))
    if external_forbidden:
        print("external nodes outside %s (normal vehicle stack, not a failure): %s"
              % (own_ns, sorted(external_forbidden)))

    external = 0
    for (topic, namespace, node_name) in sorted(node.seen_publishers):
        if namespace == own_ns:
            failures.append("%s is published by shadow node %s%s" % (topic, namespace, node_name))
        else:
            external += 1
    if external:
        print("external publisher(s) on %s: %d (coexistence, not a failure)"
              % ("/".join(FORBIDDEN_TOPICS), external))

    if node.health is None:
        failures.append("health_ok was never received (adapter/silent latch broken)")
    else:
        if args.expect_healthy and node.health is not True:
            failures.append("expected healthy inputs, health_ok=%s" % node.health)
        if args.expect_unhealthy and node.health is not False:
            failures.append("expected unhealthy inputs, health_ok=%s" % node.health)

    if node.shadow_count == 0:
        failures.append("cmd_vel_shadow published no samples")
    else:
        max_xy = max(math.hypot(s[0], s[1]) for s in node.shadow_samples)
        max_z = max(abs(s[2]) for s in node.shadow_samples)
        max_wz = max(abs(s[3]) for s in node.shadow_samples)
        if max_z > 1e-9:
            failures.append("shadow linear.z is non-zero (max %.4f): unsupported axis" % max_z)
        if max_wz > 1e-9:
            failures.append("shadow angular.z is non-zero (max %.4f): yaw must be isolated" % max_wz)
        if args.expect_motion and max_xy <= 0.05:
            failures.append("expected motion: max |v_xy| = %.4f m/s" % max_xy)
        if args.expect_zero and max_xy > 1e-9:
            failures.append("expected zero shadow output: max |v_xy| = %.4f m/s" % max_xy)
        print("shadow samples=%d  max|v_xy|=%.4f  max|wz|=%.4f  max|vz|=%.4f"
              % (node.shadow_count, max_xy, max_wz, max_z))

    print("shadow nodes in %s: %s" % (own_ns, sorted(shadow_nodes)))
    print("health_ok: %s (samples=%d)" % (node.health, node.shadow_count))
    node.destroy_node()
    rclpy.shutdown()

    if failures:
        print("\nFAIL:")
        for item in failures:
            print("  - %s" % item)
        sys.exit(2)
    print("\nPASS: shadow graph is isolated and the gating expectation holds")


if __name__ == "__main__":
    main()
