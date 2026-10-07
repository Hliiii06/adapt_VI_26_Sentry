#!/usr/bin/env python3
"""shadow_guard 门控逻辑的确定性单测（不需要 ROS 图，失败返回非零）。

覆盖复审指出的语义漏洞：**地图过期必须始终停车**，且撤销/锁止不是可关闭的可选项。
直接构造节点状态后调用 `decide()`，不 spin、不启动任何 ROS 图。

用例：
  1. 地图过期 + 其他输入健康 + 候选 0.4 m/s  → 必须为零（不是 pass）；
  2. 地图新鲜 + 其他输入健康 + 候选 0.4 m/s  → 放行 0.4；
  3. 已锁止后心跳恢复、但**没有新任务** → 仍然为零；
  4. 收到**编号更大的新任务**授权且地图已恢复 → 解锁并放行；
  5. 启动阶段（从未收到心跳）→ 为零但不锁止；
  6. 不存在的参数 `revoke_on_map_stale` 不得再被声明（防止有人重新引入这个双语义开关）。
"""

import math
import sys
import unittest

import rclpy
from scan_planner_msgs.msg import TaskAuthorization

from sentry_scan_adapter.shadow_guard import ShadowGuard


class GuardLogicTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = ShadowGuard()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        node = self.node
        now = node._now_s()
        node.graph_violation = ""
        node.map_latched = False
        node.latched_task_id = 0
        node.last_task_id = 0
        node.map_ever_fresh = True
        node.task_active = True
        node.map_recv = now
        node.map_stamp = now
        node.health = True
        node.health_recv = now
        node.body_recv = now
        node.candidate_recv = now
        node.candidate.linear.x = 0.4
        node.candidate.linear.y = 0.0
        node.candidate.linear.z = 0.0
        node.candidate.angular.x = 0.0
        node.candidate.angular.y = 0.0
        node.candidate.angular.z = 0.0
        node._now = now
        node._publish_reset = lambda reason, when: setattr(node, "_reset_count",
                                                            getattr(node, "_reset_count", 0) + 1)

    def _stale_map(self):
        self.node.map_recv = self.node._now - 5.0
        self.node.map_stamp = self.node._now - 5.0

    def test_stale_map_always_stops_even_with_healthy_inputs(self):
        self._stale_map()
        twist, reason = self.node.decide()
        self.assertEqual(reason, "map_latched")
        self.assertEqual((twist.linear.x, twist.linear.y), (0.0, 0.0),
                         "a stale map must zero the output, not pass 0.4 m/s")
        self.assertTrue(self.node.map_latched)

    def test_fresh_map_passes_candidate(self):
        twist, reason = self.node.decide()
        self.assertTrue(reason.startswith("pass"))
        self.assertAlmostEqual(twist.linear.x, 0.4, places=6)

    def test_latch_survives_map_recovery_without_new_task(self):
        self._stale_map()
        self.node.decide()
        self.assertTrue(self.node.map_latched)
        # 心跳恢复：没有新任务，仍然必须为零。
        self.node.map_recv = self.node._now
        self.node.map_stamp = self.node._now
        twist, reason = self.node.decide()
        self.assertEqual(reason, "map_latched")
        self.assertEqual(twist.linear.x, 0.0)

    def test_new_task_clears_latch(self):
        self._stale_map()
        self.node.decide()
        latched = self.node.latched_task_id
        msg = TaskAuthorization()
        msg.active = True
        msg.task_id = latched + 1
        self.node.map_recv = self.node._now
        self.node.map_stamp = self.node._now
        self.node.on_task_active(msg)
        self.assertFalse(self.node.map_latched, "a newer task on a recovered map must clear the latch")
        twist, reason = self.node.decide()
        self.assertTrue(reason.startswith("pass"))
        self.assertAlmostEqual(twist.linear.x, 0.4, places=6)

    def test_stale_map_without_task_does_not_latch(self):
        # 启动阶段（还没有任务）地图过期：只需输出零，不应锁止——
        # 否则第一个任务授权会被误当成"解锁"，并在日志里制造一次无意义的 latch/clear。
        self.node.task_active = False
        self._stale_map()
        twist, reason = self.node.decide()
        self.assertEqual(twist.linear.x, 0.0)
        self.assertEqual(reason, "map_update_stale")
        self.assertFalse(self.node.map_latched)

    def test_startup_without_heartbeat_stops_but_does_not_latch(self):
        self.node.map_ever_fresh = False
        self.node.map_recv = None
        self.node.map_stamp = None
        twist, reason = self.node.decide()
        self.assertEqual(twist.linear.x, 0.0)
        self.assertIn("map_update", reason)
        self.assertFalse(self.node.map_latched)

    def test_revoke_toggle_parameter_is_gone(self):
        names = {name for name, _ in self.node.get_parameters_by_prefix("").items()} \
            if hasattr(self.node, "get_parameters_by_prefix") else set()
        # 参数未声明时 get_parameters_by_prefix('') 不一定列出全部；直接用 has_parameter 断言。
        self.assertFalse(self.node.has_parameter("revoke_on_map_stale"),
                         "the dual-semantics toggle must not be re-introduced")


if __name__ == "__main__":
    result = unittest.main(exit=False, verbosity=2).result
    sys.exit(0 if result.wasSuccessful() else 2)
