#!/usr/bin/env python3
"""适配层坐标/速度数学的确定性单测（不需要 ROS 图，失败返回非零）。

覆盖 B3 要求的"frame/杆臂/速度样例"：已知平移 + yaw=90° 的数值正确、
缺 TF 时拒绝、未补偿杆臂时降级。运行前需 source ROS 与工作区 install。
"""

import math
import sys
import unittest

import rclpy
from geometry_msgs.msg import TransformStamped

from sentry_scan_adapter.rm_input_adapter import RmInputAdapter, _pose_from_transform, _rotate


def _tf(translation=(0.0, 0.0, 0.0), yaw=0.0):
    msg = TransformStamped()
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = translation
    msg.transform.rotation.z = math.sin(yaw * 0.5)
    msg.transform.rotation.w = math.cos(yaw * 0.5)
    return msg


class AdapterMathTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = RmInputAdapter()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node.imu_to_body_offset = None
        self.node.require_center_velocity = False
        self.node.require_angular_velocity = False
        self.node.velocity_frame_alias = ""
        self.node._velocity = None
        self.node.channels["tf"].error = ""
        self._lookup_result = _tf()
        self._lookup_calls = []

        def fake_lookup(target, source, stamp_s, allow_future=False, required=True):
            self._lookup_calls.append((target, source, stamp_s, required))
            return self._lookup_result

        self.node._lookup = fake_lookup
        now = self.node._now_s()
        self.now = now

    def _set_velocity(self, linear, angular, frame="odom"):
        self.node._velocity = (self.now, self.now, frame, linear, angular)

    def test_rotate_yaw_90(self):
        q = _tf(yaw=math.pi / 2).transform.rotation
        x, y, z = _rotate(q, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 1.0, places=6)
        self.assertAlmostEqual(z, 0.0, places=6)

    def test_lever_arm_compensation(self):
        # v_center = v_point + omega x r；yaw=0 时 body 系偏移直接就是规划系向量。
        self.node.imu_to_body_offset = [1.0, 0.0, 0.0]
        self._lookup_result = _tf(yaw=0.0)
        self._set_velocity((1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        linear, angular, reason = self.node.build_velocity(self.now)
        self.assertAlmostEqual(linear[0], 1.0, places=6)
        self.assertAlmostEqual(linear[1], 1.0, places=6)
        self.assertAlmostEqual(linear[2], 0.0, places=6)
        self.assertEqual(reason, "center_velocity_compensated")

    def test_lever_arm_with_body_yaw_90(self):
        # body yaw=90°：body 系 (1,0,0) -> 规划系 (0,1,0)；omega x r = (-1,0,0)。
        self.node.imu_to_body_offset = [1.0, 0.0, 0.0]
        self._lookup_result = _tf(yaw=math.pi / 2)
        self._set_velocity((1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        linear, _, reason = self.node.build_velocity(self.now)
        self.assertAlmostEqual(linear[0], 0.0, places=6)
        self.assertAlmostEqual(linear[1], 0.0, places=6)
        self.assertEqual(reason, "center_velocity_compensated")

    def test_require_center_velocity_without_offset_degrades(self):
        self.node.require_center_velocity = True
        self._set_velocity((1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        linear, _, reason = self.node.build_velocity(self.now)
        self.assertEqual(linear, (0.0, 0.0, 0.0))
        self.assertEqual(reason, "lever_arm_uncompensated")

    def test_unresolved_velocity_frame_degrades_without_health_error(self):
        self._lookup_result = None
        self._set_velocity((1.0, 0.0, 0.0), (0.0, 0.0, 0.0), frame="world")
        linear, _, reason = self.node.build_velocity(self.now)
        self.assertEqual(linear, (0.0, 0.0, 0.0))
        self.assertTrue(reason.startswith("velocity_frame_unresolved"))
        self.assertEqual(self.node.channels["tf"].error, "",
                         "velocity frame resolution must not mark required TF unhealthy")
        self.assertTrue(any(call[3] is False for call in self._lookup_calls),
                        "velocity frame lookup must be a non-required lookup")

    def test_velocity_frame_alias_is_declared_equivalence(self):
        self.node.velocity_frame_alias = "world"
        self._set_velocity((0.5, 0.0, 0.0), (0.0, 0.0, 0.0), frame="world")
        linear, _, reason = self.node.build_velocity(self.now)
        self.assertEqual(linear[0], 0.5)
        self.assertEqual(reason, "lever_arm_uncompensated")
        self.assertEqual(self._lookup_calls, [], "alias must skip the TF lookup entirely")

    def test_stale_source_stamp_is_degraded(self):
        self._set_velocity((1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.node._velocity = (self.now, self.now - 5.0, "odom", (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        linear, _, reason = self.node.build_velocity(self.now)
        self.assertEqual(linear, (0.0, 0.0, 0.0))
        self.assertEqual(reason, "velocity_stale_stamp")

    def test_accept_stamp_rejects_old_and_backwards(self):
        channel = self.node.channels["odom"]
        channel.last_stamp = None
        channel.error = ""
        self.assertTrue(self.node._accept_stamp(channel, self.now - 0.1, self.now))
        # 来源年龄单独验证：时间戳仍在前进，但已经超过 max_source_age。
        channel.last_stamp = self.now - 2.0
        channel.error = ""
        self.assertFalse(self.node._accept_stamp(channel, self.now - 1.0, self.now))
        self.assertIn("old", channel.error)
        # 时间倒退：比上一帧旧即拒绝（与年龄无关）。
        channel.last_stamp = self.now - 0.1
        channel.error = ""
        self.assertFalse(self.node._accept_stamp(channel, self.now - 0.2, self.now),
                         "timestamp going backwards must be rejected")
        self.assertIn("backwards", channel.error)

    def test_pose_from_transform(self):
        pose = _pose_from_transform(_tf(translation=(1.0, 2.0, 3.0), yaw=math.pi / 2).transform)
        self.assertAlmostEqual(pose.position.x, 1.0)
        self.assertAlmostEqual(pose.position.y, 2.0)
        self.assertAlmostEqual(pose.position.z, 3.0)
        self.assertAlmostEqual(pose.orientation.w, math.cos(math.pi / 4), places=6)


if __name__ == "__main__":
    result = unittest.main(exit=False, verbosity=2).result
    sys.exit(0 if result.wasSuccessful() else 2)
