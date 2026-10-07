#!/usr/bin/env python3
"""适配层坐标/速度数学的确定性单测（不需要 ROS 图，失败返回非零）。

覆盖 B3 要求的"frame/杆臂/速度样例"：已知平移 + yaw=90° 的数值正确、
缺 TF 时拒绝、未补偿杆臂时降级。运行前需 source ROS 与工作区 install。
"""

import math
import struct
import sys
import unittest

import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header

from sentry_scan_adapter.rm_input_adapter import (RmInputAdapter, _count_finite_points,
                                                  _pose_from_transform, _rotate)


def _xyz_fields(offsets=(0, 4, 8), datatype=PointField.FLOAT32):
    return [PointField(name=name, offset=off, datatype=datatype, count=1)
            for name, off in zip(("x", "y", "z"), offsets)]


def _raw_cloud(fields, data, width, height, point_step, row_step, bigendian=False):
    msg = PointCloud2()
    msg.header.frame_id = "camera_init"
    msg.height, msg.width = height, width
    msg.fields = fields
    msg.is_bigendian = bigendian
    msg.point_step, msg.row_step = point_step, row_step
    msg.data = data
    msg.is_dense = True
    return msg


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

    def test_future_stamp_rejected_without_polluting_history(self):
        channel = self.node.channels["odom"]
        channel.error = ""
        channel.last_stamp = self.now - 0.1
        # 1 小时之后的坏 stamp：必须拒绝，且不得写入 last_stamp。
        self.assertFalse(self.node._accept_stamp(channel, self.now + 3600.0, self.now),
                         "a stamp far in the future must be rejected")
        self.assertIn("future", channel.error)
        self.assertAlmostEqual(channel.last_stamp, self.now - 0.1, places=6,
                               msg="rejected future stamp must not become history")
        # 之后正常（略早于当前）的时间戳仍应被接受，证明历史没有被污染。
        channel.error = ""
        self.assertTrue(self.node._accept_stamp(channel, self.now - 0.05, self.now),
                        "a normal stamp after a rejected future stamp must still be accepted")
        # 容差内的小幅超前允许（同一回调里 TF/消息时间戳的常见抖动）。
        channel.error = ""
        self.assertTrue(self.node._accept_stamp(channel, self.now + 0.01, self.now))

    def test_cloud_validity_check(self):
        header = Header()
        header.frame_id = "camera_init"
        good = create_cloud_xyz32(header, [(0.0, 0.0, 0.1), (1.0, 2.0, 0.3)])
        count, why = _count_finite_points(good)
        self.assertEqual((count, why), (2, ""))

        nan = create_cloud_xyz32(header, [(float("nan"), float("nan"), float("nan"))] * 50)
        count_nan, why_nan = _count_finite_points(nan)
        self.assertEqual(count_nan, 0, "an all-NaN cloud has zero finite points")
        self.assertEqual(why_nan, "")

        partial = create_cloud_xyz32(header, [(float("nan"),) * 3, (1.0, 1.0, 1.0)] * 20)
        count_partial, _ = _count_finite_points(partial)
        self.assertEqual(count_partial, 20)

        broken = PointCloud2()
        broken.header.frame_id = "camera_init"
        broken.width, broken.height, broken.point_step, broken.row_step = 10, 1, 8, 80
        broken.fields = good.fields
        broken.data = b"\x00" * 80
        count_broken, why_broken = _count_finite_points(broken)
        self.assertEqual(count_broken, 0)
        self.assertIn("point_step", why_broken)

    def test_cloud_layout_row_padding_and_endianness(self):
        # 行填充布局：2 行 x 2 点，point_step=16（xyz + 4 字节填充），row_step=40（尾部再填充 8 字节）。
        def payload(value):
            row = b"".join(struct.pack("<fff", value, value, value) + b"\x00" * 4 for _ in range(2))
            return (row + b"\x00" * 8) * 2

        fields = _xyz_fields()
        good = _raw_cloud(fields, payload(1.5), width=2, height=2, point_step=16, row_step=40)
        count, why = _count_finite_points(good)
        self.assertEqual((count, why), (4, ""), "row padding must not create or drop points")

        nan = float("nan")
        nan_cloud = _raw_cloud(fields, payload(nan), width=2, height=2, point_step=16, row_step=40)
        count_nan, why_nan = _count_finite_points(nan_cloud)
        self.assertEqual((count_nan, why_nan), (0, ""),
                         "row-padded all-NaN cloud has zero finite points")

        # 大端：同样的有限值按 >f 打包，必须显式拒绝而不是误读成有效点。
        big = _raw_cloud(
            fields,
            (b"".join(struct.pack(">fff", 1.0, 2.0, 3.0) for _ in range(2)) + b"\x00" * 8) * 2,
            width=2, height=2, point_step=16, row_step=40, bigendian=True)
        count_big, why_big = _count_finite_points(big)
        self.assertEqual(count_big, 0)
        self.assertIn("is_bigendian", why_big)

        # 非 FLOAT32 字段（FLOAT64）同样必须拒绝。
        wide = _raw_cloud(_xyz_fields(offsets=(0, 8, 16), datatype=PointField.FLOAT64),
                          b"\x00" * (24 * 2 * 2), width=2, height=2, point_step=24, row_step=48)
        count_wide, why_wide = _count_finite_points(wide)
        self.assertEqual(count_wide, 0)
        self.assertIn("FLOAT32", why_wide)

        # 截断数据必须拒绝。
        short = _raw_cloud(fields, b"\x00" * 10, width=2, height=2, point_step=16, row_step=40)
        count_short, why_short = _count_finite_points(short)
        self.assertEqual(count_short, 0)
        self.assertTrue(why_short)

    def test_pose_from_transform(self):
        pose = _pose_from_transform(_tf(translation=(1.0, 2.0, 3.0), yaw=math.pi / 2).transform)
        self.assertAlmostEqual(pose.position.x, 1.0)
        self.assertAlmostEqual(pose.position.y, 2.0)
        self.assertAlmostEqual(pose.position.z, 3.0)
        self.assertAlmostEqual(pose.orientation.w, math.cos(math.pi / 4), places=6)


if __name__ == "__main__":
    result = unittest.main(exit=False, verbosity=2).result
    sys.exit(0 if result.wasSuccessful() else 2)
