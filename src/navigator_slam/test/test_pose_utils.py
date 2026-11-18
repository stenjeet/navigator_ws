"""Unit tests for the pose helpers (no ROS required).

Run locally from the repository root::

    PYTHONPATH=src/navigator_slam python3 -m unittest \
        discover -s src/navigator_slam/test -p 'test_pose_utils.py' -v
"""

import math
import unittest
from collections import namedtuple

import numpy as np

from navigator_slam.pose_utils import (
    POSE_COVARIANCE_INDICES,
    pose_covariance,
    quaternion_to_yaw,
)

Quaternion = namedtuple('Quaternion', 'x y z w')


def yaw_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


class TestPoseCovariance(unittest.TestCase):
    def test_indices_match_the_odometry_layout(self):
        # x, y, z, roll, pitch, yaw on both axes; x-z and y-z must not be used.
        expected = (0, 1, 5, 6, 7, 11, 30, 31, 35)
        self.assertEqual(POSE_COVARIANCE_INDICES, expected)

    def test_extracts_the_planar_block(self):
        covariance = [0.0] * 36
        values = {
            0: 1.0, 1: 2.0, 5: 3.0,
            6: 4.0, 7: 5.0, 11: 6.0,
            30: 7.0, 31: 8.0, 35: 9.0,
        }
        for index, value in values.items():
            covariance[index] = value
        # Values at the wrong positions (x-z, y-z, z-z) must be ignored.
        for index in (2, 8, 14, 20):
            covariance[index] = 100.0

        block = pose_covariance(covariance)
        np.testing.assert_allclose(
            block,
            [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]],
        )

    def test_zero_covariance(self):
        np.testing.assert_allclose(pose_covariance([0.0] * 36), np.zeros((3, 3)))

    def test_accepts_numpy_input(self):
        block = pose_covariance(np.arange(36, dtype=float))
        self.assertEqual(block.shape, (3, 3))
        np.testing.assert_allclose(block[0], [0.0, 1.0, 5.0])


class TestQuaternionToYaw(unittest.TestCase):
    def test_identity(self):
        self.assertAlmostEqual(quaternion_to_yaw(Quaternion(0, 0, 0, 1)), 0.0)

    def test_quarter_turn(self):
        self.assertAlmostEqual(
            quaternion_to_yaw(yaw_quaternion(math.pi / 2.0)), math.pi / 2.0
        )

    def test_negative_angle_is_wrapped(self):
        yaw = -0.75 * math.pi
        self.assertAlmostEqual(quaternion_to_yaw(yaw_quaternion(yaw)), yaw)


if __name__ == '__main__':
    unittest.main()
