"""Unit tests for the pure EKF implementation (no ROS required).

Run locally from the repository root::

    PYTHONPATH=src/navigator_localization \
        python3 -m unittest discover -s src/navigator_localization/test -v
"""

import math
import unittest

import numpy as np

from navigator_localization.ekf import EKFLocalization, wrap_angle

# Same anchors as navigator_sim/world.py (A1..A4).
ANCHORS = [
    (0.50, 0.50, 1.20),
    (7.40, 0.80, 1.20),
    (7.10, 7.50, 1.20),
    (0.70, 6.90, 1.20),
]
# Same tag mounting offset as world.UWB_TAG_OFFSET.
TAG_OFFSET = (-0.15, 0.12, 0.46)


def true_ranges(pose, anchors, tag_offset=TAG_OFFSET):
    """Reference range model mirroring navigator_sim.uwb_model_node."""
    x, y, theta = pose
    dx, dy, dz = tag_offset
    c, s = math.cos(theta), math.sin(theta)
    tx = x + dx * c - dy * s
    ty = y + dx * s + dy * c
    ranges = []
    for ax, ay, az in anchors:
        ex, ey, ez = tx - ax, ty - ay, dz - az
        ranges.append(math.sqrt(ex * ex + ey * ey + ez * ez))
    return np.array(ranges, dtype=float)


class TestPredict(unittest.TestCase):
    def test_straight_line(self):
        ekf = EKFLocalization([0.0, 0.0, 0.0], np.zeros((3, 3)), use_tag_offset=False)
        x, _ = ekf.predict(1.0, 0.0, 1.0, np.zeros((3, 3)))
        np.testing.assert_allclose(x, [1.0, 0.0, 0.0], atol=1e-12)

    def test_turn_in_place(self):
        ekf = EKFLocalization([0.0, 0.0, 0.0], np.zeros((3, 3)))
        x, _ = ekf.predict(0.0, math.pi / 2.0, 1.0, np.zeros((3, 3)))
        self.assertAlmostEqual(x[2], math.pi / 2.0)

    def test_covariance_grows(self):
        ekf = EKFLocalization([0.0, 0.0, 0.0], np.eye(3) * 0.01)
        Q = np.diag([0.1, 0.1, 0.01])
        _, P = ekf.predict(0.5, 0.1, 0.05, Q)
        self.assertTrue(np.all(np.linalg.eigvalsh(P) > 0.0))


class TestUpdate(unittest.TestCase):
    def test_all_nan_is_noop(self):
        ekf = EKFLocalization([1.0, 2.0, 0.3], np.eye(3))
        x0 = ekf.x.copy()
        P0 = ekf.P.copy()
        x, P = ekf.update([np.nan] * 4, ANCHORS, 0.05 ** 2)
        np.testing.assert_allclose(x, x0)
        np.testing.assert_allclose(P, P0)

    def test_reduces_covariance(self):
        ekf = EKFLocalization([2.0, 3.0, 0.4], np.eye(3) * 0.5)
        before = np.trace(ekf.P)
        ekf.update(true_ranges([2.0, 3.0, 0.4], ANCHORS), ANCHORS, 0.05 ** 2)
        self.assertLess(np.trace(ekf.P), before)

    def test_partial_dropout(self):
        ekf = EKFLocalization([2.0, 3.0, 0.4], np.eye(3) * 0.5)
        ranges = true_ranges([2.0, 3.0, 0.4], ANCHORS)
        ranges[[1, 3]] = np.nan
        before = np.trace(ekf.P)
        ekf.update(ranges, ANCHORS, 0.05 ** 2)
        self.assertLess(np.trace(ekf.P), before)

    def test_static_measurement_cannot_observe_heading(self):
        """A single UWB tag fixes only its own (x, y): H is rank 2.

        Heading becomes observable only through motion (turns), so a static
        range-only update must not be expected to recover theta.
        """
        ekf = EKFLocalization([2.0, 3.0, 0.5], np.eye(3), tag_offset=TAG_OFFSET)
        H = np.vstack([ekf.measurement_and_jacobian(a)[1] for a in ANCHORS])
        self.assertEqual(np.linalg.matrix_rank(H), 2)

    def test_converges_along_trajectory_with_turns(self):
        """Moving with turns makes the heading observable; the EKF tracks it."""
        dt = 0.05
        true = [1.5, 1.5, 0.0]
        controls = []
        for _ in range(2):
            controls += [(0.4, 0.0)] * int(round(1.2 / 0.4 / dt))
            controls += [(0.0, 0.6)] * int(round((math.pi / 2.0) / 0.6 / dt))

        ekf = EKFLocalization(
            [1.7, 1.3, 0.1],
            np.diag([0.2 ** 2, 0.2 ** 2, 0.2 ** 2]),
            tag_offset=TAG_OFFSET,
        )
        Q = np.diag([1e-4, 1e-4, 1e-4])
        for v, omega in controls:
            true[0] += v * dt * math.cos(true[2])
            true[1] += v * dt * math.sin(true[2])
            true[2] = wrap_angle(true[2] + omega * dt)
            ekf.predict(v, omega, dt, Q)
            ekf.update(true_ranges(true, ANCHORS), ANCHORS, 0.05 ** 2)

        np.testing.assert_allclose(ekf.x[:2], true[:2], atol=0.02)
        self.assertLess(abs(wrap_angle(ekf.x[2] - true[2])), 0.02)

    def test_jacobian_matches_finite_difference(self):
        ekf = EKFLocalization([1.3, 2.1, 0.7], np.eye(3), tag_offset=TAG_OFFSET)
        anchor = ANCHORS[0]
        _, H = ekf.measurement_and_jacobian(anchor)

        eps = 1e-6
        saved = ekf.x.copy()
        numeric = np.zeros(3)
        for i in range(3):
            ekf.x = saved.copy()
            ekf.x[i] += eps
            r_plus, _ = ekf.measurement_and_jacobian(anchor)
            ekf.x = saved.copy()
            ekf.x[i] -= eps
            r_minus, _ = ekf.measurement_and_jacobian(anchor)
            numeric[i] = (r_plus - r_minus) / (2.0 * eps)
        ekf.x = saved

        np.testing.assert_allclose(H.reshape(3), numeric, atol=1e-5)


class TestCovariance(unittest.TestCase):
    def test_symmetric_psd_after_steps(self):
        ekf = EKFLocalization([1.0, 1.0, 0.1], np.eye(3) * 0.2, tag_offset=TAG_OFFSET)
        for _ in range(20):
            ekf.predict(0.35, 0.0, 0.05, np.diag([1e-4, 1e-4, 1e-6]))
            ekf.update(true_ranges(ekf.x, ANCHORS), ANCHORS, 0.05 ** 2)
        np.testing.assert_allclose(ekf.P, ekf.P.T, atol=1e-12)
        self.assertTrue(np.all(np.linalg.eigvalsh(ekf.P) >= -1e-12))


if __name__ == '__main__':
    unittest.main()
