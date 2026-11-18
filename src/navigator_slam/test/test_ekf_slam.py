"""Unit tests for the joint EKF-SLAM (stage 8), no ROS required.

Run locally from the repository root::

    PYTHONPATH=src/navigator_slam:src/navigator_sim \
        python3 -m unittest discover -s src/navigator_slam/test -v

The loop-closure and robustness tests replay the exact segment logic of
``navigator_sim.motion_node`` (square trajectory, ``loops`` passes) against the
landmark positions of ``navigator_sim.world`` and synthesise the observations
analytically, so they need ``navigator_sim`` on ``PYTHONPATH`` and are skipped
otherwise.
"""

import ast
import math
import os
import unittest

import numpy as np

from navigator_slam.ekf_slam import (
    CHI2_2DOF_99,
    EKFSLAM,
    LandmarkEstimate,
    Observation,
    select_noise,
    wrap_angle,
)

try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML is a ROS dependency
    yaml = None

try:
    from navigator_sim import world
except ImportError:  # pragma: no cover - standalone run without navigator_sim
    world = None

# Same anchors / tag offset as navigator_sim/world.py.
ANCHORS = [
    (0.50, 0.50, 1.20),
    (7.40, 0.80, 1.20),
    (7.10, 7.50, 1.20),
    (0.70, 6.90, 1.20),
]
TAG_OFFSET = (-0.15, 0.12, 0.46)

# Filter model noise (ekf_slam.yaml). The synthetic observations are noisier
# than a perfect detector but cleaner than the model, so the filter keeps a
# conservative R - that is what a real circle-fit detector looks like.
SIGMA_RANGE = 0.05
SIGMA_BEARING = 0.05

START = (1.5, 1.5, 0.0)
DT = 0.05
SIDE = 4.0
LINEAR_VELOCITY = 0.35
ANGULAR_VELOCITY = 0.60
SEED = 20251118
Q = np.diag([1.0e-4, 1.0e-4, 1.0e-4])

LANDMARKS = [(entry[1], entry[2]) for entry in world.LANDMARKS] if world else []

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(HERE)
CONFIG_PATH = os.path.join(PACKAGE_DIR, 'config', 'ekf_slam.yaml')
NODE_PATH = os.path.join(PACKAGE_DIR, 'navigator_slam', 'ekf_slam_node.py')


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------
def make_slam(**overrides):
    """Filter with the defaults of ``ekf_slam.yaml`` (small, fast test set-up)."""
    params = dict(
        initial_pose=START,
        initial_covariance=np.diag([0.10 ** 2, 0.10 ** 2, 0.0873 ** 2]),
        tag_offset=TAG_OFFSET,
        sigma_range=SIGMA_RANGE,
        sigma_bearing=SIGMA_BEARING,
        initial_landmark_std=0.10,
        max_landmarks=20,
        gate=CHI2_2DOF_99,
    )
    params.update(overrides)
    return EKFSLAM(**params)


def true_ranges(pose, anchors=ANCHORS, tag_offset=TAG_OFFSET):
    """Reference UWB range model mirroring navigator_sim.uwb_model_node."""
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


def noisy_ranges(pose, rng, sigma=0.05, dropout=0.0):
    ranges = true_ranges(pose) + rng.normal(0.0, sigma, size=len(ANCHORS))
    if dropout > 0.0:
        ranges[rng.random(len(ANCHORS)) < dropout] = np.nan
    return ranges


def simulate_trajectory(loops=1, control_noise=0.0, rng=None):
    """Replay ``motion_node.step`` and return ``(states, controls)``.

    ``states[i]`` is the true pose before step ``i`` and ``controls[i]`` the
    commanded ``(v, omega, dt_used)`` of that step, exactly as published on
    ``/cmd_vel``. ``control_noise`` perturbs the *executed* velocity (wheel
    slip), so dead reckoning drifts while ``/cmd_vel`` stays ideal.
    """
    segments = []
    for _ in range(max(1, int(loops))):
        for _ in range(4):
            segments.append(('straight', SIDE))
            segments.append(('turn', math.pi / 2.0))

    x, y, yaw = START
    states = [np.array([x, y, yaw])]
    controls = []
    index = 0
    remaining = segments[0][1]
    while index < len(segments):
        kind, _ = segments[index]
        if kind == 'straight':
            v, omega, rate = LINEAR_VELOCITY, 0.0, LINEAR_VELOCITY
        else:
            v, omega, rate = 0.0, ANGULAR_VELOCITY, ANGULAR_VELOCITY

        step = rate * DT
        if step >= remaining:
            step = remaining
            dt_used = step / rate if rate > 0.0 else 0.0
        else:
            dt_used = DT

        v_true, omega_true = v, omega
        if control_noise > 0.0 and rng is not None:
            v_true = v * (1.0 + control_noise * rng.standard_normal())
            omega_true = omega + 0.5 * control_noise * rng.standard_normal()

        x += v_true * math.cos(yaw) * dt_used
        y += v_true * math.sin(yaw) * dt_used
        yaw = wrap_angle(yaw + omega_true * dt_used)

        controls.append((v, omega, dt_used))
        states.append(np.array([x, y, yaw]))

        remaining -= step
        if remaining <= 1e-9:
            index += 1
            if index < len(segments):
                remaining = segments[index][1]
    return states, controls


def observe(pose, rng, sigma_range=0.02, sigma_bearing=0.02, max_range=4.0,
            false_negative=0.0):
    """Synthetic landmark observations from the true pose (base frame)."""
    observations = []
    for lx, ly in LANDMARKS:
        dx, dy = lx - pose[0], ly - pose[1]
        distance = math.hypot(dx, dy)
        if distance > max_range:
            continue
        if false_negative > 0.0 and rng.random() < false_negative:
            continue
        bearing = wrap_angle(math.atan2(dy, dx) - pose[2])
        observations.append(Observation(
            range=distance + rng.normal(0.0, sigma_range),
            bearing=wrap_angle(bearing + rng.normal(0.0, sigma_bearing)),
            radius=0.09,
            range_variance=0.0,
            bearing_variance=0.0,
        ))
    return observations


def run_slam(loops=2, use_uwb=True, seed=SEED, control_noise=0.0,
             observation_noise=(0.02, 0.02), false_negative=0.0, outage=None,
             update_every=2, max_range=4.0, ekf_kwargs=None):
    """Replay the trajectory through the filter and collect diagnostics."""
    rng = np.random.default_rng(seed)
    states, controls = simulate_trajectory(
        loops=loops, control_noise=control_noise, rng=rng
    )
    slam = make_slam(**(ekf_kwargs or {}))
    steps_per_lap = len(controls) // max(1, int(loops))
    lap_ids = [set() for _ in range(int(loops))]

    for step, (v, omega, dt_used) in enumerate(controls):
        slam.predict(v, omega, dt_used, Q)
        if step % update_every != 0:
            continue

        if use_uwb and not (outage and outage[0] <= step < outage[1]):
            slam.update_uwb(noisy_ranges(states[step + 1], rng), ANCHORS, 0.05 ** 2)

        observations = observe(
            states[step + 1], rng,
            sigma_range=observation_noise[0],
            sigma_bearing=observation_noise[1],
            max_range=max_range,
            false_negative=false_negative,
        )
        associations = slam.associate(observations)
        slam.update_landmarks(observations, associations)
        for observation, association in zip(observations, associations):
            if association is None:
                slam.add_landmark(observation)

        lap = min(step // steps_per_lap, int(loops) - 1)
        lap_ids[lap].update(a for a in associations if a is not None)

    return {
        'slam': slam,
        'states': states,
        'controls': controls,
        'lap_ids': lap_ids,
    }


def pose_error(slam, true_pose):
    x, y, _ = slam.mu[:3]
    return math.hypot(x - true_pose[0], y - true_pose[1])


def landmark_errors(slam, positions=LANDMARKS):
    """Distance from every estimated landmark to the nearest true landmark."""
    errors = []
    for landmark in slam.landmarks():
        errors.append(min(
            math.hypot(landmark.x - lx, landmark.y - ly) for lx, ly in positions
        ))
    return errors


def assert_symmetric_psd(test_case, matrix, label=''):
    matrix = np.asarray(matrix, dtype=float)
    test_case.assertTrue(
        np.allclose(matrix, matrix.T, atol=1e-12),
        '%s covariance is not symmetric' % label,
    )
    eigenvalues = np.linalg.eigvalsh(matrix)
    test_case.assertGreaterEqual(
        float(eigenvalues.min()), -1e-12,
        '%s covariance is not PSD (min eigenvalue %r)'
        % (label, float(eigenvalues.min())),
    )


# ----------------------------------------------------------------------
# State initialisation and augmentation
# ----------------------------------------------------------------------
class TestInitialState(unittest.TestCase):
    def test_robot_only_state(self):
        slam = make_slam()
        self.assertEqual(slam.state_dim, 3)
        self.assertEqual(slam.Sigma.shape, (3, 3))
        self.assertEqual(slam.landmark_count, 0)
        np.testing.assert_allclose(slam.mu, START)
        self.assertEqual(slam.landmarks(), [])

    def test_add_landmark_grows_the_state(self):
        slam = make_slam()
        landmark_id = slam.add_landmark(Observation(range=2.0, bearing=0.3))
        self.assertEqual(landmark_id, 0)
        self.assertEqual(slam.state_dim, 5)
        self.assertEqual(slam.Sigma.shape, (5, 5))
        self.assertEqual(slam.ids, [0])
        assert_symmetric_psd(self, slam.Sigma, 'augmented')

        expected = (
            START[0] + 2.0 * math.cos(0.3),
            START[1] + 2.0 * math.sin(0.3),
        )
        np.testing.assert_allclose(slam.mu[3:5], expected, atol=1e-12)
        estimate = slam.landmarks()[0]
        self.assertIsInstance(estimate, LandmarkEstimate)
        self.assertEqual(estimate.id, 0)
        self.assertAlmostEqual(estimate.x, expected[0])
        self.assertAlmostEqual(estimate.y, expected[1])
        self.assertEqual(estimate.covariance.shape, (2, 2))

    def test_add_landmark_keeps_the_pose(self):
        slam = make_slam()
        before = slam.mu.copy()
        slam.add_landmark(Observation(range=1.5, bearing=-1.0))
        np.testing.assert_allclose(slam.mu[:3], before)
        self.assertEqual(slam.Sigma[3, 2], slam.Sigma[2, 3])

    def test_new_landmark_covariance_has_a_floor(self):
        slam = make_slam(initial_landmark_std=0.2)
        slam.add_landmark(Observation(range=1.0, bearing=0.0,
                                     range_variance=0.0,
                                     bearing_variance=0.0))
        covariance = slam.landmark_covariance(0)
        self.assertGreaterEqual(covariance[0, 0], 0.2 ** 2 - 1e-12)
        self.assertGreaterEqual(covariance[1, 1], 0.2 ** 2 - 1e-12)
        assert_symmetric_psd(self, covariance, 'new landmark')

    def test_add_landmark_accepts_an_explicit_measurement_covariance(self):
        slam = make_slam()
        slam.add_landmark(Observation(range=1.0, bearing=0.0), R=np.eye(2) * 1.0)
        covariance = slam.landmark_covariance(0)
        self.assertGreater(covariance[0, 0], 1.0)
        self.assertGreater(covariance[1, 1], 1.0)
        assert_symmetric_psd(self, slam.Sigma, 'explicit R')

    def test_landmark_cap(self):
        slam = make_slam(max_landmarks=2)
        self.assertEqual(slam.add_landmark(Observation(1.0, 0.0)), 0)
        self.assertEqual(slam.add_landmark(Observation(2.0, 0.5)), 1)
        self.assertIsNone(slam.add_landmark(Observation(3.0, 0.7)))
        self.assertEqual(slam.landmark_count, 2)
        self.assertEqual(slam.state_dim, 7)

    def test_invalid_observation_is_rejected(self):
        slam = make_slam()
        self.assertIsNone(slam.add_landmark(Observation(float('nan'), 0.0)))
        self.assertIsNone(slam.add_landmark(Observation(-1.0, 0.0)))
        self.assertEqual(slam.landmark_count, 0)

    def test_unknown_landmark_id_raises(self):
        slam = make_slam()
        with self.assertRaises(KeyError):
            slam.landmark_position(7)


# ----------------------------------------------------------------------
# Prediction
# ----------------------------------------------------------------------
class TestPredict(unittest.TestCase):
    def test_straight_line(self):
        slam = make_slam()
        slam.predict(1.0, 0.0, 1.0, np.zeros((3, 3)))
        np.testing.assert_allclose(slam.mu, [2.5, 1.5, 0.0], atol=1e-12)

    def test_turn_in_place_wraps_theta(self):
        slam = make_slam()
        slam.predict(0.0, 3.0, 1.0, np.zeros((3, 3)))
        self.assertLessEqual(abs(slam.mu[2]), math.pi)

    def test_many_steps_keep_symmetry_and_psd(self):
        slam = make_slam()
        slam.add_landmark(Observation(range=2.0, bearing=0.4))
        slam.add_landmark(Observation(range=3.0, bearing=-0.8))
        robot_trace = []
        for index in range(200):
            slam.predict(LINEAR_VELOCITY, 0.1 * math.sin(index * 0.1), DT, Q)
            robot_trace.append(float(np.trace(slam.robot_covariance)))
            self.assertTrue(np.all(np.isfinite(slam.mu)))
        assert_symmetric_psd(self, slam.Sigma, 'predicted')
        self.assertGreater(robot_trace[-1], robot_trace[0])
        # Landmark blocks carry no process noise but grow through the
        # cross-correlations with the robot.
        self.assertGreater(float(np.trace(slam.landmark_covariance(0))), 0.0)

    def test_landmark_means_are_unchanged_by_prediction(self):
        slam = make_slam()
        slam.add_landmark(Observation(range=2.0, bearing=0.4))
        positions = slam.mu[3:5].copy()
        slam.predict(0.3, 0.2, 0.1, Q)
        np.testing.assert_allclose(slam.mu[3:5], positions)

    def test_cross_covariance_is_carried(self):
        slam = make_slam(initial_pose=(1.5, 1.5, 0.4))
        slam.add_landmark(Observation(range=2.0, bearing=0.4))
        self.assertNotEqual(slam.Sigma[0, 3], 0.0)
        before = slam.Sigma.copy()
        slam.predict(0.3, 0.1, 0.1, Q)
        # The whole cross-covariance block moves with the robot block.
        self.assertFalse(
            np.allclose(slam.Sigma[:3, 3:], before[:3, 3:], atol=1e-12)
        )
        self.assertGreater(float(np.abs(slam.Sigma[1, 4])), 0.0)

    def test_zero_dt_is_a_noop(self):
        slam = make_slam()
        before = slam.mu.copy()
        slam.predict(1.0, 1.0, 0.0, Q)
        np.testing.assert_allclose(slam.mu, before)


# ----------------------------------------------------------------------
# UWB updates
# ----------------------------------------------------------------------
class TestUwbUpdate(unittest.TestCase):
    def test_reduces_the_robot_trace(self):
        slam = make_slam(initial_covariance=np.eye(3) * 0.5)
        before = float(np.trace(slam.robot_covariance))
        slam.update_uwb(true_ranges(START), ANCHORS, 0.05 ** 2)
        self.assertLess(float(np.trace(slam.robot_covariance)), before)
        assert_symmetric_psd(self, slam.Sigma, 'uwb')

    def test_all_nan_is_a_noop(self):
        slam = make_slam()
        mu = slam.mu.copy()
        sigma = slam.Sigma.copy()
        slam.update_uwb([np.nan] * 4, ANCHORS, 0.05 ** 2)
        np.testing.assert_allclose(slam.mu, mu)
        np.testing.assert_allclose(slam.Sigma, sigma)

    def test_dropouts_are_skipped(self):
        slam = make_slam(initial_covariance=np.eye(3) * 0.5)
        ranges = true_ranges(START)
        ranges[[1, 3]] = np.nan
        before = float(np.trace(slam.robot_covariance))
        slam.update_uwb(ranges, ANCHORS, 0.05 ** 2)
        self.assertLess(float(np.trace(slam.robot_covariance)), before)
        assert_symmetric_psd(self, slam.Sigma, 'partial uwb')

    def test_partial_update_matches_manual_subset(self):
        ranges = true_ranges(START)
        ranges[[0, 2]] = np.nan
        valid_anchors = [ANCHORS[1], ANCHORS[3]]
        partial = make_slam(initial_covariance=np.eye(3))
        partial.update_uwb(ranges, ANCHORS, 0.05 ** 2)

        subset = make_slam(initial_covariance=np.eye(3))
        subset.update_uwb(
            [ranges[1], ranges[3]], valid_anchors, 0.05 ** 2
        )
        np.testing.assert_allclose(partial.mu, subset.mu, atol=1e-12)
        np.testing.assert_allclose(partial.Sigma, subset.Sigma, atol=1e-12)

    def test_per_anchor_noise(self):
        slam = make_slam(initial_covariance=np.eye(3))
        before = float(np.trace(slam.robot_covariance))
        slam.update_uwb(true_ranges(START), ANCHORS, [0.05 ** 2] * 4)
        self.assertLess(float(np.trace(slam.robot_covariance)), before)
        slam.update_uwb(
            true_ranges(START), ANCHORS, np.eye(4) * 0.05 ** 2
        )
        assert_symmetric_psd(self, slam.Sigma, 'matrix R')

    def test_anchor_count_mismatch_raises(self):
        slam = make_slam()
        with self.assertRaises(ValueError):
            slam.update_uwb([1.0, 2.0], ANCHORS, 0.05 ** 2)

    def test_noise_selection(self):
        valid = np.array([True, False, True])
        np.testing.assert_allclose(
            select_noise(0.5, valid), np.eye(2) * 0.5
        )
        np.testing.assert_allclose(
            select_noise(np.array([1.0, 2.0, 3.0]), valid), np.diag([1.0, 3.0])
        )
        with self.assertRaises(ValueError):
            select_noise(np.array([1.0, 2.0]), valid)

    def test_trajectory_converges_like_stage_5(self):
        """UWB-only run (no landmarks): the same behaviour as the stage-5 EKF."""
        rng = np.random.default_rng(SEED)
        states, controls = simulate_trajectory(loops=1)
        slam = make_slam(
            initial_pose=(1.7, 1.3, 0.1),
            initial_covariance=np.diag([0.2 ** 2, 0.2 ** 2, 0.2 ** 2]),
        )
        for step, (v, omega, dt_used) in enumerate(controls):
            slam.predict(v, omega, dt_used, Q)
            if step % 2 == 0:
                slam.update_uwb(
                    noisy_ranges(states[step + 1], rng), ANCHORS, 0.05 ** 2
                )
        self.assertLess(pose_error(slam, states[-1]), 0.10)
        self.assertLess(abs(wrap_angle(slam.mu[2] - states[-1][2])), 0.10)
        assert_symmetric_psd(self, slam.Sigma, 'uwb trajectory')


# ----------------------------------------------------------------------
# Landmark update and association
# ----------------------------------------------------------------------
class TestLandmarkUpdate(unittest.TestCase):
    def setUp(self):
        self.true_landmark = (3.0, 2.0)

    def observation_from(self, pose, noise=0.0, rng=None):
        dx = self.true_landmark[0] - pose[0]
        dy = self.true_landmark[1] - pose[1]
        distance = math.hypot(dx, dy)
        bearing = wrap_angle(math.atan2(dy, dx) - pose[2])
        if rng is not None:
            distance += rng.normal(0.0, noise)
            bearing += rng.normal(0.0, noise)
        return Observation(range=distance, bearing=wrap_angle(bearing))

    def test_same_landmark_twice_keeps_one_id(self):
        slam = make_slam()
        first = self.observation_from(START)
        slam.add_landmark(first)

        slam.predict(0.0, 0.5, 1.0, Q)  # turn in place
        second = self.observation_from(slam.mu[:3])
        associations = slam.associate([second])
        self.assertEqual(associations, [0])
        slam.update_landmarks([second], associations)
        self.assertEqual(slam.landmark_count, 1)
        self.assertLess(
            math.hypot(
                slam.mu[3] - self.true_landmark[0],
                slam.mu[4] - self.true_landmark[1],
            ),
            0.02,
        )

    def test_far_observation_does_not_merge(self):
        slam = make_slam()
        slam.add_landmark(self.observation_from(START))

        far = Observation(range=3.0, bearing=0.0)
        self.assertIsNone(slam.associate([far])[0])
        new_id = slam.add_landmark(far)
        self.assertEqual(new_id, 1)
        self.assertEqual(slam.landmark_count, 2)

    def test_outlier_is_rejected_and_does_not_change_the_state(self):
        slam = make_slam()
        slam.add_landmark(self.observation_from(START))
        # Refine the estimate so that the gate is tight.
        for _ in range(3):
            observation = self.observation_from(slam.mu[:3])
            slam.update_landmarks([observation], [0])

        mu = slam.mu.copy()
        sigma = slam.Sigma.copy()
        outlier = self.observation_from(slam.mu[:3])
        outlier.range += 1.5
        associations = slam.associate([outlier])
        self.assertEqual(associations, [None])
        slam.update_landmarks([outlier], associations)
        np.testing.assert_allclose(slam.mu, mu)
        np.testing.assert_allclose(slam.Sigma, sigma)

    def test_update_converges_over_many_scans(self):
        rng = np.random.default_rng(SEED)
        slam = make_slam()
        slam.add_landmark(self.observation_from(START, 0.02, rng))
        for _ in range(40):
            slam.predict(LINEAR_VELOCITY, 0.0, DT, Q)
            observation = self.observation_from(slam.mu[:3], 0.02, rng)
            associations = slam.associate([observation])
            self.assertEqual(associations, [0])
            slam.update_landmarks([observation], associations)
        self.assertLess(
            math.hypot(
                slam.mu[3] - self.true_landmark[0],
                slam.mu[4] - self.true_landmark[1],
            ),
            0.10,
        )
        assert_symmetric_psd(self, slam.Sigma, 'landmark update')

    def test_batch_update_matches_sequential_update(self):
        slam = make_slam()
        slam.add_landmark(Observation(range=2.0, bearing=0.3))
        slam.add_landmark(Observation(range=3.0, bearing=-0.6))
        observations = [
            Observation(range=2.05, bearing=0.29),
            Observation(range=3.10, bearing=-0.58),
        ]
        batch = make_slam()
        batch.add_landmark(Observation(range=2.0, bearing=0.3))
        batch.add_landmark(Observation(range=3.0, bearing=-0.6))
        batch.update_landmarks(observations, [0, 1])

        sequential = make_slam()
        sequential.add_landmark(Observation(range=2.0, bearing=0.3))
        sequential.add_landmark(Observation(range=3.0, bearing=-0.6))
        sequential.update_landmarks([observations[0]], [0])
        sequential.update_landmarks([observations[1]], [1])

        # Both are consistent EKF updates of independent measurements of
        # different landmarks, so they agree closely.
        np.testing.assert_allclose(batch.mu, sequential.mu, atol=1e-9)

    def test_observation_variance_is_added_to_the_model(self):
        slam = make_slam()
        base = slam.measurement_covariance(Observation(1.0, 0.0))
        noisy = slam.measurement_covariance(
            Observation(1.0, 0.0, range_variance=0.01, bearing_variance=0.002)
        )
        self.assertAlmostEqual(base[0, 0], SIGMA_RANGE ** 2)
        self.assertAlmostEqual(noisy[0, 0], SIGMA_RANGE ** 2 + 0.01)
        self.assertAlmostEqual(noisy[1, 1], SIGMA_BEARING ** 2 + 0.002)
        # Negative detector variances are ignored, not subtracted.
        negative = slam.measurement_covariance(
            Observation(1.0, 0.0, range_variance=-1.0)
        )
        self.assertAlmostEqual(negative[0, 0], SIGMA_RANGE ** 2)

    def test_association_length_must_match(self):
        slam = make_slam()
        slam.add_landmark(Observation(range=2.0, bearing=0.0))
        with self.assertRaises(ValueError):
            slam.update_landmarks([Observation(range=2.0, bearing=0.0)], [])

    def test_invalid_observation_is_not_associated(self):
        slam = make_slam()
        slam.add_landmark(Observation(range=2.0, bearing=0.0))
        self.assertEqual(
            slam.associate([Observation(float('nan'), 0.0)]), [None]
        )
        self.assertEqual(slam.associate([]), [])


# ----------------------------------------------------------------------
# Jacobians
# ----------------------------------------------------------------------
class TestJacobians(unittest.TestCase):
    def setUp(self):
        self.slam = make_slam(
            initial_pose=(2.0, 3.0, 0.4),
            initial_covariance=np.eye(3) * 0.1,
        )
        self.slam.add_landmark(Observation(range=2.5, bearing=0.3))
        self.slam.add_landmark(Observation(range=3.5, bearing=-0.9))

    def test_uwb_range_jacobian_matches_finite_difference(self):
        _, H = self.slam.uwb_measurement_and_jacobian(ANCHORS[0])
        self.assertEqual(H.shape, (1, self.slam.state_dim))
        np.testing.assert_allclose(H[0, 3:], 0.0)

        step = 1e-7
        reference = self.slam.mu.copy()
        for column in range(3):
            self.slam.mu = reference.copy()
            self.slam.mu[column] += step
            plus, _ = self.slam.uwb_measurement_and_jacobian(ANCHORS[0])
            self.slam.mu = reference.copy()
            self.slam.mu[column] -= step
            minus, _ = self.slam.uwb_measurement_and_jacobian(ANCHORS[0])
            numeric = (plus - minus) / (2.0 * step)
            self.assertAlmostEqual(H[0, column], numeric, places=6)
        self.slam.mu = reference.copy()

    def test_landmark_jacobian_matches_finite_difference(self):
        landmark_id = 1
        _, H = self.slam.predict_observation(landmark_id)
        self.assertEqual(H.shape, (2, self.slam.state_dim))

        start = 3 + 2 * self.slam.landmark_index(landmark_id)
        columns = [0, 1, 2, start, start + 1]
        step = 1e-7
        reference = self.slam.mu.copy()
        for column in columns:
            self.slam.mu = reference.copy()
            self.slam.mu[column] += step
            plus, _ = self.slam.predict_observation(landmark_id)
            self.slam.mu = reference.copy()
            self.slam.mu[column] -= step
            minus, _ = self.slam.predict_observation(landmark_id)
            numeric = (plus - minus) / (2.0 * step)
            np.testing.assert_allclose(
                H[:, column], numeric, atol=1e-6,
                err_msg='column %d' % column,
            )
        self.slam.mu = reference.copy()

    def test_jacobian_has_zero_columns_outside_the_pair(self):
        _, H = self.slam.predict_observation(0)
        start_other = 3 + 2 * self.slam.landmark_index(1)
        np.testing.assert_allclose(H[:, start_other:start_other + 2], 0.0)

    def test_innovation_is_wrapped(self):
        slam = make_slam(initial_pose=(0.0, 0.0, 3.0))
        slam.add_landmark(Observation(range=2.0, bearing=3.0))
        h, _ = slam.predict_observation(0)
        self.assertLessEqual(abs(h[1]), math.pi)
        # Re-observing with the equivalent +2*pi bearing still associates.
        observation = Observation(range=2.0, bearing=h[1] + 2.0 * math.pi)
        self.assertEqual(slam.associate([observation]), [0])


# ----------------------------------------------------------------------
# Full closed loop
# ----------------------------------------------------------------------
@unittest.skipIf(world is None, 'navigator_sim is not on PYTHONPATH')
class TestLoopClosure(unittest.TestCase):
    def test_double_loop_closes_and_keeps_the_ids(self):
        result = run_slam(loops=2)
        slam = result['slam']
        final = result['states'][-1]

        error = pose_error(slam, final)
        self.assertLess(
            error, 0.15, 'final pose error %.3f m' % error
        )
        self.assertLess(
            abs(wrap_angle(slam.mu[2] - final[2])), 0.10,
            'final yaw error %.3f rad' % abs(wrap_angle(slam.mu[2] - final[2])),
        )

        self.assertEqual(
            slam.landmark_count, len(LANDMARKS),
            'expected %d landmarks, got %d (id explosion)'
            % (len(LANDMARKS), slam.landmark_count),
        )
        errors = landmark_errors(slam)
        self.assertLess(
            float(np.mean(errors)), 0.25,
            'mean landmark error %.3f m (%s)' % (np.mean(errors), errors),
        )
        self.assertLess(max(errors), 0.35, 'max landmark error %r' % max(errors))

        first_lap, second_lap = result['lap_ids']
        self.assertTrue(first_lap)
        self.assertEqual(
            first_lap, second_lap,
            'second lap introduced new ids: %s vs %s' % (first_lap, second_lap),
        )
        self.assertTrue(np.all(np.isfinite(slam.mu)))
        assert_symmetric_psd(self, slam.Sigma, 'slam')

    def test_covariance_shrinks_with_observations(self):
        result = run_slam(loops=1)
        slam = result['slam']
        self.assertEqual(slam.landmark_count, len(LANDMARKS))
        for landmark in slam.landmarks():
            self.assertLess(float(np.trace(landmark.covariance)), 0.25 ** 2)


# ----------------------------------------------------------------------
# Robustness
# ----------------------------------------------------------------------
@unittest.skipIf(world is None, 'navigator_sim is not on PYTHONPATH')
class TestRobustness(unittest.TestCase):
    def test_thirty_percent_missing_observations(self):
        result = run_slam(loops=2, false_negative=0.3, seed=SEED + 1)
        slam = result['slam']
        final = result['states'][-1]
        self.assertTrue(np.all(np.isfinite(slam.mu)))
        self.assertLess(pose_error(slam, final), 0.30)
        self.assertEqual(slam.landmark_count, len(LANDMARKS))
        self.assertLess(float(np.mean(landmark_errors(slam))), 0.25)

    def test_without_uwb_the_map_drifts_but_stays_locally_consistent(self):
        """No absolute reference: the map frame drifts, the geometry survives.

        In this polygon every landmark is re-observed on each side of the
        square, so the landmark measurements alone keep the *local* pose
        accurate even with UWB switched off (and unmodelled wheel slip). What
        is lost without the absolute range reference is the global anchor: the
        pose covariance is an order of magnitude larger and the landmark
        positions in the ``map`` frame drift by a few centimetres, while their
        geometry in the robot frame stays correct.
        """
        with_uwb = run_slam(loops=2, use_uwb=True, control_noise=0.04,
                            seed=SEED + 2)
        without_uwb = run_slam(loops=2, use_uwb=False, control_noise=0.04,
                               seed=SEED + 2)

        slam = without_uwb['slam']
        final = without_uwb['states'][-1]
        self.assertTrue(np.all(np.isfinite(slam.mu)))
        self.assertEqual(slam.landmark_count, len(LANDMARKS))
        self.assertLess(pose_error(slam, final), 0.25)

        # Uncertainty and map-frame drift grow without the absolute anchor.
        self.assertGreater(
            float(np.trace(slam.robot_covariance)),
            5.0 * float(np.trace(with_uwb['slam'].robot_covariance)),
        )
        self.assertGreater(
            float(np.mean(landmark_errors(slam))),
            3.0 * float(np.mean(landmark_errors(with_uwb['slam']))),
        )

        # The map is expressed in the drifted dead-reckoning frame, so compare
        # the landmark geometry *in the robot frame*: every landmark must still
        # reproduce the true relative geometry.
        true_relative = [
            robot_frame((lx, ly), final) for lx, ly in LANDMARKS
        ]
        worst = 0.0
        for landmark in slam.landmarks():
            estimated = robot_frame((landmark.x, landmark.y), slam.mu[:3])
            worst = max(
                worst,
                min(
                    math.hypot(estimated[0] - t[0], estimated[1] - t[1])
                    for t in true_relative
                ),
            )
        self.assertLess(worst, 0.20, 'landmark geometry error %.3f m' % worst)

    def test_uwb_outage_keeps_the_state_finite(self):
        result = run_slam(loops=1, outage=(300, 700), seed=SEED + 3)
        slam = result['slam']
        self.assertTrue(np.all(np.isfinite(slam.mu)))
        self.assertTrue(np.all(np.isfinite(slam.Sigma)))
        self.assertLess(pose_error(slam, result['states'][-1]), 0.30)
        self.assertLess(float(np.mean(landmark_errors(slam))), 0.25)
        assert_symmetric_psd(self, slam.Sigma, 'outage')


def robot_frame(point, pose):
    """Coordinates of a map-frame point in the robot frame of ``pose``."""
    dx = point[0] - pose[0]
    dy = point[1] - pose[1]
    c, s = math.cos(-pose[2]), math.sin(-pose[2])
    return (c * dx - s * dy, s * dx + c * dy)


# ----------------------------------------------------------------------
# Landmark confirmation
# ----------------------------------------------------------------------
class TestLandmarkConfirmation(unittest.TestCase):
    """Candidates need several consistent sightings before they enter the state."""

    LANDMARK = (4.0, 0.45)

    @staticmethod
    def observation(pose, point, scale=1.0):
        dx = point[0] - pose[0]
        dy = point[1] - pose[1]
        return Observation(
            range=math.hypot(dx, dy) + scale * 0.0,
            bearing=wrap_angle(math.atan2(dy, dx) - pose[2]),
            radius=0.09,
            range_variance=0.01 ** 2,
            bearing_variance=0.01 ** 2,
        )

    def test_candidate_needs_several_sightings(self):
        slam = make_slam(min_landmark_hits=3)
        observation = self.observation(START, self.LANDMARK)
        self.assertIsNone(slam.confirm_observation(observation))
        self.assertIsNone(slam.confirm_observation(observation))
        self.assertEqual(slam.landmark_count, 0)
        self.assertEqual(len(slam.tentative), 1)

        new_id = slam.confirm_observation(observation)
        self.assertEqual(new_id, 0)
        self.assertEqual(slam.landmark_count, 1)
        self.assertEqual(len(slam.tentative), 0)
        np.testing.assert_allclose(
            slam.landmark_position(new_id), self.LANDMARK, atol=0.02
        )

    def test_one_hit_creates_immediately(self):
        slam = make_slam(min_landmark_hits=1)
        new_id = slam.confirm_observation(self.observation(START, self.LANDMARK))
        self.assertEqual(new_id, 0)
        self.assertEqual(slam.landmark_count, 1)
        self.assertEqual(len(slam.tentative), 0)

    def test_scattered_sightings_do_not_confirm(self):
        slam = make_slam(min_landmark_hits=3)
        for step in range(6):
            point = (4.0 + 1.5 * step, 0.45)
            self.assertIsNone(
                slam.confirm_observation(self.observation(START, point))
            )
        self.assertEqual(slam.landmark_count, 0)
        self.assertGreaterEqual(len(slam.tentative), 6)

    def test_candidates_do_not_consume_the_landmark_cap(self):
        slam = make_slam(min_landmark_hits=5, max_landmarks=1)
        for step in range(4):
            slam.confirm_observation(
                self.observation(START, (4.0 + 1.0 * step, 0.45))
            )
        self.assertEqual(slam.landmark_count, 0)

        # A fresh candidate (far from the scattered ones) needs five sightings.
        observation = self.observation(START, (0.45, 4.0))
        for _ in range(4):
            self.assertIsNone(slam.confirm_observation(observation))
        self.assertEqual(slam.landmark_count, 0)
        self.assertEqual(slam.confirm_observation(observation), 0)
        self.assertEqual(slam.landmark_count, 1)

    def test_process_observations_reports_new_ids(self):
        slam = make_slam(min_landmark_hits=1)
        observations = [
            self.observation(START, self.LANDMARK),
            self.observation(START, (0.45, 4.0)),
        ]
        associations, added = slam.process_observations(observations)
        self.assertEqual(associations, [None, None])
        self.assertEqual(sorted(added), [0, 1])
        self.assertEqual(slam.landmark_count, 2)

        associations, added = slam.process_observations(observations)
        self.assertEqual(associations, [0, 1])
        self.assertEqual(added, [])

    def test_invalid_observation_is_ignored(self):
        slam = make_slam(min_landmark_hits=1)
        self.assertIsNone(slam.confirm_observation(Observation(float('nan'), 0.0)))
        self.assertIsNone(slam.confirm_observation(Observation(-1.0, 0.0)))



# ----------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------
@unittest.skipIf(world is None, 'navigator_sim is not on PYTHONPATH')
@unittest.skipIf(yaml is None, 'PyYAML is not installed')
class TestConfig(unittest.TestCase):
    def setUp(self):
        with open(CONFIG_PATH, 'r') as stream:
            self.params = yaml.safe_load(stream)['ekf_slam_node'][
                'ros__parameters'
            ]

    def test_anchors_and_tag_offset_match_world(self):
        flat = [float(v) for v in self.params['anchors']]
        expected = []
        for _, ax, ay, az in world.UWB_ANCHORS:
            expected.extend([ax, ay, az])
        self.assertEqual(flat, expected)
        self.assertEqual(
            tuple(self.params['tag_offset']), tuple(world.UWB_TAG_OFFSET)
        )

    def test_initial_pose_and_noise_are_consistent(self):
        self.assertGreater(self.params['gate'], 0.0)
        self.assertLessEqual(self.params['gate'], CHI2_2DOF_99)
        self.assertGreater(self.params['sigma_range'], 0.0)
        self.assertGreater(self.params['sigma_bearing'], 0.0)
        self.assertGreater(self.params['initial_landmark_std'], 0.0)
        self.assertGreaterEqual(self.params['max_landmarks'], len(world.LANDMARKS))
        self.assertGreater(self.params['publish_rate_hz'], 0.0)
        self.assertGreater(self.params['predict_rate_hz'], 0.0)
        self.assertEqual(self.params['frame_id'], 'map')
        self.assertTrue(self.params['use_uwb'])


@unittest.skipIf(yaml is None, 'PyYAML is not installed')
class TestConfigMatchesNode(unittest.TestCase):
    """Every parameter the node declares must be in ekf_slam.yaml."""

    def test_parameter_names_match_the_config(self):
        if not os.path.exists(NODE_PATH):
            self.skipTest('node source is not available')
        with open(NODE_PATH, 'r') as stream:
            tree = ast.parse(stream.read())
        declared = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            function = node.func
            if not isinstance(function, ast.Attribute):
                continue
            if function.attr != 'declare_parameter' or not node.args:
                continue
            argument = node.args[0]
            if isinstance(argument, ast.Constant):
                declared.append(argument.value)
        self.assertTrue(declared, 'no declare_parameter calls found')

        with open(CONFIG_PATH, 'r') as stream:
            params = yaml.safe_load(stream)['ekf_slam_node']['ros__parameters']
        self.assertEqual(sorted(declared), sorted(params))


if __name__ == '__main__':
    unittest.main()
