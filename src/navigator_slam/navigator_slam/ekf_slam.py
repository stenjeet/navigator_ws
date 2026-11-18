"""Joint EKF-SLAM: robot pose and landmark coordinates in one filter (stage 8).

The filter keeps the planar robot pose ``base_footprint`` in ``map`` *together*
with the estimated landmark positions::

    mu    = [x, y, theta, m1x, m1y, ..., mNx, mNy]
    Sigma = cov(mu)                       # (2N + 3) x (2N + 3)

Keeping both blocks in one covariance is what makes SLAM work: the
cross-correlations between the robot and the landmarks turn every re-observed
landmark into a pose correction (loop closure), and the pose uncertainty is
propagated into the landmark initialisation.

Models
------
* Prediction - differential drive from the commanded ``(v, omega)``::

      x     += v dt cos(theta)
      y     += v dt sin(theta)
      theta += omega dt

  with the full process Jacobian ``F`` (identity outside the robot block) and
  ``Q`` non-zero only on the robot block, so the landmark cross-covariances are
  carried along.
* UWB update - the range model of the stage-5 localization EKF (``ekf.py``) is
  reused verbatim but not imported: the tag is mounted at ``tag_offset`` in
  ``base_link``, rotated with ``theta``, and the range is the 3D distance to the
  anchor. ``NaN`` marks a dropout and is skipped.
* Landmark update - range/bearing from the ``base_footprint`` origin::

      h(x) = [ d, wrap(atan2(dy, dx) - theta) ],  d = hypot(dx, dy)

  with the analytic 2 x (2N + 3) Jacobian (robot block, associated landmark
  block) and the Joseph form for the covariance. Observations are pose-free, so
  the lidar mounting offset does not enter the model.
* Initialisation - inverse observation model from the current robot pose; the
  new landmark block and its cross-covariances to the *whole* state are filled
  from the robot Jacobian, so later observations of other landmarks still carry
  the correct correlations.

Measurement noise
-----------------
``LandmarkObservation`` carries its own variances produced by the detector
(circle-fit residuals). The filter adds its configured model floor:
``var = sigma**2 + max(obs_variance, 0)``. Summing (instead of taking the
maximum) is the conservative choice: the result is never smaller than either
input, so a detector that underestimates its error can never make the filter
overconfident, while a zero variance falls back to the model value.

Data association
----------------
``associate`` predicts every known landmark in the robot frame, forms the
innovation covariance from the **joint** Sigma blocks (robot block + landmark
block + their cross terms) plus ``R`` and associates greedily by the smallest
Mahalanobis distance below a chi-square gate (2 degrees of freedom, 9.21 for
99 %). Each landmark and each observation is used at most once per scan.

The module has no ROS dependencies (numpy only) so it can be unit-tested
directly; the ROS wrapper lives in ``ekf_slam_node.py``. It is deliberately
standalone from stage 7: the node converts the ``navigator_interfaces``
messages into the plain dataclasses below.
"""

import math
from dataclasses import dataclass

import numpy as np

from navigator_slam.tentative import TentativeTracks

ROBOT_DIM = 3
LANDMARK_DIM = 2

#: Chi-square quantile for 2 degrees of freedom and a 99 % confidence level.
CHI2_2DOF_99 = 9.21


def wrap_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Observation:
    """One landmark observation in the ``base_footprint`` frame.

    Mirrors ``navigator_interfaces/LandmarkObservation`` without importing it:
    ``range``/``bearing`` are the measurement, ``radius`` is informational and
    the two variances describe the detector's own uncertainty.
    """

    range: float
    bearing: float
    radius: float = 0.0
    range_variance: float = 0.0
    bearing_variance: float = 0.0

    def is_finite(self):
        return (
            math.isfinite(self.range)
            and math.isfinite(self.bearing)
            and self.range > 0.0
        )


@dataclass
class LandmarkEstimate:
    """Snapshot of one landmark of the joint state."""

    id: int
    x: float
    y: float
    covariance: np.ndarray  # 2x2 position covariance [m^2]


class EKFSLAM:
    """Joint EKF-SLAM over the robot pose and the landmark map."""

    def __init__(
        self,
        initial_pose=(0.0, 0.0, 0.0),
        initial_covariance=None,
        tag_offset=(0.0, 0.0, 0.0),
        sigma_range=0.05,
        sigma_bearing=0.05,
        initial_landmark_std=0.10,
        max_landmarks=50,
        gate=CHI2_2DOF_99,
        min_landmark_hits=8,
        tentative_gate=0.35,
        max_tentative_tracks=200,
    ):
        pose = np.array(initial_pose, dtype=float).reshape(ROBOT_DIM)
        if initial_covariance is None:
            initial_covariance = np.diag([0.1 ** 2, 0.1 ** 2, 0.09 ** 2])
        covariance = np.asarray(initial_covariance, dtype=float).reshape(
            ROBOT_DIM, ROBOT_DIM
        )
        if sigma_range <= 0.0 or sigma_bearing <= 0.0:
            raise ValueError('sigma_range and sigma_bearing must be positive')
        if initial_landmark_std < 0.0:
            raise ValueError('initial_landmark_std must be non-negative')
        if gate <= 0.0:
            raise ValueError('association gate must be positive')
        max_landmarks = int(max_landmarks)
        if max_landmarks < 1:
            raise ValueError('max_landmarks must be at least 1')

        self.mu = pose
        self.mu[2] = wrap_angle(self.mu[2])
        self.Sigma = 0.5 * (covariance + covariance.T)
        self.tag_offset = tuple(float(v) for v in tag_offset)
        self.sigma_range = float(sigma_range)
        self.sigma_bearing = float(sigma_bearing)
        self.initial_landmark_std = float(initial_landmark_std)
        self.max_landmarks = max_landmarks
        self.gate = float(gate)
        #: Landmark ids in state order: block ``k`` is ``mu[3 + 2k : 5 + 2k]``.
        self.ids = []
        #: Candidates that need ``min_landmark_hits`` sightings before they
        #: enter the state; wall corners would otherwise become phantoms.
        self.tentative = TentativeTracks(
            gate=tentative_gate,
            min_hits=min_landmark_hits,
            max_tracks=max_tentative_tracks,
        )

    # ------------------------------------------------------------------
    # State accessors
    # ------------------------------------------------------------------
    @property
    def state_dim(self):
        return int(self.mu.size)

    @property
    def landmark_count(self):
        return len(self.ids)

    @property
    def pose(self):
        return self.mu[:ROBOT_DIM].copy()

    @property
    def robot_covariance(self):
        return self.Sigma[:ROBOT_DIM, :ROBOT_DIM].copy()

    def landmark_index(self, landmark_id):
        """State block index of ``landmark_id`` (raises ``KeyError``)."""
        try:
            return self.ids.index(int(landmark_id))
        except ValueError:
            raise KeyError('unknown landmark id %r' % (landmark_id,))

    def landmark_position(self, landmark_id):
        index = self.landmark_index(landmark_id)
        start = ROBOT_DIM + LANDMARK_DIM * index
        return self.mu[start:start + LANDMARK_DIM].copy()

    def landmark_covariance(self, landmark_id):
        index = self.landmark_index(landmark_id)
        start = ROBOT_DIM + LANDMARK_DIM * index
        return self.Sigma[
            start:start + LANDMARK_DIM, start:start + LANDMARK_DIM
        ].copy()

    def landmarks(self):
        """All landmarks as :class:`LandmarkEstimate` snapshots."""
        result = []
        for index, landmark_id in enumerate(self.ids):
            start = ROBOT_DIM + LANDMARK_DIM * index
            x, y = self.mu[start:start + LANDMARK_DIM]
            result.append(LandmarkEstimate(
                id=int(landmark_id),
                x=float(x),
                y=float(y),
                covariance=self.Sigma[
                    start:start + LANDMARK_DIM, start:start + LANDMARK_DIM
                ].copy(),
            ))
        return result

    # ------------------------------------------------------------------
    # Prediction
    # ------------------------------------------------------------------
    def predict(self, v, omega, dt, process_noise):
        """Advance the state and the covariance with the differential drive.

        ``process_noise`` is the 3x3 robot process covariance; it is embedded in
        a ``(2N+3) x (2N+3)`` matrix with zeros on the landmark blocks.
        """
        v = float(v)
        omega = float(omega)
        dt = float(dt)
        if dt <= 0.0:
            return self.mu.copy(), self.Sigma.copy()

        n = self.state_dim
        theta = float(self.mu[2])
        cos_theta, sin_theta = math.cos(theta), math.sin(theta)

        self.mu[0] += v * dt * cos_theta
        self.mu[1] += v * dt * sin_theta
        self.mu[2] = wrap_angle(theta + omega * dt)

        F = np.eye(n)
        F[0, 2] = -v * dt * sin_theta
        F[1, 2] = v * dt * cos_theta

        Q = np.zeros((n, n))
        Q[:ROBOT_DIM, :ROBOT_DIM] = np.asarray(
            process_noise, dtype=float
        ).reshape(ROBOT_DIM, ROBOT_DIM)

        # Apple Accelerate's BLAS raises spurious divide-by-zero/overflow flags
        # on well-conditioned products; ignore the flags and verify the result.
        with np.errstate(divide='ignore', over='ignore', invalid='ignore'):
            self.Sigma = F @ self.Sigma @ F.T + Q
        self._symmetrize()
        self._check_finite()
        return self.mu.copy(), self.Sigma.copy()

    # ------------------------------------------------------------------
    # UWB range model (same as the stage-5 localization EKF)
    # ------------------------------------------------------------------
    def _tag_xy(self, tag_offset=None):
        """World position of the UWB tag for the current pose estimate."""
        x, y, theta = self.mu[:ROBOT_DIM]
        dx, dy, _ = self.tag_offset if tag_offset is None else tag_offset
        c, s = math.cos(theta), math.sin(theta)
        return x + dx * c - dy * s, y + dx * s + dy * c

    def uwb_measurement_and_jacobian(self, anchor, tag_offset=None):
        """Return ``(range, H)`` for an anchor ``(ax, ay, az)``.

        ``H`` is the ``1 x (2N+3)`` Jacobian row of the range; the landmark
        columns are zero because UWB only observes the robot. ``tag_offset``
        overrides the offset the filter was built with.
        """
        ax, ay, az = (float(v) for v in anchor)
        offset = self.tag_offset if tag_offset is None else tuple(
            float(v) for v in tag_offset
        )
        theta = float(self.mu[2])
        dx, dy, dz = offset
        c, s = math.cos(theta), math.sin(theta)

        tx, ty = self._tag_xy(offset)
        ex = tx - ax
        ey = ty - ay
        ez = dz - az
        r = math.sqrt(ex * ex + ey * ey + ez * ez)
        if r < 1e-9:
            r = 1e-9

        dr_dtheta = (ex * (-dx * s - dy * c) + ey * (dx * c - dy * s)) / r
        H = np.zeros((1, self.state_dim))
        H[0, 0] = ex / r
        H[0, 1] = ey / r
        H[0, 2] = dr_dtheta
        return r, H

    def update_uwb(self, ranges, anchors, measurement_noise, tag_offset=None):
        """Correct the state with one range per anchor (``NaN`` = dropout).

        ``measurement_noise`` accepts a scalar variance, a per-anchor variance
        vector or a full covariance matrix, like the stage-5 EKF.
        """
        z = np.asarray(ranges, dtype=float).reshape(-1)
        anchors = list(anchors)
        if len(anchors) != len(z):
            raise ValueError(
                'expected %d anchors, got %d' % (len(z), len(anchors))
            )

        valid = np.isfinite(z)
        n_valid = int(np.count_nonzero(valid))
        if n_valid == 0:
            return self.mu.copy(), self.Sigma.copy()

        n = self.state_dim
        H = np.zeros((n_valid, n))
        h = np.zeros(n_valid)
        row = 0
        for i in range(len(z)):
            if not valid[i]:
                continue
            r, H_i = self.uwb_measurement_and_jacobian(
                anchors[i], tag_offset
            )
            h[row] = r
            H[row, :] = H_i[0]
            row += 1

        R = select_noise(measurement_noise, valid)
        self._correct(z[valid] - h, H, R)
        return self.mu.copy(), self.Sigma.copy()

    # ------------------------------------------------------------------
    # Landmark observation model
    # ------------------------------------------------------------------
    def measurement_covariance(self, observation):
        """``2x2`` covariance of one landmark observation.

        Model floor (``sigma_range**2`` / ``sigma_bearing**2``) plus the
        detector's own variance; see the module docstring for why the two are
        summed and not maxed.
        """
        range_var = self.sigma_range ** 2 + max(
            float(observation.range_variance), 0.0
        )
        bearing_var = self.sigma_bearing ** 2 + max(
            float(observation.bearing_variance), 0.0
        )
        return np.diag([range_var, bearing_var])

    def predict_observation(self, landmark_id):
        """Return ``(h, H)`` of the range/bearing model for one landmark.

        ``h`` is ``[d, azimuth]`` with the azimuth wrapped to (-pi, pi];
        ``H`` is the ``2 x (2N+3)`` analytic Jacobian.
        """
        index = self.landmark_index(landmark_id)
        start = ROBOT_DIM + LANDMARK_DIM * index
        lx, ly = self.mu[start:start + LANDMARK_DIM]
        x, y, theta = self.mu[:ROBOT_DIM]

        dx = lx - x
        dy = ly - y
        d2 = max(dx * dx + dy * dy, 1e-12)
        d = math.sqrt(d2)

        H = np.zeros((2, self.state_dim))
        # Range: only the robot x/y and the landmark x/y matter.
        H[0, 0] = -dx / d
        H[0, 1] = -dy / d
        H[0, start] = dx / d
        H[0, start + 1] = dy / d

        # Bearing atan2(dy, dx) - theta.
        H[1, 0] = dy / d2
        H[1, 1] = -dx / d2
        H[1, 2] = -1.0
        H[1, start] = -dy / d2
        H[1, start + 1] = dx / d2

        h = np.array([d, wrap_angle(math.atan2(dy, dx) - theta)])
        return h, H

    # ------------------------------------------------------------------
    # Data association
    # ------------------------------------------------------------------
    def associate(self, observations, gate=None):
        """Greedy nearest-neighbour association by Mahalanobis distance.

        Returns one entry per observation: the matched landmark id or ``None``.
        The innovation covariance uses the joint Sigma blocks (robot,
        landmark and their cross-covariance) plus ``R``; pairs above ``gate``
        (chi-square 2 dof) are rejected, and each landmark and each observation
        is used at most once.
        """
        if gate is None:
            gate = self.gate
        gate = float(gate)

        result = [None] * len(observations)
        if not self.ids:
            return result

        candidates = []
        for obs_index, observation in enumerate(observations):
            if not observation.is_finite():
                continue
            z = np.array([float(observation.range), wrap_angle(observation.bearing)])
            R = self.measurement_covariance(observation)
            for landmark_id in self.ids:
                h, H = self.predict_observation(landmark_id)
                innovation = z - h
                innovation[1] = wrap_angle(innovation[1])
                S = H @ self.Sigma @ H.T + R
                try:
                    solved = np.linalg.solve(S, innovation)
                except np.linalg.LinAlgError:  # pragma: no cover - degenerate S
                    continue
                distance = float(innovation @ solved)
                if math.isfinite(distance) and distance <= gate:
                    candidates.append((distance, obs_index, landmark_id))

        candidates.sort(key=lambda item: item[0])
        used_observations = set()
        used_landmarks = set()
        for distance, obs_index, landmark_id in candidates:
            if obs_index in used_observations or landmark_id in used_landmarks:
                continue
            used_observations.add(obs_index)
            used_landmarks.add(landmark_id)
            result[obs_index] = landmark_id
        return result

    # ------------------------------------------------------------------
    # Landmark update and initialisation
    # ------------------------------------------------------------------
    def update_landmarks(self, observations, associations):
        """Batch update with the associated landmark observations.

        ``associations`` is the output of :meth:`associate`; ``None`` entries
        are ignored, so unmatched observations must be passed to
        :meth:`add_landmark` instead.
        """
        if len(associations) != len(observations):
            raise ValueError(
                'expected %d associations, got %d'
                % (len(observations), len(associations))
            )
        pairs = [
            (observation, association)
            for observation, association in zip(observations, associations)
            if association is not None
        ]
        if not pairs:
            return self.mu.copy(), self.Sigma.copy()

        n = self.state_dim
        m = len(pairs)
        H = np.zeros((2 * m, n))
        innovation = np.zeros(2 * m)
        R = np.zeros((2 * m, 2 * m))
        for row, (observation, landmark_id) in enumerate(pairs):
            h, H_i = self.predict_observation(landmark_id)
            H[2 * row:2 * row + 2, :] = H_i
            innovation[2 * row] = float(observation.range) - h[0]
            innovation[2 * row + 1] = wrap_angle(
                float(observation.bearing) - h[1]
            )
            R[2 * row:2 * row + 2, 2 * row:2 * row + 2] = (
                self.measurement_covariance(observation)
            )
        self._correct(innovation, H, R)
        return self.mu.copy(), self.Sigma.copy()

    def add_landmark(self, observation, R=None, position=None):
        """Initialise a new landmark from an unmatched observation.

        ``R`` is the optional ``2x2`` polar measurement covariance; by default
        it is built from ``sigma_range``/``sigma_bearing`` and the observation's
        own variances (see :meth:`measurement_covariance`). ``position``
        overrides the landmark position (used by the confirmation buffer, which
        averages several sightings). The diagonal of the new block is floored by
        ``initial_landmark_std**2``, so a single observation cannot make a
        landmark overconfident even when the detector reports a tiny variance.
        Returns the new id, or ``None`` when the observation is invalid or the
        state already holds ``max_landmarks`` landmarks.
        """
        if not observation.is_finite():
            return None
        if self.landmark_count >= self.max_landmarks:
            return None

        x, y, theta = self.mu[:ROBOT_DIM]
        psi = float(theta) + float(observation.bearing)
        cos_psi, sin_psi = math.cos(psi), math.sin(psi)
        distance = float(observation.range)

        # Inverse observation model: position of the landmark in the map frame.
        if position is None:
            lx = x + distance * cos_psi
            ly = y + distance * sin_psi
        else:
            lx, ly = (float(value) for value in position)

        # Jacobian w.r.t. the robot pose and the polar measurement noise.
        G = np.array([
            [1.0, 0.0, -distance * sin_psi],
            [0.0, 1.0, distance * cos_psi],
        ])
        M = np.array([
            [cos_psi, -distance * sin_psi],
            [sin_psi, distance * cos_psi],
        ])
        if R is None:
            R = self.measurement_covariance(observation)
        else:
            R = np.asarray(R, dtype=float).reshape(LANDMARK_DIM, LANDMARK_DIM)

        n = self.state_dim
        block = G @ self.Sigma[:ROBOT_DIM, :ROBOT_DIM] @ G.T + M @ R @ M.T
        floor = self.initial_landmark_std ** 2
        block[np.diag_indices(LANDMARK_DIM)] = np.maximum(
            np.diag(block), floor
        )

        # Cross-covariance of the new landmark with the whole existing state.
        cross = G @ self.Sigma[:ROBOT_DIM, :]

        augmented = np.zeros((n + LANDMARK_DIM, n + LANDMARK_DIM))
        augmented[:n, :n] = self.Sigma
        augmented[n:, :n] = cross
        augmented[:n, n:] = cross.T
        augmented[n:, n:] = block

        new_id = max(self.ids) + 1 if self.ids else 0
        self.ids.append(new_id)
        self.mu = np.concatenate([self.mu, [lx, ly]])
        self.Sigma = augmented
        self._symmetrize()
        self._check_finite()
        return new_id

    def confirm_observation(self, observation):
        """Feed an unmatched observation to the confirmation buffer.

        Returns the new landmark id once the candidate has been seen
        ``min_landmark_hits`` times; ``None`` while it is still tentative, when
        the observation is invalid or when the state is full. With
        ``min_landmark_hits == 1`` the landmark is created immediately.
        """
        if not observation.is_finite():
            return None
        if self.landmark_count >= self.max_landmarks:
            return None
        pose_x, pose_y, pose_theta = self.mu[:ROBOT_DIM]
        psi = float(pose_theta) + float(observation.bearing)
        candidate_x = pose_x + float(observation.range) * math.cos(psi)
        candidate_y = pose_y + float(observation.range) * math.sin(psi)

        confirmed, landmark_x, landmark_y = self.tentative.update(
            candidate_x, candidate_y
        )
        if not confirmed:
            return None
        return self.add_landmark(
            observation, position=(landmark_x, landmark_y)
        )

    def process_observations(self, observations):
        """Associate, update and initialise landmarks in one step.

        Returns ``(associations, added_ids)``: the association of every
        observation (``None`` when unmatched) and the ids of the landmarks that
        were confirmed during this scan.
        """
        associations = self.associate(observations)
        self.update_landmarks(observations, associations)
        added = []
        for observation, association in zip(observations, associations):
            if association is not None:
                continue
            new_id = self.confirm_observation(observation)
            if new_id is not None:
                added.append(new_id)
        return associations, added

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _correct(self, innovation, H, R):
        """Joseph-form EKF update shared by all measurement types."""
        innovation = np.asarray(innovation, dtype=float).reshape(-1)
        H = np.asarray(H, dtype=float)
        R = np.asarray(R, dtype=float)

        S = H @ self.Sigma @ H.T + R
        PHt = self.Sigma @ H.T
        try:
            K = np.linalg.solve(S, PHt.T).T
        except np.linalg.LinAlgError:  # pragma: no cover - degenerate S
            K = PHt @ np.linalg.pinv(S)

        self.mu = self.mu + K @ innovation
        self.mu[2] = wrap_angle(self.mu[2])

        identity = np.eye(self.state_dim)
        with np.errstate(divide='ignore', over='ignore', invalid='ignore'):
            A = identity - K @ H
            self.Sigma = A @ self.Sigma @ A.T + K @ R @ K.T
        self._symmetrize()
        self._check_finite()

    def _check_finite(self):
        if not (np.all(np.isfinite(self.mu)) and np.all(np.isfinite(self.Sigma))):
            raise FloatingPointError('EKF-SLAM state became non-finite')

    def _symmetrize(self):
        self.Sigma = 0.5 * (self.Sigma + self.Sigma.T)


def select_noise(noise, valid):
    """Measurement-noise covariance for the valid subset of measurements.

    Accepts a scalar variance, a per-anchor variance vector or a full
    covariance matrix; ``valid`` is the boolean mask of the observed entries.
    """
    arr = np.asarray(noise, dtype=float)
    n = len(valid)
    n_valid = int(np.count_nonzero(valid))
    if arr.ndim == 0:
        return np.eye(n_valid) * float(arr)
    if arr.ndim == 1:
        if len(arr) != n:
            raise ValueError('per-anchor noise must have %d entries' % n)
        return np.diag(arr[valid])
    if arr.shape != (n, n):
        raise ValueError('noise matrix must be %dx%d' % (n, n))
    return arr[np.ix_(valid, valid)]
