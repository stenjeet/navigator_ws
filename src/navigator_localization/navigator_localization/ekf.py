"""Extended Kalman Filter for 2D pose estimation from UWB ranges (stage 5).

The filter keeps the planar pose ``[x, y, theta]`` of ``base_footprint`` in the
``map`` frame:

* prediction uses the differential-drive motion model with the commanded
  ``(v, omega)`` as control input;
* correction uses the ranges to the known UWB anchors.

The module has no ROS dependencies so it can be unit-tested directly. The ROS
wrapper lives in ``ekf_node.py``.
"""

import math

import numpy as np

STATE_DIM = 3


def wrap_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class EKFLocalization:
    """Planar EKF: state ``[x, y, theta]``, measurements are UWB ranges."""

    def __init__(
        self,
        initial_pose,
        initial_covariance,
        tag_offset=(0.0, 0.0, 0.0),
        use_tag_offset=True,
        wrap_theta=True,
    ):
        self.x = np.asarray(initial_pose, dtype=float).reshape(STATE_DIM)
        self.P = np.asarray(initial_covariance, dtype=float).reshape(
            STATE_DIM, STATE_DIM
        )
        self.tag_offset = tuple(float(v) for v in tag_offset)
        self.use_tag_offset = bool(use_tag_offset)
        self.wrap_theta = bool(wrap_theta)

    # ------------------------------------------------------------------
    # Measurement model
    # ------------------------------------------------------------------
    def _offset(self):
        if self.use_tag_offset:
            return self.tag_offset
        return 0.0, 0.0, 0.0

    def _tag_xy(self):
        """World position of the UWB tag for the current pose estimate."""
        x, y, theta = self.x
        dx, dy, _ = self._offset()
        c, s = math.cos(theta), math.sin(theta)
        return x + dx * c - dy * s, y + dx * s + dy * c

    def measurement_and_jacobian(self, anchor):
        """Return ``(range, H)`` for an anchor ``(ax, ay, az)``.

        ``H`` is the 1x3 Jacobian of the range w.r.t. ``[x, y, theta]``.
        """
        ax, ay, az = anchor
        theta = self.x[2]
        dx, dy, dz = self._offset()
        c, s = math.cos(theta), math.sin(theta)

        tx, ty = self._tag_xy()
        ex = tx - ax
        ey = ty - ay
        ez = dz - az
        r = math.sqrt(ex * ex + ey * ey + ez * ez)
        if r < 1e-9:
            r = 1e-9

        dr_dtheta = (ex * (-dx * s - dy * c) + ey * (dx * c - dy * s)) / r
        H = np.array([[ex / r, ey / r, dr_dtheta]], dtype=float)
        return r, H

    # ------------------------------------------------------------------
    # EKF steps
    # ------------------------------------------------------------------
    def predict(self, v, omega, dt, process_noise):
        """Advance the state with the differential-drive model."""
        theta = self.x[2]
        c, s = math.cos(theta), math.sin(theta)

        self.x = np.array(
            [
                self.x[0] + v * dt * c,
                self.x[1] + v * dt * s,
                self.x[2] + omega * dt,
            ],
            dtype=float,
        )
        if self.wrap_theta:
            self.x[2] = wrap_angle(self.x[2])

        F = np.array(
            [
                [1.0, 0.0, -v * dt * s],
                [0.0, 1.0, v * dt * c],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        Q = np.asarray(process_noise, dtype=float).reshape(STATE_DIM, STATE_DIM)
        self.P = F @ self.P @ F.T + Q
        self._symmetrize()
        return self.x.copy(), self.P.copy()

    def update(self, ranges, anchors, measurement_noise):
        """Correct the state with one range per anchor (NaN = dropout)."""
        z = np.asarray(ranges, dtype=float).reshape(-1)
        if len(anchors) != len(z):
            raise ValueError(
                'expected %d anchors, got %d' % (len(z), len(anchors))
            )

        valid = np.isfinite(z)
        n_valid = int(np.count_nonzero(valid))
        if n_valid == 0:
            return self.x.copy(), self.P.copy()

        H = np.zeros((n_valid, STATE_DIM), dtype=float)
        h = np.zeros(n_valid, dtype=float)
        row = 0
        for i in range(len(z)):
            if not valid[i]:
                continue
            r, H_i = self.measurement_and_jacobian(anchors[i])
            h[row] = r
            H[row, :] = H_i
            row += 1

        R = self._select_noise(measurement_noise, valid)
        innovation = z[valid] - h

        S = H @ self.P @ H.T + R
        PHt = self.P @ H.T
        # K = P H^T S^-1, solved without an explicit inverse.
        K = np.linalg.solve(S, PHt.T).T

        self.x = self.x + K @ innovation
        if self.wrap_theta:
            self.x[2] = wrap_angle(self.x[2])

        # Joseph form keeps P symmetric positive semi-definite.
        identity = np.eye(STATE_DIM)
        A = identity - K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T
        self._symmetrize()
        return self.x.copy(), self.P.copy()

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------
    @staticmethod
    def _select_noise(noise, valid):
        """Build the measurement-noise covariance for the valid subset.

        Accepts a scalar variance, a per-anchor variance vector or a full
        covariance matrix; ``valid`` is the boolean mask of observed ranges.
        """
        arr = np.asarray(noise, dtype=float)
        n = len(valid)
        if arr.ndim == 0:
            return np.eye(int(np.count_nonzero(valid))) * float(arr)
        if arr.ndim == 1:
            if len(arr) != n:
                raise ValueError('per-anchor noise must have %d entries' % n)
            return np.diag(arr[valid])
        if arr.shape != (n, n):
            raise ValueError('noise matrix must be %dx%d' % (n, n))
        return arr[np.ix_(valid, valid)]

    def _symmetrize(self):
        self.P = 0.5 * (self.P + self.P.T)
