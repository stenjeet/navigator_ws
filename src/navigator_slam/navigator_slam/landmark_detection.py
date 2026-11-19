"""Landmark extraction and data association (course project stage 7).

The detector turns one 2D lidar scan into a short list of circular landmarks and
the associator matches those observations against an existing landmark map. Both
live here without ROS imports, so they can be unit-tested on a machine without
ROS; the ROS wrapper is ``landmark_node.py``.

Extraction pipeline
-------------------
1. Every finite beam inside ``[min_range, max_range]`` becomes a point in the
   ``base_footprint`` frame. The lidar is mounted at ``lidar_offset`` with
   identity rotation, so a point is ``offset_xy + range * (cos, sin)(beam)``.
2. Beams are split into clusters wherever the range jumps by more than
   ``jump_threshold + |angle_increment| * mean_range`` (an adaptive threshold:
   a metric floor for the object depth change plus the tangential step of the
   scan). Invalid beams (``inf``/``NaN``, out of range) break a cluster, but up
   to ``MAX_SCAN_GAP`` of them are bridged so a lidar dropout does not fragment
   a landmark. A full 360 degree scan is cyclic, so the first and last cluster
   are merged when they are neighbours across the scan boundary.
3. A cluster is a landmark candidate when it holds ``min_points..max_points``
   points, its width is below ``max_cluster_width`` and it passes the circle
   test: a circle must exist with a radius inside ``radius_min..radius_max``
   and a radial RMS residual below ``max_residual``. A landmark of radius
   0.09 m seen from 1.5-8 m is only a few beams wide, so the algebraic (Kasa)
   least-squares fit is ill-conditioned there; such clusters use the
   chord/sagitta arc fit instead (:func:`fit_arc`), which does not need a
   well-conditioned normal equation. A flat wall has (almost) no sagitta, so
   its arc radius blows up and the cluster is rejected; the same radius limits
   remove the 0.30/0.42 m cylinders, and ``max_cluster_width`` removes the
   walls, the box wall and the triangle. The Kasa fit is only trusted when
   the cluster spans at least ``MIN_FIT_SPAN`` radians.

Observation covariance
----------------------
The measurement noise documented for the lidar model is ``sigma_range = 0.02 m``
plus a ``0.01 m`` systematic bias; the bias is treated as an additional
independent 1-sigma term. With ``n`` points, radial RMS residual ``res`` and
cluster half-width ``h`` the reported variances are

    range_variance   = (sigma_range^2 + bias^2) / n + res^2 + h^2
    bearing_variance = sigma_bearing^2
                       + (sigma_range^2 + bias^2) / (n * range^2)
                       + angle_increment^2 / 12

``h^2`` covers the radial centre uncertainty of a short arc (bounded by the
landmark radius, about 0.09 m) and the last term is the beam quantisation. For a
well-conditioned fit ``h^2`` is replaced by the conditioning term
``(sigma_range^2 + bias^2) / (n * sin^2(span / 2))``. Both variances are clamped
to be finite and strictly positive.

Association
-----------
``associate_observations`` predicts every known landmark in the robot frame and
forms the innovation covariance ``S = H P H^T + R`` with the 3x3 pose covariance
``P`` and ``R`` = observation noise + the landmark covariance rotated into the
robot frame (landmarks are held fixed here; the joint EKF-SLAM keeps their
cross-covariances in its own state). Pairs are accepted greedily in ascending
Mahalanobis distance (``d^2 = nu^T S^-1 nu``) while ``d^2 <= gate``; a landmark
id and an observation are used at most once per call.
"""

import math
from dataclasses import dataclass

import numpy as np

#: Systematic lidar range bias from the sensor model (metres, 1-sigma term).
LIDAR_BIAS = 0.01

#: Angular span (radians) below which the algebraic circle fit is considered
#: ill-conditioned; such clusters use the chord/sagitta arc fit instead.
MIN_FIT_SPAN = 0.25

#: Number of consecutive invalid beams bridged inside a cluster, so a lidar
#: dropout does not split a landmark or a wall into fragments.
MAX_SCAN_GAP = 2

#: Smallest admissible variance, so covariance matrices stay invertible.
_MIN_VARIANCE = 1e-9


@dataclass
class LandmarkObservation:
    """One landmark seen in a scan, in the ``base_footprint`` frame.

    ``range`` is the distance from the ``base_footprint`` origin to the fitted
    landmark centre and ``bearing`` is the angle relative to the robot heading,
    in ``(-pi, pi]``. ``range_variance`` [m^2] and ``bearing_variance`` [rad^2]
    are the 1-sigma variances of the two measurements.
    """

    range: float
    bearing: float
    radius: float
    range_variance: float
    bearing_variance: float


@dataclass
class LandmarkEstimate:
    """A landmark of the map: id, position in the map frame and 2x2 covariance."""

    id: int
    x: float
    y: float
    covariance: np.ndarray

    def __post_init__(self):
        self.id = int(self.id)
        self.x = float(self.x)
        self.y = float(self.y)
        self.covariance = np.asarray(self.covariance, dtype=float).reshape(2, 2)


@dataclass
class CircleFit:
    """Result of an algebraic circle fit, in the frame of the fitted points."""

    x: float
    y: float
    radius: float
    residual: float


@dataclass
class ScanCluster:
    """One scan cluster: its points in the base frame and its angular span."""

    points: np.ndarray
    span: float


@dataclass
class DetectionParams:
    """Detection thresholds; the defaults mirror ``config/landmarks.yaml``."""

    #: Metric part of the adaptive cluster jump threshold [m].
    jump_threshold: float = 0.15
    #: Admissible number of beams per cluster.
    min_points: int = 3
    max_points: int = 25
    #: Largest physical cluster width accepted as a small landmark [m]; the
    #: 0.09 m landmarks are never wider than 0.18 m, so wider clusters are
    #: walls or bigger obstacles.
    max_cluster_width: float = 0.20
    #: Admissible fitted landmark radius [m].
    radius_min: float = 0.04
    radius_max: float = 0.25
    #: Largest radial RMS residual of an accepted circle fit [m].
    max_residual: float = 0.05
    #: Beams beyond this range are ignored [m] (None disables the extra cap).
    max_range: float = 8.0
    #: Beams closer than this are ignored [m].
    min_range: float = 0.12
    #: Lidar range noise and bias budget used by the covariance model [m].
    sigma_range: float = 0.02
    #: Bearing noise floor used by the covariance model [rad].
    sigma_bearing: float = 0.02


def wrap_angle(angle):
    """Wrap an angle to ``(-pi, pi]``."""
    return math.atan2(math.sin(angle), math.cos(angle))


def fit_circle(points):
    """Algebraic least-squares circle fit (Kasa) of 2D ``points``.

    Returns a :class:`CircleFit` with the centre, radius and the radial RMS
    residual, or ``None`` when there are fewer than three points or the points
    are numerically collinear (the fit is then not defined).
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if points.shape[0] < 3:
        return None

    centred = points - points.mean(axis=0)
    singular = np.linalg.svd(centred, compute_uv=False)
    if singular[0] <= 1e-9 or singular[-1] <= 1e-9 * singular[0]:
        return None

    x = points[:, 0]
    y = points[:, 1]
    design = np.column_stack([2.0 * x, 2.0 * y, np.ones_like(x)])
    try:
        solution, _, rank, _ = np.linalg.lstsq(
            design, x * x + y * y, rcond=None
        )
    except np.linalg.LinAlgError:  # pragma: no cover - defensive
        return None
    if rank < 3:
        return None

    cx, cy = float(solution[0]), float(solution[1])
    radius_sq = float(solution[2]) + cx * cx + cy * cy
    if not math.isfinite(radius_sq) or radius_sq <= 0.0:
        return None

    radius = math.sqrt(radius_sq)
    residual = float(
        np.sqrt(np.mean((np.hypot(x - cx, y - cy) - radius) ** 2))
    )
    if not math.isfinite(residual):
        return None
    return CircleFit(x=cx, y=cy, radius=radius, residual=residual)


def fit_arc(points):
    """Circle through a short arc, from its chord and sagitta.

    ``points`` are ordered along the scan. The chord joins the first and the
    last point, the sagitta is the offset of the middle point from the chord
    (positive towards the lidar, which sits near the origin). A short arc does
    not resolve the radius in a least-squares fit, but the chord/sagitta
    relation ``h = R (1 - cos(beta))``, ``c = 2 R sin(beta)`` does: it gives
    the radius and the centre without a singular linear system.

    Returns ``(x, y, radius)`` of the centre in the point frame, or ``None``
    when the arc is degenerate (zero chord, no bulge towards the lidar).
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if points.shape[0] < 3:
        return None

    chord = points[-1] - points[0]
    length = float(np.linalg.norm(chord))
    if length <= 1e-9:
        return None

    normal = np.array([-chord[1], chord[0]], dtype=float) / length
    middle = 0.5 * (points[0] + points[-1])
    if float(normal @ middle) < 0.0:
        normal = -normal       # normal points away from the lidar
    sagitta = float((points[points.shape[0] // 2] - middle) @ (-normal))
    if sagitta <= 0.0:
        return None

    beta = 2.0 * math.atan(2.0 * sagitta / length)
    if not 0.0 < beta < math.pi:
        return None
    radius = length / (2.0 * math.sin(beta))
    if not math.isfinite(radius) or radius <= 0.0:
        return None

    centre = middle + (radius - sagitta) * normal
    return float(centre[0]), float(centre[1]), float(radius)


def _range_limits(range_min, range_max, params):
    low = max(float(range_min), float(params.min_range))
    high = float(range_max)
    if params.max_range is not None:
        high = min(high, float(params.max_range))
    return low, high


def _jump_limit(range_a, range_b, angle_step, params):
    """Largest range jump that still joins two beams ``angle_step`` apart."""
    noise_floor = 3.0 * float(params.sigma_range)
    step = abs(float(angle_step)) * 0.5 * (range_a + range_b)
    return max(float(params.jump_threshold), noise_floor) + step


def _is_full_circle(count, angle_increment):
    increment = abs(float(angle_increment))
    if count < 3 or increment <= 0.0:
        return False
    return abs(count * increment - 2.0 * math.pi) <= 0.5 * increment


def segment_clusters(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    lidar_offset=(0.18, 0.0, 0.34),
    params=DetectionParams(),
):
    """Split a scan into clusters of points in the ``base_footprint`` frame.

    Returns a list of :class:`ScanCluster`. Clusters follow the beam order;
    up to :data:`MAX_SCAN_GAP` consecutive invalid beams are bridged so that a
    lidar dropout does not cut a landmark (or a wall) into fragments. A full
    360 degree scan is closed cyclically, so a landmark on the ``+x`` axis is
    not cut in two at ``bearing = 0``.
    """
    ranges = np.asarray(ranges, dtype=float).reshape(-1)
    count = ranges.size
    if count == 0:
        return []

    low, high = _range_limits(range_min, range_max, params)
    valid = np.isfinite(ranges) & (ranges >= low) & (ranges <= high)
    if not valid.any():
        return []

    increment = float(angle_increment)
    groups = []
    current = []
    gap = 0
    for beam in range(count):
        if not valid[beam]:
            if current:
                gap += 1
                if gap > MAX_SCAN_GAP:
                    groups.append(current)
                    current = []
                    gap = 0
            continue
        if current:
            jump = abs(ranges[beam] - ranges[current[-1]])
            step = (gap + 1) * increment
            if jump > _jump_limit(ranges[current[-1]], ranges[beam], step,
                                  params):
                groups.append(current)
                current = []
        current.append(beam)
        gap = 0
    if current:
        groups.append(current)

    # Close the ring: the last beam and the first beam are neighbours.
    if len(groups) > 1 and _is_full_circle(count, increment):
        head, tail = groups[0], groups[-1]
        ring_gap = (head[0] - tail[-1]) % count - 1
        if ring_gap <= MAX_SCAN_GAP:
            step = (ring_gap + 1) * increment
            jump = abs(ranges[head[0]] - ranges[tail[-1]])
            if jump <= _jump_limit(ranges[tail[-1]], ranges[head[0]], step,
                                   params):
                groups = [tail + head] + groups[1:-1]

    angles = float(angle_min) + np.arange(count) * increment
    dx = float(lidar_offset[0])
    dy = float(lidar_offset[1])
    clusters = []
    for group in groups:
        beams = np.asarray(group, dtype=int)
        radii = ranges[beams]
        phi = angles[beams]
        # Angular extent from the first to the last beam, cyclic-safe.
        span_beams = int((beams[-1] - beams[0]) % count)
        clusters.append(
            ScanCluster(
                points=np.column_stack(
                    [dx + radii * np.cos(phi), dy + radii * np.sin(phi)]
                ),
                span=span_beams * abs(increment),
            )
        )
    return clusters


def _observation_from_cluster(cluster, angle_increment, params):
    """Turn one cluster into a landmark observation, or ``None`` to reject it."""
    points = cluster.points
    count = points.shape[0]
    if count < int(params.min_points) or count > int(params.max_points):
        return None

    centroid = points.mean(axis=0)
    width = 2.0 * float(np.max(np.linalg.norm(points - centroid, axis=1)))
    if width > float(params.max_cluster_width):
        return None

    increment = abs(float(angle_increment))
    span = float(cluster.span)
    point_var = float(params.sigma_range) ** 2 + LIDAR_BIAS ** 2

    fit = fit_circle(points)
    if span >= MIN_FIT_SPAN:
        # Enough beams to resolve the curvature: trust the fit or reject the
        # cluster. This is where walls and the big cylinders are thrown away.
        if fit is None:
            return None
        if not (params.radius_min <= fit.radius <= params.radius_max):
            return None
        if fit.residual > float(params.max_residual):
            return None
        centre = np.array([fit.x, fit.y], dtype=float)
        radius = fit.radius
        residual = fit.residual
        conditioning = point_var / max(
            count * math.sin(0.5 * span) ** 2, 1e-6
        )
    else:
        # Short arc: the least-squares fit is ill-conditioned. Use the
        # chord/sagitta arc fit instead, which recovers the radius of a small
        # cylinder from the curvature of the cluster; a wall has no sagitta
        # and yields an out-of-limit radius, so it is rejected here.
        arc = fit_arc(points)
        if arc is None:
            return None
        arc_x, arc_y, radius = arc
        if not (params.radius_min <= radius <= params.radius_max):
            return None
        centre = np.array([arc_x, arc_y], dtype=float)
        residual = float(
            np.sqrt(np.mean((np.linalg.norm(points - centre, axis=1) - radius) ** 2))
        )
        conditioning = 0.25 * width * width

    distance = float(np.linalg.norm(centre))
    if not math.isfinite(distance) or distance <= 0.0:
        return None
    if distance < float(params.min_range):
        return None
    if params.max_range is not None and distance > float(params.max_range):
        return None

    bearing = math.atan2(float(centre[1]), float(centre[0]))
    range_variance = (
        point_var / count + residual * residual + conditioning
    )
    bearing_variance = (
        float(params.sigma_bearing) ** 2
        + point_var / (count * max(distance, float(params.min_range)) ** 2)
        + increment * increment / 12.0
    )
    return LandmarkObservation(
        range=distance,
        bearing=float(wrap_angle(bearing)),
        radius=float(radius),
        range_variance=float(max(range_variance, _MIN_VARIANCE)),
        bearing_variance=float(max(bearing_variance, _MIN_VARIANCE)),
    )


def extract_landmarks(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    lidar_offset=(0.18, 0.0, 0.34),
    params=DetectionParams(),
):
    """Extract circular landmark observations from one lidar scan.

    ``ranges`` are the beam ranges in the ``lidar_link`` frame (``inf``/``NaN``
    marks a dropout, beams outside ``[range_min, range_max]`` are ignored).
    Observations are expressed in ``base_footprint``: ``range`` is measured from
    the robot origin and ``bearing`` from the robot heading, so the small lidar
    offset is already compensated. Returns a list of
    :class:`LandmarkObservation`, possibly empty.
    """
    observations = []
    for cluster in segment_clusters(
        ranges,
        angle_min,
        angle_increment,
        range_min,
        range_max,
        lidar_offset=lidar_offset,
        params=params,
    ):
        observation = _observation_from_cluster(cluster, angle_increment, params)
        if observation is not None:
            observations.append(observation)
    return observations


def _landmark_fields(landmark):
    """``(id, x, y, covariance)`` of a :class:`LandmarkEstimate` or 4-tuple."""
    if hasattr(landmark, 'id'):
        return (
            int(landmark.id),
            float(landmark.x),
            float(landmark.y),
            np.asarray(landmark.covariance, dtype=float).reshape(2, 2),
        )
    identifier, x, y, covariance = landmark
    return (
        int(identifier),
        float(x),
        float(y),
        np.asarray(covariance, dtype=float).reshape(2, 2),
    )


def _predict(pose, landmark):
    """Range-bearing prediction and Jacobian of one landmark in the robot frame.

    ``pose`` is ``(x, y, yaw)`` of ``base_footprint`` in the map frame. Returns
    ``(prediction, H)`` with ``prediction = [range, bearing]`` and ``H`` the 2x3
    Jacobian w.r.t. ``[x, y, yaw]``, or ``None`` for a degenerate range.
    """
    x, y, yaw = (float(v) for v in pose)
    _, lx, ly, _ = _landmark_fields(landmark)
    dx = lx - x
    dy = ly - y
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    robot_x = cos_yaw * dx + sin_yaw * dy
    robot_y = -sin_yaw * dx + cos_yaw * dy
    distance = math.hypot(robot_x, robot_y)
    if distance <= 1e-6:
        return None

    bearing = math.atan2(robot_y, robot_x)
    inverse_sq = 1.0 / (distance * distance)
    H = np.array(
        [
            [-dx / distance, -dy / distance, 0.0],
            [dy * inverse_sq, -dx * inverse_sq, -1.0],
        ],
        dtype=float,
    )
    return np.array([distance, bearing], dtype=float), H


def associate_observations(
    observations,
    robot_pose,
    landmarks,
    pose_covariance,
    gate=9.21,
    max_range=None,
):
    """Associate observations with known landmarks.

    ``robot_pose`` is ``(x, y, yaw)`` in the map frame, ``landmarks`` is a
    sequence of :class:`LandmarkEstimate` (objects with ``id``/``x``/``y``/
    ``covariance``) and ``pose_covariance`` the 3x3 covariance of the pose.
    Each known landmark is predicted in the robot frame, the innovation
    covariance is ``S = H P H^T + R`` with ``R`` = observation noise plus the
    landmark covariance rotated into the robot frame and mapped into the
    ``(range, bearing)`` plane (inverse polar Jacobian), and pairs are accepted
    greedily by ascending Mahalanobis distance while ``d^2 <= gate``.

    Returns one entry per observation: the matched landmark id or ``None``.
    ``max_range`` drops observations farther away than the sensor can see.
    """
    matches = [None] * len(observations)
    if not observations or not landmarks:
        return matches

    pose = tuple(float(v) for v in robot_pose)
    covariance = np.asarray(pose_covariance, dtype=float).reshape(3, 3)
    cos_yaw, sin_yaw = math.cos(pose[2]), math.sin(pose[2])
    to_robot = np.array(
        [[cos_yaw, sin_yaw], [-sin_yaw, cos_yaw]], dtype=float
    )

    limit = float('inf') if max_range is None else float(max_range)

    # Per landmark: the range-bearing prediction and the part of S that does
    # not depend on the observation (pose + landmark covariance). The landmark
    # covariance lives in metres; it enters the polar innovation covariance
    # through the inverse of the polar Jacobian.
    predictions = []
    for landmark in landmarks:
        predicted = _predict(pose, landmark)
        if predicted is None:
            continue
        prediction, H = predicted
        distance, bearing = prediction
        if distance > limit or distance < 1e-3:
            continue
        _, _, _, landmark_covariance = _landmark_fields(landmark)
        cos_bearing, sin_bearing = math.cos(bearing), math.sin(bearing)
        polar = np.array(
            [
                [cos_bearing, -distance * sin_bearing],
                [sin_bearing, distance * cos_bearing],
            ],
            dtype=float,
        )
        inverse_polar = np.linalg.inv(polar)
        landmark_noise = (
            inverse_polar
            @ (to_robot @ landmark_covariance @ to_robot.T)
            @ inverse_polar.T
        )
        marginal = H @ covariance @ H.T + landmark_noise
        marginal = 0.5 * (marginal + marginal.T)
        if not np.all(np.isfinite(marginal)):
            continue
        predictions.append(
            (_landmark_fields(landmark)[0], prediction, marginal)
        )

    candidates = []
    for index, observation in enumerate(observations):
        range_variance = max(float(observation.range_variance), _MIN_VARIANCE)
        bearing_variance = max(
            float(observation.bearing_variance), _MIN_VARIANCE
        )
        if not (math.isfinite(range_variance) and math.isfinite(bearing_variance)):
            continue
        if not math.isfinite(observation.range):
            continue
        if observation.range > limit or observation.range <= 0.0:
            continue
        for landmark_id, prediction, marginal in predictions:
            innovation = np.array(
                [
                    float(observation.range) - prediction[0],
                    wrap_angle(float(observation.bearing) - prediction[1]),
                ],
                dtype=float,
            )
            innovation_cov = marginal + np.diag(
                [range_variance, bearing_variance]
            )
            if np.linalg.det(innovation_cov) <= 1e-12:
                continue
            distance_sq = float(
                innovation @ np.linalg.solve(innovation_cov, innovation)
            )
            if math.isfinite(distance_sq) and distance_sq <= float(gate):
                candidates.append((distance_sq, index, landmark_id))

    candidates.sort()
    used_landmarks = set()
    for distance_sq, index, landmark_id in candidates:
        if matches[index] is not None or landmark_id in used_landmarks:
            continue
        matches[index] = landmark_id
        used_landmarks.add(landmark_id)
    return matches
