"""Unit tests for landmark extraction and data association (stage 7).

Run locally from the repository root::

    PYTHONPATH=src/navigator_slam:src/navigator_sim \
        python3 -m unittest discover -s src/navigator_slam/test -v

The polygon tests ray-cast against ``navigator_sim.world`` with an independent
oracle (the same pattern as ``test_occupancy_grid.py``), so a systematic error
in the projection or in the scan model cannot be hidden by reusing the
simulation code.
"""

import math
import os
import unittest

import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML is a ROS dependency
    yaml = None

from navigator_slam.landmark_detection import (
    DetectionParams,
    LandmarkEstimate,
    LandmarkObservation,
    _predict,
    associate_observations,
    extract_landmarks,
    fit_arc,
    fit_circle,
    segment_clusters,
    wrap_angle,
)

try:
    from navigator_sim import world
except ImportError:  # pragma: no cover - standalone run without navigator_sim
    world = None

BEAMS = 360
INCREMENT = 2.0 * math.pi / BEAMS
RANGE_MIN = 0.12
RANGE_MAX = 8.0
LIDAR_OFFSET = (0.18, 0.00, 0.34)

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(os.path.dirname(HERE), 'config', 'landmarks.yaml')


# --------------------------------------------------------------------------
# Independent geometry oracle
# --------------------------------------------------------------------------
def ray_aabb(ox, oy, dx, dy, xmin, xmax, ymin, ymax):
    """Ray/box test (slab method). Returns the entry distance or ``None``."""
    t_enter, t_exit = -math.inf, math.inf
    for origin, direction, low, high in (
        (ox, dx, xmin, xmax),
        (oy, dy, ymin, ymax),
    ):
        if abs(direction) < 1e-12:
            if origin < low or origin > high:
                return None
            continue
        t1 = (low - origin) / direction
        t2 = (high - origin) / direction
        if t1 > t2:
            t1, t2 = t2, t1
        t_enter = max(t_enter, t1)
        t_exit = min(t_exit, t2)
    if t_enter > t_exit or t_exit < 0.0:
        return None
    return t_enter if t_enter >= 0.0 else 0.0


def ray_circle(ox, oy, dx, dy, cx, cy, radius):
    """Ray/circle test. Returns the nearest non-negative hit or ``None``."""
    ex, ey = ox - cx, oy - cy
    b = 2.0 * (dx * ex + dy * ey)
    c = ex * ex + ey * ey - radius * radius
    discriminant = b * b - 4.0 * c
    if discriminant < 0.0:
        return None
    root = math.sqrt(discriminant)
    hits = [(-b - root) / 2.0, (-b + root) / 2.0]
    hits = [t for t in hits if t >= 0.0]
    return min(hits) if hits else None


def ray_segment(ox, oy, dx, dy, x1, y1, x2, y2):
    """Ray/segment test. Returns the distance or ``None``."""
    rx, ry = x2 - x1, y2 - y1
    denominator = dx * ry - dy * rx
    if abs(denominator) < 1e-12:
        return None
    ex, ey = x1 - ox, y1 - oy
    t = (ex * ry - ey * rx) / denominator
    if t < 0.0:
        return None
    s = (ex * dy - ey * dx) / denominator
    if not 0.0 <= s <= 1.0:
        return None
    return t


def lidar_position(pose, lidar_offset):
    """Planar position of the lidar for a base pose ``(x, y, yaw)``."""
    x, y, yaw = pose
    dx, dy, _ = lidar_offset
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return x + dx * cos_yaw - dy * sin_yaw, y + dx * sin_yaw + dy * cos_yaw


def polygon_scan(pose, lidar_offset=None, beams=BEAMS):
    """Full 360 beam scan of the test polygon from ``pose`` (oracle ray caster)."""
    if lidar_offset is None:
        lidar_offset = world.LIDAR_OFFSET
    lx, ly = lidar_position(pose, lidar_offset)
    increment = 2.0 * math.pi / beams
    boxes = list(world.WALLS) + [box[:4] for box in world.BOX_OBSTACLES]
    circles = [(x, y, r) for (x, y, r, _) in world.CYLINDER_OBSTACLES]
    circles += [(x, y, r) for (_, x, y, r, _, _) in world.LANDMARKS]
    vertices = world.TRIANGLE_OBSTACLE['vertices']

    ranges = []
    for beam in range(beams):
        phi = pose[2] + beam * increment
        dx, dy = math.cos(phi), math.sin(phi)
        best = math.inf
        for xmin, xmax, ymin, ymax in boxes:
            hit = ray_aabb(lx, ly, dx, dy, xmin, xmax, ymin, ymax)
            if hit is not None:
                best = min(best, hit)
        for cx, cy, radius in circles:
            hit = ray_circle(lx, ly, dx, dy, cx, cy, radius)
            if hit is not None:
                best = min(best, hit)
        for index in range(3):
            x1, y1 = vertices[index]
            x2, y2 = vertices[(index + 1) % 3]
            hit = ray_segment(lx, ly, dx, dy, x1, y1, x2, y2)
            if hit is not None:
                best = min(best, hit)
        ranges.append(best if best <= RANGE_MAX else math.inf)
    return ranges, increment


def circle_scan(centre, radius, lidar_offset=(0.0, 0.0, 0.0), beams=BEAMS):
    """Scan of a single circular object (no walls) from ``lidar_offset``."""
    lx, ly = lidar_offset[0], lidar_offset[1]
    increment = 2.0 * math.pi / beams
    ranges = []
    for beam in range(beams):
        phi = beam * increment
        hit = ray_circle(lx, ly, math.cos(phi), math.sin(phi),
                         centre[0], centre[1], radius)
        ranges.append(hit if hit is not None else math.inf)
    return ranges, increment


def observation_to_base(observation):
    """``(x, y)`` of an observation in the base frame."""
    return (
        observation.range * math.cos(observation.bearing),
        observation.range * math.sin(observation.bearing),
    )


def observation_to_map(observation, pose):
    """``(x, y)`` of an observation in the map frame."""
    x, y, yaw = pose
    bx, by = observation_to_base(observation)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return x + cos_yaw * bx - sin_yaw * by, y + sin_yaw * bx + cos_yaw * by


class TestCircleFit(unittest.TestCase):
    def test_algebraic_fit_recovers_noisy_arc(self):
        angles = np.linspace(0.4, 2.4, 40)
        noise = np.random.default_rng(4).normal(0.0, 0.008, angles.size)
        centre = np.array([0.35, -0.20])
        radius = 0.32
        directions = np.column_stack([np.cos(angles), np.sin(angles)])
        points = centre + (radius + noise)[:, None] * directions

        fit = fit_circle(points)
        self.assertIsNotNone(fit)
        self.assertAlmostEqual(fit.x, centre[0], delta=0.02)
        self.assertAlmostEqual(fit.y, centre[1], delta=0.02)
        self.assertAlmostEqual(fit.radius, radius, delta=0.02)
        self.assertLess(fit.residual, 0.02)

    def test_collinear_and_degenerate_points_are_rejected(self):
        self.assertIsNone(fit_circle([[0.0, 0.0], [1.0, 0.0]]))
        collinear = [[float(t), 0.5 * float(t)] for t in (1.0, 2.0, 3.0, 4.0)]
        self.assertIsNone(fit_circle(collinear))
        self.assertIsNone(fit_circle([[1.0, 1.0]] * 4))

    def test_arc_fit_recovers_a_short_landmark_arc(self):
        # Three beams on a 0.09 m cylinder 4 m away: the least-squares fit is
        # ill-conditioned here, the chord/sagitta fit is not.
        distance, radius = 4.0, 0.09
        ranges = []
        for offset in (-INCREMENT, 0.0, INCREMENT):
            ranges.append(
                distance * math.cos(offset)
                - math.sqrt(radius ** 2 - distance ** 2 * math.sin(offset) ** 2)
            )
        points = np.array([
            [r * math.cos(k * INCREMENT), r * math.sin(k * INCREMENT)]
            for r, k in zip(ranges, (-1, 0, 1))
        ])
        arc = fit_arc(points)
        self.assertIsNotNone(arc)
        self.assertAlmostEqual(arc[0], distance, delta=0.02)
        self.assertAlmostEqual(arc[1], 0.0, delta=0.01)
        self.assertAlmostEqual(arc[2], radius, delta=0.01)


class TestExtraction(unittest.TestCase):
    def extract(self, ranges, angle_min=0.0, angle_increment=INCREMENT,
                lidar_offset=(0.0, 0.0, 0.0), params=None):
        return extract_landmarks(
            ranges,
            angle_min,
            angle_increment,
            RANGE_MIN,
            RANGE_MAX,
            lidar_offset=lidar_offset,
            params=params if params is not None else DetectionParams(),
        )

    def test_empty_and_invalid_scans_return_no_observations(self):
        self.assertEqual(self.extract([]), [])
        self.assertEqual(self.extract([math.inf] * BEAMS), [])
        self.assertEqual(self.extract([math.nan] * BEAMS), [])
        self.assertEqual(self.extract([0.05] * BEAMS), [])

    def test_dropouts_and_a_wrap_cluster_create_no_observation(self):
        ranges = [math.inf] * BEAMS
        ranges[0] = 3.0
        ranges[1] = math.nan
        ranges[359] = 3.0
        ranges[120] = 2.0
        ranges[121] = 2.0
        # Only two neighbouring beams per fragment: below min_points.
        self.assertEqual(self.extract(ranges), [])

    def test_wrap_around_cluster_is_one_observation(self):
        ranges, increment = circle_scan((2.0, 0.0), 0.09)
        observations = self.extract(ranges, angle_increment=increment)
        self.assertEqual(len(observations), 1)
        observation = observations[0]
        self.assertAlmostEqual(observation.range, 2.0, delta=0.02)
        self.assertAlmostEqual(observation.bearing, 0.0, delta=0.01)
        self.assertAlmostEqual(observation.radius, 0.09, delta=0.02)

    def test_short_arc_uses_the_chord_sagitta_fallback(self):
        # A 5-beam arc at 2 m with the inner beams pulled outwards: the
        # least-squares circle is far from the truth, the chord/sagitta fit
        # still recovers it and the observation is kept.
        distance, radius = 2.0, 0.09
        ranges = []
        for index in range(-2, 3):
            offset = index * INCREMENT
            ranges.append(
                distance * math.cos(offset)
                - math.sqrt(radius ** 2 - distance ** 2 * math.sin(offset) ** 2)
            )
        ranges[1] += 0.03
        ranges[3] += 0.03
        scan = [math.inf] * BEAMS
        for index, value in enumerate(ranges):
            scan[(index - 2) % BEAMS] = value

        clusters = [
            cluster for cluster in segment_clusters(
                scan, 0.0, INCREMENT, RANGE_MIN, RANGE_MAX
            )
            if cluster.points.shape[0] >= 3
        ]
        self.assertEqual(len(clusters), 1)
        cluster = clusters[0]
        least_squares = fit_circle(cluster.points)
        chord_sagitta = fit_arc(cluster.points)
        self.assertIsNotNone(least_squares)
        self.assertIsNotNone(chord_sagitta)
        # The least-squares radius is biased, the fallback is not.
        self.assertGreater(abs(least_squares.radius - radius), 0.02)
        self.assertLess(abs(chord_sagitta[2] - radius), 0.005)
        self.assertLess(cluster.span, 0.25)

        observations = self.extract(scan)
        self.assertEqual(len(observations), 1)
        x, y = observation_to_base(observations[0])
        self.assertAlmostEqual(x, distance, delta=0.05)
        self.assertAlmostEqual(y, 0.0, delta=0.03)
        self.assertTrue(0.04 <= observations[0].radius <= 0.25)
        self.assertTrue(math.isfinite(observations[0].range_variance))
        self.assertGreater(observations[0].range_variance, 0.0)
        self.assertGreater(observations[0].bearing_variance, 0.0)

    def test_wall_like_clusters_are_rejected(self):
        # Flat wall 4 m ahead: a straight line of points, no sagitta.
        wall = [math.inf] * BEAMS
        for index in range(170, 190):
            wall[index] = 4.0 / math.cos((index - 180) * INCREMENT)
        self.assertEqual(self.extract(wall), [])

        # A wide wall patch (more beams) must also be rejected.
        wide = [math.inf] * BEAMS
        for index in range(150, 211):
            wide[index] = 4.0 / math.cos((index - 180) * INCREMENT)
        self.assertEqual(self.extract(wide), [])

    def test_big_cylinders_are_not_landmarks(self):
        for centre, radius in (((3.0, 0.0), 0.45), ((2.0, 0.0), 0.35)):
            ranges, increment = circle_scan(centre, radius)
            self.assertEqual(self.extract(ranges, angle_increment=increment), [])

    def test_parameters_are_honoured(self):
        ranges, increment = circle_scan((2.0, 0.0), 0.09)
        params = DetectionParams(radius_max=0.05)
        self.assertEqual(
            self.extract(ranges, angle_increment=increment, params=params), []
        )
        params = DetectionParams(max_range=1.0)
        self.assertEqual(
            self.extract(ranges, angle_increment=increment, params=params), []
        )


@unittest.skipIf(world is None, 'navigator_sim is not on PYTHONPATH')
class TestAgainstPolygonGeometry(unittest.TestCase):
    """The detector must find the polygon landmarks and nothing bigger."""

    POSE = (6.0, 4.0, math.pi)

    def setUp(self):
        self.ranges, self.increment = polygon_scan(self.POSE)
        self.observations = extract_landmarks(
            self.ranges,
            0.0,
            self.increment,
            RANGE_MIN,
            RANGE_MAX,
            lidar_offset=world.LIDAR_OFFSET,
        )

    def test_at_least_three_landmarks_are_detected(self):
        self.assertGreaterEqual(len(self.observations), 3)

    def test_detections_match_the_polygon_landmarks(self):
        truth = [(name, x, y) for (name, x, y, _, _, _) in world.LANDMARKS]
        found = {}
        for observation in self.observations:
            map_x, map_y = observation_to_map(observation, self.POSE)
            name, distance = min(
                ((name, math.hypot(map_x - x, map_y - y))
                 for name, x, y in truth),
                key=lambda item: item[1],
            )
            self.assertLess(
                distance, 0.15,
                'observation (%.2f, %.2f) is not a landmark (%.2f m from %s)'
                % (map_x, map_y, distance, name),
            )
            found[name] = distance

        # The three landmarks in view from this pose are all detected.
        self.assertEqual(set(found), {'L1', 'L2', 'L3'})
        for distance in found.values():
            self.assertLess(distance, 0.15)

    def test_radii_and_variances_are_physical(self):
        params = DetectionParams()
        for observation in self.observations:
            self.assertGreaterEqual(observation.radius, params.radius_min)
            self.assertLessEqual(observation.radius, params.radius_max)
            self.assertTrue(math.isfinite(observation.range))
            self.assertTrue(math.isfinite(observation.bearing))
            for variance in (
                observation.range_variance,
                observation.bearing_variance,
            ):
                self.assertTrue(math.isfinite(variance))
                self.assertGreater(variance, 0.0)

    def test_big_obstacles_are_not_reported(self):
        big_circles = [(x, y) for (x, y, _, _) in world.CYLINDER_OBSTACLES]
        boxes = [
            (xmin - 0.2, xmax + 0.2, ymin - 0.2, ymax + 0.2)
            for (xmin, xmax, ymin, ymax, _) in world.BOX_OBSTACLES
        ]
        triangle = world.TRIANGLE_OBSTACLE['vertices']
        for observation in self.observations:
            map_x, map_y = observation_to_map(observation, self.POSE)
            for cx, cy in big_circles:
                self.assertGreater(
                    math.hypot(map_x - cx, map_y - cy), 0.4,
                    'big cylinder at (%.2f, %.2f) reported' % (cx, cy),
                )
            for xmin, xmax, ymin, ymax in boxes:
                inside = xmin <= map_x <= xmax and ymin <= map_y <= ymax
                self.assertFalse(inside, 'L-wall reported at (%.2f, %.2f)'
                                 % (map_x, map_y))
            for index in range(3):
                x1, y1 = triangle[index]
                x2, y2 = triangle[(index + 1) % 3]
                edge = math.hypot(x2 - x1, y2 - y1)
                # Distance from the observation to the triangle edge.
                t = ((map_x - x1) * (x2 - x1) + (map_y - y1) * (y2 - y1))
                t = min(max(t / (edge * edge), 0.0), 1.0)
                distance = math.hypot(
                    map_x - (x1 + t * (x2 - x1)),
                    map_y - (y1 + t * (y2 - y1)),
                )
                self.assertGreater(distance, 0.2, 'triangle reported')

    def test_noisy_scan_still_detects_the_landmarks(self):
        import random

        rng = random.Random(8)
        noisy = []
        for distance in self.ranges:
            if not math.isfinite(distance) or rng.random() < 0.05:
                noisy.append(math.inf)
            else:
                noise = 0.04 * (rng.random() - 0.5)     # uniform +-0.02 m
                noisy.append(min(max(distance + 0.01 + noise, RANGE_MIN),
                                 RANGE_MAX))
        observations = extract_landmarks(
            noisy, 0.0, self.increment, RANGE_MIN, RANGE_MAX,
            lidar_offset=world.LIDAR_OFFSET,
        )
        truth = [(name, x, y) for (name, x, y, _, _, _) in world.LANDMARKS]
        found = set()
        for observation in observations:
            map_x, map_y = observation_to_map(observation, self.POSE)
            name, distance = min(
                ((name, math.hypot(map_x - x, map_y - y))
                 for name, x, y in truth),
                key=lambda item: item[1],
            )
            self.assertLess(distance, 0.15)
            found.add(name)
        self.assertEqual(found, {'L1', 'L2', 'L3'})


class TestGeometry(unittest.TestCase):
    def test_observations_are_in_the_base_frame(self):
        # Lidar 1 m ahead of the robot origin; the landmark is at base (2, 1),
        # i.e. 45 degrees off the lidar axis but 26.6 degrees off the heading.
        lidar_offset = (1.0, 0.0, 0.34)
        ranges, increment = circle_scan((2.0, 1.0), 0.09, lidar_offset)
        observations = extract_landmarks(
            ranges, 0.0, increment, RANGE_MIN, RANGE_MAX,
            lidar_offset=lidar_offset,
        )
        self.assertEqual(len(observations), 1)
        observation = observations[0]
        self.assertAlmostEqual(observation.range, math.hypot(2.0, 1.0),
                               delta=0.01)
        self.assertAlmostEqual(observation.bearing, math.atan2(1.0, 2.0),
                               delta=0.01)
        lidar_bearing = math.atan2(1.0, 2.0 - 1.0)
        self.assertGreater(abs(observation.bearing - lidar_bearing), 0.2)

    def test_bearing_is_wrapped_to_the_half_open_interval(self):
        ranges, increment = circle_scan((-2.0, 0.0), 0.09, LIDAR_OFFSET)
        observations = extract_landmarks(
            ranges, 0.0, increment, RANGE_MIN, RANGE_MAX,
            lidar_offset=LIDAR_OFFSET,
        )
        self.assertEqual(len(observations), 1)
        self.assertAlmostEqual(abs(observations[0].bearing), math.pi, delta=0.02)
        self.assertEqual(observations[0].bearing,
                         wrap_angle(observations[0].bearing))


class TestAssociation(unittest.TestCase):
    POSE = (4.0, 3.0, 0.0)          # landmark 2.55 m straight ahead (south)

    def setUp(self):
        self.landmark = LandmarkEstimate(
            id=7, x=4.0, y=0.45, covariance=np.eye(2) * 1e-4
        )
        self.pose_covariance = np.diag([0.0025, 0.0025, 1e-4])

    def observation(self, distance, bearing, range_variance=1e-4,
                    bearing_variance=1e-4):
        return LandmarkObservation(
            range=distance,
            bearing=bearing,
            radius=0.09,
            range_variance=range_variance,
            bearing_variance=bearing_variance,
        )

    def test_correct_observation_keeps_the_landmark_id(self):
        observation = self.observation(2.55, -math.pi / 2.0)
        first = associate_observations(
            [observation], self.POSE, [self.landmark], self.pose_covariance
        )
        second = associate_observations(
            [observation], self.POSE, [self.landmark], self.pose_covariance
        )
        self.assertEqual(first, [7])
        self.assertEqual(second, [7])

    def test_observation_two_metres_away_is_rejected(self):
        observation = self.observation(4.55, -math.pi / 2.0)
        matches = associate_observations(
            [observation], self.POSE, [self.landmark], self.pose_covariance
        )
        self.assertEqual(matches, [None])

    def test_observation_behind_the_robot_is_rejected_by_a_large_gate(self):
        # Range is right, the bearing is 180 degrees off: even a gate 100x the
        # 99% chi-square threshold must not associate it.
        observation = self.observation(2.55, math.pi)
        matches = associate_observations(
            [observation], self.POSE, [self.landmark], self.pose_covariance,
            gate=1000.0,
        )
        self.assertEqual(matches, [None])

    def test_ids_are_not_reused_within_one_scan(self):
        observation = self.observation(2.55, -math.pi / 2.0)
        matches = associate_observations(
            [observation, observation], self.POSE, [self.landmark],
            self.pose_covariance,
        )
        self.assertEqual(matches, [7, None])

    def test_two_landmarks_are_matched_greedily(self):
        other = LandmarkEstimate(
            id=9, x=4.0, y=3.45, covariance=np.eye(2) * 1e-4
        )
        towards_landmark = self.observation(2.55, -math.pi / 2.0)
        towards_other = self.observation(0.45, math.pi / 2.0)
        # Reversed order: the greedy pass must still use each id once.
        matches = associate_observations(
            [towards_other, towards_landmark], self.POSE,
            [self.landmark, other], np.diag([1e-4, 1e-4, 1e-6]),
        )
        self.assertEqual(matches, [9, 7])

    def test_landmark_covariance_widens_the_gate(self):
        # 0.35 m range error: rejected for a well-known landmark, accepted once
        # the landmark covariance is large (the association uses R + C).
        observation = self.observation(2.90, -math.pi / 2.0)
        tight = associate_observations(
            [observation], self.POSE,
            [LandmarkEstimate(7, 4.0, 0.45, np.eye(2) * 1e-4)],
            self.pose_covariance,
        )
        loose = associate_observations(
            [observation], self.POSE,
            [LandmarkEstimate(7, 4.0, 0.45, np.eye(2) * 0.25)],
            self.pose_covariance,
        )
        self.assertEqual(tight, [None])
        self.assertEqual(loose, [7])

    def test_max_range_and_empty_inputs(self):
        observation = self.observation(2.55, -math.pi / 2.0)
        self.assertEqual(
            associate_observations(
                [observation], self.POSE, [self.landmark],
                self.pose_covariance, max_range=1.0,
            ),
            [None],
        )
        self.assertEqual(
            associate_observations([], self.POSE, [self.landmark],
                                   self.pose_covariance),
            [],
        )
        self.assertEqual(
            associate_observations([observation], self.POSE, [],
                                   self.pose_covariance),
            [None],
        )

    def test_prediction_jacobian_matches_finite_differences(self):
        landmark = LandmarkEstimate(3, 4.6, 1.9, np.eye(2))
        pose = (1.2, 0.8, 0.7)
        prediction, jacobian = _predict(pose, landmark)
        self.assertAlmostEqual(
            prediction[0], math.hypot(4.6 - 1.2, 1.9 - 0.8), places=9
        )
        step = 1e-7
        numeric = np.zeros((2, 3))
        for index in range(3):
            plus = list(pose)
            minus = list(pose)
            plus[index] += step
            minus[index] -= step
            ahead = _predict(tuple(plus), landmark)[0]
            behind = _predict(tuple(minus), landmark)[0]
            numeric[0, index] = (ahead[0] - behind[0]) / (2.0 * step)
            numeric[1, index] = (
                wrap_angle(ahead[1] - behind[1]) / (2.0 * step)
            )
        np.testing.assert_allclose(jacobian, numeric, atol=1e-6)


@unittest.skipIf(yaml is None, 'PyYAML is not installed')
class TestConfig(unittest.TestCase):
    def setUp(self):
        with open(CONFIG_PATH, 'r') as stream:
            self.params = yaml.safe_load(stream)['landmark_node'][
                'ros__parameters'
            ]

    def test_detection_defaults_match_the_config(self):
        defaults = DetectionParams()
        for field in defaults.__dataclass_fields__:
            self.assertIn(field, self.params, 'missing %s in landmarks.yaml'
                          % field)
            self.assertAlmostEqual(
                float(self.params[field]), float(getattr(defaults, field)),
                msg='default of %s differs from landmarks.yaml' % field,
            )

    def test_topics_and_map_parameters(self):
        self.assertEqual(self.params['scan_topic'], '/scan_noisy')
        self.assertTrue(self.params['pose_topic'].endswith('/odom'))
        self.assertTrue(self.params['observation_topic'].startswith('/'))
        self.assertTrue(self.params['map_topic'].startswith('/'))
        self.assertTrue(self.params['marker_topic'].startswith('/'))
        self.assertEqual(self.params['frame_id'], 'base_footprint')
        self.assertEqual(self.params['map_frame_id'], 'map')
        self.assertGreater(self.params['publish_rate_hz'], 0.0)
        self.assertGreater(self.params['max_landmarks'], 0)
        self.assertGreater(self.params['map_alpha'], 0.0)
        self.assertLessEqual(self.params['map_alpha'], 1.0)

    def test_limits_are_ordered_and_consistent(self):
        params = self.params
        self.assertLess(params['radius_min'], params['radius_max'])
        self.assertLess(params['min_points'], params['max_points'])
        self.assertLess(params['min_range'], params['max_range'])
        self.assertEqual(params['gate'], 9.21)
        # The 0.09 m landmarks must fit into the radius window and their
        # silhouette (2 * 0.09 m) into the cluster width limit.
        if world is not None:
            radius = world.LANDMARKS[0][3]
            self.assertGreater(
                radius, params['radius_min']
            )
            self.assertLess(radius, params['radius_max'])
            self.assertGreaterEqual(
                params['max_cluster_width'], 2.0 * radius
            )
            self.assertEqual(
                tuple(params['lidar_offset']), tuple(world.LIDAR_OFFSET)
            )


if __name__ == '__main__':
    unittest.main()
