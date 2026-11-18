"""Unit tests for the pure log-odds occupancy grid (stage 6).

Run locally from the repository root::

    PYTHONPATH=src/navigator_slam:src/navigator_sim \
        python3 -m unittest discover -s src/navigator_slam/test -v
"""

import math
import unittest

import numpy as np

from navigator_slam.occupancy_grid import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    LogOddsGrid,
    compare_grids,
    distance_to_occupied,
    log_odds,
    nearest_sample,
    probability,
)

try:
    from navigator_sim import world
except ImportError:  # pragma: no cover - standalone run without navigator_sim
    world = None

RESOLUTION = 0.05


def make_grid(min_x=-0.5, min_y=-0.5, max_x=8.5, max_y=8.5, resolution=RESOLUTION):
    return LogOddsGrid.from_bounds(min_x, min_y, max_x, max_y, resolution)


def beam_scan(grid, pose, ranges, angle_min=0.0, angle_increment=0.0,
              lidar_offset=(0.0, 0.0, 0.0)):
    return grid.update_scan(
        pose,
        ranges,
        angle_min,
        angle_increment,
        range_min=0.12,
        range_max=8.0,
        lidar_offset=lidar_offset,
    )


class TestProbability(unittest.TestCase):
    def test_roundtrip(self):
        for value in (0.2, 0.5, 0.7, 0.95):
            self.assertAlmostEqual(probability(log_odds(value)), value, places=12)

    def test_invalid_probability_raises(self):
        for value in (0.0, 1.0, -0.1, 1.5):
            with self.assertRaises(ValueError):
                log_odds(value)


class TestGeometry(unittest.TestCase):
    def setUp(self):
        self.grid = LogOddsGrid(100, 100, 0.1, origin=(0.0, 0.0))

    def test_world_to_grid_uses_cell_centres(self):
        self.assertEqual(self.grid.world_to_grid(0.05, 0.05), (0, 0))
        self.assertEqual(self.grid.world_to_grid(9.95, 9.95), (99, 99))
        self.assertEqual(self.grid.world_to_grid(1.05, 2.05), (10, 20))

    def test_world_to_grid_outside(self):
        self.assertIsNone(self.grid.world_to_grid(-0.01, 1.0))
        self.assertIsNone(self.grid.world_to_grid(10.0, 1.0))

    def test_grid_to_world_is_inverse_of_world_to_grid(self):
        x, y = self.grid.grid_to_world(10, 20)
        self.assertEqual(self.grid.world_to_grid(x, y), (10, 20))

    def test_from_bounds_covers_requested_area(self):
        grid = make_grid()
        self.assertEqual(grid.width, 180)
        self.assertEqual(grid.height, 180)
        self.assertEqual(grid.origin, (-0.5, -0.5))
        self.assertEqual(grid.world_to_grid(-0.49, -0.49), (0, 0))
        self.assertEqual(grid.world_to_grid(8.49, 8.49), (179, 179))
        self.assertIsNone(grid.world_to_grid(8.51, 4.0))

    def test_invalid_configuration_raises(self):
        with self.assertRaises(ValueError):
            LogOddsGrid(0, 10, 0.1)
        with self.assertRaises(ValueError):
            LogOddsGrid(10, 10, 0.0)
        with self.assertRaises(ValueError):
            LogOddsGrid(10, 10, 0.1, log_odds_min=0.5, log_odds_max=2.0)
        with self.assertRaises(ValueError):
            LogOddsGrid(10, 10, 0.1, occupied_threshold=0.3, free_threshold=0.6)
        with self.assertRaises(ValueError):
            LogOddsGrid.from_bounds(1.0, 1.0, 0.0, 2.0, 0.1)


class TestLikelihoodUpdate(unittest.TestCase):
    def test_initial_map_is_unknown(self):
        grid = make_grid()
        data = grid.occupancy_data()
        self.assertTrue(np.all(data == UNKNOWN))
        self.assertEqual(grid.stats(), {'occupied': 0, 'free': 0, 'unknown': 32400})

    def test_single_beam_marks_free_cells_and_occupied_end(self):
        grid = LogOddsGrid(100, 100, 0.1, origin=(0.0, 0.0))
        free, occupied = beam_scan(grid, (1.0, 1.0, 0.0), [2.0])

        self.assertGreater(free, 0)
        self.assertEqual(occupied, 1)
        for x in (1.5, 2.0, 2.5):
            self.assertEqual(grid.occupancy_at(x, 1.0), FREE, 'x=%.1f' % x)
        self.assertEqual(grid.occupancy_at(3.0, 1.0), OCCUPIED)
        self.assertEqual(grid.occupancy_at(3.6, 1.0), UNKNOWN)
        self.assertEqual(grid.occupancy_at(0.5, 1.0), UNKNOWN)

    def test_pose_yaw_rotates_the_beam(self):
        grid = LogOddsGrid(100, 100, 0.1, origin=(0.0, 0.0))
        beam_scan(grid, (2.0, 2.0, math.pi / 2.0), [1.0])
        self.assertEqual(grid.occupancy_at(2.0, 3.0), OCCUPIED)
        self.assertEqual(grid.occupancy_at(2.0, 2.5), FREE)
        self.assertEqual(grid.occupancy_at(3.0, 2.0), UNKNOWN)

    def test_lidar_offset_is_rotated_with_the_pose(self):
        grid = LogOddsGrid(100, 100, 0.1, origin=(0.0, 0.0))
        # Lidar 0.5 m ahead of the robot: robot at (2, 2) facing +y puts the
        # lidar at (2, 2.5); a beam along +y ends at (2, 3.5). Cells between
        # the robot and the lidar are not observed.
        beam_scan(
            grid,
            (2.0, 2.0, math.pi / 2.0),
            [1.0],
            lidar_offset=(0.5, 0.0, 0.0),
        )
        self.assertEqual(grid.occupancy_at(2.0, 3.5), OCCUPIED)
        self.assertEqual(grid.occupancy_at(2.0, 3.0), FREE)
        self.assertEqual(grid.occupancy_at(2.0, 2.55), FREE)
        self.assertEqual(grid.occupancy_at(2.0, 2.2), UNKNOWN)

    def test_beam_sweep_covers_angles(self):
        grid = LogOddsGrid(100, 100, 0.1, origin=(0.0, 0.0))
        # 4 beams of 1.0 m from (5, 5) with 90 deg spacing.
        beam_scan(
            grid,
            (5.0, 5.0, 0.0),
            [1.0, 1.0, 1.0, 1.0],
            angle_min=0.0,
            angle_increment=math.pi / 2.0,
        )
        for x, y in ((6.0, 5.0), (5.0, 6.0), (4.0, 5.0), (5.0, 4.0)):
            self.assertEqual(grid.occupancy_at(x, y), OCCUPIED, '(%s, %s)' % (x, y))

    def test_dropout_and_nan_are_ignored(self):
        grid = LogOddsGrid(100, 100, 0.1, origin=(0.0, 0.0))
        free, occupied = beam_scan(
            grid, (1.0, 1.0, 0.0), [float('inf'), float('nan'), -1.0]
        )
        self.assertEqual((free, occupied), (0, 0))
        self.assertTrue(np.all(grid.occupancy_data() == UNKNOWN))

    def test_ranges_outside_the_sensor_model_are_ignored(self):
        grid = LogOddsGrid(100, 100, 0.1, origin=(0.0, 0.0))
        free, occupied = beam_scan(grid, (1.0, 1.0, 0.0), [0.05, 9.0])
        self.assertEqual((free, occupied), (0, 0))

    def test_beams_longer_than_max_range_are_ignored(self):
        grid = LogOddsGrid(100, 100, 0.1, origin=(0.0, 0.0))
        free, occupied = grid.update_scan(
            (1.0, 1.0, 0.0),
            [5.0],
            0.0,
            0.0,
            range_max=8.0,
            max_range=2.0,
        )
        self.assertEqual((free, occupied), (0, 0))
        self.assertEqual(grid.occupancy_at(3.0, 1.0), UNKNOWN)

    def test_log_odds_is_clamped(self):
        grid = make_grid()
        x, y = 4.0, 4.0
        ix, iy = grid.world_to_grid(x, y)
        pose = (x - 1.0, y, 0.0)
        for _ in range(200):
            beam_scan(grid, pose, [1.0])
            self.assertLessEqual(grid.log_odds[iy, ix], grid.log_odds_max + 1e-9)
        self.assertAlmostEqual(grid.log_odds[iy, ix], grid.log_odds_max)
        self.assertEqual(grid.occupancy_at(x, y), OCCUPIED)

    def test_repeated_identical_scans_are_stable(self):
        grid = make_grid()
        for _ in range(5):
            beam_scan(grid, (2.0, 1.5, 0.0), [1.0, 2.0, 3.0], angle_increment=0.3)
        first = grid.occupancy_data()
        beam_scan(grid, (2.0, 1.5, 0.0), [1.0, 2.0, 3.0], angle_increment=0.3)
        np.testing.assert_array_equal(first, grid.occupancy_data())


class TestComparison(unittest.TestCase):
    def test_identical_maps_match_fully(self):
        a = make_grid()
        b = make_grid()
        beam_scan(a, (2.0, 2.0, 0.0), [1.0])
        beam_scan(b, (2.0, 2.0, 0.0), [1.0])
        metrics = compare_grids(a, b)
        self.assertEqual(metrics['iou_occupied'], 1.0)
        self.assertEqual(metrics['agreement'], 1.0)
        self.assertEqual(metrics['only_a'], 0)
        self.assertEqual(metrics['only_b'], 0)
        self.assertGreater(metrics['occupied_a'], 0)

    def test_different_obstacle_positions_reduce_agreement(self):
        a = make_grid()
        b = make_grid()
        # A sees the obstacle at 3.0 m from the lidar, B at 2.5 m: the cell hit
        # by B is free in A, and the cell hit by A is unknown in B.
        beam_scan(a, (1.0, 1.0, 0.0), [3.0])
        beam_scan(b, (1.0, 1.0, 0.0), [2.5])
        metrics = compare_grids(a, b)
        self.assertLess(metrics['iou_occupied'], 1.0)
        self.assertLess(metrics['agreement'], 1.0)
        self.assertGreater(metrics['only_a'], 0)
        self.assertGreater(metrics['only_b'], 0)
        self.assertGreater(metrics['known_both'], 0)

    def test_empty_maps_report_nan(self):
        metrics = compare_grids(make_grid(), make_grid())
        self.assertTrue(math.isnan(metrics['iou_occupied']))
        self.assertTrue(math.isnan(metrics['agreement']))

    def test_different_geometry_is_rejected(self):
        with self.assertRaises(ValueError):
            compare_grids(make_grid(), make_grid(resolution=0.1))
        with self.assertRaises(ValueError):
            compare_grids(make_grid(), make_grid(min_x=-1.0))


class TestDistances(unittest.TestCase):
    def test_zero_on_sources(self):
        occupied = np.zeros((5, 5), dtype=bool)
        occupied[2, 2] = True
        distances = distance_to_occupied(occupied, resolution=0.5)
        self.assertEqual(distances[2, 2], 0.0)
        self.assertTrue(np.all(distances < np.inf))

    def test_axis_and_diagonal_steps(self):
        occupied = np.zeros((5, 5), dtype=bool)
        occupied[0, 0] = True
        distances = distance_to_occupied(occupied, resolution=0.5)
        self.assertAlmostEqual(distances[0, 3], 1.5)          # 3 straight steps
        self.assertAlmostEqual(distances[2, 2], 2 * math.sqrt(2.0) * 0.5)
        self.assertAlmostEqual(distances[4, 0], 2.0)

    def test_no_occupied_cells_gives_infinity(self):
        distances = distance_to_occupied(np.zeros((4, 4), dtype=bool))
        self.assertTrue(np.all(np.isinf(distances)))

    def test_shifted_map_has_offset_sized_distance(self):
        a = make_grid()
        b = make_grid()
        beam_scan(a, (1.0, 1.0, 0.0), [3.0])
        beam_scan(b, (1.0, 1.0, 0.0), [3.2])
        metrics = compare_grids(a, b)
        self.assertAlmostEqual(metrics['mean_distance_ab'], 0.2, delta=0.06)
        self.assertAlmostEqual(metrics['max_distance_ab'], 0.2, delta=0.06)

    def test_identical_maps_have_zero_distance(self):
        a = make_grid()
        b = make_grid()
        beam_scan(a, (1.0, 1.0, 0.0), [3.0])
        beam_scan(b, (1.0, 1.0, 0.0), [3.0])
        metrics = compare_grids(a, b)
        self.assertEqual(metrics['mean_distance_ab'], 0.0)
        self.assertEqual(metrics['max_distance_ba'], 0.0)


class TestNearestSample(unittest.TestCase):
    def test_picks_closest_timestamp(self):
        times = [1.0, 1.1, 1.2]
        self.assertEqual(nearest_sample(times, 1.04), 0)
        self.assertEqual(nearest_sample(times, 1.06), 1)
        self.assertEqual(nearest_sample(times, 99.0), 2)

    def test_empty_list_raises(self):
        with self.assertRaises(ValueError):
            nearest_sample([], 0.0)


def ray_aabb(ox, oy, dx, dy, xmin, xmax, ymin, ymax):
    """Independent ray/box test used as the test oracle (slab method)."""
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
    """Independent ray/circle test used as the test oracle."""
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
    """Independent ray/segment test used as the test oracle."""
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


@unittest.skipIf(world is None, 'navigator_sim is not on PYTHONPATH')
class TestAgainstPolygonGeometry(unittest.TestCase):
    """The mapper must reproduce the polygon geometry from ``world.py``.

    Ranges come from an independent ray caster built only from the ``world``
    geometry tables, so a systematic error in the grid projection cannot be
    hidden by reusing the lidar model implementation.
    """

    POSE = (1.5, 1.5, 0.0)
    BEAMS = 360

    def setUp(self):
        self.grid = make_grid()
        self.angle_increment = 2.0 * math.pi / self.BEAMS
        self.lidar_x, self.lidar_y = self._lidar_position(self.POSE)
        self.ranges = [
            self._raycast(self.POSE[2] + j * self.angle_increment)
            for j in range(self.BEAMS)
        ]

    @staticmethod
    def _lidar_position(pose):
        x, y, yaw = pose
        dx, dy, _ = world.LIDAR_OFFSET
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        return x + dx * cos_yaw - dy * sin_yaw, y + dx * sin_yaw + dy * cos_yaw

    def _raycast(self, phi):
        dx, dy = math.cos(phi), math.sin(phi)
        best = math.inf
        boxes = list(world.WALLS) + [box[:4] for box in world.BOX_OBSTACLES]
        for xmin, xmax, ymin, ymax in boxes:
            hit = ray_aabb(self.lidar_x, self.lidar_y, dx, dy, xmin, xmax, ymin, ymax)
            if hit is not None:
                best = min(best, hit)
        circles = [(x, y, r) for (x, y, r, _) in world.CYLINDER_OBSTACLES]
        circles += [(x, y, r) for (_, x, y, r, _, _) in world.LANDMARKS]
        for cx, cy, radius in circles:
            hit = ray_circle(self.lidar_x, self.lidar_y, dx, dy, cx, cy, radius)
            if hit is not None:
                best = min(best, hit)
        vertices = world.TRIANGLE_OBSTACLE['vertices']
        for index in range(3):
            x1, y1 = vertices[index]
            x2, y2 = vertices[(index + 1) % 3]
            hit = ray_segment(self.lidar_x, self.lidar_y, dx, dy, x1, y1, x2, y2)
            if hit is not None:
                best = min(best, hit)
        return best

    def _update(self, pose):
        self.grid.update_scan(
            pose,
            self.ranges,
            0.0,
            self.angle_increment,
            range_min=0.12,
            range_max=8.0,
            lidar_offset=world.LIDAR_OFFSET,
        )

    def test_beam_endpoints_are_occupied_and_rays_are_free(self):
        self._update(self.POSE)

        checked = 0
        for index in range(0, self.BEAMS, 15):
            distance = self.ranges[index]
            if not math.isfinite(distance) or distance < 0.5:
                continue
            phi = self.POSE[2] + index * self.angle_increment
            end_x = self.lidar_x + distance * math.cos(phi)
            end_y = self.lidar_y + distance * math.sin(phi)
            self.assertEqual(
                self.grid.occupancy_at(end_x, end_y), OCCUPIED,
                'beam %d endpoint (%.2f, %.2f)' % (index, end_x, end_y),
            )
            along = 0.5 * distance
            self.assertEqual(
                self.grid.occupancy_at(
                    self.lidar_x + along * math.cos(phi),
                    self.lidar_y + along * math.sin(phi),
                ),
                FREE,
                'beam %d midpoint' % index,
            )
            checked += 1
        self.assertGreater(checked, 20)

    def test_interior_is_free_and_map_is_well_formed(self):
        self._update(self.POSE)
        data = self.grid.occupancy_data()

        self.assertEqual(self.grid.occupancy_at(1.5, 1.5), FREE)
        self.assertEqual(self.grid.occupancy_at(3.0, 1.5), FREE)
        self.assertGreater(np.count_nonzero(data == OCCUPIED), 200)
        self.assertGreater(np.count_nonzero(data == FREE), 1000)

        occupied = np.argwhere(data == OCCUPIED)
        xs = self.grid.origin[0] + (occupied[:, 1] + 0.5) * self.grid.resolution
        ys = self.grid.origin[1] + (occupied[:, 0] + 0.5) * self.grid.resolution
        margin = world.WALL_THICKNESS + 0.1
        self.assertGreaterEqual(xs.min(), -margin)
        self.assertLessEqual(xs.max(), 8.0 + margin)
        self.assertGreaterEqual(ys.min(), -margin)
        self.assertLessEqual(ys.max(), 8.0 + margin)

    def test_true_and_shifted_poses_produce_different_maps(self):
        shifted = (self.POSE[0] + 0.20, self.POSE[1], self.POSE[2] + 0.10)
        self._update(self.POSE)
        shifted_grid = make_grid()
        shifted_grid.update_scan(
            shifted,
            self.ranges,
            0.0,
            self.angle_increment,
            range_min=0.12,
            range_max=8.0,
            lidar_offset=world.LIDAR_OFFSET,
        )
        metrics = compare_grids(self.grid, shifted_grid)
        self.assertLess(metrics['iou_occupied'], 0.9)
        self.assertGreater(metrics['only_a'], 0)


if __name__ == '__main__':
    unittest.main()
