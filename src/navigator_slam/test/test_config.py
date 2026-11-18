"""Consistency checks between occupancy.yaml and the simulation geometry.

The mapper must use exactly the lidar offset the simulation publishes and a
grid that covers the whole polygon, otherwise the map would be shifted or
clipped. The checks are skipped when ``navigator_sim`` is not importable.
"""

import math
import os
import unittest

try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML is a ROS dependency
    yaml = None

try:
    from navigator_sim import world
except ImportError:
    world = None

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(os.path.dirname(HERE), 'config', 'occupancy.yaml')


@unittest.skipIf(world is None, 'navigator_sim is not on PYTHONPATH')
@unittest.skipIf(yaml is None, 'PyYAML is not installed')
class TestConfigMatchesWorld(unittest.TestCase):
    def setUp(self):
        with open(CONFIG_PATH, 'r') as stream:
            self.params = yaml.safe_load(stream)['occupancy_grid_node'][
                'ros__parameters'
            ]

    def test_lidar_offset_matches_world(self):
        self.assertEqual(
            tuple(self.params['lidar_offset']),
            tuple(world.LIDAR_OFFSET),
        )

    def test_grid_covers_the_arena_and_walls(self):
        margin = world.WALL_THICKNESS
        arena_max_x = max(box[1] for box in world.WALLS)
        arena_max_y = max(box[3] for box in world.WALLS)
        self.assertLessEqual(self.params['grid_min_x'], -margin)
        self.assertLessEqual(self.params['grid_min_y'], -margin)
        self.assertGreaterEqual(self.params['grid_max_x'], arena_max_x + margin)
        self.assertGreaterEqual(self.params['grid_max_y'], arena_max_y + margin)

    def test_resolution_is_fine_enough_for_obstacles(self):
        resolution = self.params['resolution']
        self.assertGreater(resolution, 0.0)
        self.assertLessEqual(resolution, 0.10)
        # Grid dimensions must stay cheap for 10 Hz updates.
        width = int(
            (self.params['grid_max_x'] - self.params['grid_min_x']) / resolution
        )
        height = int(
            (self.params['grid_max_y'] - self.params['grid_min_y']) / resolution
        )
        self.assertLessEqual(width * height, 500 * 500)

    def test_scan_range_is_usable(self):
        # The cap must be wide enough to see across the polygon and must not
        # exceed the modelled lidar range (sensors.yaml: 8.0 m).
        half_diagonal = 0.5 * math.hypot(
            max(box[1] for box in world.WALLS),
            max(box[3] for box in world.WALLS),
        )
        self.assertGreaterEqual(self.params['max_range'], half_diagonal)
        self.assertLessEqual(self.params['max_range'], 8.0)


class TestConfigSelfConsistency(unittest.TestCase):
    @unittest.skipIf(yaml is None, 'PyYAML is not installed')
    def test_probabilities_are_ordered(self):
        with open(CONFIG_PATH, 'r') as stream:
            params = yaml.safe_load(stream)['occupancy_grid_node'][
                'ros__parameters'
            ]
        self.assertLess(params['free_threshold'], params['occupied_threshold'])
        self.assertLess(params['p_free'], 0.5)
        self.assertGreater(params['p_occupied'], 0.5)
        self.assertLess(params['log_odds_min'], 0.0)
        self.assertGreater(params['log_odds_max'], 0.0)

        # One beam must be enough to publish a cell as free/occupied.
        self.assertLessEqual(params['p_free'], params['free_threshold'])
        self.assertGreaterEqual(params['p_occupied'], params['occupied_threshold'])
        # The clamp must not cut the evidence of a single beam.
        from navigator_slam.occupancy_grid import log_odds

        self.assertLessEqual(log_odds(params['p_occupied']), params['log_odds_max'])
        self.assertGreaterEqual(log_odds(params['p_free']), params['log_odds_min'])

    @unittest.skipIf(yaml is None, 'PyYAML is not installed')
    def test_topics_are_configured(self):
        with open(CONFIG_PATH, 'r') as stream:
            params = yaml.safe_load(stream)['occupancy_grid_node'][
                'ros__parameters'
            ]
        self.assertEqual(params['scan_topic'], '/scan_noisy')
        self.assertTrue(params['pose_topic'].endswith('/odom'))
        self.assertTrue(params['map_topic'].startswith('/'))
        self.assertGreater(params['publish_rate_hz'], 0.0)


if __name__ == '__main__':
    unittest.main()
