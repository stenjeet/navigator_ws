"""Consistency checks between ekf.yaml and the simulation geometry.

The localization package must use exactly the anchors and tag offset that the
simulation publishes, otherwise the ranges would be interpreted in a different
geometry. The checks are skipped when ``navigator_sim`` is not imported (e.g. a
standalone local run without the whole workspace on ``PYTHONPATH``).
"""

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
CONFIG_PATH = os.path.join(os.path.dirname(HERE), 'config', 'ekf.yaml')


@unittest.skipIf(world is None, 'navigator_sim is not on PYTHONPATH')
@unittest.skipIf(yaml is None, 'PyYAML is not installed')
class TestConfigMatchesWorld(unittest.TestCase):
    def setUp(self):
        with open(CONFIG_PATH, 'r') as stream:
            self.params = yaml.safe_load(stream)['ekf_node']['ros__parameters']

    def test_anchors_match_world(self):
        flat = self.params['anchors']
        anchors = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]
        expected = [tuple(anchor[1:4]) for anchor in world.UWB_ANCHORS]
        self.assertEqual(anchors, expected)

    def test_tag_offset_matches_world(self):
        self.assertEqual(
            tuple(self.params['tag_offset']),
            tuple(world.UWB_TAG_OFFSET),
        )


if __name__ == '__main__':
    unittest.main()
