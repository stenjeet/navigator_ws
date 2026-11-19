"""Unit tests for landmark confirmation (no ROS required)."""

import unittest

from navigator_slam.tentative import TentativeTracks


class TestTentativeTracks(unittest.TestCase):
    def test_requires_min_hits_before_confirmation(self):
        tracks = TentativeTracks(gate=0.3, min_hits=3)
        confirmed, x, y = tracks.update(4.0, 0.45)
        self.assertFalse(confirmed)
        self.assertEqual(len(tracks), 1)
        self.assertFalse(tracks.update(4.02, 0.44)[0])
        confirmed, x, y = tracks.update(3.99, 0.46)
        self.assertTrue(confirmed)
        self.assertEqual(len(tracks), 0)
        self.assertAlmostEqual(x, 4.0, delta=0.03)
        self.assertAlmostEqual(y, 0.45, delta=0.03)

    def test_min_hits_one_confirms_every_sighting(self):
        tracks = TentativeTracks(min_hits=1)
        confirmed, x, y = tracks.update(1.0, 2.0)
        self.assertTrue(confirmed)
        self.assertEqual((x, y), (1.0, 2.0))
        self.assertEqual(len(tracks), 0)

    def test_distant_sightings_do_not_confirm(self):
        tracks = TentativeTracks(gate=0.2, min_hits=2)
        self.assertFalse(tracks.update(0.0, 0.0)[0])
        self.assertFalse(tracks.update(1.0, 0.0)[0])
        self.assertFalse(tracks.update(2.0, 0.0)[0])
        self.assertEqual(len(tracks), 3)

    def test_separate_candidates_are_tracked_independently(self):
        tracks = TentativeTracks(gate=0.15, min_hits=2)
        self.assertFalse(tracks.update(0.0, 0.0)[0])
        self.assertFalse(tracks.update(5.0, 5.0)[0])
        self.assertEqual(len(tracks), 2)
        confirmed, x, y = tracks.update(0.02, 0.01)
        self.assertTrue(confirmed)
        self.assertAlmostEqual(x, 0.01, delta=0.02)
        self.assertEqual(len(tracks), 1)
        confirmed, x, y = tracks.update(5.01, 4.99)
        self.assertTrue(confirmed)
        self.assertAlmostEqual(x, 5.0, delta=0.02)
        self.assertEqual(len(tracks), 0)

    def test_max_tracks_evicts_the_oldest(self):
        tracks = TentativeTracks(gate=0.1, min_hits=5, max_tracks=2)
        tracks.update(0.0, 0.0)
        tracks.update(1.0, 0.0)
        tracks.update(2.0, 0.0)
        self.assertEqual(len(tracks), 2)
        self.assertAlmostEqual(tracks.positions()[0][0], 1.0)

    def test_invalid_configuration_raises(self):
        with self.assertRaises(ValueError):
            TentativeTracks(gate=0.0)
        with self.assertRaises(ValueError):
            TentativeTracks(min_hits=0)
        with self.assertRaises(ValueError):
            TentativeTracks(max_tracks=0)

    def test_clear_drops_candidates(self):
        tracks = TentativeTracks(min_hits=3)
        tracks.update(1.0, 1.0)
        tracks.clear()
        self.assertEqual(len(tracks), 0)


if __name__ == '__main__':
    unittest.main()
