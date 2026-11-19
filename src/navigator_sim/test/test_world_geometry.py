"""Geometry and consistency tests for the test polygon (stage 2).

Run locally without ROS::

    PYTHONPATH=src/navigator_sim python3 -m unittest discover -s src/navigator_sim/test -v

The trajectory is a fixed 4 x 4 m square (corners 1.5..5.5 m).  These tests keep
the scene physically plausible: every obstacle stays inside the arena, clear of
the path, anchors are not hidden inside cylinders, and the URDF sensor mounts
match ``world.LIDAR_OFFSET`` / ``world.UWB_TAG_OFFSET``.
"""

import math
import os
import unittest
import xml.etree.ElementTree as ET

from navigator_sim import world

PATH = [
    ((1.5, 1.5), (5.5, 1.5)),
    ((5.5, 1.5), (5.5, 5.5)),
    ((5.5, 5.5), (1.5, 5.5)),
    ((1.5, 5.5), (1.5, 1.5)),
]
#: Robot hull radius (0.24 m) plus a 0.21 m safety margin.
REQUIRED_CLEARANCE = 0.45
ANCHOR_MARKER_RADIUS = 0.12
HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.normpath(os.path.join(
    HERE, '..', '..', 'navigator_description', 'urdf', 'navigator_m.urdf'))


def point_segment_distance(point, start, end):
    """Shortest distance from ``point`` to the segment ``start``-``end``."""
    px, py = point
    ax, ay = start
    bx, by = end
    vx, vy = bx - ax, by - ay
    length2 = vx * vx + vy * vy
    t = 0.0 if length2 == 0.0 else (
        (px - ax) * vx + (py - ay) * vy) / length2
    t = min(max(t, 0.0), 1.0)
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def sample_segment(start, end, step=0.02):
    """Points along a segment, endpoints included."""
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(1, int(math.ceil(length / step)))
    return [
        (start[0] + (end[0] - start[0]) * i / count,
         start[1] + (end[1] - start[1]) * i / count)
        for i in range(count + 1)
    ]


def obstacle_points():
    """Perimeter samples of every obstacle as ``{name: [(x, y), ...]}``."""
    points = {}
    for index, (xmin, xmax, ymin, ymax, _) in enumerate(world.BOX_OBSTACLES):
        corners = ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax))
        points['box%d' % index] = []
        for edge in range(4):
            points['box%d' % index].extend(
                sample_segment(corners[edge], corners[(edge + 1) % 4]))
    for index, (cx, cy, radius, _) in enumerate(world.CYLINDER_OBSTACLES):
        points['cylinder%d' % index] = [
            (cx + radius * math.cos(2.0 * math.pi * i / 180.0),
             cy + radius * math.sin(2.0 * math.pi * i / 180.0))
            for i in range(180)
        ]
    triangle = world.TRIANGLE_OBSTACLE['vertices']
    points['triangle'] = []
    for index in range(3):
        points['triangle'].extend(
            sample_segment(triangle[index], triangle[(index + 1) % 3]))
    return points


class TestPolygonGeometry(unittest.TestCase):
    def test_outer_walls_close_the_arena(self):
        self.assertEqual(len(world.WALLS), 4)
        self.assertGreater(world.WALL_THICKNESS, 0.0)
        self.assertGreater(world.WALL_HEIGHT, 0.0)
        # The four wall strips must cover all four sides of the 8 x 8 m arena.
        self.assertTrue(any(
            xmin <= 0.0 and xmax >= 8.0 and ymin < 0.0 < ymax
            for (xmin, xmax, ymin, ymax) in world.WALLS))
        self.assertTrue(any(
            xmin <= 0.0 and xmax >= 8.0 and ymin < 8.0 < ymax
            for (xmin, xmax, ymin, ymax) in world.WALLS))
        self.assertTrue(any(
            ymin <= 0.0 and ymax >= 8.0 and xmin < 0.0 < xmax
            for (xmin, xmax, ymin, ymax) in world.WALLS))
        self.assertTrue(any(
            ymin <= 0.0 and ymax >= 8.0 and xmin < 8.0 < xmax
            for (xmin, xmax, ymin, ymax) in world.WALLS))

    def test_obstacles_stay_inside_the_arena(self):
        inner_min, inner_max = 0.06, 7.94
        for index, (xmin, xmax, ymin, ymax, height) in enumerate(
                world.BOX_OBSTACLES):
            self.assertGreater(height, 0.0)
            self.assertGreaterEqual(xmin, inner_min, 'box %d' % index)
            self.assertGreaterEqual(ymin, inner_min, 'box %d' % index)
            self.assertLessEqual(xmax, inner_max, 'box %d' % index)
            self.assertLessEqual(ymax, inner_max, 'box %d' % index)
        for index, (cx, cy, radius, height) in enumerate(
                world.CYLINDER_OBSTACLES):
            self.assertGreater(radius, 0.0)
            self.assertGreater(height, 0.0)
            self.assertGreaterEqual(cx - radius, inner_min,
                                    'cylinder %d' % index)
            self.assertGreaterEqual(cy - radius, inner_min,
                                    'cylinder %d' % index)
            self.assertLessEqual(cx + radius, inner_max,
                                 'cylinder %d' % index)
            self.assertLessEqual(cy + radius, inner_max,
                                 'cylinder %d' % index)
        for x, y in world.TRIANGLE_OBSTACLE['vertices']:
            self.assertGreaterEqual(x, inner_min)
            self.assertGreaterEqual(y, inner_min)
            self.assertLessEqual(x, inner_max)
            self.assertLessEqual(y, inner_max)
        self.assertGreater(world.TRIANGLE_OBSTACLE['height'], 0.0)

    def test_obstacles_clear_the_square_trajectory(self):
        measured = {}
        for name, points in obstacle_points().items():
            gap = min(
                point_segment_distance(point, start, end)
                for point in points
                for start, end in PATH
            )
            measured[name] = gap
            self.assertGreaterEqual(
                gap, REQUIRED_CLEARANCE,
                '%s is only %.2f m from the path' % (name, gap))
        # Guard against silently moving an obstacle into the corridor: the
        # current layout keeps at least the documented 0.45 m everywhere.
        self.assertLess(min(measured.values()), 1.0)

    def test_wall_boxes_join_into_one_solid_wall(self):
        joined = []
        for first in range(len(world.BOX_OBSTACLES)):
            for second in range(first + 1, len(world.BOX_OBSTACLES)):
                a = world.BOX_OBSTACLES[first]
                b = world.BOX_OBSTACLES[second]
                if (min(a[1], b[1]) > max(a[0], b[0])
                        and min(a[3], b[3]) > max(a[2], b[2])):
                    joined.append((first, second))
        self.assertEqual(joined, [(0, 1), (1, 2)])

    def test_landmarks_and_anchors_are_inside_the_arena(self):
        self.assertEqual(len(world.LANDMARKS), 4)
        for name, x, y, radius, height, color in world.LANDMARKS:
            self.assertTrue(0.06 <= x <= 7.94, name)
            self.assertTrue(0.06 <= y <= 7.94, name)
            self.assertGreater(radius, 0.0, name)
            self.assertGreater(height, 0.0, name)
            self.assertEqual(len(color), 3, name)
        self.assertEqual(len(world.UWB_ANCHORS), 4)
        for name, x, y, z in world.UWB_ANCHORS:
            self.assertTrue(0.06 <= x <= 7.94, name)
            self.assertTrue(0.06 <= y <= 7.94, name)
            self.assertGreater(z, 0.0, name)

    def test_cylinders_do_not_hide_the_anchors(self):
        for index, (cx, cy, radius, _) in enumerate(world.CYLINDER_OBSTACLES):
            for name, ax, ay, _ in world.UWB_ANCHORS:
                distance = math.hypot(cx - ax, cy - ay)
                self.assertGreaterEqual(
                    distance, radius + ANCHOR_MARKER_RADIUS,
                    'cylinder %d covers anchor %s' % (index, name))


class TestSceneMetadata(unittest.TestCase):
    def test_scene_palette_is_complete_and_valid(self):
        required = {
            'floor', 'walls', 'box_obstacle', 'cylinder_obstacle',
            'triangle_obstacle', 'uwb_anchor', 'label',
        }
        self.assertTrue(required.issubset(world.COLORS))
        for name, color in world.COLORS.items():
            self.assertEqual(len(color), 3, name)
            for channel in color:
                self.assertTrue(0.0 <= channel <= 1.0, name)

    def test_sensor_offsets_match_the_urdf(self):
        if not os.path.exists(URDF_PATH):
            self.skipTest('URDF is not available in this tree')
        root = ET.parse(URDF_PATH).getroot()

        def origin(joint_name):
            joint = root.find(".//joint[@name='%s']" % joint_name)
            self.assertIsNotNone(joint, joint_name)
            values = joint.find('origin').get('xyz').split()
            return tuple(float(value) for value in values)

        for axis, expected in enumerate(world.LIDAR_OFFSET):
            self.assertAlmostEqual(origin('lidar_joint')[axis], expected,
                                   places=6)
        for axis, expected in enumerate(world.UWB_TAG_OFFSET):
            self.assertAlmostEqual(origin('uwb_tag_joint')[axis], expected,
                                   places=6)

        # Regression: the hull was compacted to r=0.24 m, h=0.22 m.
        base = root.find(".//link[@name='base_link']")
        geometry = base.find('visual/geometry/cylinder')
        self.assertAlmostEqual(float(geometry.get('radius')), 0.24, places=6)
        self.assertAlmostEqual(float(geometry.get('length')), 0.22, places=6)

        # Regression: casters must not sink below the floor plane.
        for joint_name in ('front_caster_joint', 'rear_caster_joint'):
            self.assertGreaterEqual(origin(joint_name)[2], 0.04, joint_name)


if __name__ == '__main__':
    unittest.main()
