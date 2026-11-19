"""Single source of truth for the test polygon geometry.

Layout revision: the obstacles were rearranged so that they stay at least
0.45 m clear of the square trajectory (side 4.0 m, corners 1.5..5.5 m), while
all four SLAM landmarks remain observable from at least one side of the path.
Both the RViz marker publisher and the lidar ray-casting model import this
module so that the visual map and the measured distances never diverge.

All 2D rectangles are stored as (x_min, x_max, y_min, y_max) [m].
"""

# --------------------------------------------------------------------------
# Scene palette: (r, g, b) in [0, 1]. The marker publisher reads these values,
# so the rendered scene and the documentation share one colour definition.
# --------------------------------------------------------------------------
COLORS = {
    'floor': (0.90, 0.91, 0.93),             # light grey
    'walls': (0.24, 0.26, 0.30),             # graphite
    'box_obstacle': (0.15, 0.38, 0.78),      # blue L-wall
    'cylinder_obstacle': (0.95, 0.45, 0.08),  # orange cylinders
    'triangle_obstacle': (0.08, 0.65, 0.60),  # teal prism
    'uwb_anchor': (1.00, 0.55, 0.00),        # amber anchors
    'label': (0.10, 0.10, 0.10),             # anchor labels
}

# --------------------------------------------------------------------------
# Outer walls. Arena is 8 x 8 m; walls are 0.12 m thick and 0.50 m high,
# centred on the perimeter (matches the RViz markers in the report).
# --------------------------------------------------------------------------
WALL_THICKNESS = 0.12
WALL_HEIGHT = 0.50

WALLS = [
    (0.00, 8.00, -0.06, 0.06),   # south
    (0.00, 8.00, 7.94, 8.06),    # north
    (-0.06, 0.06, 0.00, 8.00),   # west
    (7.94, 8.06, 0.00, 8.00),    # east
]

# --------------------------------------------------------------------------
# Internal axis-aligned obstacles (blue L-shaped wall, north-west quadrant).
# Horizontal part: 2.00 x 0.22 x 0.70 at y = 4.55..4.77,
# vertical part:   0.22 x 1.97 x 0.70 at x = 2.20..2.42.
# The L opens towards the south-east; every edge stays >= 0.70 m from the path.
# --------------------------------------------------------------------------
BOX_OBSTACLES = [
    (2.20, 4.20, 4.55, 4.77, 0.70),   # L-wall, horizontal part
    (2.20, 2.42, 2.80, 4.77, 0.70),   # L-wall, vertical part
]

# --------------------------------------------------------------------------
# Cylindrical obstacles: (x, y, radius, height) [m].
# One in the south-east outer corridor, one in the north-west outer corridor;
# both leave >= 0.48 m to the trajectory and keep all landmarks visible.
# --------------------------------------------------------------------------
CYLINDER_OBSTACLES = [
    (6.55, 2.30, 0.30, 0.60),   # cylinder 1, east corridor
    (0.60, 6.55, 0.42, 0.85),   # cylinder 2, west/north corridor
]

# --------------------------------------------------------------------------
# Triangular obstacle (teal prism, north-east quadrant). Vertices
# (4.45, 4.55), (5.00, 4.30), (4.70, 5.00); height 0.55 m. The closest vertex
# keeps 0.50 m to the northern/eastern path edges.
# --------------------------------------------------------------------------
TRIANGLE_OBSTACLE = {
    'vertices': [
        (4.45, 4.55),
        (5.00, 4.30),
        (4.70, 5.00),
    ],
    'height': 0.55,
}

# --------------------------------------------------------------------------
# UWB anchors: (name, x, y, z) [m]. Table 2.3.
# --------------------------------------------------------------------------
UWB_ANCHORS = [
    ('A1', 0.50, 0.50, 1.20),
    ('A2', 7.40, 0.80, 1.20),
    ('A3', 7.10, 7.50, 1.20),
    ('A4', 0.70, 6.90, 1.20),
]

# --------------------------------------------------------------------------
# SLAM landmarks (report section 2.4): (name, x, y, radius, height, (r,g,b)).
# Coordinates are assumed (the report does not list them); each landmark is a
# small coloured cylinder placed near the inner side of a wall, clear of the
# square trajectory. They are part of the lidar ray casting so that they can be
# extracted later (stage 7). Heights exceed the lidar plane (0.34 m) so they
# occlude the beam.
# --------------------------------------------------------------------------
LANDMARKS = [
    ('L1', 4.00, 0.45, 0.09, 0.45, (0.85, 0.08, 0.08)),   # south, red
    ('L2', 7.55, 4.00, 0.09, 0.45, (0.08, 0.75, 0.15)),   # east, green
    ('L3', 4.00, 7.55, 0.09, 0.45, (0.08, 0.25, 0.85)),   # north, blue
    ('L4', 0.45, 4.00, 0.09, 0.45, (0.95, 0.70, 0.05)),   # west, yellow
]

# --------------------------------------------------------------------------
# Sensor mounting offsets relative to base_link (table 2.1), [m].
# --------------------------------------------------------------------------
LIDAR_OFFSET = (0.18, 0.00, 0.34)
UWB_TAG_OFFSET = (-0.15, 0.12, 0.46)
