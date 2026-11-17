"""Single source of truth for the test polygon geometry.

Values follow the report tables (2.2, 2.3) and are verified collision-free
against the square trajectory (side 4.0 m, corners 1.5..5.5 m). Both the RViz
marker publisher and the lidar ray-casting model import this module so that the
visual map and the measured distances never diverge.

All 2D rectangles are stored as (x_min, x_max, y_min, y_max) [m].
"""

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
# Internal axis-aligned obstacles (purple L-shaped wall).
# Table 2.2: horizontal part centre (3.10, 4.85) 2.20 x 0.22 x 0.70,
#            vertical part   centre (2.10, 3.75) 0.22 x 2.40 x 0.70.
# --------------------------------------------------------------------------
BOX_OBSTACLES = [
    (2.00, 4.20, 4.74, 4.96, 0.70),   # L-wall, horizontal part
    (1.99, 2.21, 2.55, 4.95, 0.70),   # L-wall, vertical part
]

# --------------------------------------------------------------------------
# Cylindrical obstacles: (x, y, radius, height) [m].
# Table 2.2: cylinder 1 centre (6.35, 2.65) r=0.35 h=0.60,
#            cylinder 2 centre (6.25, 6.25) r=0.45 h=0.80.
# --------------------------------------------------------------------------
CYLINDER_OBSTACLES = [
    (6.35, 2.65, 0.35, 0.60),   # cylinder 1
    (6.25, 6.25, 0.45, 0.80),   # cylinder 2
]

# --------------------------------------------------------------------------
# Triangular obstacle (green). Table 2.2: vertices (4.35, 3.00), (5.05, 3.10),
# (4.65, 3.75), height 0.55 m.
# --------------------------------------------------------------------------
TRIANGLE_OBSTACLE = {
    'vertices': [
        (4.35, 3.00),
        (5.05, 3.10),
        (4.65, 3.75),
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
