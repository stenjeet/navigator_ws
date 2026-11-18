#!/usr/bin/env python3
"""Compare occupancy-grid maps built from true and estimated poses (stage 6).

Run alongside the mapping launch file, which starts two mapper instances on the
same lidar scans::

    ros2 launch navigator_bringup mapping.launch.py use_rviz:=false pose_source:=both &
    python3 tools/map_compare.py --duration 62

Produces in ``results/``:

    result_map_compare.png   true map, estimated map and their difference
    map_compare_data.npz     raw grids for ``--replot``

Metrics printed to the console (and used in the figure title):

    distance            mean/max distance from the occupied cells of one map to
                        the nearest occupied cell of the other (metres) - the
                        primary accuracy measure, ~ one cell for a good pose;
    occupied IoU        intersection over union of occupied cells (sensitive:
                        a one-cell shift already halves it for thin walls);
    agreement           share of equal cells among those known in both maps;
    only truth/estimate occupied in one map only.

Re-render without the simulation::

    python3 tools/map_compare.py --replot
"""

import argparse
import os
import sys
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np

WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUT_DIR = os.path.join(WORKSPACE_DIR, 'results')
DEFAULT_DURATION = 62.0
DATA_NAME = 'map_compare_data.npz'

try:
    from navigator_slam.occupancy_grid import (
        FREE,
        OCCUPIED,
        UNKNOWN,
        compare_occupancy_data,
    )
except ImportError:  # workspace not built/sourced: fall back to the source tree
    sys.path.insert(0, os.path.join(WORKSPACE_DIR, 'src', 'navigator_slam'))
    from navigator_slam.occupancy_grid import (  # noqa: E402
        FREE,
        OCCUPIED,
        UNKNOWN,
        compare_occupancy_data,
    )

MAP_TOPICS = {
    'ground_truth': '/map/ground_truth',
    'estimate': '/map/estimate',
}

COLOR_UNKNOWN = (0.78, 0.78, 0.78)
COLOR_FREE = (1.0, 1.0, 1.0)
COLOR_OCCUPIED = (0.08, 0.08, 0.08)


def record(duration):
    """Collect the latest map from every topic (maps grow during the run)."""
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node

    rclpy.init()
    node = Node('map_compare')
    received = {}

    def make_callback(key):
        def callback(msg):
            received[key] = msg
        return callback

    for key, topic in MAP_TOPICS.items():
        node.create_subscription(OccupancyGrid, topic, make_callback(key), 1)
    node.get_logger().info(
        'recording %.1f s of %s'
        % (duration, ', '.join(MAP_TOPICS.values()))
    )

    started = time.monotonic()
    try:
        while time.monotonic() - started < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    missing = [key for key in MAP_TOPICS if key not in received]
    if missing:
        raise SystemExit(
            'no map on %s; is mapping.launch.py running with pose_source:=both?'
            % ', '.join(MAP_TOPICS[key] for key in missing)
        )
    return received


def messages_to_arrays(messages):
    """Convert the two OccupancyGrid messages into (height, width) int8 maps."""
    true_map = messages['ground_truth']
    estimated = messages['estimate']

    if (true_map.info.width, true_map.info.height) != (
        estimated.info.width, estimated.info.height
    ):
        raise SystemExit(
            'map sizes differ: %dx%d vs %dx%d'
            % (
                true_map.info.width, true_map.info.height,
                estimated.info.width, estimated.info.height,
            )
        )
    if abs(true_map.info.resolution - estimated.info.resolution) > 1e-9:
        raise SystemExit('map resolutions differ')
    if (
        abs(true_map.info.origin.position.x - estimated.info.origin.position.x) > 1e-6
        or abs(true_map.info.origin.position.y - estimated.info.origin.position.y) > 1e-6
    ):
        raise SystemExit('map origins differ')

    shape = (true_map.info.height, true_map.info.width)
    arrays = {
        key: np.asarray(messages[key].data, dtype=np.int8).reshape(shape)
        for key in MAP_TOPICS
    }
    resolution = float(true_map.info.resolution)
    origin = (
        float(true_map.info.origin.position.x),
        float(true_map.info.origin.position.y),
    )
    return arrays['ground_truth'], arrays['estimate'], resolution, origin


def map_rgb(data):
    """Colour image for one occupancy map (unknown/free/occupied)."""
    image = np.empty(data.shape + (3,), dtype=float)
    image[data == UNKNOWN] = COLOR_UNKNOWN
    image[data == FREE] = COLOR_FREE
    image[data == OCCUPIED] = COLOR_OCCUPIED
    # Keep any unexpected value visible instead of leaving it uninitialised.
    known = (data == UNKNOWN) | (data == FREE) | (data == OCCUPIED)
    image[~known] = (1.0, 0.0, 1.0)
    return image


def difference_rgb(true_data, estimated_data):
    """Colour image of the occupied-cell differences between the two maps."""
    image = np.full(true_data.shape + (3,), 0.92, dtype=float)
    true_occupied = true_data == OCCUPIED
    estimated_occupied = estimated_data == OCCUPIED
    image[true_occupied & estimated_occupied] = (0.05, 0.05, 0.05)
    image[true_occupied & ~estimated_occupied] = (0.85, 0.10, 0.10)
    image[~true_occupied & estimated_occupied] = (0.10, 0.40, 0.90)
    return image


def render(true_data, estimated_data, resolution, origin, metrics, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    height, width = true_data.shape
    extent = [
        origin[0],
        origin[0] + width * resolution,
        origin[1],
        origin[1] + height * resolution,
    ]

    figure, axes = plt.subplots(1, 3, figsize=(16.0, 5.8))
    panels = (
        ('Map from true pose', map_rgb(true_data)),
        ('Map from EKF estimate', map_rgb(estimated_data)),
        (
            'Difference (mean distance = %.3f m)'
            % metrics['mean_distance_ab'],
            difference_rgb(true_data, estimated_data),
        ),
    )
    for axes_item, (title, image) in zip(axes, panels):
        axes_item.imshow(
            image,
            origin='lower',
            extent=extent,
            interpolation='nearest',
        )
        axes_item.set_title(title)
        axes_item.set_xlabel('x [m]')
        axes_item.set_ylabel('y [m]')
        axes_item.set_xlim(extent[0], extent[1])
        axes_item.set_ylim(extent[2], extent[3])
        axes_item.set_aspect('equal', adjustable='box')
        axes_item.grid(True, color='0.65', linewidth=0.4, alpha=0.4)

    axes[0].legend(
        handles=[
            Patch(facecolor=COLOR_OCCUPIED, label='occupied'),
            Patch(facecolor=COLOR_FREE, edgecolor='0.6', label='free'),
            Patch(facecolor=COLOR_UNKNOWN, label='unknown'),
        ],
        loc='upper right',
        fontsize=8,
    )
    axes[2].legend(
        handles=[
            Patch(facecolor=(0.05, 0.05, 0.05), label='occupied in both'),
            Patch(facecolor=(0.85, 0.10, 0.10), label='occupied only in true map'),
            Patch(facecolor=(0.10, 0.40, 0.90), label='occupied only in estimate'),
        ],
        loc='upper right',
        fontsize=8,
    )

    figure.suptitle(
        'Occupancy grid: true vs estimated pose   '
        'mean distance %.3f m, IoU %.3f, agreement %.4f'
        % (
            metrics['mean_distance_ab'],
            metrics['iou_occupied'],
            metrics['agreement'],
        )
    )
    figure.tight_layout()
    path = os.path.join(out_dir, 'result_map_compare.png')
    figure.savefig(path, dpi=140)
    plt.close(figure)
    return path


def main():
    parser = argparse.ArgumentParser(
        description='Compare occupancy maps built from true and estimated poses.'
    )
    parser.add_argument('--duration', type=float, default=DEFAULT_DURATION,
                        help='recording duration in seconds (default: %(default)s)')
    parser.add_argument('--out', default=DEFAULT_OUT_DIR,
                        help='output directory (default: %(default)s)')
    parser.add_argument('--replot', action='store_true',
                        help='re-render from the saved npz instead of recording')
    args = parser.parse_args()

    data_path = os.path.join(args.out, DATA_NAME)
    if args.replot:
        if not os.path.exists(data_path):
            raise SystemExit('no data at %s; run once without --replot' % data_path)
        with np.load(data_path) as data:
            true_data = data['ground_truth']
            estimated_data = data['estimate']
            resolution = float(data['resolution'])
            origin = tuple(float(value) for value in data['origin'])
    else:
        true_data, estimated_data, resolution, origin = messages_to_arrays(
            record(args.duration)
        )
        os.makedirs(args.out, exist_ok=True)
        np.savez_compressed(
            data_path,
            ground_truth=true_data,
            estimate=estimated_data,
            resolution=resolution,
            origin=np.asarray(origin, dtype=float),
        )

    metrics = compare_occupancy_data(
        true_data, estimated_data, resolution=resolution
    )
    figure_path = render(
        true_data, estimated_data, resolution, origin, metrics, args.out
    )

    print(
        'occupied cells: true %d, estimate %d, in both %d'
        % (metrics['occupied_a'], metrics['occupied_b'], metrics['both_occupied'])
    )
    print(
        'only true pose %d, only estimate %d, cells known in both %d'
        % (metrics['only_a'], metrics['only_b'], metrics['known_both'])
    )
    print(
        'occupied distance true->estimate: mean %.3f m, max %.3f m'
        % (metrics['mean_distance_ab'], metrics['max_distance_ab'])
    )
    print(
        'occupied distance estimate->true: mean %.3f m, max %.3f m'
        % (metrics['mean_distance_ba'], metrics['max_distance_ba'])
    )
    print('occupied IoU: %.4f' % metrics['iou_occupied'])
    print('cell agreement (known in both): %.4f' % metrics['agreement'])
    print('figure: %s' % figure_path)


if __name__ == '__main__':
    main()
