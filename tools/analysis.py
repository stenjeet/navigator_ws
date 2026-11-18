#!/usr/bin/env python3
"""Efficiency analysis: localization RMSE and map accuracy across scenarios.

Every scenario launches ``navigator_bringup/mapping.launch.py`` headless with
its own copies of ``sensors.yaml`` and ``ekf.yaml``, records ``/estimate/error``
(and ``/slam/error`` when stage 8 runs) plus the final ``/map/ground_truth`` and
``/map/estimate`` grids, and aggregates the run into one summary and two
figures.

Built-in scenario table (default profile, 62 s of simulation per scenario; the
values are starting points for the report and can be changed in a YAML file
passed with ``--scenarios``):

    scenario             sensors.yaml                     ekf.yaml
    -------------------  -------------------------------  ------------------
    baseline             nominal                          nominal
    uwb_sigma_0.02       uwb_sigma 0.02                   uwb_sigma 0.02
    uwb_sigma_0.10       uwb_sigma 0.10                   uwb_sigma 0.10
    uwb_sigma_0.20       uwb_sigma 0.20                   uwb_sigma 0.20
    uwb_mismatch_r_low   uwb_sigma 0.10                   uwb_sigma 0.05
    dropout_0.10         uwb_dropout_probability 0.10     nominal
    dropout_0.25         uwb_dropout_probability 0.25     nominal
    outage_20_40         outage 20..40 s                  nominal
    anchors_wide         anchors in the arena corners     same anchors
    anchors_compact      anchors near the start pose      same anchors

``--quick`` runs ``baseline``, ``uwb_sigma_0.10`` and ``dropout_0.25`` with a
30 s duration per scenario (one lap of the square trajectory lasts ~62 s, so a
quick run covers roughly half a lap).

Scenario overrides are applied on top of the package configs by a deep merge:
``sensors`` holds flat ``sensors.yaml`` parameters, ``ekf`` holds flat
``ekf.yaml`` parameters. The generated copies live in ``<out>/configs`` and are
passed to the simulation through the ``sensors_params`` / ``ekf_params`` launch
arguments of ``simulation.launch.py``. ``mapping.launch.py`` includes that file
without forwarding extra arguments, so the paths also travel in the
``NAVIGATOR_SENSORS_PARAMS`` / ``NAVIGATOR_EKF_PARAMS`` environment variables,
which ``simulation.launch.py`` uses as the defaults of those two arguments.

Outputs (``--out`` defaults to ``results/analysis``, figures to ``results/``):

    summary.md            Markdown table plus auto-generated findings
    summary.csv           machine-readable metrics, one row per scenario
    analysis_data.npz     metrics for ``--replot``
    ../analysis_rmse.png  position and yaw RMSE per scenario
    ../analysis_maps.png  occupied IoU and mean occupied-cell distance

Re-render everything from the saved data without touching ROS::

    python3 tools/analysis.py --replot

The tool requires matplotlib and numpy, a built workspace
(``colcon build --symlink-install``) and a sourced ROS 2 environment
(``./run_analysis.sh`` does both checks for you).
"""

import argparse
import copy
import csv
import io
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np
import yaml

WORKSPACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUT_DIR = os.path.join(WORKSPACE_DIR, 'results', 'analysis')
SENSORS_CONFIG_PATH = os.path.join(
    WORKSPACE_DIR, 'src', 'navigator_sim', 'config', 'sensors.yaml'
)
EKF_CONFIG_PATH = os.path.join(
    WORKSPACE_DIR, 'src', 'navigator_localization', 'config', 'ekf.yaml'
)

DEFAULT_DURATION = 62.0
QUICK_DURATION = 30.0
DEFAULT_STARTUP_TIMEOUT = 25.0

DATA_NAME = 'analysis_data.npz'
SUMMARY_MD = 'summary.md'
SUMMARY_CSV = 'summary.csv'
RMSE_FIGURE = 'analysis_rmse.png'
MAPS_FIGURE = 'analysis_maps.png'

NAN = float('nan')

ERROR_TOPICS = {
    'estimate': '/estimate/error',
    'slam': '/slam/error',
}
MAP_TOPICS = {
    'ground_truth': '/map/ground_truth',
    'estimate': '/map/estimate',
}

# The launch file gives the two sensor models these node names, while
# ``sensors.yaml`` keeps every sensor parameter in one shared ``sensor_models``
# section. Scenario copies therefore repeat the parameters under the real node
# names so that the overrides are actually applied.
SENSOR_NODE_NAMES = ('uwb_model_node', 'lidar_model_node')
SENSOR_SHARED_PARAMS = ('random_seed',)

LEFTOVER_PATTERNS = (
    'lib/navigator_sim',
    'lib/navigator_localization',
    'lib/navigator_slam',
)

BASELINE_COLOR = '#d95f02'
SERIES_COLOR = '#1b6ca8'

# Switch the position RMSE panel to a log scale only when the scenarios span
# more than this factor, so a tight cluster is not flattened by the axis.
LOG_SCALE_SPREAD = 1.5

try:
    from navigator_slam.occupancy_grid import compare_occupancy_data
except ImportError:  # workspace not built/sourced: fall back to the source tree
    sys.path.insert(0, os.path.join(WORKSPACE_DIR, 'src', 'navigator_slam'))
    from navigator_slam.occupancy_grid import compare_occupancy_data  # noqa: E402

ANCHOR_HEIGHT = 1.20
ANCHORS_WIDE = ((0.4, 0.4), (7.6, 0.4), (7.6, 7.6), (0.4, 7.6))
ANCHORS_COMPACT = ((2.0, 2.0), (2.6, 2.0), (2.0, 2.6), (2.6, 2.6))

METRIC_KEYS = (
    'duration',
    'samples',
    'rmse_x',
    'rmse_y',
    'rmse_pos',
    'rmse_yaw',
    'max_pos',
    'max_yaw',
    'iou_occupied',
    'mean_distance',
    'max_distance',
    'slam_samples',
    'slam_rmse_pos',
    'slam_rmse_yaw',
    'slam_max_pos',
    'wall_time',
)


class ScenarioError(RuntimeError):
    """A scenario could not be recorded (launch failure, missing topics)."""


# ----------------------------------------------------------------------
# Scenario table
# ----------------------------------------------------------------------
def flat_anchors(points, height=ANCHOR_HEIGHT):
    """Flat ``x, y, z`` triples for the ``anchors`` parameter."""
    return [
        coordinate
        for x, y in points
        for coordinate in (float(x), float(y), float(height))
    ]


DEFAULT_PROFILE = [
    {
        'name': 'baseline',
        'sensors': {},
        'ekf': {},
    },
    {
        'name': 'uwb_sigma_0.02',
        'sensors': {'uwb_sigma': 0.02},
        'ekf': {'uwb_sigma': 0.02},
    },
    {
        'name': 'uwb_sigma_0.10',
        'sensors': {'uwb_sigma': 0.10},
        'ekf': {'uwb_sigma': 0.10},
    },
    {
        'name': 'uwb_sigma_0.20',
        'sensors': {'uwb_sigma': 0.20},
        'ekf': {'uwb_sigma': 0.20},
    },
    {
        'name': 'uwb_mismatch_r_low',
        'sensors': {'uwb_sigma': 0.10},
        'ekf': {'uwb_sigma': 0.05},
    },
    {
        'name': 'dropout_0.10',
        'sensors': {'uwb_dropout_probability': 0.10},
        'ekf': {},
    },
    {
        'name': 'dropout_0.25',
        'sensors': {'uwb_dropout_probability': 0.25},
        'ekf': {},
    },
    {
        'name': 'outage_20_40',
        'sensors': {'outage_start_s': 20.0, 'outage_end_s': 40.0},
        'ekf': {},
    },
    {
        'name': 'anchors_wide',
        'sensors': {'anchors': flat_anchors(ANCHORS_WIDE)},
        'ekf': {'anchors': flat_anchors(ANCHORS_WIDE)},
    },
    {
        'name': 'anchors_compact',
        'sensors': {'anchors': flat_anchors(ANCHORS_COMPACT)},
        'ekf': {'anchors': flat_anchors(ANCHORS_COMPACT)},
    },
]

QUICK_NAMES = ('baseline', 'uwb_sigma_0.10', 'dropout_0.25')


def deep_merge(base, override):
    """Recursively merge ``override`` into a copy of ``base``.

    Dictionaries are merged key by key, everything else (lists, scalars) is
    replaced by the override. ``base`` is never modified.
    """
    merged = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def build_scenarios(quick=False, duration=None, table=None):
    """Expand the scenario table into runnable specifications.

    ``quick`` keeps ``QUICK_NAMES`` and a 30 s default duration; a custom
    ``table`` (from ``--scenarios``) is used as-is, because its scenario names
    and durations are not known in advance. An explicit ``duration`` always
    wins.
    """
    custom = table is not None
    items = copy.deepcopy(DEFAULT_PROFILE if table is None else table)
    if quick and not custom:
        items = [item for item in items if item.get('name') in QUICK_NAMES]
    default_duration = (
        QUICK_DURATION if quick and not custom else DEFAULT_DURATION
    )

    scenarios = []
    seen = set()
    for item in items:
        name = str(item.get('name') or '').strip()
        if not name:
            raise ValueError('every scenario needs a non-empty name')
        if name in seen:
            raise ValueError('duplicate scenario name %r' % name)
        seen.add(name)

        scenario = {
            'name': name,
            'duration': float(item.get('duration', default_duration)),
            'sensors': dict(item.get('sensors') or {}),
            'ekf': dict(item.get('ekf') or {}),
            'use_slam': bool(item.get('use_slam', False)),
        }
        if duration is not None:
            scenario['duration'] = float(duration)
        if not math.isfinite(scenario['duration']) or scenario['duration'] <= 0.0:
            raise ValueError('scenario %r needs a positive duration' % name)
        scenarios.append(scenario)

    if not scenarios:
        raise ValueError('the scenario table is empty')
    return scenarios


def load_scenario_file(path):
    """Read a ``--scenarios`` YAML file into a scenario table.

    The file is either a list of scenarios or a mapping with ``defaults``
    (merged into every scenario) and ``scenarios``.
    """
    document = load_yaml(path)
    if isinstance(document, list):
        defaults, items = {}, document
    elif isinstance(document, dict):
        defaults = document.get('defaults') or {}
        items = document.get('scenarios') or []
    else:
        raise ValueError('%s: expected a list or a mapping' % path)
    if not isinstance(items, list) or not items:
        raise ValueError('%s: no scenarios defined' % path)

    table = []
    for item in items:
        if not isinstance(item, dict):
            raise ValueError('%s: every scenario must be a mapping' % path)
        merged = deep_merge(defaults, item)
        table.append(merged)
    return table


# ----------------------------------------------------------------------
# Configuration files
# ----------------------------------------------------------------------
def load_yaml(path):
    with open(path, 'r') as stream:
        return yaml.safe_load(stream)


def load_node_params(path):
    """Flat ``ros__parameters`` mapping of a single-section parameter file."""
    document = load_yaml(path)
    if not isinstance(document, dict) or not document:
        raise ValueError('%s: expected a parameter file' % path)
    sections = list(document.items())
    if len(sections) > 1:
        raise ValueError('%s: expected exactly one node section' % path)
    node_name, section = sections[0]
    parameters = (section or {}).get('ros__parameters')
    if not isinstance(parameters, dict):
        raise ValueError('%s: section %r has no ros__parameters' % (path, node_name))
    return dict(parameters)


def load_ekf_document(path=EKF_CONFIG_PATH):
    """Raw ``ekf.yaml`` document, kept so the scenario copy keeps its shape."""
    document = load_yaml(path)
    if not isinstance(document, dict) or not document:
        raise ValueError('%s: expected a parameter file' % path)
    return document


def write_sensor_config(path, base_params, overrides=None):
    """Write a ``sensors.yaml`` copy with the scenario overrides applied.

    Both sensor nodes launched by ``simulation.launch.py`` receive this file,
    so the parameters are repeated under ``uwb_model_node`` and
    ``lidar_model_node`` (the names the launch file gives the nodes, while
    ``sensors.yaml`` keeps everything in one shared ``sensor_models`` section).
    ``uwb_*``/``outage_*``/``anchors`` overrides go to the UWB node and
    ``lidar_*`` overrides to the lidar node; each section only carries the
    parameters of its own node.
    """
    overrides = overrides or {}
    uwb_base = {
        key: value for key, value in base_params.items()
        if not key.startswith('lidar_')
    }
    lidar_base = {
        key: value for key, value in base_params.items()
        if key in SENSOR_SHARED_PARAMS or key.startswith('lidar_')
    }
    uwb_overrides = {
        key: value for key, value in overrides.items()
        if not key.startswith('lidar_')
    }
    lidar_overrides = {
        key: value for key, value in overrides.items()
        if key.startswith('lidar_')
    }
    document = {
        'uwb_model_node': {
            'ros__parameters': deep_merge(uwb_base, uwb_overrides),
        },
        'lidar_model_node': {
            'ros__parameters': deep_merge(lidar_base, lidar_overrides),
        },
    }
    write_yaml(path, document)
    return path


def write_ekf_config(path, document, overrides=None):
    """Write an ``ekf.yaml`` copy with the scenario overrides applied."""
    node_name, section = list(document.items())[0]
    parameters = deep_merge((section or {}).get('ros__parameters') or {}, overrides)
    write_yaml(path, {node_name: {'ros__parameters': parameters}})
    return path


def write_yaml(path, document):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, 'w') as stream:
        yaml.safe_dump(document, stream, sort_keys=False, default_flow_style=False)


def write_scenario_configs(scenario, config_dir, sensor_base, ekf_document):
    """Write ``<name>_sensors.yaml`` and ``<name>_ekf.yaml`` for a scenario."""
    os.makedirs(config_dir, exist_ok=True)
    sensors_path = os.path.join(config_dir, '%s_sensors.yaml' % scenario['name'])
    ekf_path = os.path.join(config_dir, '%s_ekf.yaml' % scenario['name'])
    write_sensor_config(sensors_path, sensor_base, scenario.get('sensors'))
    write_ekf_config(ekf_path, ekf_document, scenario.get('ekf'))
    return sensors_path, ekf_path


def format_scenario_plan(scenarios):
    """One human-readable line per scenario, printed before the run."""
    lines = []
    for scenario in scenarios:
        overrides = []
        for prefix, section in (
            ('sensors', scenario.get('sensors', {})),
            ('ekf', scenario.get('ekf', {})),
        ):
            for key, value in section.items():
                if isinstance(value, list):
                    overrides.append('%s.%s=[%d values]' % (prefix, key, len(value)))
                else:
                    overrides.append('%s.%s=%s' % (prefix, key, value))
        lines.append(
            '  %-20s %5.0f s  %s'
            % (scenario['name'], scenario['duration'], ', '.join(overrides) or 'nominal')
        )
    return lines


# ----------------------------------------------------------------------
# Metrics
# ----------------------------------------------------------------------
def finite_number(value):
    """``value`` as a float when it is a finite number, otherwise ``None``."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def format_value(value, spec='%.3f', missing='n/a'):
    """Format a metric for the Markdown table without NaN/inf."""
    number = finite_number(value)
    return missing if number is None else spec % number


def format_int(value, missing='n/a'):
    number = finite_number(value)
    return missing if number is None else '%d' % int(round(number))


def format_csv(value, spec='%.6g'):
    """Format a metric for the CSV file; missing values stay empty."""
    number = finite_number(value)
    return '' if number is None else spec % number


def aggregate_errors(samples):
    """RMSE and peak values of ``(x, y, yaw)`` error samples.

    Rows with a non-finite component are dropped, so a partially broken error
    stream does not poison the aggregate. Yaw is in radians, as published by
    ``/estimate/error`` and ``/slam/error``.
    """
    values = np.asarray(
        np.empty((0, 3)) if samples is None else samples, dtype=float
    ).reshape(-1, 3)
    if values.shape[0]:
        values = values[np.all(np.isfinite(values), axis=1)]

    if values.shape[0] == 0:
        return {
            'samples': 0,
            'rmse_x': NAN,
            'rmse_y': NAN,
            'rmse_pos': NAN,
            'rmse_yaw': NAN,
            'max_pos': NAN,
            'max_yaw': NAN,
        }

    error_x = values[:, 0]
    error_y = values[:, 1]
    error_yaw = values[:, 2]
    position = np.hypot(error_x, error_y)
    return {
        'samples': int(values.shape[0]),
        'rmse_x': float(np.sqrt(np.mean(error_x ** 2))),
        'rmse_y': float(np.sqrt(np.mean(error_y ** 2))),
        'rmse_pos': float(np.sqrt(np.mean(position ** 2))),
        'rmse_yaw': float(np.sqrt(np.mean(error_yaw ** 2))),
        'max_pos': float(np.max(position)),
        'max_yaw': float(np.max(np.abs(error_yaw))),
    }


def map_metrics(ground_truth, estimate, resolution):
    """IoU and chamfer-style distance of the two final occupancy grids."""
    missing = {
        'iou_occupied': NAN,
        'mean_distance': NAN,
        'max_distance': NAN,
    }
    if ground_truth is None or estimate is None:
        return dict(missing)
    if np.asarray(ground_truth).shape != np.asarray(estimate).shape:
        return dict(missing)
    if finite_number(resolution) is None or float(resolution) <= 0.0:
        return dict(missing)

    result = compare_occupancy_data(
        np.asarray(ground_truth), np.asarray(estimate),
        resolution=float(resolution),
    )
    return {
        'iou_occupied': float(result['iou_occupied']),
        'mean_distance': float(result['mean_distance_ab']),
        'max_distance': float(result['max_distance_ab']),
    }


def scenario_row(scenario, recording, wall_time=None, log_path=None):
    """One summary row: scenario metadata plus the recorded metrics."""
    row = {
        'name': scenario['name'],
        'status': 'ok',
        'duration': float(scenario['duration']),
        'use_slam': bool(scenario.get('use_slam', False)),
        'spec': {
            'sensors': dict(scenario.get('sensors') or {}),
            'ekf': dict(scenario.get('ekf') or {}),
        },
    }
    row.update(aggregate_errors(recording.get('error')))

    slam = aggregate_errors(recording.get('slam_error'))
    row['slam_samples'] = slam['samples']
    row['slam_rmse_pos'] = slam['rmse_pos']
    row['slam_rmse_yaw'] = slam['rmse_yaw']
    row['slam_max_pos'] = slam['max_pos']

    row.update(map_metrics(
        recording.get('map_ground_truth'),
        recording.get('map_estimate'),
        recording.get('resolution'),
    ))
    if wall_time is not None:
        row['wall_time'] = float(wall_time)
    if log_path:
        row['log'] = log_path
    return row


def failed_row(scenario, message):
    """A row that marks a scenario without usable data."""
    row = scenario_row(scenario, {})
    row['status'] = 'failed'
    lines = [line.strip() for line in str(message).splitlines() if line.strip()]
    row['error'] = lines[0] if lines else 'failed'
    return row


# ----------------------------------------------------------------------
# Recording (ROS 2)
# ----------------------------------------------------------------------
def tail_text(path, lines=15):
    """Last ``lines`` of a log file, for error messages."""
    if not path or not os.path.exists(path):
        return ''
    try:
        with open(path, 'r', errors='replace') as stream:
            content = stream.readlines()
    except OSError:
        return ''
    return ''.join(content[-lines:])


def check_process(process, log_path):
    """Fail as soon as the launch process is gone."""
    code = process.poll()
    if code is not None:
        raise ScenarioError(
            'the launch process exited with code %s\n%s'
            % (code, tail_text(log_path))
        )


def message_to_grid(message):
    """``(data, resolution)`` of an ``OccupancyGrid`` message."""
    height = int(message.info.height)
    width = int(message.info.width)
    data = np.asarray(message.data, dtype=np.int8).reshape(height, width)
    return data, float(message.info.resolution)


def record_scenario(duration, process, log_path, expect_slam=False,
                    startup_timeout=DEFAULT_STARTUP_TIMEOUT):
    """Record one scenario while the launch subprocess runs.

    Waits for the first ``/estimate/error`` sample (a clear failure otherwise),
    then collects ``duration`` seconds of ``/estimate/error``, ``/slam/error``
    and the final ``/map/ground_truth`` + ``/map/estimate`` grids.
    """
    try:
        import rclpy
        from geometry_msgs.msg import Vector3Stamped
        from nav_msgs.msg import OccupancyGrid
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
    except ImportError as error:
        raise ScenarioError(
            'ROS 2 Python packages are not available (%s); source the ROS 2 '
            'environment first (./run_analysis.sh does it)' % error
        )

    estimate = []
    slam = []
    grids = {}

    rclpy.init()
    node = Node('stage9_analysis_recorder')
    try:
        def error_callback(target):
            def callback(message):
                target.append((
                    float(message.vector.x),
                    float(message.vector.y),
                    float(message.vector.z),
                ))
            return callback

        def map_callback(key):
            def callback(message):
                grids[key] = message
            return callback

        node.create_subscription(
            Vector3Stamped, ERROR_TOPICS['estimate'],
            error_callback(estimate), 50,
        )
        node.create_subscription(
            Vector3Stamped, ERROR_TOPICS['slam'],
            error_callback(slam), 50,
        )
        for key, topic in MAP_TOPICS.items():
            node.create_subscription(
                OccupancyGrid, topic, map_callback(key), 1,
            )

        waiting_until = time.monotonic() + float(startup_timeout)
        while not estimate:
            check_process(process, log_path)
            if time.monotonic() > waiting_until:
                raise ScenarioError(
                    'no message on %s within %.0f s (is the EKF running and the '
                    'workspace built?)\n%s'
                    % (
                        ERROR_TOPICS['estimate'], startup_timeout,
                        tail_text(log_path),
                    )
                )
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except ExternalShutdownException:
                raise KeyboardInterrupt

        deadline = time.monotonic() + float(duration)
        while time.monotonic() < deadline:
            check_process(process, log_path)
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except ExternalShutdownException:
                raise KeyboardInterrupt
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    ground_truth = estimate_map = None
    resolution = NAN
    if 'ground_truth' in grids and 'estimate' in grids:
        ground_truth, resolution = message_to_grid(grids['ground_truth'])
        estimate_map, estimate_resolution = message_to_grid(grids['estimate'])
        if abs(estimate_resolution - resolution) > 1e-9:
            print(
                '    warning: map resolutions differ (%.3f vs %.3f m)'
                % (resolution, estimate_resolution), flush=True,
            )
            resolution = NAN
    elif grids:
        missing = [key for key in MAP_TOPICS if key not in grids]
        print(
            '    warning: no map on %s; map metrics are unavailable'
            % ', '.join(MAP_TOPICS[key] for key in missing), flush=True,
        )
    else:
        print(
            '    warning: no occupancy grid received on %s; map metrics are '
            'unavailable' % ', '.join(MAP_TOPICS.values()), flush=True,
        )

    if expect_slam and not slam:
        print(
            '    warning: no message on %s although the scenario expects SLAM'
            % ERROR_TOPICS['slam'], flush=True,
        )

    return {
        'error': np.asarray(estimate, dtype=float).reshape(-1, 3),
        'slam_error': np.asarray(slam, dtype=float).reshape(-1, 3),
        'map_ground_truth': ground_truth,
        'map_estimate': estimate_map,
        'resolution': resolution,
    }


# ----------------------------------------------------------------------
# Child process management
# ----------------------------------------------------------------------
def cleanup_leftovers():
    """Kill simulation nodes left behind by an interrupted scenario."""
    for pattern in LEFTOVER_PATTERNS:
        subprocess.run(
            ['pkill', '-9', '-f', pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )


def launch_command(sensors_path, ekf_path):
    """The launch command of one scenario (mirrors ``run_map_compare.sh``)."""
    return [
        'ros2', 'launch', 'navigator_bringup', 'mapping.launch.py',
        'use_rviz:=false',
        'pose_source:=both',
        'sensors_params:=%s' % sensors_path,
        'ekf_params:=%s' % ekf_path,
    ]


def child_environment(sensors_path, ekf_path):
    """Environment for the launch subprocess.

    ``mapping.launch.py`` does not forward extra launch arguments to the
    included ``simulation.launch.py``, so the scenario config paths are also
    exported; ``simulation.launch.py`` uses them as the defaults of the
    ``sensors_params`` and ``ekf_params`` arguments.
    """
    environment = dict(os.environ)
    environment['NAVIGATOR_SENSORS_PARAMS'] = os.path.abspath(sensors_path)
    environment['NAVIGATOR_EKF_PARAMS'] = os.path.abspath(ekf_path)
    environment.setdefault('PYTHONUNBUFFERED', '1')
    return environment


def stop_launch(process, timeout=8.0):
    """Send SIGINT (the Ctrl-C equivalent), then kill whatever is left."""
    if process is None:
        return
    if process.poll() is None:
        try:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            process.kill()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                pass
    cleanup_leftovers()


def workspace_checks(workspace=WORKSPACE_DIR):
    """Fail loudly unless the workspace is built and ROS 2 is available."""
    if not os.path.exists(os.path.join(workspace, 'install', 'setup.bash')):
        raise SystemExit(
            'the workspace at %s is not built (no install/setup.bash); run '
            'colcon build --symlink-install first' % workspace
        )
    if shutil.which('ros2') is None:
        raise SystemExit(
            'ros2 is not on PATH; source ${ROS_SETUP:-/opt/ros/jazzy/setup.bash} '
            'or run ./run_analysis.sh instead of the tool directly'
        )
    try:
        import rclpy  # noqa: F401
    except ImportError as error:
        raise SystemExit(
            'the ROS 2 Python packages are not importable (%s); source '
            '${ROS_SETUP:-/opt/ros/jazzy/setup.bash} and install/setup.bash '
            'first, or run ./run_analysis.sh' % error
        )


def run_one_scenario(scenario, args, config_dir, sensor_base, ekf_document):
    """Configure, launch, record and stop a single scenario."""
    sensors_path, ekf_path = write_scenario_configs(
        scenario, config_dir, sensor_base, ekf_document
    )
    log_path = os.path.join(config_dir, '%s_launch.log' % scenario['name'])
    command = launch_command(sensors_path, ekf_path)

    cleanup_leftovers()
    started = time.monotonic()
    with open(log_path, 'wb') as log_stream:
        process = subprocess.Popen(
            command,
            cwd=WORKSPACE_DIR,
            env=child_environment(sensors_path, ekf_path),
            stdout=log_stream,
            stderr=subprocess.STDOUT,
        )
        try:
            recording = record_scenario(
                scenario['duration'],
                process,
                log_path,
                expect_slam=scenario.get('use_slam', False),
                startup_timeout=getattr(
                    args, 'startup_timeout', DEFAULT_STARTUP_TIMEOUT
                ),
            )
        finally:
            stop_launch(process)

    return scenario_row(
        scenario, recording,
        wall_time=time.monotonic() - started,
        log_path=log_path,
    )


def run_scenarios(scenarios, args, out_dir):
    """Run every scenario sequentially and return the summary rows."""
    config_dir = os.path.join(out_dir, 'configs')
    os.makedirs(config_dir, exist_ok=True)
    sensor_base = load_node_params(SENSORS_CONFIG_PATH)
    ekf_document = load_ekf_document(EKF_CONFIG_PATH)

    rows = []
    for index, scenario in enumerate(scenarios, 1):
        print(
            '[%d/%d] %s, %.0f s'
            % (index, len(scenarios), scenario['name'], scenario['duration']),
            flush=True,
        )
        try:
            row = run_one_scenario(
                scenario, args, config_dir, sensor_base, ekf_document
            )
        except ScenarioError as error:
            print('    failed: %s' % error, file=sys.stderr, flush=True)
            rows.append(failed_row(scenario, str(error)))
            continue
        rows.append(row)
        print('    %s' % format_row_line(row), flush=True)
    return rows


def format_row_line(row):
    """Compact console line for one finished scenario."""
    line = (
        'RMSE pos %s m (x %s, y %s, yaw %s deg), samples %s; '
        'map IoU %s, mean distance %s m; SLAM RMSE pos %s m'
        % (
            format_value(row.get('rmse_pos')),
            format_value(row.get('rmse_x')),
            format_value(row.get('rmse_y')),
            format_value(
                None if finite_number(row.get('rmse_yaw')) is None
                else math.degrees(row['rmse_yaw'])
            ),
            format_int(row.get('samples')),
            format_value(row.get('iou_occupied')),
            format_value(row.get('mean_distance')),
            format_value(row.get('slam_rmse_pos')),
        )
    )
    if row.get('status') == 'failed':
        line = 'no data (%s)' % row.get('error', 'failed')
    return line


# ----------------------------------------------------------------------
# Summary rendering
# ----------------------------------------------------------------------
TABLE_COLUMNS = (
    ('Scenario', lambda row: str(row['name'])),
    ('Duration [s]', lambda row: format_value(row.get('duration'), '%.0f')),
    ('Samples', lambda row: format_int(row.get('samples'))),
    ('RMSE x [m]', lambda row: format_value(row.get('rmse_x'))),
    ('RMSE y [m]', lambda row: format_value(row.get('rmse_y'))),
    ('RMSE pos [m]', lambda row: format_value(row.get('rmse_pos'))),
    ('RMSE yaw [deg]', lambda row: format_value(yaw_degrees(row.get('rmse_yaw')))),
    ('max \\|e\\| [m]', lambda row: format_value(row.get('max_pos'))),
    ('IoU occupied', lambda row: format_value(row.get('iou_occupied'))),
    ('Mean dist [m]', lambda row: format_value(row.get('mean_distance'))),
    ('SLAM RMSE [m]', lambda row: format_value(row.get('slam_rmse_pos'))),
)

SUMMARY_NOTES = (
    'Metrics: `RMSE` is computed from `/estimate/error` (`x`, `y` and the yaw '
    'error in radians, shown in degrees); `max |e|` is the peak position error; '
    '`IoU occupied` compares the occupied cells of the final '
    '`/map/ground_truth` and `/map/estimate` grids and `Mean dist` is the mean '
    'distance from an occupied cell of one map to the nearest occupied cell of '
    'the other. `n/a` marks a scenario without data, for example the SLAM '
    'column when stage 8 is not part of the workspace.'
)


def yaw_degrees(value):
    number = finite_number(value)
    return None if number is None else math.degrees(number)


def scenario_parameter(row, section, key, default=None):
    """A recorded scenario override from the ``sensors`` or ``ekf`` section."""
    spec = row.get('spec') or {}
    return (spec.get(section) or {}).get(key, default)



def render_summary_markdown(rows, title='Stage 9 - efficiency analysis'):
    """Markdown summary: table plus a generated findings paragraph."""
    lines = ['# ' + title, '']
    if not rows:
        lines.append('No scenario has been recorded yet.')
        return '\n'.join(lines) + '\n'

    durations = {
        round(float(row['duration']))
        for row in rows
        if finite_number(row.get('duration')) is not None
    }
    if len(durations) == 1:
        horizon = '%.0f s of simulation per scenario' % durations.pop()
    else:
        horizon = 'a mixed horizon per scenario'
    lines.append(
        '%d scenarios, %s, recorded from `mapping.launch.py use_rviz:=false '
        'pose_source:=both`.' % (len(rows), horizon)
    )
    lines.append('')

    lines.append('| ' + ' | '.join(name for name, _ in TABLE_COLUMNS) + ' |')
    lines.append('|' + '|'.join('---' for _ in TABLE_COLUMNS) + '|')
    for row in rows:
        lines.append(
            '| ' + ' | '.join(format_cell(row) for _, format_cell in TABLE_COLUMNS) + ' |'
        )
    lines.append('')
    lines.append(SUMMARY_NOTES)
    lines.append('')

    failures = [row for row in rows if row.get('status') == 'failed']
    if failures:
        lines.append(
            'Failed scenarios: %s (see `configs/*_launch.log`).'
            % ', '.join(row['name'] for row in failures)
        )
        lines.append('')

    lines.append('## Findings')
    lines.append('')
    lines.append(build_findings(rows))
    return '\n'.join(lines) + '\n'


def build_findings(rows):
    """Auto-generated paragraph with the best/worst scenarios."""
    usable = [row for row in rows if finite_number(row.get('rmse_pos')) is not None]
    if not usable:
        return (
            'No scenario produced a localization error stream, so the scenarios '
            'cannot be compared.'
        )

    best = min(usable, key=lambda row: row['rmse_pos'])
    worst = max(usable, key=lambda row: row['rmse_pos'])
    sentences = []
    ratio = None
    if best['rmse_pos'] > 0.0:
        ratio = worst['rmse_pos'] / best['rmse_pos']
    sentence = (
        'The lowest position RMSE is %.3f m (`%s`) and the highest is %.3f m '
        '(`%s`)'
        % (best['rmse_pos'], best['name'], worst['rmse_pos'], worst['name'])
    )
    if ratio is not None and math.isfinite(ratio):
        sentence += ', a factor of %.1f' % ratio
    yaw_values = [
        math.degrees(row['rmse_yaw']) for row in usable
        if finite_number(row.get('rmse_yaw')) is not None
    ]
    if yaw_values:
        if max(yaw_values) - min(yaw_values) < 1e-9:
            sentence += '. Yaw RMSE is %.2f deg in every scenario.' % yaw_values[0]
        else:
            sentence += (
                '. Yaw RMSE stays between %.2f and %.2f deg.'
                % (min(yaw_values), max(yaw_values))
            )
    else:
        sentence += '.'
    sentences.append(sentence)

    baseline = next(
        (row for row in usable if row['name'] == 'baseline'), None
    )
    if baseline is not None and baseline is not best:
        sentences.append(
            'The nominal model (`baseline`) reaches %.3f m, i.e. the scenarios '
            'above change the error by %.3f m.'
            % (baseline['rmse_pos'], worst['rmse_pos'] - baseline['rmse_pos'])
        )

    sweep = sorted(
        (
            row for row in usable
            if row['name'].startswith('uwb_sigma_')
            and finite_number(scenario_parameter(row, 'ekf', 'uwb_sigma')) is not None
        ),
        key=lambda row: float(scenario_parameter(row, 'ekf', 'uwb_sigma')),
    )
    if len(sweep) >= 2:
        sentences.append(
            'Sweeping the UWB noise assumed by both the model and the filter '
            'from %.2f to %.2f m moves the position RMSE from %.3f to %.3f m.'
            % (
                float(scenario_parameter(sweep[0], 'ekf', 'uwb_sigma')),
                float(scenario_parameter(sweep[-1], 'ekf', 'uwb_sigma')),
                sweep[0]['rmse_pos'],
                sweep[-1]['rmse_pos'],
            )
        )

    mismatch = next(
        (row for row in usable if row['name'] == 'uwb_mismatch_r_low'), None
    )
    if mismatch is not None:
        sentences.append(
            'An over-confident filter (model 0.10 m, filter 0.05 m) gives '
            '%.3f m, %s than the matching 0.10 m scenario.'
            % (
                mismatch['rmse_pos'],
                'better' if _better_than_matching(rows, mismatch) else 'worse',
            )
        )

    dropouts = [row for row in usable if row['name'].startswith('dropout_')]
    if dropouts:
        worst_dropout = max(dropouts, key=lambda row: row['rmse_pos'])
        sentences.append(
            'UWB dropouts matter up to %.3f m in position RMSE (`%s`).'
            % (worst_dropout['rmse_pos'], worst_dropout['name'])
        )

    outage = next(
        (row for row in usable if row['name'].startswith('outage_')), None
    )
    if outage is not None:
        start = finite_number(scenario_parameter(outage, 'sensors', 'outage_start_s'))
        end = finite_number(scenario_parameter(outage, 'sensors', 'outage_end_s'))
        length = None if start is None or end is None else end - start
        sentences.append(
            'The scenario `%s` loses UWB coverage%s and ends with a position '
            'RMSE of %.3f m.'
            % (
                outage['name'],
                '' if length is None or length <= 0.0
                else ' for %.0f s' % length,
                outage['rmse_pos'],
            )
        )

    mapped = [
        row for row in rows if finite_number(row.get('iou_occupied')) is not None
    ]
    if mapped:
        best_map = max(mapped, key=lambda row: row['iou_occupied'])
        worst_map = min(mapped, key=lambda row: row['iou_occupied'])
        distances = [
            row['mean_distance'] for row in mapped
            if finite_number(row.get('mean_distance')) is not None
        ]
        sentence = (
            'Occupied IoU peaks at %.3f (`%s`) and falls to %.3f (`%s`)'
            % (
                best_map['iou_occupied'], best_map['name'],
                worst_map['iou_occupied'], worst_map['name'],
            )
        )
        if distances:
            sentence += (
                '; the mean occupied-cell distance ranges from %.3f to %.3f m.'
                % (min(distances), max(distances))
            )
        else:
            sentence += '.'
        sentences.append(sentence)
    else:
        sentences.append(
            'No scenario produced both occupancy grids, so the map metrics are '
            'unavailable.'
        )

    slam = [
        row for row in rows if finite_number(row.get('slam_rmse_pos')) is not None
    ]
    if slam:
        sentences.append(
            'With EKF-SLAM enabled the position RMSE is %.3f..%.3f m (%d of %d '
            'scenarios).'
            % (
                min(row['slam_rmse_pos'] for row in slam),
                max(row['slam_rmse_pos'] for row in slam),
                len(slam), len(rows),
            )
        )

    return ' '.join(sentences)


def _better_than_matching(rows, mismatch):
    """True when the mismatched filter beats the matching 0.10 m scenario."""
    matching = next(
        (
            row for row in rows
            if row['name'].startswith('uwb_sigma_')
            and finite_number(row['rmse_pos']) is not None
            and finite_number(scenario_parameter(row, 'ekf', 'uwb_sigma')) is not None
            and abs(float(scenario_parameter(row, 'ekf', 'uwb_sigma')) - 0.10) < 1e-9
        ),
        None,
    )
    if matching is None:
        return True
    return mismatch['rmse_pos'] <= matching['rmse_pos']


CSV_COLUMNS = (
    ('scenario', lambda row: str(row['name'])),
    ('status', lambda row: str(row.get('status', 'ok'))),
    ('duration_s', lambda row: format_csv(row.get('duration'), '%.1f')),
    ('samples', lambda row: format_csv(row.get('samples'), '%.0f')),
    ('rmse_x_m', lambda row: format_csv(row.get('rmse_x'))),
    ('rmse_y_m', lambda row: format_csv(row.get('rmse_y'))),
    ('rmse_pos_m', lambda row: format_csv(row.get('rmse_pos'))),
    ('rmse_yaw_deg', lambda row: format_csv(yaw_degrees(row.get('rmse_yaw')))),
    ('max_pos_m', lambda row: format_csv(row.get('max_pos'))),
    ('max_yaw_deg', lambda row: format_csv(yaw_degrees(row.get('max_yaw')))),
    ('iou_occupied', lambda row: format_csv(row.get('iou_occupied'))),
    ('mean_distance_m', lambda row: format_csv(row.get('mean_distance'))),
    ('max_distance_m', lambda row: format_csv(row.get('max_distance'))),
    ('slam_samples', lambda row: format_csv(row.get('slam_samples'), '%.0f')),
    ('slam_rmse_pos_m', lambda row: format_csv(row.get('slam_rmse_pos'))),
    ('slam_rmse_yaw_deg', lambda row: format_csv(yaw_degrees(row.get('slam_rmse_yaw')))),
    ('wall_time_s', lambda row: format_csv(row.get('wall_time'), '%.1f')),
)


def render_summary_csv(rows):
    """CSV text with one row per scenario; missing metrics stay empty."""
    buffer = io.StringIO()
    writer = csv.writer(buffer, lineterminator='\n')
    writer.writerow([name for name, _ in CSV_COLUMNS])
    for row in rows:
        writer.writerow([format_value(row) for _, format_value in CSV_COLUMNS])
    return buffer.getvalue()


def write_summary(out_dir, rows):
    """Write ``summary.md`` and ``summary.csv``; return both paths."""
    os.makedirs(out_dir, exist_ok=True)
    markdown_path = os.path.join(out_dir, SUMMARY_MD)
    csv_path = os.path.join(out_dir, SUMMARY_CSV)
    with open(markdown_path, 'w') as stream:
        stream.write(render_summary_markdown(rows))
    with open(csv_path, 'w', newline='') as stream:
        stream.write(render_summary_csv(rows))
    return markdown_path, csv_path


# ----------------------------------------------------------------------
# Figures
# ----------------------------------------------------------------------
def scenario_colors(rows):
    """Baseline is highlighted, every other scenario shares one colour."""
    return [
        BASELINE_COLOR if str(row['name']) == 'baseline' else SERIES_COLOR
        for row in rows
    ]


def _scenario_axis(axes, rows):
    positions = np.arange(len(rows))
    axes.set_xticks(positions)
    axes.set_xticklabels([str(row['name']) for row in rows], rotation=30,
                         ha='right', fontsize=8)
    axes.grid(True, axis='y', alpha=0.3)
    return positions


def _missing_note(axes, values):
    if not np.all(np.isfinite(values)):
        axes.text(
            0.02, 0.98, 'n/a: scenario without data',
            transform=axes.transAxes, fontsize=7, va='top', color='0.35',
        )


def render_rmse_figure(rows, path):
    """Position and yaw RMSE per scenario (log scale when values allow)."""
    figure, (position_axes, yaw_axes) = plt.subplots(1, 2, figsize=(13.0, 4.8))
    colors = scenario_colors(rows)
    position = np.array(
        [NAN if finite_number(row.get('rmse_pos')) is None
         else float(row['rmse_pos']) for row in rows], dtype=float,
    )
    yaw = np.array(
        [NAN if yaw_degrees(row.get('rmse_yaw')) is None
         else yaw_degrees(row['rmse_yaw']) for row in rows], dtype=float,
    )

    positions = _scenario_axis(position_axes, rows)
    position_axes.bar(positions, position, color=colors, width=0.65)
    position_axes.set_ylabel('RMSE position [m]')
    finite_position = position[np.isfinite(position)]
    spread = (
        finite_position.max() / finite_position.min()
        if finite_position.size and np.all(finite_position > 0.0) else 1.0
    )
    if spread >= LOG_SCALE_SPREAD:
        position_axes.set_yscale('log')
        position_axes.set_title('Position RMSE (log scale)')
    else:
        position_axes.set_title('Position RMSE')
    _missing_note(position_axes, position)

    _scenario_axis(yaw_axes, rows)
    yaw_axes.bar(positions, yaw, color=colors, width=0.65)
    yaw_axes.set_ylabel('RMSE yaw [deg]')
    yaw_axes.set_title('Yaw RMSE')
    _missing_note(yaw_axes, yaw)

    handles = [Patch(facecolor=BASELINE_COLOR, label='baseline')]
    if any(str(row['name']) != 'baseline' for row in rows):
        handles.append(Patch(facecolor=SERIES_COLOR, label='other scenarios'))
    position_axes.legend(handles=handles, loc='upper left', fontsize=8)

    durations = {
        round(float(row['duration']))
        for row in rows
        if finite_number(row.get('duration')) is not None
    }
    horizon = '%.0f s' % durations.pop() if len(durations) == 1 else 'mixed'
    figure.suptitle(
        'Localization RMSE per scenario (%d scenarios, %s each)' % (len(rows), horizon)
    )
    figure.tight_layout()
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    figure.savefig(path, dpi=140)
    plt.close(figure)
    return path


def render_maps_figure(rows, path):
    """Occupied IoU and mean occupied-cell distance per scenario."""
    figure, (iou_axes, distance_axes) = plt.subplots(1, 2, figsize=(13.0, 4.8))
    colors = scenario_colors(rows)
    iou = np.array(
        [NAN if finite_number(row.get('iou_occupied')) is None
         else float(row['iou_occupied']) for row in rows], dtype=float,
    )
    distance = np.array(
        [NAN if finite_number(row.get('mean_distance')) is None
         else float(row['mean_distance']) for row in rows], dtype=float,
    )

    positions = _scenario_axis(iou_axes, rows)
    iou_axes.bar(positions, iou, color=colors, width=0.65)
    iou_axes.set_ylabel('occupied IoU')
    iou_axes.set_ylim(0.0, 1.05)
    iou_axes.set_title('Occupancy IoU, estimate vs ground truth')
    _missing_note(iou_axes, iou)

    _scenario_axis(distance_axes, rows)
    distance_axes.bar(positions, distance, color=colors, width=0.65)
    distance_axes.set_ylabel('mean occupied-cell distance [m]')
    distance_axes.set_title('Mean occupied-cell distance (lower is better)')
    _missing_note(distance_axes, distance)

    handles = [Patch(facecolor=BASELINE_COLOR, label='baseline')]
    if any(str(row['name']) != 'baseline' for row in rows):
        handles.append(Patch(facecolor=SERIES_COLOR, label='other scenarios'))
    iou_axes.legend(handles=handles, loc='upper right', fontsize=8)

    figure.suptitle('Map accuracy per scenario (final occupancy grids)')
    figure.tight_layout()
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    figure.savefig(path, dpi=140)
    plt.close(figure)
    return path


def render_figures(rows, figures_dir):
    """Render both figures; return their paths."""
    return (
        render_rmse_figure(rows, os.path.join(figures_dir, RMSE_FIGURE)),
        render_maps_figure(rows, os.path.join(figures_dir, MAPS_FIGURE)),
    )


# ----------------------------------------------------------------------
# Saved data (--replot)
# ----------------------------------------------------------------------
def save_dataset(path, rows):
    """Store the summary rows in ``analysis_data.npz`` for ``--replot``."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    metrics = np.array(
        [
            [NAN if finite_number(row.get(key)) is None else float(row[key])
             for key in METRIC_KEYS]
            for row in rows
        ],
        dtype=float,
    ).reshape(-1, len(METRIC_KEYS))
    np.savez_compressed(
        path,
        names=np.array([str(row['name']) for row in rows], dtype=str),
        metrics=metrics,
        metric_keys=np.array(METRIC_KEYS, dtype=str),
        statuses=np.array([str(row.get('status', 'ok')) for row in rows], dtype=str),
        use_slam=np.array(
            [bool(row.get('use_slam', False)) for row in rows], dtype=bool
        ),
        specs=np.array(
            [json.dumps(row.get('spec') or {}, sort_keys=True) for row in rows],
            dtype=str,
        ),
    )
    return path


def load_dataset(path):
    """Rebuild summary rows from ``analysis_data.npz``."""
    if not os.path.exists(path):
        raise SystemExit(
            'no saved analysis at %s; run the tool once without --replot' % path
        )
    with np.load(path) as data:
        names = [str(name) for name in data['names']]
        metrics = np.asarray(data['metrics'], dtype=float)
        keys = (
            [str(key) for key in data['metric_keys']]
            if 'metric_keys' in data else list(METRIC_KEYS)
        )
        statuses = (
            [str(status) for status in data['statuses']]
            if 'statuses' in data else ['ok'] * len(names)
        )
        use_slam = [bool(value) for value in data['use_slam']]
        specs = [json.loads(str(spec) or '{}') for spec in data['specs']]

    rows = []
    for index, name in enumerate(names):
        row = {
            'name': name,
            'status': statuses[index],
            'use_slam': use_slam[index],
            'spec': specs[index],
        }
        for column, key in enumerate(keys):
            row[key] = float(metrics[index, column])
        for key in ('samples', 'slam_samples'):
            number = finite_number(row.get(key))
            row[key] = int(round(number)) if number is not None else 0
        rows.append(row)
    return rows


def run_replot(out_dir, figures_dir):
    """Re-render summary and figures from the saved npz."""
    data_path = os.path.join(out_dir, DATA_NAME)
    rows = load_dataset(data_path)
    markdown_path, csv_path = write_summary(out_dir, rows)
    rmse_path, maps_path = render_figures(rows, figures_dir)
    print('re-rendered from %s' % data_path)
    print('summary: %s\n         %s' % (markdown_path, csv_path))
    print('figures: %s\n         %s' % (rmse_path, maps_path))
    return rows


# ----------------------------------------------------------------------
# Command line
# ----------------------------------------------------------------------
EPILOG = """\
scenario table (default profile, 62 s per scenario):
  baseline            nominal sensors.yaml and ekf.yaml
  uwb_sigma_0.02      uwb_sigma 0.02 m in the model and in the filter
  uwb_sigma_0.10      uwb_sigma 0.10 m in both
  uwb_sigma_0.20      uwb_sigma 0.20 m in both
  uwb_mismatch_r_low  model 0.10 m, filter 0.05 m (optimistic filter)
  dropout_0.10        uwb_dropout_probability 0.10
  dropout_0.25        uwb_dropout_probability 0.25
  outage_20_40        every UWB range is NaN from 20 s to 40 s
  anchors_wide        anchors at (0.4,0.4), (7.6,0.4), (7.6,7.6), (0.4,7.6)
  anchors_compact     anchors at (2.0,2.0), (2.6,2.0), (2.0,2.6), (2.6,2.6)

--quick keeps baseline, uwb_sigma_0.10 and dropout_0.25 and uses 30 s per
scenario. An explicit duration (positional or --duration) wins.

--scenarios FILE.yaml replaces the table, for example:
  defaults:
    duration: 45
  scenarios:
    - name: baseline
      sensors: {}
      ekf: {}
    - name: sigma_0.10
      sensors: {uwb_sigma: 0.10}
      ekf: {uwb_sigma: 0.10}

outputs: <out>/summary.md, <out>/summary.csv, <out>/analysis_data.npz,
<figures>/analysis_rmse.png and <figures>/analysis_maps.png
"""


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Run every scenario of the stage-9 table and compare '
                    'localization RMSE and map accuracy.',
        epilog=EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        'duration_positional', nargs='?', type=float, metavar='DURATION',
        help='recording time per scenario in seconds (default: 62, 30 with '
             '--quick)',
    )
    parser.add_argument(
        '--duration', type=float, default=None,
        help='recording time per scenario in seconds (same as the positional '
             'argument)',
    )
    parser.add_argument(
        '--quick', action='store_true',
        help='run only baseline, uwb_sigma_0.10 and dropout_0.25 (30 s each)',
    )
    parser.add_argument(
        '--scenarios', default=None, metavar='FILE.yaml',
        help='YAML file that replaces the built-in scenario table',
    )
    parser.add_argument(
        '--out', default=DEFAULT_OUT_DIR,
        help='output directory for summary and data (default: %(default)s)',
    )
    parser.add_argument(
        '--figures', default=None,
        help='figure directory (default: the parent of --out)',
    )
    parser.add_argument(
        '--replot', action='store_true',
        help='re-render the summary and figures from the saved npz; no ROS',
    )
    parser.add_argument(
        '--startup-timeout', type=float, default=DEFAULT_STARTUP_TIMEOUT,
        help='seconds to wait for /estimate/error after launching a scenario '
             '(default: %(default)s)',
    )
    return parser.parse_args(argv)


def scenarios_for_args(args):
    """Scenario list selected by the command line."""
    duration = (
        args.duration_positional if args.duration_positional is not None
        else args.duration
    )
    if duration is not None and (not math.isfinite(duration) or duration <= 0.0):
        raise SystemExit('the duration must be a positive number of seconds')
    table = load_scenario_file(args.scenarios) if args.scenarios else None
    return build_scenarios(quick=args.quick, duration=duration, table=table)


def main(argv=None):
    args = parse_args(argv)
    out_dir = os.path.abspath(args.out)
    figures_dir = (
        os.path.abspath(args.figures) if args.figures
        else os.path.dirname(out_dir) or os.curdir
    )

    if args.replot:
        run_replot(out_dir, figures_dir)
        return 0

    workspace_checks(WORKSPACE_DIR)
    scenarios = scenarios_for_args(args)
    total = sum(scenario['duration'] for scenario in scenarios)
    print('analysis: %d scenarios, %.0f s of simulation in total'
          % (len(scenarios), total))
    print('output: %s' % out_dir)
    print('figures: %s' % figures_dir)
    print('estimated runtime: %.0f min (recording plus ~12 s per scenario)'
          % (total / 60.0 + len(scenarios) * 12.0 / 60.0))
    for line in format_scenario_plan(scenarios):
        print(line)
    print()

    try:
        rows = run_scenarios(scenarios, args, out_dir)
    except KeyboardInterrupt:
        cleanup_leftovers()
        print('\ninterrupted; no summary written', file=sys.stderr)
        return 130

    markdown_path, csv_path = write_summary(out_dir, rows)
    data_path = save_dataset(os.path.join(out_dir, DATA_NAME), rows)
    rmse_path, maps_path = render_figures(rows, figures_dir)

    print()
    print(render_summary_markdown(rows))
    print('summary: %s\n         %s' % (markdown_path, csv_path))
    print('figures: %s\n         %s' % (rmse_path, maps_path))
    print('data:    %s' % data_path)

    failures = [row for row in rows if row.get('status') == 'failed']
    if failures:
        print(
            '\nfailed scenarios: %s'
            % ', '.join(row['name'] for row in failures),
            file=sys.stderr,
        )
        for row in failures:
            print('  %s: %s' % (row['name'], row.get('error', '')), file=sys.stderr)
        print(
            'launch logs: %s' % os.path.join(out_dir, 'configs'), file=sys.stderr
        )
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
