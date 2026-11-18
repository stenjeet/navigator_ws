"""Tests for the stage-9 analysis tool (``tools/analysis.py``).

The tool is a script, not an installed module, so it is loaded by path with
``importlib``. Only the pure helpers are covered: scenario expansion, deep
merging of the package configs, metric aggregation, summary rendering and the
``--replot`` path. Running scenarios needs ROS 2 and is not tested here; the
rendering tests are skipped when matplotlib is missing.
"""

import csv
import contextlib
import importlib.util
import io
import math
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

try:
    import numpy as np
except ImportError:  # pragma: no cover - numpy is a tool dependency
    np = None

try:
    from navigator_slam.occupancy_grid import FREE, OCCUPIED, UNKNOWN
except ImportError:  # pragma: no cover - PYTHONPATH is set by the test command
    FREE = OCCUPIED = UNKNOWN = None

HERE = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
TOOL_PATH = os.path.join(WORKSPACE, 'tools', 'analysis.py')
TMP_ROOT = os.path.join(WORKSPACE, '.tmp')

EXPECTED_FULL_NAMES = (
    'baseline',
    'uwb_sigma_0.02',
    'uwb_sigma_0.10',
    'uwb_sigma_0.20',
    'uwb_mismatch_r_low',
    'dropout_0.10',
    'dropout_0.25',
    'outage_20_40',
    'anchors_wide',
    'anchors_compact',
)
EXPECTED_QUICK_NAMES = ('baseline', 'uwb_sigma_0.10', 'dropout_0.25')


def load_analysis_module():
    """Import ``tools/analysis.py`` as a module (needs matplotlib and yaml)."""
    spec = importlib.util.spec_from_file_location(
        'stage9_analysis_tool', TOOL_PATH
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


try:
    ANALYSIS = load_analysis_module()
except ImportError as error:  # pragma: no cover - depends on the environment
    ANALYSIS = None
    IMPORT_ERROR = error

requires_analysis = unittest.skipIf(
    ANALYSIS is None,
    'tools/analysis.py needs matplotlib, numpy and PyYAML'
)


class TempDirMixin:
    """A scratch directory inside the worktree ``.tmp/``, removed afterwards."""

    def setUp(self):
        os.makedirs(TMP_ROOT, exist_ok=True)
        self.temp_dir = tempfile.mkdtemp(prefix='analysis_test_', dir=TMP_ROOT)
        self.addCleanup(shutil.rmtree, self.temp_dir, True)


def make_row(name, rmse_pos=0.10, iou=0.40, sensors=None, ekf=None,
             samples=600, **extra):
    """A summary row shaped like ``scenario_row`` output."""
    row = {
        'name': name,
        'status': 'ok',
        'duration': 62.0,
        'spec': {'sensors': sensors or {}, 'ekf': ekf or {}},
        'samples': samples,
        'rmse_x': rmse_pos / math.sqrt(2.0),
        'rmse_y': rmse_pos / math.sqrt(2.0),
        'rmse_pos': rmse_pos,
        'rmse_yaw': 0.03,
        'max_pos': 3.0 * rmse_pos,
        'max_yaw': 0.12,
        'iou_occupied': iou,
        'mean_distance': 0.05,
        'max_distance': 0.15,
        'wall_time': 70.0,
    }
    row.update(extra)
    return row


def synthetic_rows():
    """Five scenarios with a declining map quality, like the real table."""
    return [
        make_row('baseline', rmse_pos=0.05, iou=0.46),
        make_row('uwb_sigma_0.10', rmse_pos=0.08, iou=0.40,
                 sensors={'uwb_sigma': 0.10}, ekf={'uwb_sigma': 0.10}),
        make_row('uwb_sigma_0.20', rmse_pos=0.14, iou=0.31,
                 sensors={'uwb_sigma': 0.20}, ekf={'uwb_sigma': 0.20}),
        make_row('dropout_0.25', rmse_pos=0.11, iou=0.28,
                 sensors={'uwb_dropout_probability': 0.25}),
        make_row('outage_20_40', rmse_pos=0.20, iou=0.22,
                 sensors={'outage_start_s': 20.0, 'outage_end_s': 40.0}),
    ]


@unittest.skipIf(np is None, 'numpy is not installed')
@requires_analysis
class TestDeepMerge(unittest.TestCase):
    def test_merges_nested_sections(self):
        base = {'node': {'ros__parameters': {'a': 1, 'b': 2}}}
        merged = ANALYSIS.deep_merge(base, {'node': {'ros__parameters': {'b': 3}}})
        self.assertEqual(merged, {'node': {'ros__parameters': {'a': 1, 'b': 3}}})

    def test_replaces_lists_and_scalars(self):
        base = {'anchors': [], 'uwb_sigma': 0.05, 'name': 'x'}
        merged = ANALYSIS.deep_merge(
            base, {'anchors': [1.0, 2.0, 3.0], 'uwb_sigma': 0.2}
        )
        self.assertEqual(merged['anchors'], [1.0, 2.0, 3.0])
        self.assertEqual(merged['uwb_sigma'], 0.2)
        self.assertEqual(merged['name'], 'x')

    def test_does_not_modify_the_base(self):
        base = {'node': {'ros__parameters': {'a': 1}}}
        ANALYSIS.deep_merge(base, {'node': {'ros__parameters': {'a': 2}}})
        self.assertEqual(base['node']['ros__parameters']['a'], 1)

    def test_empty_override_returns_a_copy(self):
        base = {'a': [1, 2]}
        merged = ANALYSIS.deep_merge(base, {})
        merged['a'].append(3)
        self.assertEqual(base['a'], [1, 2])


@unittest.skipIf(np is None, 'numpy is not installed')
@requires_analysis
class TestScenarioExpansion(unittest.TestCase):
    def test_full_profile(self):
        scenarios = ANALYSIS.build_scenarios()
        self.assertEqual(tuple(s['name'] for s in scenarios), EXPECTED_FULL_NAMES)
        for scenario in scenarios:
            self.assertEqual(scenario['duration'], 62.0)

    def test_quick_profile(self):
        scenarios = ANALYSIS.build_scenarios(quick=True)
        self.assertEqual(tuple(s['name'] for s in scenarios), EXPECTED_QUICK_NAMES)
        for scenario in scenarios:
            self.assertEqual(scenario['duration'], 30.0)

    def test_explicit_duration_wins(self):
        for quick in (False, True):
            scenarios = ANALYSIS.build_scenarios(quick=quick, duration=45.0)
            self.assertTrue(all(s['duration'] == 45.0 for s in scenarios))

    def test_sigma_scenarios_override_model_and_filter(self):
        by_name = {s['name']: s for s in ANALYSIS.build_scenarios()}
        for sigma in (0.02, 0.10, 0.20):
            scenario = by_name['uwb_sigma_%.2f' % sigma]
            self.assertEqual(scenario['sensors']['uwb_sigma'], sigma)
            self.assertEqual(scenario['ekf']['uwb_sigma'], sigma)

    def test_mismatch_scenario_is_optimistic(self):
        by_name = {s['name']: s for s in ANALYSIS.build_scenarios()}
        scenario = by_name['uwb_mismatch_r_low']
        self.assertEqual(scenario['sensors']['uwb_sigma'], 0.10)
        self.assertEqual(scenario['ekf']['uwb_sigma'], 0.05)

    def test_dropout_and_outage_scenarios(self):
        by_name = {s['name']: s for s in ANALYSIS.build_scenarios()}
        self.assertEqual(
            by_name['dropout_0.10']['sensors']['uwb_dropout_probability'], 0.10
        )
        self.assertEqual(
            by_name['dropout_0.25']['sensors']['uwb_dropout_probability'], 0.25
        )
        self.assertEqual(by_name['dropout_0.25']['ekf'], {})
        outage = by_name['outage_20_40']
        self.assertEqual(outage['sensors']['outage_start_s'], 20.0)
        self.assertEqual(outage['sensors']['outage_end_s'], 40.0)

    def test_anchor_layouts_are_flat_triples_in_both_files(self):
        by_name = {s['name']: s for s in ANALYSIS.build_scenarios()}
        wide = by_name['anchors_wide']
        compact = by_name['anchors_compact']
        for scenario in (wide, compact):
            sensor_anchors = scenario['sensors']['anchors']
            ekf_anchors = scenario['ekf']['anchors']
            self.assertEqual(sensor_anchors, ekf_anchors)
            self.assertEqual(len(sensor_anchors), 12)
            self.assertEqual(len(sensor_anchors) % 3, 0)
        self.assertEqual(tuple(wide['ekf']['anchors'][:3]), (0.4, 0.4, 1.2))
        self.assertEqual(tuple(compact['ekf']['anchors'][:3]), (2.0, 2.0, 1.2))

    def test_duplicate_names_are_rejected(self):
        table = [
            {'name': 'same', 'sensors': {}, 'ekf': {}},
            {'name': 'same', 'sensors': {}, 'ekf': {}},
        ]
        with self.assertRaises(ValueError):
            ANALYSIS.build_scenarios(table=table)

    def test_empty_table_is_rejected(self):
        with self.assertRaises(ValueError):
            ANALYSIS.build_scenarios(table=[])

    def test_custom_table_is_used_as_is(self):
        table = [
            {'name': 'custom_a', 'duration': 15.0, 'sensors': {}, 'ekf': {}},
            {'name': 'custom_b', 'sensors': {'uwb_sigma': 0.5}, 'ekf': {}},
        ]
        scenarios = ANALYSIS.build_scenarios(quick=True, table=table)
        self.assertEqual([s['name'] for s in scenarios], ['custom_a', 'custom_b'])
        self.assertEqual(scenarios[0]['duration'], 15.0)
        # Without an explicit duration a custom scenario keeps the profile
        # default (62 s).
        self.assertEqual(scenarios[1]['duration'], 62.0)

    def test_override_keys_exist_in_the_package_configs(self):
        sensor_base = ANALYSIS.load_node_params(ANALYSIS.SENSORS_CONFIG_PATH)
        ekf_base = ANALYSIS.load_node_params(ANALYSIS.EKF_CONFIG_PATH)
        # `anchors` is deliberately absent from sensors.yaml: the node default
        # is world.UWB_ANCHORS and an empty YAML sequence would be stored as an
        # uninitialized parameter, so scenarios pass a full list only when they
        # change the layout.
        scenario_only = {'anchors'}
        for scenario in ANALYSIS.build_scenarios():
            for key in scenario['sensors']:
                if key in scenario_only:
                    continue
                self.assertIn(key, sensor_base, 'unknown sensors.yaml key %r' % key)
            for key in scenario['ekf']:
                self.assertIn(key, ekf_base, 'unknown ekf.yaml key %r' % key)

    def test_scenario_file_merges_defaults(self):
        path = os.path.join(TMP_ROOT, 'scenario_file_test.yaml')
        os.makedirs(TMP_ROOT, exist_ok=True)
        ANALYSIS.write_yaml(path, {
            'defaults': {'duration': 12.0},
            'scenarios': [
                {'name': 'quick_a', 'sensors': {'uwb_sigma': 0.3}},
                {'name': 'quick_b', 'duration': 5.0, 'ekf': {'uwb_sigma': 0.3}},
            ],
        })
        self.addCleanup(os.remove, path)
        table = ANALYSIS.load_scenario_file(path)
        scenarios = ANALYSIS.build_scenarios(table=table)
        self.assertEqual([s['name'] for s in scenarios], ['quick_a', 'quick_b'])
        self.assertEqual(scenarios[0]['duration'], 12.0)
        self.assertEqual(scenarios[0]['sensors'], {'uwb_sigma': 0.3})
        self.assertEqual(scenarios[1]['duration'], 5.0)
        self.assertEqual(scenarios[1]['ekf'], {'uwb_sigma': 0.3})


@unittest.skipIf(np is None, 'numpy is not installed')
@requires_analysis
class TestConfigWriting(TempDirMixin, unittest.TestCase):
    def setUp(self):
        super().setUp()
        self.sensor_base = ANALYSIS.load_node_params(ANALYSIS.SENSORS_CONFIG_PATH)
        self.ekf_document = ANALYSIS.load_ekf_document(ANALYSIS.EKF_CONFIG_PATH)
        self.sensor_file = ANALYSIS.SENSORS_CONFIG_PATH
        self.ekf_file = ANALYSIS.EKF_CONFIG_PATH

    def read_text(self, path):
        with open(path, 'r') as stream:
            return stream.read()

    def test_sensor_config_uses_the_launched_node_names(self):
        path = os.path.join(self.temp_dir, 'sensors.yaml')
        ANALYSIS.write_sensor_config(
            path, self.sensor_base, {'uwb_sigma': 0.2, 'outage_start_s': 5.0}
        )
        document = ANALYSIS.load_yaml(path)
        self.assertEqual(
            set(document), set(ANALYSIS.SENSOR_NODE_NAMES)
        )
        uwb = document['uwb_model_node']['ros__parameters']
        lidar = document['lidar_model_node']['ros__parameters']
        self.assertEqual(uwb['uwb_sigma'], 0.2)
        self.assertEqual(uwb['outage_start_s'], 5.0)
        self.assertEqual(uwb['outage_end_s'], 0.0)
        self.assertNotIn('lidar_sigma', uwb)
        # The default anchor layout stays in the node (world.UWB_ANCHORS): an
        # empty YAML sequence would be stored as an uninitialized parameter.
        self.assertNotIn('anchors', uwb)
        self.assertEqual(lidar['lidar_sigma'], self.sensor_base['lidar_sigma'])
        self.assertEqual(lidar['random_seed'], self.sensor_base['random_seed'])
        self.assertNotIn('uwb_sigma', lidar)
        self.assertNotIn('anchors', lidar)

    def test_wide_anchor_override_reaches_both_nodes(self):
        path = os.path.join(self.temp_dir, 'sensors.yaml')
        anchors = ANALYSIS.flat_anchors(ANALYSIS.ANCHORS_WIDE)
        ANALYSIS.write_sensor_config(path, self.sensor_base, {'anchors': anchors})
        document = ANALYSIS.load_yaml(path)
        self.assertEqual(
            document['uwb_model_node']['ros__parameters']['anchors'], anchors
        )

    def test_anchor_override_is_written_when_requested(self):
        path = os.path.join(self.temp_dir, 'sensors_anchors.yaml')
        anchors = [0.4, 0.4, 1.2, 7.6, 0.4, 1.2, 7.6, 7.6, 1.2, 0.4, 7.6, 1.2]
        ANALYSIS.write_sensor_config(path, self.sensor_base, {'anchors': anchors})
        document = ANALYSIS.load_yaml(path)
        uwb = document['uwb_model_node']['ros__parameters']
        self.assertEqual(uwb['anchors'], anchors)
        self.assertNotIn('anchors', document['lidar_model_node']['ros__parameters'])

    def test_ekf_config_keeps_the_section_and_applies_overrides(self):
        path = os.path.join(self.temp_dir, 'ekf.yaml')
        ANALYSIS.write_ekf_config(path, self.ekf_document, {'uwb_sigma': 0.2})
        document = ANALYSIS.load_yaml(path)
        self.assertEqual(list(document), ['ekf_node'])
        parameters = document['ekf_node']['ros__parameters']
        self.assertEqual(parameters['uwb_sigma'], 0.2)
        self.assertEqual(parameters['initial_x'], 1.5)
        self.assertEqual(len(parameters['anchors']), 12)

    def test_scenario_configs_are_named_after_the_scenario(self):
        scenario = {
            'name': 'demo',
            'duration': 1.0,
            'sensors': {'uwb_sigma': 0.11},
            'ekf': {'uwb_sigma': 0.11},
        }
        sensors_path, ekf_path = ANALYSIS.write_scenario_configs(
            scenario, self.temp_dir, self.sensor_base, self.ekf_document
        )
        self.assertEqual(os.path.basename(sensors_path), 'demo_sensors.yaml')
        self.assertEqual(os.path.basename(ekf_path), 'demo_ekf.yaml')
        sensors = ANALYSIS.load_yaml(sensors_path)
        ekf = ANALYSIS.load_yaml(ekf_path)
        self.assertEqual(
            sensors['uwb_model_node']['ros__parameters']['uwb_sigma'], 0.11
        )
        self.assertEqual(ekf['ekf_node']['ros__parameters']['uwb_sigma'], 0.11)

    def test_package_configs_are_not_modified(self):
        before = (self.read_text(self.sensor_file), self.read_text(self.ekf_file))
        ANALYSIS.write_scenario_configs(
            {'name': 'demo', 'sensors': {'uwb_sigma': 0.3},
             'ekf': {'uwb_sigma': 0.3}},
            self.temp_dir, self.sensor_base, self.ekf_document,
        )
        after = (self.read_text(self.sensor_file), self.read_text(self.ekf_file))
        self.assertEqual(before, after)


@unittest.skipIf(np is None, 'numpy is not installed')
@requires_analysis
class TestErrorAggregation(unittest.TestCase):
    def test_constant_error(self):
        samples = np.array([[0.3, 0.4, 0.0]] * 10)
        metrics = ANALYSIS.aggregate_errors(samples)
        self.assertEqual(metrics['samples'], 10)
        self.assertAlmostEqual(metrics['rmse_x'], 0.3)
        self.assertAlmostEqual(metrics['rmse_y'], 0.4)
        self.assertAlmostEqual(metrics['rmse_pos'], 0.5)
        self.assertAlmostEqual(metrics['max_pos'], 0.5)

    def test_yaw_rmse_and_peak(self):
        samples = np.array([
            [0.0, 0.0, 0.1],
            [0.0, 0.0, -0.1],
            [0.0, 0.0, 0.3],
        ])
        metrics = ANALYSIS.aggregate_errors(samples)
        self.assertAlmostEqual(metrics['rmse_yaw'], 0.1 * math.sqrt(11.0 / 3.0))
        self.assertAlmostEqual(metrics['max_yaw'], 0.3)

    def test_non_finite_samples_are_dropped(self):
        samples = np.array([
            [0.1, 0.0, 0.0],
            [float('nan'), 0.0, 0.0],
            [0.1, 0.0, float('inf')],
        ])
        metrics = ANALYSIS.aggregate_errors(samples)
        self.assertEqual(metrics['samples'], 1)
        self.assertAlmostEqual(metrics['rmse_pos'], 0.1)

    def test_empty_stream_returns_nan_metrics(self):
        for samples in (None, np.empty((0, 3)), np.empty((0, 0))):
            metrics = ANALYSIS.aggregate_errors(samples)
            self.assertEqual(metrics['samples'], 0)
            for key in ('rmse_x', 'rmse_y', 'rmse_pos', 'rmse_yaw', 'max_pos'):
                self.assertTrue(math.isnan(metrics[key]))

    def test_aggregate_errors_reshape_flat_input(self):
        metrics = ANALYSIS.aggregate_errors([0.2, 0.0, 0.0, 0.2, 0.0, 0.0])
        self.assertEqual(metrics['samples'], 2)
        self.assertAlmostEqual(metrics['rmse_x'], 0.2)


@unittest.skipIf(np is None, 'numpy is not installed')
@unittest.skipIf(OCCUPIED is None, 'navigator_slam is not on PYTHONPATH')
@requires_analysis
class TestMapMetrics(unittest.TestCase):
    def grid(self, size=20, border=True):
        data = np.full((size, size), UNKNOWN, dtype=np.int8)
        data[:, :] = FREE
        if border:
            data[0, :] = OCCUPIED
            data[-1, :] = OCCUPIED
            data[:, 0] = OCCUPIED
            data[:, -1] = OCCUPIED
        return data

    def test_identical_maps(self):
        metrics = ANALYSIS.map_metrics(self.grid(), self.grid(), 0.05)
        self.assertAlmostEqual(metrics['iou_occupied'], 1.0)
        self.assertAlmostEqual(metrics['mean_distance'], 0.0)

    def test_shifted_maps(self):
        left = np.full((20, 20), FREE, dtype=np.int8)
        right = np.full((20, 20), FREE, dtype=np.int8)
        left[5, 5] = OCCUPIED
        right[6, 5] = OCCUPIED
        metrics = ANALYSIS.map_metrics(left, right, 0.05)
        self.assertAlmostEqual(metrics['iou_occupied'], 0.0)
        self.assertAlmostEqual(metrics['mean_distance'], 0.05)
        self.assertAlmostEqual(metrics['max_distance'], 0.05)

    def test_missing_or_mismatched_maps_are_blank(self):
        for estimate in (None, np.zeros((3, 3), dtype=np.int8)):
            metrics = ANALYSIS.map_metrics(self.grid(), estimate, 0.05)
            self.assertTrue(math.isnan(metrics['iou_occupied']))
            self.assertTrue(math.isnan(metrics['mean_distance']))

    def test_invalid_resolution_is_blank(self):
        metrics = ANALYSIS.map_metrics(self.grid(), self.grid(), float('nan'))
        self.assertTrue(math.isnan(metrics['iou_occupied']))


@unittest.skipIf(np is None, 'numpy is not installed')
@requires_analysis
class TestSummaryRendering(TempDirMixin, unittest.TestCase):
    def assert_no_nan_text(self, text):
        lowered = text.lower()
        for token in ('nan', 'inf'):
            self.assertNotIn(token, lowered, 'summary contains %r' % token)

    def test_markdown_lists_every_scenario(self):
        rows = synthetic_rows()
        markdown = ANALYSIS.render_summary_markdown(rows)
        for row in rows:
            self.assertIn(row['name'], markdown)
        self.assertIn('| Scenario |', markdown)
        self.assertIn('## Findings', markdown)
        self.assert_no_nan_text(markdown)

    def test_missing_metrics_become_not_available(self):
        row = make_row('broken', rmse_pos=float('nan'), iou=float('nan'))
        markdown = ANALYSIS.render_summary_markdown([row])
        self.assertIn('n/a', markdown)
        self.assert_no_nan_text(markdown)

    def test_csv_round_trip(self):
        rows = synthetic_rows()
        rows.append(make_row('without_map', iou=float('nan')))
        text = ANALYSIS.render_summary_csv(rows)
        self.assert_no_nan_text(text)
        parsed = list(csv.DictReader(text.splitlines()))
        self.assertEqual(len(parsed), len(rows))
        self.assertEqual(parsed[0]['scenario'], 'baseline')
        self.assertAlmostEqual(float(parsed[0]['rmse_pos_m']), 0.05)
        self.assertEqual(parsed[-1]['iou_occupied'], '')
        self.assertEqual(parsed[-1]['status'], 'ok')

    def test_findings_name_the_best_and_worst_scenario(self):
        rows = synthetic_rows()
        findings = ANALYSIS.build_findings(rows)
        self.assertIn('baseline', findings)
        self.assertIn('outage_20_40', findings)
        self.assert_no_nan_text(findings)
        # The sweep sentence needs two sigma scenarios with an EKF value.
        self.assertIn('Sweeping the UWB noise', findings)

    def test_findings_without_any_data(self):
        findings = ANALYSIS.build_findings([])
        self.assert_no_nan_text(findings)
        self.assertTrue(findings)

    def test_failed_scenario_is_reported(self):
        broken = make_row('broken', rmse_pos=float('nan'), iou=float('nan'))
        broken['status'] = 'failed'
        broken['error'] = 'launch process exited with code 1'
        markdown = ANALYSIS.render_summary_markdown([make_row('baseline'), broken])
        self.assertIn('Failed scenarios: broken', markdown)
        self.assert_no_nan_text(markdown)

    def test_write_summary_creates_both_files(self):
        rows = synthetic_rows()
        markdown_path, csv_path = ANALYSIS.write_summary(self.temp_dir, rows)
        self.assertTrue(os.path.getsize(markdown_path) > 0)
        self.assertTrue(os.path.getsize(csv_path) > 0)
        self.assertEqual(os.path.basename(markdown_path), 'summary.md')
        self.assertEqual(os.path.basename(csv_path), 'summary.csv')


@unittest.skipIf(np is None, 'numpy is not installed')
@requires_analysis
class TestDatasetAndReplot(TempDirMixin, unittest.TestCase):
    def test_dataset_round_trip(self):
        rows = synthetic_rows()
        rows.append(make_row('without_map', iou=float('nan')))
        path = os.path.join(self.temp_dir, 'analysis_data.npz')
        ANALYSIS.save_dataset(path, rows)
        loaded = ANALYSIS.load_dataset(path)
        self.assertEqual([row['name'] for row in loaded],
                         [row['name'] for row in rows])
        for original, restored in zip(rows, loaded):
            self.assertAlmostEqual(restored['rmse_pos'], original['rmse_pos'])
            self.assertEqual(restored['duration'], original['duration'])
            self.assertEqual(restored['samples'], original['samples'])
            self.assertEqual(restored['spec'], original['spec'])
        self.assertTrue(math.isnan(loaded[-1]['iou_occupied']))
        self.assertEqual(loaded[-1]['samples'], 600)

    def test_load_dataset_without_file_is_explicit(self):
        with self.assertRaises(SystemExit):
            ANALYSIS.load_dataset(os.path.join(self.temp_dir, 'missing.npz'))

    def test_figures_render_from_synthetic_data(self):
        rows = synthetic_rows()
        rmse_path, maps_path = ANALYSIS.render_figures(rows, self.temp_dir)
        for path in (rmse_path, maps_path):
            self.assertTrue(os.path.exists(path))
            self.assertGreater(os.path.getsize(path), 5000)

    def test_figures_render_without_any_data(self):
        rows = [
            make_row('broken', rmse_pos=float('nan'), iou=float('nan')),
            make_row('empty', rmse_pos=float('nan'), iou=float('nan')),
        ]
        rmse_path, maps_path = ANALYSIS.render_figures(rows, self.temp_dir)
        self.assertTrue(os.path.exists(rmse_path))
        self.assertTrue(os.path.exists(maps_path))

    def test_replot_writes_summary_and_figures(self):
        out_dir = os.path.join(self.temp_dir, 'analysis')
        figures_dir = os.path.join(self.temp_dir, 'figures')
        ANALYSIS.save_dataset(
            os.path.join(out_dir, 'analysis_data.npz'), synthetic_rows()
        )
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            rows = ANALYSIS.run_replot(out_dir, figures_dir)
        self.assertEqual(len(rows), 5)
        self.assertIn('summary.md', buffer.getvalue())
        for name in ('summary.md', 'summary.csv', 'analysis_data.npz'):
            self.assertTrue(os.path.exists(os.path.join(out_dir, name)), name)
        for name in ('analysis_rmse.png', 'analysis_maps.png'):
            self.assertTrue(os.path.exists(os.path.join(figures_dir, name)), name)


@unittest.skipIf(np is None, 'numpy is not installed')
@requires_analysis
class TestRunnerPieces(TempDirMixin, unittest.TestCase):
    """The non-ROS parts of the scenario runner."""

    def test_launch_command_matches_the_contract(self):
        command = ANALYSIS.launch_command('/cfg/sensors.yaml', '/cfg/ekf.yaml')
        self.assertEqual(
            command[:4],
            ['ros2', 'launch', 'navigator_bringup', 'mapping.launch.py'],
        )
        self.assertIn('use_rviz:=false', command)
        self.assertIn('pose_source:=both', command)
        self.assertIn('sensors_params:=/cfg/sensors.yaml', command)
        self.assertIn('ekf_params:=/cfg/ekf.yaml', command)

    def test_child_environment_is_unbuffered(self):
        environment = ANALYSIS.child_environment()
        self.assertEqual(environment['PYTHONUNBUFFERED'], '1')

    def test_unbuilt_workspace_fails_loudly(self):
        with self.assertRaises(SystemExit):
            ANALYSIS.workspace_checks(self.temp_dir)

    def test_stop_launch_of_a_finished_process_is_harmless(self):
        original = ANALYSIS.cleanup_leftovers
        ANALYSIS.cleanup_leftovers = lambda: None
        self.addCleanup(setattr, ANALYSIS, 'cleanup_leftovers', original)
        process = subprocess.Popen([sys.executable, '-c', 'pass'])
        process.wait()
        ANALYSIS.stop_launch(process)

    def test_failed_row_keeps_only_the_first_line(self):
        row = ANALYSIS.failed_row(
            {'name': 'broken', 'duration': 1.0, 'sensors': {}, 'ekf': {}},
            'launch process exited with code 1\n[ERROR] details\n',
        )
        self.assertEqual(row['status'], 'failed')
        self.assertEqual(row['error'], 'launch process exited with code 1')
        self.assertTrue(math.isnan(row['rmse_pos']))


if __name__ == '__main__':
    unittest.main()
