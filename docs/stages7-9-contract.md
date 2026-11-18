# Contract for stages 7-9 (landmarks, EKF-SLAM, analysis)

This document is the interface agreement between the three stage branches. It
exists so that the stages can be developed in parallel git worktrees and merged
without renegotiating topics, messages or file ownership.

## Ground rules

- Artifacts (code, comments, identifiers, commit messages) are in English.
- Every stage keeps the project convention: **pure Python module(s) without ROS**
  plus a thin ROS wrapper. Pure modules must not import `rclpy`, message
  packages or `navigator_interfaces` at import time, so they can be unit-tested
  on a machine without ROS (`py_compile` + `unittest`).
- Only files listed under "Ownership" may be touched. In particular **do not
  modify** `README.md`, `WSL_SETUP.md`, `PLAN.md`, `src/navigator_description/**`,
  `src/navigator_localization/**`, `src/navigator_sim/**` (except stage 9, see
  below), `src/navigator_slam/navigator_slam/occupancy_grid*.py`,
  `src/navigator_slam/test/test_occupancy_grid.py`,
  `src/navigator_slam/test/test_config.py`, `mapping.launch.py`,
  `tools/map_compare.py`, `run_map_compare.sh`. Documentation is updated
  centrally after the branches are merged.
- Configs: one YAML per stage under `src/navigator_slam/config/`; every declared
  parameter appears there with a comment, and defaults in code match the YAML.
- Tests: `unittest` (no `pytest`-only features), placed in
  `src/navigator_slam/test/`. Tests that need the polygon geometry import
  `navigator_sim.world` and are skipped when it is not importable.
- Before committing: `python3 -m py_compile` on every new file, YAML parse
  check, `PYTHONPATH=src/navigator_slam:src/navigator_sim python3 -m unittest
  discover -s src/navigator_slam/test`, and `git status` must be clean after the
  commit. Never run `colcon` (ROS is not installed on the development machine).
- Commit in small steps on your own branch, prefix `feat(slam):` /
  `feat(tools):` / `test(slam):`.

## Frames and geometry (from stages 1-6)

- `map -> odom` is a fixed identity transform; pose topics are already in `map`.
- Pose is `base_footprint` in `map`; the lidar is mounted at
  `world.LIDAR_OFFSET = (0.18, 0.0, 0.34)` with identity rotation, so a scan
  point `(r, a)` in `lidar_link` is `(r cos a, r sin a)` from the lidar.
- Lidar scans: `/scan_noisy`, `frame_id = lidar_link`, `angle_min = 0`,
  360 beams at 10 Hz, `inf` = dropout, range limits 0.12..8.0 m.
- Ground truth: `/ground_truth/odom` (nav_msgs/Odometry, `map`).
- EKF localization (stage 5): `/estimate/odom`, `/estimate/error`.
- Occupancy grid (stage 6): `/map/estimate`, `/map/ground_truth`
  (nav_msgs/OccupancyGrid, 180x180, resolution 0.05, origin (-0.5, -0.5)).
- Landmarks of the polygon: `navigator_sim.world.LANDMARKS`
  (`('L1', x, y, radius=0.09, height=0.45, colour)`); they are part of the lidar
  ray-casting geometry, so they appear in `/scan_noisy`.
- `/cmd_vel` carries the commanded `(v, omega)` used for prediction.

## Messages (`src/navigator_interfaces`, already committed on the base branch)

```
LandmarkObservation.msg
    float64 range              # base_footprint origin -> landmark centre [m]
    float64 bearing            # angle relative to heading, (-pi, pi] [rad]
    float64 radius             # fitted landmark radius [m]
    float64 range_variance     # [m^2]
    float64 bearing_variance   # [rad^2]

LandmarkObservationArray.msg
    std_msgs/Header header     # stamp = scan stamp, frame_id = base_footprint
    LandmarkObservation[] observations

Landmark.msg
    int32 id
    float64 x
    float64 y
    float64[4] covariance      # 2x2 row-major [xx, xy, yx, yy]

LandmarkMap.msg
    std_msgs/Header header     # frame_id = map
    Landmark[] landmarks
```

Observations are **pose-free** (base frame), so the detector needs no map pose
and the joint EKF-SLAM can use them directly.

## Stage 7 — landmark extraction and data association

Ownership:

```
src/navigator_slam/navigator_slam/landmark_detection.py     # pure
src/navigator_slam/navigator_slam/landmark_node.py          # ROS wrapper
src/navigator_slam/config/landmarks.yaml
src/navigator_slam/test/test_landmark_detection.py
src/navigator_bringup/launch/landmarks.launch.py
```

`landmark_detection.py` API (names are the contract used by stage 8):

```
@dataclass LandmarkObservation: range, bearing, radius, range_variance, bearing_variance
def extract_landmarks(ranges, angle_min, angle_increment, range_min, range_max,
                      lidar_offset=(0.18, 0.0, 0.34), params=DetectionParams()) -> list[LandmarkObservation]
def associate_observations(observations, robot_pose, landmarks, pose_covariance,
                           gate=9.21, max_range=None) -> list[int | None]
```

- `extract_landmarks` clusters the scan points in the base frame by range jumps
  (adaptive: angular step plus a metric threshold), fits a circle per cluster
  (algebraic least squares), rejects clusters outside `min_points..max_points`,
  radius outside `radius_min..radius_max` and with a fit residual above
  `max_residual`; falls back to the cluster centroid when the fit is rejected.
  Points farther than `max_range` and non-finite ranges are skipped.
- `associate_observations` predicts each known landmark in the robot frame,
  forms the innovation covariance `H P_pose H^T + R` and associates greedily by
  the smallest Mahalanobis distance below `gate` (chi-square, 2 dof, 9.21 for
  99%); unmatched observations get `None`.
- `DetectionParams` is a plain dataclass; the node fills it from parameters.

`landmark_node` (entry point already registered in `setup.py`, node name
`landmark_node`):

- subscribes `/scan_noisy`; optional `pose_topic` (`/estimate/odom` by default;
  empty string disables map tracking and markers);
- publishes `/landmarks/observations` (`LandmarkObservationArray`) on every
  scan; when a pose is available also `/landmarks/map` (`LandmarkMap`) and
  `/landmarks/markers` (`visualization_msgs/MarkerArray`, spheres + id labels)
  at `publish_rate_hz`;
- keeps a simple landmark map: new id on first observation, exponential moving
  average of position with a running covariance per id; params
  `max_landmarks`, `gate`, `map_alpha`;
- parameters as in `landmarks.yaml` (scan/pose/observation/map/marker topics,
  detection params, association gate, `publish_rate_hz`, `frame_id`).

Tests must include: a synthetic 360-beam scan ray-cast against
`navigator_sim.world` that yields at least 3 landmark observations, circle fit
on a noisy arc, rejection of a wall-like wide cluster, association stability
(same landmark observed twice keeps the id, a far observation gets a new id) and
no-pose behaviour (observations only).

`landmarks.launch.py`: includes `simulation.launch.py` with
`use_localization:=true`, adds `landmark_node` with `landmarks.yaml`; launch
argument `use_rviz` (default `true`).

## Stage 8 — EKF-SLAM

Ownership:

```
src/navigator_slam/navigator_slam/ekf_slam.py               # pure
src/navigator_slam/navigator_slam/ekf_slam_node.py          # ROS wrapper
src/navigator_slam/config/ekf_slam.yaml
src/navigator_slam/test/test_ekf_slam.py
src/navigator_bringup/launch/slam.launch.py
```

- State `mu = [x, y, theta, m1x, m1y, ..., mNx, mNy]`, covariance `Sigma`.
- `predict(v, omega, dt, Q)`: differential-drive model, covariance propagated
  with the process Jacobian; cross-covariances to landmarks are kept.
- `update_uwb(ranges, anchors, R, tag_offset)`: same range model and Jacobian as
  stage 5 (`navigator_localization.ekf` may be used as a reference, do not
  import it), `NaN` = dropout.
- `update_landmarks(observations, associations)`: range-bearing model
  `h = [d, wrap(atan2(dy, dx) - theta)]`, analytic Jacobians for the robot block
  and the associated landmark block, Joseph form for the covariance.
- `add_landmark(observation, R)`: inverse observation model, initial position
  from the robot pose, covariance from `G Sigma G^T + G R G^T` with the landmark
  rows filled.
- `associate(observations, gate)`: reuse
  `navigator_slam.landmark_detection.associate_observations` with the current
  `Sigma` and predicted landmark positions; import it at module level (stage 7
  is merged before stage 8 is developed).
- `ekf_slam_node` (entry point `ekf_slam_node`, node name `ekf_slam_node`):
  predict from `/cmd_vel` at `predict_rate_hz`; on
  `/landmarks/observations` update/add; on `/uwb/noisy_ranges` update (parameter
  `use_uwb`); publishes `/slam/odom`, `/slam/path`, `/slam/landmarks`
  (`LandmarkMap`), `/slam/markers` (`MarkerArray`), and `/slam/error`
  (`geometry_msgs/Vector3Stamped`, error against `/ground_truth/odom`) plus a
  periodic RMSE log. Does **not** publish TF (ground truth owns
  `odom -> base_footprint`). Parameters in `ekf_slam.yaml`: initial pose and
  sigma, `q_x/q_y/q_yaw`, `uwb_sigma`, anchors, tag offset, `use_uwb`,
  `predict_rate_hz`, `gate`, `sigma_range`, `sigma_bearing`,
  `initial_landmark_std`, `max_landmarks`, `path_max_poses`, `path_every_n`.
- Tests: covariance symmetry/PSD after augmentation and updates; the same
  landmark observed twice keeps one id; a closed double loop (`loops: 2`
  trajectory, synthetic landmarks from `world.LANDMARKS`) gives final pose error
  below 0.15 m and landmark error below 0.25 m; a rejected outlier observation
  (55th percentile gate) is not associated; without UWB the filter still tracks
  (drift allowed) and with UWB outages it does not diverge.

`slam.launch.py`: includes `simulation.launch.py` with
`use_localization:=false`, adds `landmark_node` with `pose_topic:=` (empty, pose
tracking off) and `ekf_slam_node` with `ekf_slam.yaml`; argument `use_rviz`.

## Stage 9 — efficiency analysis

Ownership:

```
tools/analysis.py
run_analysis.sh
src/navigator_slam/test/test_analysis.py
src/navigator_bringup/launch/simulation.launch.py   # launch arguments only
src/navigator_sim/navigator_sim/uwb_model_node.py   # scenario parameters only
src/navigator_sim/config/sensors.yaml               # additions with comments
```

Allowed changes to existing files, strictly backwards compatible:

- `simulation.launch.py`: add launch arguments `sensors_params` and `ekf_params`
  (path to a YAML file, default = the package config) that are passed to the
  nodes instead of the defaults. Existing behaviour with no arguments must not
  change.
- `uwb_model_node.py`: add parameters `anchors` (flat `x, y, z` triples; empty
  list means `world.UWB_ANCHORS`) and `outage_start_s` / `outage_end_s`
  (`0.0 / 0.0` = disabled; inside the window every range is `NaN`). Defaults
  reproduce the current output bit-for-bit. Keep the existing seeded RNG order.
- `sensors.yaml`: only additions with comments (new defaults).

`tools/analysis.py`:

- scenario table (built-in, overridable with `--scenarios FILE.yaml`), each item:
  name, duration, sensor overrides (`uwb_sigma`, `uwb_dropout_probability`,
  `uwb_biases`, `anchors`), EKF overrides (`uwb_sigma`, `anchor layout`),
  optional `use_slam`;
- runs scenarios sequentially: writes temporary YAMLs, launches
  `navigator_bringup/mapping.launch.py use_rviz:=false pose_source:=both
  sensors_params:=... ekf_params:=...` as a subprocess, records
  `/estimate/error` (RMSE x/y/yaw), `/slam/error` when present (optional),
  and the two final maps (occupied IoU and mean distance, reusing
  `navigator_slam.occupancy_grid.compare_occupancy_data`);
- writes `results/analysis/summary.md`, `results/analysis/summary.csv`,
  `results/analysis_rmse.png` (RMSE vs scenario, log scale for sigma) and
  `results/analysis_maps.png` (IoU/distance per scenario); `--replot` renders
  from the saved CSV/NPZ without running the simulation;
- `--quick` selects a short subset (baseline + 2 sigma values + 1 dropout, 30 s),
  the default profile runs the full table at 62 s per scenario.

`run_analysis.sh` mirrors `run_map_compare.sh`: sources ROS, checks `install/`,
starts the analysis, kills leftovers (`lib/navigator_sim`,
`lib/navigator_localization`, `lib/navigator_slam`), prints where the results are.

Tests (`test_analysis.py`, skip if matplotlib import fails): RMSE aggregation,
scenario expansion/override merging, summary table formatting, and
`--replot`-style rendering from a synthetic CSV.

## Integration order

1. `feature/stage7-landmarks` is merged first.
2. `feature/stage8-ekf-slam` is rebased/merged on top (it imports stage 7).
3. `feature/stage9-analysis` is merged last (it touches `simulation.launch.py`
   and `uwb_model_node.py`).
4. Central documentation (`README.md`, `WSL_SETUP.md`, `PLAN.md`, RViz displays)
   is updated after the merge, not in stage branches.
