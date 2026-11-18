#!/usr/bin/env bash
# Run the stage-9 efficiency analysis: every scenario launches the simulation
# headless with its own sensor/EKF parameters, records localization RMSE and
# map accuracy, and writes results/analysis/summary.md + CSV + two figures.
#
#   ./run_analysis.sh                 # full table, 62 s per scenario
#   ./run_analysis.sh --quick         # 3 scenarios, 30 s each
#   ./run_analysis.sh 90 --quick      # quick table, 90 s per scenario
#   ./run_analysis.sh --replot        # re-render from the saved npz, no ROS
#
# The tool launches the simulation itself (one run per scenario) and stops it
# again, so this script only checks the environment and passes the arguments
# through. Requires matplotlib and numpy and a built workspace
# (colcon build --symlink-install).
set -eo pipefail
cd "$(dirname "$0")"

# ROS setup scripts are not compatible with `set -u` (unbound variables).
# shellcheck disable=SC1091
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [ ! -f install/setup.bash ]; then
  echo "Workspace is not built yet. Run: colcon build --symlink-install" >&2
  exit 1
fi
# shellcheck disable=SC1091
source install/setup.bash
set -u

# Kill leftovers of an interrupted run before the first scenario starts (the
# tool repeats this between scenarios and closes every launch with SIGINT).
pkill -9 -f "lib/navigator_sim" 2>/dev/null || true
pkill -9 -f "lib/navigator_localization" 2>/dev/null || true
pkill -9 -f "lib/navigator_slam" 2>/dev/null || true

exec python3 tools/analysis.py "$@"
