#!/usr/bin/env bash
# Record ground truth + EKF estimate and render localization figures into results/.
#
#   ./run_localization_metrics.sh          # 62 s recording
#   ./run_localization_metrics.sh 90       # 90 s recording
#
# Requires matplotlib and numpy (python3-matplotlib, python3-numpy) and a built
# workspace (colcon build --symlink-install).
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

DURATION="${1:-62}"

python3 tools/localization_metrics.py --duration "$DURATION" &
METRICS_PID=$!
sleep 3

ros2 launch navigator_bringup localization.launch.py use_rviz:=false &
SIM_PID=$!

set +e
wait "$METRICS_PID"
STATUS=$?
set -e

kill -INT "$SIM_PID" 2>/dev/null || true
sleep 3
kill -9 "$SIM_PID" 2>/dev/null || true
pkill -9 -f "lib/navigator_sim" 2>/dev/null || true
pkill -9 -f "lib/navigator_localization" 2>/dev/null || true

echo "figures: $(pwd)/results"
exit "$STATUS"
