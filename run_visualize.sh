#!/usr/bin/env bash
# Record a headless simulation run and render the figures into results/.
#
#   ./run_visualize.sh          # 62 s recording
#   ./run_visualize.sh 90       # 90 s recording
#
# Requires matplotlib and numpy for python3.
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

python3 tools/visualize.py --duration "$DURATION" &
VIZ_PID=$!
sleep 3

ros2 launch navigator_bringup simulation.launch.py use_rviz:=false &
SIM_PID=$!

wait "$VIZ_PID"

kill -INT "$SIM_PID" 2>/dev/null || true
sleep 3
kill -9 "$SIM_PID" 2>/dev/null || true
pkill -9 -f "lib/navigator_sim" 2>/dev/null || true

echo "figures: $(pwd)/results"
