#!/usr/bin/env bash
# Record occupancy maps built from true and estimated poses and compare them.
#
#   ./run_map_compare.sh          # 62 s recording
#   ./run_map_compare.sh 90       # 90 s recording
#
# Starts two mapper instances on the same lidar scans (pose_source:=both): one
# uses /ground_truth/odom, the other the EKF estimate /estimate/odom. Requires
# matplotlib and numpy and a built workspace (colcon build --symlink-install).
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

python3 tools/map_compare.py --duration "$DURATION" &
COMPARE_PID=$!
sleep 3

ros2 launch navigator_bringup mapping.launch.py use_rviz:=false pose_source:=both &
SIM_PID=$!

set +e
wait "$COMPARE_PID"
STATUS=$?
set -e

kill -INT "$SIM_PID" 2>/dev/null || true
sleep 3
kill -9 "$SIM_PID" 2>/dev/null || true
pkill -9 -f "lib/navigator_sim" 2>/dev/null || true
pkill -9 -f "lib/navigator_localization" 2>/dev/null || true
pkill -9 -f "lib/navigator_slam" 2>/dev/null || true

echo "figures: $(pwd)/results"
exit "$STATUS"
