#!/usr/bin/env bash
# Build (if needed) and start the Navigator-M simulation.
#
#   ./run.sh                    # build + simulation + RViz
#   ./run.sh use_rviz:=false    # build + headless simulation
#
# Extra arguments are passed to ros2 launch.
set -eo pipefail
cd "$(dirname "$0")"

# ROS setup scripts are not compatible with `set -u` (unbound variables).
# shellcheck disable=SC1091
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

colcon build --symlink-install
# shellcheck disable=SC1091
source install/setup.bash
set -u

exec ros2 launch navigator_bringup simulation.launch.py "$@"
