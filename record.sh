#!/usr/bin/env bash
# Start a robot-view recording from anywhere: sources ROS + the workspace, then
#   ros2 run robot_recorder record "$@"      (Ctrl-C to stop; --help for options)
set -e
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [ -n "${ROS2_WS:-}" ]; then
  WS_ROOT="$ROS2_WS"
elif [ -n "${ROBOHOME_WS:-}" ]; then
  WS_ROOT="$ROBOHOME_WS"
elif [ -f "$SCRIPT_DIR/../../install/setup.bash" ]; then
  WS_ROOT="$SCRIPT_DIR/../.."
else
  WS_ROOT="$HOME/ros2_ws"
fi
source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/setup.bash"
exec ros2 run robot_recorder record "$@"
