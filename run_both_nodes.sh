#!/usr/bin/env bash
set -euo pipefail

# Run robot_logic_node and robot_with_gripper.launch together for testing.
# Usage: ./run_both_nodes.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS 2 and workspace overlays if present
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  # shellcheck source=/dev/null
  set +u
  source /opt/ros/jazzy/setup.bash || true
  set -u
fi
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  set +u
  source "$HOME/ros2_ws/install/setup.bash" || true
  set -u
fi

LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"

ROBOT_LOG="$LOG_DIR/robot_logic_node.log"

echo "Starting robot_logic_node (background). Logs: $ROBOT_LOG"
ros2 run robot_control_main robot_logic_node > "$ROBOT_LOG" 2>&1 &
PID_ROBOT=$!
echo "robot_logic_node PID=$PID_ROBOT"

cleanup() {
  echo "Cleaning up..."
  if ps -p "$PID_ROBOT" > /dev/null 2>&1; then
    echo "Killing robot_logic_node (PID $PID_ROBOT)"
    kill "$PID_ROBOT" || true
    wait "$PID_ROBOT" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

# Small delay to let the background node initialize
sleep 1

echo "Launching robot_with_gripper.launch (foreground)"
ros2 launch robot_control_main robot_with_gripper.launch.py

echo "Launch exited — script will now cleanup"
