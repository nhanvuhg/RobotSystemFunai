#!/usr/bin/env bash
set -euo pipefail

# Start all robot system processes:
# - Dobot bringup
# - Robot logic node
# - Gripper node
# - CSI camera node
# - YOLO detection launch
# Usage: ./run_all_three.sh

WS="$HOME/ros2_ws"

# Source ROS 2 and workspace overlays if present
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  # shellcheck source=/dev/null
  set +u
  
  # Disable SHM transport to avoid RTPS_TRANSPORT_SHM errors
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/fastdds_no_shm.xml"
  
  source /opt/ros/jazzy/setup.bash || true
  set -u
fi
if [ -f "$WS/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  set +u
  source "$WS/install/setup.bash" || true
  set -u
fi

LOG_DIR="$WS/logs"
mkdir -p "$LOG_DIR"

# ✅ Check for existing processes before starting
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔍 Checking for duplicate processes..."

# Use || true to prevent grep exit code from triggering set -e
EXISTING=$(ps aux | grep -E "(robot_logic_node|dobot_bringup|gripper_festo|csi_camera_node|yolo_ros_hailort)" | grep -v -E "(grep|tail)" | wc -l || true)

if [ "${EXISTING:-0}" -gt 0 ]; then
    echo "⚠️  WARNING: Found $EXISTING existing process(es)!"
    echo ""
    ps aux | grep -E "(robot_logic_node|dobot_bringup|gripper_festo|csi_camera_node|yolo_ros_hailort)" | grep -v -E "(grep|tail)" || true
    echo ""
    echo "Please stop them first with:"
    echo "  ./stop_all.sh"
    echo ""
    exit 1
fi
echo "✅ No duplicates found. Starting processes..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

DOBOT_LOG="$LOG_DIR/dobot_bringup.log"
ROBOT_LOG="$LOG_DIR/robot_logic_node.log"
GRIPPER_LOG="$LOG_DIR/gripper_festo_node.log"
CAMERA_LOG="$LOG_DIR/csi_camera_node.log"
YOLO_LOG="$LOG_DIR/yolo_detection.log"

echo "Starting dobot_bringup_v3 nova5.launch.py (background). Log: $DOBOT_LOG"
ros2 launch dobot_bringup_v3 nova5.launch.py > "$DOBOT_LOG" 2>&1 &
PID_DOBOT=$!
echo "dobot bringup PID=$PID_DOBOT"

echo "Starting robot_logic_node with params file (background). Log: $ROBOT_LOG"
ros2 run robot_control_main robot_logic_node --ros-args --params-file "$WS/src/robot_control_main/config/joint_pose_params.yaml" > "$ROBOT_LOG" 2>&1 &
PID_ROBOT=$!
echo "robot_logic_node PID=$PID_ROBOT"

echo "Starting gripper node (background via venv wrapper). Log: $GRIPPER_LOG"
"$WS/run_gripper_node.sh" > "$GRIPPER_LOG" 2>&1 &
PID_GRIPPER=$!
echo "gripper_festo_node PID=$PID_GRIPPER"

echo "Starting CSI camera node (background). Log: $CAMERA_LOG"
ros2 run csi_camera csi_camera_node > "$CAMERA_LOG" 2>&1 &
PID_CAMERA=$!
echo "csi_camera_node PID=$PID_CAMERA"

echo "Starting YOLO detection launch (background). Log: $YOLO_LOG"
ros2 launch yolo_ros_hailort_cpp system_csi_dual_model.launch.py > "$YOLO_LOG" 2>&1 &
PID_YOLO=$!
echo "YOLO detection PID=$PID_YOLO"

cleanup() {
  echo "Cleaning up..."
  for pid in "$PID_YOLO" "$PID_CAMERA" "$PID_GRIPPER" "$PID_ROBOT" "$PID_DOBOT"; do
    if ps -p "$pid" > /dev/null 2>&1; then
      echo "Killing PID $pid"
      kill "$pid" || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}

trap cleanup EXIT INT TERM

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ All processes started successfully!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Tail logs with:"
echo "  tail -f $ROBOT_LOG $GRIPPER_LOG $DOBOT_LOG"
echo "  tail -f $CAMERA_LOG $YOLO_LOG"
echo ""

# Wait indefinitely (cleanup will run on Ctrl-C)
while true; do sleep 3600; done
