# Kill child processes when script exits
cleanup() {
  echo "🛑 Shutting down nodes..."
  pkill -P $$ 2>/dev/null
  exit 0
}

# Bắt các signal
trap cleanup SIGINT SIGTERM SIGHUP

# Định nghĩa workspace và log
WS_DIR="/home/pi/ros2_ws"
mkdir -p "$WS_DIR/logs"
rm -f "$WS_DIR/logs"/*.log
# Tạo sẵn file log để tail không lỗi
touch "$WS_DIR/logs/csi_camera.log" "$WS_DIR/logs/yolo_ros_hailort_cpp.log" "$WS_DIR/logs/robot_control_main.log" "$WS_DIR/logs/snap7.log" "$WS_DIR/logs/bbox_drawer_cpp.log" "$WS_DIR/logs/GUI.log"

# Kiểm tra terminal (Ưu tiên cửa sổ mới nếu có màn hình)
if [ -n "$DISPLAY" ]; then
    if command -v lxterminal >/dev/null 2>&1; then
        TERM_CMD="lxterminal --title=\$TITLE --command"
    elif command -v gnome-terminal >/dev/null 2>&1; then
        TERM_CMD="gnome-terminal --title=\$TITLE -- bash -c"
    else
        TERM_CMD="BACKGROUND"
    fi
else
    TERM_CMD="BACKGROUND"
fi

start_node () {
  TITLE="$1"
  CMD="$2"
  LOG_FILE="$WS_DIR/logs/${TITLE}.log"

  # SCRIPT: Source môi trường ROS2 và chạy lệnh
  SCRIPT="bash -c '
  source /opt/ros/jazzy/setup.bash
  [ -f $WS_DIR/install/setup.bash ] && source $WS_DIR/install/setup.bash
  
  trap \"exit\" SIGINT SIGTERM
  while true; do
    echo \"[$TITLE] Starting...\"
    cd $WS_DIR
    eval \"$CMD\" 2>&1 | tee -a \"$LOG_FILE\"
    echo \"[$TITLE] Crashed! Restart in 2s...\"
    sleep 2
  done
  '"

  if [ "$TERM_CMD" = "BACKGROUND" ]; then
      echo "🚀 Starting $TITLE trong background..."
      eval "$SCRIPT" &
  else
      echo "🚀 Starting $TITLE trong terminal mới..."
      TITLE="$TITLE" eval "$TERM_CMD \"$SCRIPT\"" &
  fi
}

# 0. Dọn dẹp tiến trình cũ
echo "🧹 Cleaning up old ROS2 processes..."
pkill -f rpicam-vid 2>/dev/null
pkill -f csi_camera_node 2>/dev/null
pkill -f robot_logic_node 2>/dev/null
pkill -f snap7_node 2>/dev/null
pkill -f yolo_ros_hailort_cpp 2>/dev/null
pkill -f component_container 2>/dev/null
sudo fuser -k /dev/hailo0 2>/dev/null
sudo fuser -k /dev/video0 2>/dev/null
sleep 1

# Khởi động các Node
# 1. CSI Camera
start_node "csi_camera" "ros2 run csi_camera csi_camera_node"
sleep 2

# 2. BBox Drawer
start_node "bbox_drawer_cpp" "ros2 launch bbox_drawer_cpp overlay_two_cams.launch.py"
sleep 2

# 3. YOLO Detection
start_node "yolo_ros_hailort_cpp" "ros2 launch yolo_ros_hailort_cpp mutilcam_yolox_hailort.launch.py"
sleep 2

# 4. Robot Control Logic
start_node "robot_control_main" "ros2 run robot_control_main robot_logic_node"
sleep 1

# 5. Snap7 PLC Driver
start_node "snap7" "ros2 run snap7_driver snap7_node"
sleep 1

# 6. GUI
start_node "GUI" "export DISPLAY=:0 && ros2 run ros2_qml_gui1 ros2_qml_gui1"

# Theo dõi Logs
echo "✅ All nodes started. Tailing logs..."
sleep 2

tail -f \
  "$WS_DIR/logs/robot_control_main.log" \
  "$WS_DIR/logs/csi_camera.log" \
  "$WS_DIR/logs/yolo_ros_hailort_cpp.log" \
  "$WS_DIR/logs/snap7.log"



