#!/bin/bash

# =============================================================================
# Dual Camera System Launcher — Fixed version
#
# CHANGES vs original:
#   [FIX-1] Cleanup chỉ ở đây, không còn trong constructor của node C++.
#            Thêm wait đủ lâu (2s) sau khi kill để kernel release /dev/media*.
#   [FIX-A] fuser -k loop tăng retry từ 5 lên 8 lần, interval 1.5s
#   [FIX-B] Thêm kiểm tra /dev/media1 song song /dev/media0
#   [FIX-C] Delay cho CSI Camera node tăng từ 4s → 8s: node cần 15s timeout
#            bên trong nhưng script chỉ cần đợi đủ để node đã publish topic,
#            không cần đợi hết 15s — 8s là đủ khi camera healthy
#   [FIX-D] Thêm log rõ ràng hơn khi camera device vẫn bận
#   [REMOVED] sleep 0.5 sau pkill audio — quá ngắn, tăng lên 1s
#   [REMOVED] lần fuser -k lặp lại bên trong loop chưa có delay giữa các lần
# =============================================================================

ROS_DISTRO_PATH="/opt/ros/jazzy/setup.bash"
WS_PATH="$HOME/ros2_ws"
LOG_DIR="$WS_PATH/logs"

# [FIX-ARCH] Prevent parallel launching scripts from fighting over resources
echo "🛑 Ensuring no OTHER launchers are running..."
for pid in $(pgrep -f "run_two_camera.sh"); do
    if [ "$pid" != "$$" ]; then
        kill -TERM "$pid" 2>/dev/null
    fi
done

# [FIX-ARCH] Isolate FastDDS domain to avoid cross-talk with ghost nodes or other Pi systems
export ROS_DOMAIN_ID=42
echo "🌐 Set ROS_DOMAIN_ID=$ROS_DOMAIN_ID for Clean Isolation"

mkdir -p "$LOG_DIR"

# =============================================================================
# cleanup — chỉ gửi SIGTERM trước, đợi 1s, rồi mới SIGKILL
# =============================================================================
cleanup() {
    trap - SIGINT SIGTERM SIGHUP EXIT
    echo ""
    echo "🛑 Shutting down all nodes..."

    pkill -TERM -f "csi_dual_camera_node"   2>/dev/null
    pkill -TERM -f "yolo_ros_hailort"       2>/dev/null
    pkill -TERM -f "overlay_bboxes"         2>/dev/null
    pkill -TERM -f "snap7_node"             2>/dev/null
    pkill -TERM -f "robot_logic_node"       2>/dev/null
    pkill -TERM -f "ros2_qml_gui"           2>/dev/null
    pkill -TERM -f "rpicam-vid"             2>/dev/null

    sleep 1

    pkill -KILL -f "csi_dual_camera_node"   2>/dev/null
    pkill -KILL -f "yolo_ros_hailort"       2>/dev/null
    pkill -KILL -f "overlay_bboxes"         2>/dev/null
    pkill -KILL -f "snap7_node"             2>/dev/null
    pkill -KILL -f "robot_logic_node"       2>/dev/null
    pkill -KILL -f "ros2_qml_gui"           2>/dev/null
    pkill -KILL -f "rpicam-vid"             2>/dev/null

    echo "✅ Cleanup complete"
    exit 0
}

trap cleanup SIGINT SIGTERM SIGHUP EXIT

# =============================================================================
# Workspace check
# =============================================================================
cd "$WS_PATH" || { echo "❌ Workspace not found: $WS_PATH"; exit 1; }

if [ ! -f "$ROS_DISTRO_PATH" ]; then
    echo "❌ ROS2 setup not found: $ROS_DISTRO_PATH"
    exit 1
fi
if [ ! -f "$WS_PATH/install/setup.bash" ]; then
    echo "❌ Workspace not built yet. Run: colcon build"
    exit 1
fi

source "$ROS_DISTRO_PATH"
source "$WS_PATH/install/setup.bash"

# =============================================================================
# start_node helper — unchanged
# =============================================================================
start_node() {
    local TITLE="$1"
    local CMD="$2"
    local DELAY="${3:-1}"

    echo "🚀 Starting: $TITLE"

    lxterminal --title="$TITLE" --command="bash -c '
        source \"$ROS_DISTRO_PATH\"  || { echo \"❌ ROS2 source failed\"; exit 1; }
        source \"$WS_PATH/install/setup.bash\" || { echo \"❌ Workspace source failed\"; exit 1; }
        export ROS_DOMAIN_ID=$ROS_DOMAIN_ID

        trap \"exit 0\" SIGINT SIGTERM

        while true; do
            echo \"======================================\"
            echo \"[$TITLE] Starting at \$(date +%H:%M:%S)\"
            echo \"======================================\"
            $CMD
            EXIT_CODE=\$?
            echo \"[$TITLE] Exited (code=\$EXIT_CODE). Restart in 3s... (Ctrl+C to cancel)\"
            sleep 3
        done
    '" &

    sleep "$DELAY"
}

# =============================================================================
# System cleanup
#
# [FIX-1] Đây là NƠI DUY NHẤT chạy cleanup.
#          Constructor C++ đã được bỏ full_system_cleanup() để tránh
#          double-kill khiến kernel chưa kịp release /dev/media* lần 2.
# =============================================================================
echo "=========================================="
echo "   🎥 Dual Camera System Launcher"
echo "=========================================="
echo ""

echo "🔇 Stopping audio services (frees camera from Pipewire)..."
systemctl --user stop pipewire pipewire-pulse wireplumber 2>/dev/null
pkill -f pipewire    2>/dev/null
pkill -f wireplumber 2>/dev/null
# [REMOVED] sleep 0.5 → tăng lên 1s để service thực sự stop
sleep 1

echo "🧹 Killing stale ROS2 / camera processes..."
pkill -9 -f "ros2 run"              2>/dev/null
pkill -9 -f "ros2 launch"           2>/dev/null
pkill -9 -f "rpicam-vid"            2>/dev/null
pkill -9 -f "rpicam-hello"          2>/dev/null
pkill -9 -f "csi_dual_camera_node"  2>/dev/null
pkill -9 -f "robot_logic_node"      2>/dev/null
pkill -9 -f "yolo_ros_hailort"      2>/dev/null
pkill -9 -f "snap7_node"            2>/dev/null
pkill -9 -f "overlay_bboxes"        2>/dev/null
pkill -9 -f "ros2_qml_gui"          2>/dev/null

echo "📷 Releasing camera devices..."
for DEV in /dev/media0 /dev/media1 /dev/media2 /dev/media3 \
           /dev/video0 /dev/video1 /dev/hailo0; do
    [ -e "$DEV" ] && sudo fuser -k "$DEV" 2>/dev/null
done

# [FIX-1] Sau khi kill tất cả, đợi 2s cứng để kernel flush buffers và
#          V4L2/CFE driver release internal state.
#          Đây là khoảng thời gian quan trọng nhất — thiếu bước này là
#          nguyên nhân chính CAM0 bị EBUSY khi node khởi động.
echo "⏳ Waiting 2s for kernel to release camera devices..."
sleep 2

# Reap zombie children
while wait -n 2>/dev/null; do :; done
# Portable fallback reap:
for pid_f in /proc/[0-9]*/status; do
    grep -q "^State:.*Z" "$pid_f" 2>/dev/null && \
    wait "$(basename "$(dirname "$pid_f")")" 2>/dev/null
done

# [FIX-A] Tăng retry lên 8 lần, interval 1.5s thay vì 1s
#         Kernel Pi 5 với CFE driver cần lâu hơn Pi 4 để release media device
# [FIX-B] Kiểm tra cả /dev/media0 và /dev/media1
echo "⏳ Verifying camera devices are free..."
CAMS_FREE=true
for CAM_DEV in /dev/media0 /dev/media1; do
    [ ! -e "$CAM_DEV" ] && continue
    for i in $(seq 1 8); do
        if ! fuser "$CAM_DEV" 2>/dev/null | grep -q "[0-9]"; then
            echo "✓ $CAM_DEV is free"
            break
        fi
        echo "  [$CAM_DEV] Attempt $i/8: still busy, retrying in 1.5s..."
        sudo fuser -k "$CAM_DEV" 2>/dev/null
        sleep 1.5
        # [FIX-D] Hiển thị process nào đang giữ device để debug dễ hơn
        HOLDER=$(fuser "$CAM_DEV" 2>/dev/null)
        if [ -n "$HOLDER" ]; then
            echo "  [$CAM_DEV] Held by PID(s): $HOLDER"
            for h in $HOLDER; do
                cat "/proc/$h/comm" 2>/dev/null && true
            done
        fi
        if [ "$i" -eq 8 ]; then
            echo "⚠️  $CAM_DEV may still be busy — this will likely cause CAM startup failure"
            CAMS_FREE=false
        fi
    done
done

if [ "$CAMS_FREE" = false ]; then
    echo ""
    echo "❌ One or more camera devices could not be freed."
    echo "   Try: sudo reboot"
    echo "   Or:  sudo rmmod bcm2835_isp && sudo modprobe bcm2835_isp"
    echo "   Continuing anyway — node will retry internally..."
    echo ""
fi

echo ""
echo "✓ System ready"
echo ""

# =============================================================================
# Launch nodes
#
# [FIX-C] CSI Camera delay: 4s → 8s
#         Node bên trong dùng timeout 15s cho wait_for_first_frame,
#         nhưng script chỉ cần đợi đủ để topics được advertise.
#         8s là đủ khi camera healthy; nếu camera cần >8s thì
#         capture_loop sẽ xử lý qua startup_deadline grace period.
# =============================================================================

# 1. CSI Camera — phải chạy đầu tiên
start_node "CSI Dual Camera" \
    "ros2 run csi_camera csi_dual_camera_node 2>&1 | tee $LOG_DIR/csi_camera.log" \
    8   # [FIX-C] tăng từ 4s → 8s

# 2. YOLO — cần camera topics tồn tại
start_node "YOLO Detection" \
    "ros2 launch yolo_ros_hailort_cpp mutilcam_yolox_hailort.launch.py 2>&1 | tee $LOG_DIR/yolo.log" \
    4   # Hailo HEF load ~2-3s

# 3. BBox Drawer — cần YOLO topics
start_node "BBox Drawer" \
    "ros2 launch bbox_drawer_cpp overlay_two_cams.launch.py 2>&1 | tee $LOG_DIR/bbox_drawer.log" \
    2

# 4. Snap7 PLC — độc lập
start_node "Snap7 PLC" \
    "ros2 run snap7_driver snap7_node 2>&1 | tee $LOG_DIR/snap7.log" \
    2

# 5. Robot Logic — cần YOLO + PLC
start_node "Robot Logic" \
    "ros2 run robot_control_main robot_logic_node_dual 2>&1 | tee $LOG_DIR/robot_logic.log" \
    2

# 6. GUI — cuối cùng
start_node "GUI" \
    "export DISPLAY=:0 && ros2 run ros2_qml_gui1 ros2_qml_gui1 2>&1 | tee $LOG_DIR/gui.log" \
    1

echo ""
echo "=========================================="
echo "✅ All nodes launched!"
echo "   Logs: $LOG_DIR/"
echo "=========================================="
echo ""
echo "Press Ctrl+C to stop all nodes and exit"
echo ""

while true; do
    sleep 5
done