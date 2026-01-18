#!/bin/bash
# Stop all robot control processes cleanly
# Usage: ./stop_all.sh

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🛑 Stopping Robot Control System..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Kill robot logic node
echo "  Stopping robot_logic_node..."
pkill -9 -f "robot_logic_node" 2>/dev/null

# Kill dobot bringup
echo "  Stopping dobot_bringup..."
pkill -9 -f "dobot_bringup" 2>/dev/null

# Kill gripper node
echo "  Stopping gripper_festo_node..."
pkill -9 -f "gripper_festo_node" 2>/dev/null

# Kill camera nodes
echo "  Stopping camera nodes..."
pkill -9 -f "csi_camera_node" 2>/dev/null
pkill -9 -f "yolo_ros_hailort_cpp" 2>/dev/null
pkill -9 -f "system_csi_dual_model" 2>/dev/null

# Kill other nodes
echo "  Stopping other nodes (bbox, snap7, GUI)..."
pkill -9 -f "bbox_drawer_cpp" 2>/dev/null
pkill -9 -f "snap7_node" 2>/dev/null
pkill -9 -f "ros2_qml_gui1" 2>/dev/null
pkill -9 -f "overlay_two_cams.launch.py" 2>/dev/null

# Kill bash watchdogs
echo "  Stopping bash watchdogs..."
pkill -f "bash -c.*Source ROS2 environment" 2>/dev/null || true

# Wait for processes to terminate
sleep 1

# Verify all stopped
REMAINING=$(ps aux | grep -E "(robot_logic_node|dobot_bringup|gripper_festo|csi_camera|yolo|bbox_drawer|snap7|ros2_qml_gui1)" | grep -v grep | wc -l)

echo ""
if [ $REMAINING -eq 0 ]; then
    echo "✅ All processes stopped successfully!"
else
    echo "⚠️  Warning: $REMAINING process(es) still running:"
    ps aux | grep -E "(robot_logic_node|dobot_bringup|gripper_festo|csi_camera|yolo)" | grep -v grep
    echo ""
    echo "Try running: sudo killall -9 robot_logic_node dobot_bringup gripper_festo_node.py csi_camera_node"
fi

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
