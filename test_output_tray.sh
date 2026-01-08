#!/bin/bash
# Script test Output Tray Motion Control

echo "=========================================="
echo "Output Tray Motion Control - Test Script"
echo "=========================================="
echo ""

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 not found. Please source ROS2 setup"
    exit 1
fi

echo "✅ ROS2 found"
echo ""

# Test commands
echo "📋 Test Commands Available:"
echo ""
echo "1️⃣  Manual Mode - Slot 1:"
echo "   ros2 topic pub /robot/command_slot std_msgs/Int32 \"data: 1\""
echo ""
echo "2️⃣  Manual Mode - Slot 5:"
echo "   ros2 topic pub /robot/command_slot std_msgs/Int32 \"data: 5\""
echo ""
echo "3️⃣  Manual Mode - Slot 8:"
echo "   ros2 topic pub /robot/command_slot std_msgs/Int32 \"data: 8\""
echo ""
echo "4️⃣  Auto Mode - Camera detect Slot 3:"
echo "   ros2 topic pub /camera/ai/selected_slot std_msgs/Int32 \"data: 3\""
echo ""
echo "5️⃣  Monitor Robot Status:"
echo "   ros2 topic echo /robot/system_status"
echo ""
echo "6️⃣  Monitor Selected Slot:"
echo "   ros2 topic echo /robot/selected_slot"
echo ""

echo "=========================================="
echo ""

# Optionally start the node
read -p "Do you want to launch robot_logic_node? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "🚀 Launching robot_logic_node..."
    ros2 run robot_control_main robot_logic_node
else
    echo "📌 Skipped. Run manually:"
    echo "   ros2 run robot_control_main robot_logic_node"
fi
