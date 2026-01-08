#!/bin/bash
echo "🧹 Cleaning up ROS 2 and Hailo processes..."

# Kill ROS nodes
pkill -9 -f component_container
pkill -9 -f yolo_ros_hailort
pkill -9 -f csi_camera_node
pkill -9 -f ros2

# Kill Python processes (often hold locks)
pkill -9 -f python3

# Reset Hailo runtime (if possible via driver, usually just killing process is enough)
# But ensuring no zombie holding /dev/hailo0
fuser -k /dev/hailo0 >/dev/null 2>&1

echo "✅ Cleanup complete. You can now relaunch."
