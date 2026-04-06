#!/bin/bash
# Test Dual Camera System - Step by Step Verification
# Run this script to verify the dual camera setup

set -e  # Exit on error

echo "=========================================="
echo "🧪 Dual Camera System Verification"
echo "=========================================="
echo

# Step 1: Check if cameras are detected
echo "Step 1: Checking camera detection..."
echo "---"
rpicam-hello --list-cameras
echo
read -p "Do you see 2 cameras (CAM0 and CAM1)? (y/n): " cameras_ok

if [ "$cameras_ok" != "y" ]; then
    echo "❌ Cameras not detected!"
    echo
    echo "Troubleshooting:"
    echo "1. Check physical connections (deserializers to CAM0/CAM1 ports)"
    echo "2. Verify power to Arducam GMSL2 kit"
    echo "3. Check /boot/firmware/config.txt for camera configuration"
    echo "4. Reboot: sudo reboot"
    exit 1
fi

echo "✅ Cameras detected!"
echo

# Step 2: Test CAM0 capture
echo "Step 2: Testing CAM0 (Input Tray) capture..."
echo "---"
echo "This will display CAM0 for 5 seconds..."
timeout 5 rpicam-hello --camera 0 || true
echo
read -p "Did you see CAM0 preview? (y/n): " cam0_ok

if [ "$cam0_ok" != "y" ]; then
    echo "❌ CAM0 failed!"
    exit 1
fi

echo "✅ CAM0 working!"
echo

# Step 3: Test CAM1 capture
echo "Step 3: Testing CAM1 (Output Tray) capture..."
echo "---"
echo "This will display CAM1 for 5 seconds..."
timeout 5 rpicam-hello --camera 1 || true
echo
read -p "Did you see CAM1 preview? (y/n): " cam1_ok

if [ "$cam1_ok" != "y" ]; then
    echo "❌ CAM1 failed!"
    exit 1
fi

echo "✅ CAM1 working!"
echo

# Step 4: Source ROS workspace
echo "Step 4: Setting up ROS environment..."
echo "---"
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
echo "✅ ROS environment ready!"
echo

# Step 5: Test dual camera node
echo "Step 5: Testing dual camera node..."
echo "---"
echo "Starting csi_dual_camera_node..."
echo "(Press Ctrl+C to stop after ~10 seconds)"
echo

ros2 run csi_camera csi_dual_camera_node &
NODE_PID=$!

sleep 5

# Check topics
echo
echo "Checking camera topics..."
ros2 topic list | grep -E "cam[01]Funai"

echo
echo "Checking publish rates..."
timeout 5 ros2 topic hz /cam0Funai/image_raw &
timeout 5 ros2 topic hz /cam1Funai/image_raw &
wait

# Stop node
kill $NODE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true

echo
echo "=========================================="
echo "✅ Dual Camera System Verification Complete!"
echo "=========================================="
echo
echo "Next steps:"
echo "1. Run full system: ros2 launch csi_camera dual_camera.launch.py"
echo "2. For backup switch mode: ros2 run csi_camera csi_camera_node"
echo
