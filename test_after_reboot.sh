#!/bin/bash
# Post-Reboot Test Script
# Run this AFTER rebooting to verify dual camera setup

echo "=========================================="
echo "🔍 Dual Camera Post-Reboot Verification"
echo "=========================================="
echo

# Check camera detection
echo "Step 1: Checking camera detection..."
echo "---"
rpicam-hello --list-cameras
echo

# Check if 2 cameras detected
CAMERA_COUNT=$(rpicam-hello --list-cameras 2>&1 | grep -c "imx219")

if [ "$CAMERA_COUNT" -eq 2 ]; then
    echo "✅ SUCCESS: Detected $CAMERA_COUNT cameras!"
    echo
    
    echo "Step 2: Testing CAM0..."
    timeout 3 rpicam-hello --camera 0 -t 2000 || true
    echo "✅ CAM0 test complete"
    echo
    
    echo "Step 3: Testing CAM1..."
    timeout 3 rpicam-hello --camera 1 -t 2000 || true
    echo "✅ CAM1 test complete"
    echo
    
    echo "=========================================="
    echo "✅ Hardware Ready for Dual Camera Mode!"
    echo "=========================================="
    echo
    echo "Next step: Run dual camera node"
    echo "  cd ~/ros2_ws"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  source install/setup.bash"
    echo "  ros2 run csi_camera csi_dual_camera_node"
    echo
else
    echo "❌ FAILED: Only detected $CAMERA_COUNT camera(s)"
    echo
    echo "Troubleshooting:"
    echo "1. Check config.txt:"
    echo "   cat /boot/firmware/config.txt | grep imx219"
    echo
    echo "2. Check I2C buses:"
    echo "   i2cdetect -l"
    echo
    echo "3. Check kernel messages:"
    echo "   dmesg | grep -i imx219"
    echo
    echo "4. Verify physical connections of both cameras"
    echo
fi
