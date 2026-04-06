# Dual Camera System - Quick Reference

## 🚀 Quick Start

### Run Dual Parallel Camera Mode (NEW)
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch csi_camera dual_camera.launch.py
```

### Run Switch-Based Camera Mode (BACKUP)
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run csi_camera csi_camera_node
```

## 📊 Topics

### Dual Camera Mode
```bash
# Camera streams
/cam0Funai/image_raw  # Input Tray (CAM0)
/cam1Funai/image_raw  # Output Tray (CAM1)

# YOLO detections
/cam0/detections      # Input Tray detections
/cam1/detections      # Output Tray detections

# Check rates
ros2 topic hz /cam0Funai/image_raw
ros2 topic hz /cam1Funai/image_raw
```

### Switch Mode (Backup)
```bash
# Camera streams
/cam0Funai/image_raw  # Currently active camera
/cam1Funai/image_raw  # (published to active stream)

# Control
ros2 topic pub -1 /robot/camera_select std_msgs/Int32 "{data: 0}"  # CAM0
ros2 topic pub -1 /robot/camera_select std_msgs/Int32 "{data: 1}"  # CAM1
```

## 🔧 Troubleshooting

### No cameras detected
```bash
# Check hardware
rpicam-hello --list-cameras

# Should show:
# Available cameras:
# 0 : imx219 [3280x2464] (/base/...)
# 1 : imx219 [3280x2464] (/base/...)
```

If no cameras:
1. Check physical connections (deserializers to CAM0/CAM1)
2. Verify Arducam GMSL2 kit power
3. Check `/boot/firmware/config.txt`
4. Reboot: `sudo reboot`

### Build issues
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select csi_camera --symlink-install --merge-install
source install/setup.bash
```

### Node crashes
```bash
# Check logs
ros2 run csi_camera csi_dual_camera_node

# Kill old processes
pkill -9 rpicam-vid
pkill -9 csi_dual_camera_node
```

## 📁 Files

### New Files (Dual Camera)
- `/home/pi/ros2_ws/src/csi_camera/src/csi_dual_camera_node.cpp`
- `/home/pi/ros2_ws/src/csi_camera/launch/dual_camera.launch.py`

### Backup Files (Switch Camera)
- `/home/pi/ros2_ws/src/csi_camera/src/csi_camera_node.cpp`
- `/home/pi/ros2_ws/src/csi_camera/launch/full_camera_system.launch.py`

## 🧪 Testing

Run verification script:
```bash
cd ~/ros2_ws
./test_dual_camera.sh
```

## 🔄 Switching Between Modes

Just use different launch files:
```bash
# Dual mode
ros2 launch csi_camera dual_camera.launch.py

# Single mode (backup)
ros2 launch csi_camera full_camera_system.launch.py
```

Both executables are installed and ready to use!
