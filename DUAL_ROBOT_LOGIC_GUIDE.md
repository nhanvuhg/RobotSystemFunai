# Dual Camera Robot Logic - Quick Guide

## ✅ What Was Created

Successfully created `robot_logic_node_dual.cpp` - processes both cameras simultaneously!

---

## 📁 Files

### New Files (Dual Camera)
- `robot_logic_node_dual.cpp` - Dual camera logic
- Updated `CMakeLists.txt` - Builds both nodes

### Backup Files (Switch Camera)  
- `robot_logic_node.cpp` - Original switch-based (preserved)

---

## 🔧 Key Changes in Dual Mode

### Camera Switching Removed
- ❌ No `camera_select_pub_` publisher
- ❌ No `switchAndWaitForCamera()` calls
- ✅ Both cameras always active

### Topic Updates
```cpp
// CAM0 (Input Tray)
/cam0/detections  // was: /cam0Funai/yolo/bounding_boxes

// CAM1 (Output Tray)
/cam1/detections  // was: /cam1Funai/yolo/bounding_boxes
```

### Node Name
```
robot_logic_nova5_dual  // was: robot_logic_nova5
```

---

## 🚀 How to Run

### Dual Camera Mode (NEW)
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot_control_main robot_logic_node_dual
```

### Switch Mode (BACKUP)
```bash
ros2 run robot_control_main robot_logic_node
```

---

## 🧪 Full System Test

Launch entire dual camera system:
```bash
# Terminal 1: Dual cameras + YOLO
ros2 launch csi_camera dual_camera.launch.py

# Terminal 2: Robot logic (dual mode)
ros2 run robot_control_main robot_logic_node_dual
```

---

## 📊 System Architecture

### Before (Switch Mode)
```
┌──────────────┐
│ Robot Logic  │
└──────┬───────┘
       │ /robot/camera_select
       ▼
  ┌─────────┐
  │  Switch │
  └────┬────┘
    CAM0 OR CAM1
       │
   Single YOLO
       │
  /yolo/detections
```

### After (Dual Mode)
```
┌─────────────────┐
│ Robot Logic Dual│
└────┬──────┬─────┘
     │      │
CAM0 ▼      ▼ CAM1
/cam0/      /cam1/
détections  detections
     │      │
Always processing both!
```

---

## 🎯 Built Executables

✅ `robot_logic_node` - Switch mode (backup)  
✅ `robot_logic_node_dual` - Dual parallel mode

Both installed to: `/home/pi/ros2_ws/install/lib/robot_control_main/`

---

## 📝 Next Steps

1. Test with full system: cameras + YOLO + robot logic
2. Verify PLC commands work in dual mode
3. Check slot detection on both trays simultaneously

---

## 🔄 Switching Between Modes

Just use different executables:
```bash
# Dual mode
ros2 run robot_control_main robot_logic_node_dual

# Single mode (backup)
ros2 run robot_control_main robot_logic_node
```

No configuration changes needed!
