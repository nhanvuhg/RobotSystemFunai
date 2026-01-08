# 📷 Camera Switching System - Complete Documentation

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware & Topics](#hardware--topics)
- [Implementation Details](#implementation-details)
- [State Machine Integration](#state-machine-integration)
- [Testing Guide](#testing-guide)
- [Error Handling](#error-handling)
- [Troubleshooting](#troubleshooting)
- [API Reference](#api-reference)

---

## 🎯 Overview

### Purpose
Robot control system cần switch giữa 2 cameras:
- **Camera 1 (cam0)**: Input Tray + Buffer detection
- **Camera 2 (cam1)**: Output Tray slot detection

### Key Features
✅ **Automatic Switching**: Robot tự động switch camera dựa vào state  
✅ **Manual Control**: Operators có thể manual switch qua ROS topic  
✅ **Retry Logic**: 3 attempts với 2s delay  
✅ **Detection Validation**: Verify AI inference hoạt động sau switch  
✅ **Visual Feedback**: Status publishing cho HMI/monitoring  
✅ **Error Recovery**: Safe stop khi switch failed  

### Hardware Timing
- **I2C Command**: ~20ms
- **Hardware Switch**: ~1.5s (Arducam multiplexer)
- **Stream Restart**: ~100ms
- **Frame Stabilization**: 500ms (configurable)
- **Total Switch Time**: ~2.1s typical, **5s timeout**

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      ROBOT STATE MACHINE                         │
│  (robot_logic_nova5_optimized)                                   │
│                                                                   │
│  States requiring Camera 1:        States requiring Camera 2:    │
│  - INIT_LOAD_CHAMBER_DIRECT       - PLACE_TO_OUTPUT             │
│  - INIT_REFILL_BUFFER             - ERROR_OUTPUT_TRAY_TIMEOUT   │
│  - REFILL_BUFFER                                                 │
└──────────────────┬────────────────────────────────┬──────────────┘
                   │ Publish                        │
                   │ /robot/camera_select           │
                   │ (Int32: 1 or 2)                │
                   ▼                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                    CSI CAMERA NODE                               │
│  (csi_camera_node)                                               │
│                                                                   │
│  1. Receive /robot/camera_select                                 │
│  2. Send I2C command to Arducam HAT                              │
│  3. Release old VideoCapture                                     │
│  4. Wait 1.5s for hardware switch                                │
│  5. Open new VideoCapture stream                                 │
│  6. Publish /camera/active_id confirmation                       │
│  7. Publish frames to /ai/image_overlay                          │
└──────────────────┬────────────────────────────────┬──────────────┘
                   │ Publish                        │
                   │ /camera/active_id              │
                   │ (Int32: 1 or 2)                │
                   ▼                                │
┌─────────────────────────────────┐                │
│   ROBOT LOGIC - Camera Wait     │                │
│                                 │                │
│  1. Wait for active_id match    │                │
│  2. Wait 500ms warm-up          │                │
│  3. Validate detections         │                │
│  4. Proceed with motion         │                │
└─────────────────────────────────┘                │
                                                    │ Subscribe
                                                    │ /ai/image_overlay
                                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                      YOLO CONTAINER                              │
│  (2 nodes, shared image input)                                   │
│                                                                   │
│  ┌────────────────────────┐    ┌────────────────────────┐       │
│  │  yolo_cam0 (Model A)   │    │  yolo_cam1 (Model B)   │       │
│  │  - Input Tray model    │    │  - Output Tray model   │       │
│  │  - Subscribe: /ai/...  │    │  - Subscribe: /ai/...  │       │
│  │  - Publish:            │    │  - Publish:            │       │
│  │    /cam0/yolo/boxes    │    │    /cam1/yolo/boxes    │       │
│  └────────────────────────┘    └────────────────────────┘       │
└──────────────────┬────────────────────────────────┬──────────────┘
                   │                                │
                   │ Subscribe                      │ Subscribe
                   ▼                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                  ROBOT LOGIC - AI Processing                     │
│                                                                   │
│  camera1Callback():          camera2Callback():                  │
│  - /cam0/yolo/boxes          - /cam1/yolo/boxes                 │
│  - Process Input Tray ROIs   - Process Output Tray ROIs         │
│  - Update buffer_count       - Update selected_slot             │
│  - Store latest_cam0_boxes_  - Store latest_cam1_boxes_         │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🔌 Hardware & Topics

### ROS Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/robot/camera_select` | Int32 | Robot → CSI | Camera switch request (1 or 2) |
| `/camera/active_id` | Int32 | CSI → Robot | Camera switch confirmation (1 or 2) |
| `/camera/status` | String | Robot → HMI | Status messages for monitoring |
| `/ai/image_overlay` | Image | CSI → YOLO | **Shared image stream** (active camera) |
| `/cam0/yolo/bounding_boxes` | Detection2DArray | YOLO → Robot | Input Tray detections (Model A) |
| `/cam1/yolo/bounding_boxes` | Detection2DArray | YOLO → Robot | Output Tray detections (Model B) |
| `/robot/command_camera` | Int32 | Manual → Robot | Manual camera switch command |
| `/robot/error` | String | Robot → HMI | Error messages |

### Camera ID Mapping

```
Physical Hardware        Camera ID    ROS Topics              Purpose
─────────────────────────────────────────────────────────────────────
Camera 1 (CSI Port 0)    cam_id=1     /cam0/yolo/boxes       Input Tray + Buffer
Camera 2 (CSI Port 1)    cam_id=2     /cam1/yolo/boxes       Output Tray (8 slots)
```

### I2C Commands (Arducam Multiplexer HAT)

```bash
# Camera 1 (Port A)
i2ctransfer -f -y 4 w3@0x0c 0xff 0x55 0x01

# Camera 2 (Port B)
i2ctransfer -f -y 4 w3@0x0c 0xff 0x55 0x02

# Bus 4: Raspberry Pi 5 default I2C bus
# Address 0x0c: Arducam multiplexer HAT
```

---

## ⚙️ Implementation Details

### Class Members (Add to RobotLogicNode)

```cpp
// Publishers & Subscribers
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr camera_select_pub_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_status_pub_;
rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr camera_active_id_sub_;
rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr command_camera_sub_;

// Camera state
std::atomic<int> current_active_camera_{0};  // 0=none, 1=cam0, 2=cam1
std::mutex camera_mutex_;

// Detection validation
std::vector<Detection2D> latest_cam0_boxes_;
std::vector<Detection2D> latest_cam1_boxes_;
std::mutex detection_mutex_;
```

### Core Functions

#### 1. **requestCameraSwitch(int camera_id)**

**Purpose**: Send camera switch request to CSI node

```cpp
void RobotLogicNode::requestCameraSwitch(int camera_id)
{
    // Validate camera_id (1 or 2)
    // Check if already active
    // Publish to /robot/camera_select
    // Update visual feedback
}
```

**Usage**:
```cpp
requestCameraSwitch(1);  // Switch to Camera 1
requestCameraSwitch(2);  // Switch to Camera 2
```

---

#### 2. **waitForCameraActive(int target_camera, double timeout_sec)**

**Purpose**: Wait for CSI node to confirm camera switch

**Returns**: `true` if confirmed, `false` if timeout

```cpp
bool RobotLogicNode::waitForCameraActive(int target_camera, double timeout_sec)
{
    // Loop with 100ms spin intervals
    // Check current_active_camera_ matches target
    // Log progress every second
    // Timeout after timeout_sec seconds
}
```

**Timing**:
- Poll interval: 100ms
- Progress logs: Every 1s
- Default timeout: 5.0s
- Typical confirmation: 1.7-2.0s

---

#### 3. **waitForFirstDetection(int camera_id, double timeout_sec)**

**Purpose**: Validate AI inference is working after camera switch

**Returns**: `true` if detection received OR timeout (not an error)

```cpp
bool RobotLogicNode::waitForFirstDetection(int camera_id, double timeout_sec)
{
    // Clear old detections
    // Wait for latest_cam0_boxes_ or latest_cam1_boxes_
    // Return true even if empty scene (not an error)
}
```

**Why timeout is not an error?**
- Empty scene is valid (e.g., no cartridges in tray)
- We only want to verify YOLO is processing frames
- If timeout → log warning but continue

---

#### 4. **switchAndWaitForCamera(int camera_id)**

**Purpose**: Complete camera switch with all validation steps

**Returns**: `true` if all steps successful

```cpp
bool RobotLogicNode::switchAndWaitForCamera(int camera_id)
{
    // Step 1: requestCameraSwitch()
    // Step 2: waitForCameraActive() - 5s timeout
    // Step 3: 500ms warm-up delay
    // Step 4: waitForFirstDetection() - 3s timeout
    // Step 5: Publish CAMERA_READY status
}
```

**Total Time**:
- Best case: ~2.5s (hardware 1.7s + warm-up 0.5s + detection 0.3s)
- Worst case: ~8.5s (hardware timeout 5s + warm-up 0.5s + detection timeout 3s)

---

#### 5. **switchAndWaitForCameraWithRetry(int camera_id, int max_retries)**

**Purpose**: Camera switch with retry logic

**Returns**: `true` if any attempt successful

```cpp
bool RobotLogicNode::switchAndWaitForCameraWithRetry(int camera_id, int max_retries)
{
    for (attempt = 1 to max_retries) {
        if (switchAndWaitForCamera(camera_id)) {
            return true;  // Success
        }
        if (attempt < max_retries) {
            sleep(2s);  // Delay before retry
        }
    }
    return false;  // All attempts failed
}
```

**Retry Strategy**:
- Max attempts: 3 (configurable)
- Inter-retry delay: 2s
- Total max time: ~28s (8.5s × 3 + 2s × 2)

---

## 🔄 State Machine Integration

### States Requiring Camera 1 (Input Tray)

```cpp
void RobotLogicNode::stateInitLoadChamberDirect()
{
    // 🎥 Switch to Camera 1
    if (!switchAndWaitForCameraWithRetry(1, 3)) {
        // Error: Failed after 3 attempts
        publishError("CAMERA_SWITCH_FAILED_INPUT_TRAY");
        system_enabled_ = false;
        transitionTo(SystemState::IDLE);
        return;
    }
    
    // ✅ Camera ready, proceed with motion
    motionStub_InputTrayChamber();
    // ...
}
```

**Similar states**:
- `stateInitRefillBuffer()`
- `stateRefillBuffer()`

---

### States Requiring Camera 2 (Output Tray)

```cpp
void RobotLogicNode::statePlaceToOutput()
{
    // 🎥 Switch to Camera 2
    if (!switchAndWaitForCameraWithRetry(2, 3)) {
        // Error: Failed after 3 attempts
        publishError("CAMERA_SWITCH_FAILED_OUTPUT_TRAY");
        system_enabled_ = false;
        transitionTo(SystemState::IDLE);
        return;
    }
    
    // ✅ Camera ready, wait for slot detection
    if (selected_output_slot_ == -1) {
        // Wait for AI to find empty slot...
        return;
    }
    
    motionStub_ScaleOutputTray(selected_output_slot_);
    // ...
}
```

---

### States NOT Requiring Camera Switch

```cpp
void RobotLogicNode::stateTakeChamberToScale()
{
    // ❌ NO camera switch needed
    // Pure motion: Chamber → Scale
    motionStub_ChamberScale();
    transitionTo(SystemState::WAIT_SCALE_RESULT);
}

void RobotLogicNode::stateLoadChamberFromBuffer()
{
    // ❌ NO camera switch needed
    // Pure motion: Buffer → Chamber
    motionStub_BufferChamber();
    transitionTo(SystemState::WAIT_FILLING);
}
```

---

## 🧪 Testing Guide

### Prerequisites

```bash
# 1. Start CSI camera node
ros2 run csi_camera csi_camera_node

# 2. Start YOLO container (2 models)
ros2 launch yolo_bringup dual_yolo.launch.py

# 3. Start robot control (with camera switching)
ros2 run robot_control_main robot_logic_nova5_optimized

# 4. Monitor topics in separate terminals
ros2 topic echo /camera/active_id
ros2 topic echo /camera/status
ros2 topic echo /robot/error
```

---

### Test 1: Manual Camera Switch

**Purpose**: Verify basic camera switching works

```bash
# Terminal 1: Monitor status
ros2 topic echo /camera/status

# Terminal 2: Switch to Camera 1
ros2 topic pub -1 /robot/command_camera std_msgs/Int32 "{data: 1}"

# Expected output in logs:
# [CAMERA] 📷 Manual switch to Camera 1
# [CAMERA] 🔄 Attempt 1/3 to switch to Camera 1
# [CAMERA] 📷 Requesting Camera 1 switch...
# [CAMERA] ⏳ Waiting for Camera 1 confirmation (timeout: 5.0s)...
# [CAMERA] ✅ Camera 1 confirmed active (took 1.82s)
# [CAMERA] 🔄 Waiting 500ms for frame stabilization...
# [CAMERA] 🔍 Waiting for first detection from Camera 1...
# [CAMERA] ✅ First detection received from Camera 1 (8 objects)
# [CMD CAMERA] ✅ Manual switch successful

# Terminal 3: Switch to Camera 2
ros2 topic pub -1 /robot/command_camera std_msgs/Int32 "{data: 2}"

# Expected: Similar logs for Camera 2
```

**Success Criteria**:
- ✅ Switch completes in ~2-3s
- ✅ `/camera/active_id` publishes correct ID
- ✅ Detection received from correct YOLO model
- ✅ No error messages

---

### Test 2: Automatic Switch in State Machine

**Purpose**: Verify states trigger camera switches correctly

```bash
# Setup
ros2 service call /robot/set_manual_mode std_srvs/srv/SetBool "{data: true}"
ros2 topic pub -1 /system/start_button std_msgs/Bool "{data: true}"

# Test Input Tray state (requires Camera 1)
ros2 topic pub -1 /robot/command_row std_msgs/Int32 "{data: 2}"
ros2 topic pub -1 /robot/goto_state std_msgs/String "{data: 'INIT_LOAD_CHAMBER_DIRECT'}"

# Expected logs:
# [STATE] IDLE → INIT_LOAD_CHAMBER_DIRECT
# [CAMERA] 🔄 Attempt 1/3 to switch to Camera 1
# [CAMERA] ✅ Camera 1 confirmed active (took 1.75s)
# [MOTION] Input Tray → Chamber
# [MOTION] 🎯 Picking from Row 2
# [STATE] INIT_LOAD_CHAMBER_DIRECT → IDLE

# Test Output Tray state (requires Camera 2)
ros2 topic pub -1 /robot/command_slot std_msgs/Int32 "{data: 3}"
ros2 topic pub -1 /robot/goto_state std_msgs/String "{data: 'PLACE_TO_OUTPUT'}"

# Expected logs:
# [STATE] IDLE → PLACE_TO_OUTPUT
# [CAMERA] 🔄 Attempt 1/3 to switch to Camera 2
# [CAMERA] ✅ Camera 2 confirmed active (took 1.83s)
# [MOTION] Scale → Output Tray Slot 3
# [STATE] PLACE_TO_OUTPUT → IDLE
```

**Success Criteria**:
- ✅ State triggers correct camera ID
- ✅ Motion proceeds after camera ready
- ✅ No timeout errors
- ✅ System returns to IDLE

---

### Test 3: Retry Logic (Simulate Failure)

**Purpose**: Verify retry mechanism works

```bash
# Terminal 1: Monitor camera status
ros2 topic echo /camera/status

# Terminal 2: STOP CSI camera node (simulate hardware failure)
# Find process: ps aux | grep csi_camera
# Kill it: kill -9 <PID>

# Terminal 3: Try to switch camera
ros2 topic pub -1 /robot/command_camera std_msgs/Int32 "{data: 1}"

# Expected logs:
# [CAMERA] 🔄 Attempt 1/3 to switch to Camera 1
# [CAMERA] ⏳ Waiting for Camera 1 confirmation...
# [CAMERA] ⏱️ Still waiting... (1.0s / 5.0s)
# [CAMERA] ⏱️ Still waiting... (2.0s / 5.0s)
# [CAMERA] ⏱️ Still waiting... (3.0s / 5.0s)
# [CAMERA] ⏱️ Still waiting... (4.0s / 5.0s)
# [CAMERA] ❌ Timeout waiting for Camera 1 (5.0s elapsed)
# [CAMERA] ⚠️ Attempt 1 failed, retrying in 2 seconds...
# [CAMERA] 🔄 Attempt 2/3 to switch to Camera 1
# (repeat...)
# [CAMERA] ❌ Failed to switch to Camera 1 after 3 attempts
# [CMD CAMERA] ❌ Manual switch failed

# Terminal 4: Restart CSI node
ros2 run csi_camera csi_camera_node

# Terminal 5: Retry camera switch
ros2 topic pub -1 /robot/command_camera std_msgs/Int32 "{data: 1}"

# Expected: Success after restart
```

**Success Criteria**:
- ✅ 3 retry attempts executed
- ✅ 2s delay between retries
- ✅ Error logged after all failures
- ✅ Success after CSI node restart

---

### Test 4: Detection Validation

**Purpose**: Verify detection validation works with empty/full scenes

```bash
# Test A: Empty scene (no objects)
# 1. Clear all cartridges from input tray
# 2. Switch to Camera 1
ros2 topic pub -1 /robot/command_camera std_msgs/Int32 "{data: 1}"

# Expected logs:
# [CAMERA] 🔍 Waiting for first detection from Camera 1...
# [CAMERA] ⚠️ No detection after 3.0s (may be empty scene - not an error)
# [CMD CAMERA] ✅ Manual switch successful

# Test B: Full scene (many objects)
# 1. Fill input tray with cartridges
# 2. Switch to Camera 1
ros2 topic pub -1 /robot/command_camera std_msgs/Int32 "{data: 1}"

# Expected logs:
# [CAMERA] 🔍 Waiting for first detection from Camera 1...
# [CAMERA] ✅ First detection received from Camera 1 (15 objects)
# [CMD CAMERA] ✅ Manual switch successful
```

**Success Criteria**:
- ✅ Empty scene → Warning but continues
- ✅ Full scene → Fast detection confirmation
- ✅ No false failures

---

### Test 5: Rapid Switch Stress Test

**Purpose**: Verify system handles rapid camera switches

```bash
# Rapid switch loop (10 iterations)
for i in {1..10}; do
    echo "Iteration $i: Camera 1"
    ros2 topic pub -1 /robot/command_camera std_msgs/Int32 "{data: 1}"
    sleep 3
    
    echo "Iteration $i: Camera 2"
    ros2 topic pub -1 /robot/command_camera std_msgs/Int32 "{data: 2}"
    sleep 3
done

# Monitor for:
# - Memory leaks
# - Dropped frames
# - Timeout errors
# - Publisher/subscriber cleanup issues
```

**Success Criteria**:
- ✅ All 20 switches complete successfully
- ✅ No memory leaks (check `htop`)
- ✅ Consistent switch time (~2-3s)
- ✅ No hanging processes

---

### Test 6: Full Workflow Integration

**Purpose**: Test complete production workflow

```bash
# 1. Enable system
ros2 service call /robot/enable_system std_srvs/srv/SetBool "{data: true}"

# 2. Load new tray
ros2 topic pub -1 /system/new_tray_loaded std_msgs/Bool "{data: true}"

# 3. Start button
ros2 topic pub -1 /system/start_button std_msgs/Bool "{data: true}"

# Monitor state transitions and camera switches:
# IDLE → INIT_CHECK → INIT_LOAD_CHAMBER_DIRECT
#   ↓ (Camera 1 switch)
# WAIT_FILLING → TAKE_CHAMBER_TO_SCALE → WAIT_SCALE_RESULT
#   ↓ (No camera switch)
# PLACE_TO_OUTPUT
#   ↓ (Camera 2 switch)
# REFILL_BUFFER
#   ↓ (Camera 1 switch)
# (loop continues...)
```

**Success Criteria**:
- ✅ Correct camera for each state
- ✅ No unnecessary switches
- ✅ Smooth workflow without pauses
- ✅ All cartridges processed

---

## ⚠️ Error Handling

### Error Types

| Error | Cause | Recovery |
|-------|-------|----------|
| `CAMERA_SWITCH_FAILED_INPUT_TRAY` | Camera 1 timeout after 3 retries | Stop & manual reset |
| `CAMERA_SWITCH_FAILED_OUTPUT_TRAY` | Camera 2 timeout after 3 retries | Stop & manual reset |
| `TIMEOUT_WAITING_CAMERA_X` | CSI node not responding | Automatic retry |
| `INVALID_CAMERA_ID` | Invalid camera_id in request | Reject request |

### Error Recovery Procedure

**When camera switch fails after all retries:**

```cpp
// System behavior:
1. Publish error to /robot/error
2. Publish status to /camera/status
3. Set system_enabled_ = false
4. Transition to SystemState::IDLE
5. Wait for manual intervention
```

**Manual recovery steps:**

```bash
# 1. Check CSI camera node is running
ros2 node list | grep csi_camera

# 2. Check camera hardware
# Verify cameras detected:
libcamera-hello --list-cameras

# 3. Check I2C bus
i2cdetect -y 4

# 4. Restart CSI node if needed
ros2 lifecycle set /csi_camera_node shutdown
ros2 lifecycle set /csi_camera_node activate

# 5. Re-enable robot system
ros2 service call /robot/enable_system std_srvs/srv/SetBool "{data: true}"

# 6. Test camera switch manually
ros2 topic pub -1 /robot/command_camera std_msgs/Int32 "{data: 1}"
```

---

## 🔍 Troubleshooting

### Issue 1: Camera switch timeout

**Symptoms**:
```
[CAMERA] ❌ Timeout waiting for Camera 1 (5.0s elapsed)
```

**Possible causes**:
1. CSI camera node not running
2. I2C communication failure
3. Camera hardware disconnected
4. Wrong I2C bus/address

**Debug steps**:
```bash
# Check node is running
ros2 node list | grep csi_camera

# Check active_id topic
ros2 topic hz /camera/active_id
ros2 topic echo /camera/active_id

# Check I2C
i2cdetect -y 4  # Should show 0x0c

# Manual I2C test
i2ctransfer -f -y 4 w3@0x0c 0xff 0x55 0x01

# Check camera detection
libcamera-hello --list-cameras
```

---

### Issue 2: Detection validation timeout (warning only)

**Symptoms**:
```
[CAMERA] ⚠️ No detection after 3.0s (may be empty scene)
```

**Possible causes**:
1. Empty scene (no objects) → **Not an error**
2. YOLO node not running
3. YOLO model not loaded
4. Wrong topic subscription

**Debug steps**:
```bash
# Check YOLO is publishing
ros2 topic hz /cam0/yolo/bounding_boxes
ros2 topic hz /cam1/yolo/bounding_boxes

# Check YOLO node
ros2 node list | grep yolo

# Echo detections
ros2 topic echo /cam0/yolo/bounding_boxes --no-arr

# Check if scene is truly empty
ros2 run rqt_image_view rqt_image_view /ai/image_overlay
```

---

### Issue 3: Wrong detections published

**Symptoms**:
- Camera 1 active but getting output tray detections
- Camera 2 active but getting input tray detections

**Root cause**:
- Both YOLO models run on same `/ai/image_overlay`
- Robot needs to **ignore wrong model's output**

**Verification**:
```bash
# Check active camera
ros2 topic echo /camera/active_id

# Check which detections have data
ros2 topic echo /cam0/yolo/bounding_boxes  # Should have data when cam=1
ros2 topic echo /cam1/yolo/bounding_boxes  # Should have data when cam=2
```

**Solution**:
Robot code correctly uses `current_active_camera_` to select which detection topic to process:
```cpp
if (current_active_camera_ == 1) {
    // Use latest_cam0_boxes_
} else if (current_active_camera_ == 2) {
    // Use latest_cam1_boxes_
}
```

---

### Issue 4: Retry loop stuck

**Symptoms**:
```
[CAMERA] 🔄 Attempt 3/3 to switch to Camera 1
(nothing happens after)
```

**Possible causes**:
1. Thread deadlock
2. Callback not processed
3. Event loop blocked

**Debug steps**:
```bash
# Check CPU usage
htop  # Look for 100% CPU on robot process

# Check for deadlock
gdb -p <robot_pid>
(gdb) thread apply all bt

# Restart robot node
ros2 lifecycle set /robot_logic_nova5 shutdown
ros2 lifecycle set /robot_logic_nova5 activate
```

---

### Issue 5: Visual feedback not showing

**Symptoms**:
- No messages on `/camera/status`

**Debug steps**:
```bash
# Check topic exists
ros2 topic list | grep camera/status

# Check publisher count
ros2 topic info /camera/status

# Force publish
ros2 topic pub /camera/status std_msgs/String "{data: 'TEST'}"

# Check HMI is subscribing
ros2 topic echo /camera/status
```

---

## 📚 API Reference

### ROS Topics

#### `/robot/camera_select` (Int32)

**Direction**: Robot → CSI Camera Node  
**Purpose**: Request camera switch

```bash
# Switch to Camera 1 (Input Tray)
ros2 topic pub /robot/camera_select std_msgs/Int32 "{data: 1}"

# Switch to Camera 2 (Output Tray)
ros2 topic pub /robot/camera_select std_msgs/Int32 "{data: 2}"
```

---

#### `/camera/active_id` (Int32)

**Direction**: CSI Camera Node → Robot  
**Purpose**: Confirm active camera

```bash
# Listen for confirmations
ros2 topic echo /camera/active_id

# Expected values:
# data: 0  # No camera active
# data: 1  # Camera 1 (Input Tray) active
# data: 2  # Camera 2 (Output Tray) active
```

---

#### `/robot/command_camera` (Int32)

**Direction**: Manual Control → Robot  
**Purpose**: Manually trigger camera switch

```bash
# Switch to Camera 1
ros2 topic pub -1 /robot/command_camera std_msgs/Int32 "{data: 1}"

# Switch to Camera 2
ros2 topic pub -1 /robot/command_camera std_msgs/Int32 "{data: 2}"

# Invalid (will be rejected)
ros2 topic pub -1 /robot/command_camera std_msgs/Int32 "{data: 3}"
```

---

#### `/camera/status` (String)

**Direction**: Robot → Monitoring/HMI  
**Purpose**: Visual feedback on camera switching progress

**Message Types**:

| Status | Meaning |
|--------|---------|
| `SWITCHING_TO_CAMERA_1` | Switch request sent |
| `CAMERA_1_ACTIVE` | Hardware confirmed |
| `CAMERA_1_READY` | All validation passed |
| `MANUAL_SWITCH_REQUESTED_CAM_1` | Manual switch triggered |
| `MANUAL_SWITCH_SUCCESS_CAM_1` | Manual switch complete |
| `SWITCH_ATTEMPT_1_CAM_1` | Retry attempt 1 |
| `TIMEOUT_WAITING_CAMERA_1` | Switch timeout |
| `SWITCH_FAILED_AFTER_3_ATTEMPTS_CAM_1` | All retries failed |

```bash
# Monitor in real-time
ros2 topic echo /camera/status

# Log to file
ros2 topic echo /camera/status > camera_log.txt
```

---

### C++ API

#### `void requestCameraSwitch(int camera_id)`

Request camera switch (non-blocking)

```cpp
// Usage:
requestCameraSwitch(1);  // Request Camera 1
requestCameraSwitch(2);  // Request Camera 2
```

---

#### `bool waitForCameraActive(int target_camera, double timeout_sec = 5.0)`

Wait for camera confirmation (blocking)

```cpp
// Usage:
if (waitForCameraActive(1, 5.0)) {
    RCLCPP_INFO(get_logger(), "Camera 1 ready");
} else {
    RCLCPP_ERROR(get_logger(), "Camera 1 timeout");
}
```

**Parameters**:
- `target_camera`: 1 or 2
- `timeout_sec`: Max wait time (default: 5.0s)

**Returns**: `true` if confirmed, `false` if timeout

---

#### `bool waitForFirstDetection(int camera_id, double timeout_sec = 3.0)`

Validate AI detection working (blocking)

```cpp
// Usage:
if (waitForFirstDetection(1, 3.0)) {
    RCLCPP_INFO(get_logger(), "Detection validated");
} else {
    RCLCPP_WARN(get_logger(), "No detection (maybe empty scene)");
}
```

**Parameters**:
- `camera_id`: 1 or 2
- `timeout_sec`: Max wait time (default: 3.0s)

**Returns**: Always `true` (timeout is not an error)

---

#### `bool switchAndWaitForCamera(int camera_id)`

Complete camera switch with all validation steps

```cpp
// Usage:
if (switchAndWaitForCamera(1)) {
    // Camera 1 ready, proceed
    motionStub_InputTrayChamber();
} else {
    // Camera switch failed
    publishError("CAMERA_SWITCH_FAILED");
}
```

**Performs**:
1. Request switch
2. Wait for hardware confirmation (5s timeout)
3. 500ms warm-up delay
4. Validate detection (3s timeout)

**Total time**: ~2.5s typical, ~8.5s worst case

---

#### `bool switchAndWaitForCameraWithRetry(int camera_id, int max_retries = 3)`

Camera switch with retry logic

```cpp
// Usage:
if (switchAndWaitForCameraWithRetry(1, 3)) {
    // Success (any attempt)
    RCLCPP_INFO(get_logger(), "Camera ready");
} else {
    // All 3 attempts failed
    publishError("CAMERA_SWITCH_FAILED_AFTER_RETRIES");
    system_enabled_ = false;
    transitionTo(SystemState::IDLE);
}
```

**Parameters**:
- `camera_id`: 1 or 2
- `max_retries`: Number of attempts (default: 3)

**Returns**: `true` if any attempt succeeds

**Total time**: Up to ~28s (3 attempts × 8.5s + 2 delays × 2s)

---

## 📊 Performance Metrics

### Timing Benchmarks

| Operation | Typical | Worst Case |
|-----------|---------|------------|
| I2C Command | 20ms | 50ms |
| Hardware Switch | 1.5s | 2.0s |
| Stream Restart | 100ms | 500ms |
| Frame Stabilization | 500ms | 500ms |
| Detection Validation | 300ms | 3s |
| **Single Switch** | **2.4s** | **6s** |
| **With 3 Retries** | **2.4s** | **28s** |

### Success Rate Tracking

Monitor switch success rate:

```bash
# Count successes
ros2 topic echo /camera/status | grep -c "CAMERA.*READY"

# Count failures
ros2 topic echo /robot/error | grep -c "CAMERA_SWITCH_FAILED"

# Calculate rate
# Success Rate = Successes / (Successes + Failures) × 100%
```

---

## 🎯 Best Practices

### 1. Always Use Retry Wrapper

❌ **Bad**:
```cpp
if (!switchAndWaitForCamera(1)) {
    // Single attempt, no retry
    publishError("FAILED");
}
```

✅ **Good**:
```cpp
if (!switchAndWaitForCameraWithRetry(1, 3)) {
    // 3 attempts, more robust
    publishError("FAILED_AFTER_RETRIES");
    system_enabled_ = false;
    transitionTo(SystemState::IDLE);
}
```

---

### 2. Check Camera Before Motion

❌ **Bad**:
```cpp
void stateRefillBuffer() {
    // Start motion immediately (wrong camera!)
    motionStub_InputTrayBuffer();
}
```

✅ **Good**:
```cpp
void stateRefillBuffer() {
    // Ensure Camera 1 active first
    if (!switchAndWaitForCameraWithRetry(1, 3)) {
        handleError();
        return;
    }
    // Now safe to proceed
    motionStub_InputTrayBuffer();
}
```

---

### 3. Handle Empty Scenes Gracefully

❌ **Bad**:
```cpp
if (!waitForFirstDetection(1, 3.0)) {
    // Treat as error
    publishError("NO_DETECTION");
    return;
}
```

✅ **Good**:
```cpp
if (!waitForFirstDetection(1, 3.0)) {
    // Just log warning (empty scene is valid)
    RCLCPP_WARN(get_logger(), "No detection, continuing...");
}
// Continue regardless
```

---

### 4. Log Camera State Changes

✅ **Always log**:
```cpp
RCLCPP_INFO(get_logger(), 
    "[STATE] %s → %s (Camera %d required)",
    stateToString(current_state_).c_str(),
    stateToString(next_state).c_str(),
    required_camera_id);
```

---

### 5. Monitor Camera Status in HMI

Integrate `/camera/status` into your HMI:

```python
# Example HMI subscriber
def camera_status_callback(msg):
    status = msg.data
    if "SWITCHING" in status:
        display_status("Switching camera...", color="yellow")
    elif "READY" in status:
        display_status("Camera ready", color="green")
    elif "FAILED" in status:
        display_status("Camera error!", color="red")
        sound_alarm()
```

---

## 🚀 Deployment Checklist

Before deploying to production:

- [ ] All 6 test scenarios pass
- [ ] Camera switch time < 3s in 95% of cases
- [ ] Success rate > 99%
- [ ] No memory leaks after 1000 switches
- [ ] Error recovery tested and documented
- [ ] HMI displays camera status
- [ ] Operators trained on manual recovery
- [ ] Backup camera available if one fails
- [ ] Log rotation configured
- [ ] Monitoring alerts configured

---

## 📝 Change Log

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-01-XX | Initial implementation with retry logic |
| 1.1 | TBD | Add performance metrics logging |
| 1.2 | TBD | Optimize warm-up delay timing |

---

## 📧 Support

For issues or questions:
1. Check this documentation
2. Review logs: `ros2 topic echo /robot/error`
3. Test manually: `ros2 topic pub /robot/command_camera ...`
4. Check hardware: `i2cdetect -y 4`
5. Contact: [Your Support Contact]

---

**End of Documentation**