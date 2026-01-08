# Output Tray Motion Control - Hướng Dẫn Sử Dụng

## 📋 Tổng Quan

Hàm `motionStub_ScaleOutputTray()` đã được tối ưu hóa để xử lý 8 vị trí (slots) trên khay output. Bạn có thể điều khiển robot bằng 2 cách:

1. **Auto Mode**: Camera node detect vị trí trống, gửi slot index qua topic
2. **Manual Mode**: Gửi lệnh từ terminal để kiểm tra

---

## 🎯 Topics Subscription

### 1. **Automatic Mode - Camera Detection**
```
Topic: /camera/ai/selected_slot
Type: std_msgs/Int32
Range: 1-8 (Slot ID)
```

**Mô tả**: Khi camera node của bạn detect được slot trống, publish slot index qua topic này.

**Ví dụ code (Python)**:
```python
import rclpy
from std_msgs.msg import Int32

def publish_selected_slot(slot_id):
    """slot_id: 1-8"""
    publisher = node.create_publisher(Int32, '/camera/ai/selected_slot', 10)
    msg = Int32(data=slot_id)
    publisher.publish(msg)
```

**Flow tự động**:
```
Cartridge trên scale → Camera detect slot trống → Publish /camera/ai/selected_slot 
→ Robot tự động move tới slot & đặt cartridge
```

---

### 2. **Manual Mode - Terminal Testing**
```
Topic: /robot/command_slot
Type: std_msgs/Int32
Range: 1-8 (Slot ID)
```

**Mô tả**: Gửi lệnh từ terminal để test từng slot (chế độ Manual).

**Command từ Terminal**:
```bash
# Test Slot 1
ros2 topic pub /robot/command_slot std_msgs/Int32 "data: 1"

# Test Slot 5
ros2 topic pub /robot/command_slot std_msgs/Int32 "data: 5"

# Test Slot 8
ros2 topic pub /robot/command_slot std_msgs/Int32 "data: 8"
```

**Flow Manual**:
```
Terminal: ros2 topic pub /robot/command_slot ... "data: 3"
→ Robot nhận lệnh → Manual Mode ON → Move tới Slot 3 & đặt cartridge
```

---

## 🏗️ Kiến Trúc Motion Sequence

Hàm `motionStub_ScaleOutputTray()` thực hiện các bước:

### **Step-by-Step Motion**
```
1. Move từ scale → Wait position (Index 11)
   └─ moveToIndex(11)

2. Move down để pick cartridge từ scale
   └─ moveR(0, 0, -30)  [Down 30mm]

3. Close gripper
   └─ setDigitalOutput(8, true)

4. Move up từ scale
   └─ moveR(0, 0, 30)   [Up 30mm]

5. Move tới target slot position
   └─ moveToIndex(12 + slot_id)  [Index 13-20 cho Slot 1-8]

6. Move down để place cartridge
   └─ moveR(0, 0, -30)  [Down 30mm]

7. Release gripper
   └─ setDigitalOutput(8, false)

8. Move up từ slot
   └─ moveR(0, 0, 30)   [Up 30mm]
```

---

## 📊 YAML Motion Sequence Mapping

Từ `joint_pose_params.yaml`:

```yaml
INDEX  |  Description
-------|-------------------------------------
11     |  Scale to Output Wait Position
12     |  Wait Row Index (before placing)
13-20  |  Output Tray Slot Positions
       |  13 = Slot 1
       |  14 = Slot 2
       |  15 = Slot 3
       |  16 = Slot 4
       |  17 = Slot 5
       |  18 = Slot 6
       |  19 = Slot 7
       |  20 = Slot 8
```

---

## ✅ Sử Dụng trong State Machine

Hàm được gọi tại state `PLACE_TO_OUTPUT`:

```cpp
void statePlaceToOutput()
{
    // ... validation code ...
    
    // Get selected slot
    int slot = selected_output_slot_;  // 1-8
    
    // Execute motion
    motionStub_ScaleOutputTray(slot);
    
    // Update state
    scale_has_cartridge_ = false;
    selected_output_slot_ = -1;
}
```

---

## 🔄 Flow So Sánh: Input Tray vs Output Tray

### **Input Tray (Rows)**
```
Topic: /robot/command_row
Type: std_msgs/Int32
Range: 1-5 (Row ID)

Motion: Input Tray Row → Chamber
Indexes: 1-5 từ YAML
```

### **Output Tray (Slots)** ✨ NEW
```
Topic: /robot/command_slot  (Manual)
Topic: /camera/ai/selected_slot  (Auto)
Type: std_msgs/Int32
Range: 1-8 (Slot ID)

Motion: Scale → Output Tray Slot
Indexes: 13-20 từ YAML
```

---

## 🧪 Test Sequence

### **Test 1: Manual Mode**
```bash
# Terminal 1: Launch node
ros2 run robot_control_main robot_logic_node

# Terminal 2: Test Slot 1
ros2 topic pub /robot/command_slot std_msgs/Int32 "data: 1"

# Terminal 3: Monitor (optional)
ros2 topic echo /robot/system_status
```

### **Test 2: Auto Mode (Camera)**
```bash
# Python script để simulate camera detection
import rclpy
from std_msgs.msg import Int32

node = rclpy.create_node('camera_simulator')
pub = node.create_publisher(Int32, '/camera/ai/selected_slot', 10)

# Simulate detection của slot 3
msg = Int32(data=3)
pub.publish(msg)
```

---

## 🛠️ Mutex Protection

Cả `selected_output_slot_` đều được bảo vệ bằng mutex:

```cpp
std::mutex output_slot_selection_mutex_;

// Getter
{
    std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
    int slot = selected_output_slot_;
}

// Setter
{
    std::lock_guard<std::mutex> lock(output_slot_selection_mutex_);
    selected_output_slot_ = slot_id;
}
```

---

## 📝 Lưu Ý Quan Trọng

✅ **Đã làm**:
- [x] Tối ưu `motionStub_ScaleOutputTray()` với 8 slots
- [x] Thêm callback `selectedSlotCallback()` cho auto mode
- [x] Thêm callback `commandSlotCallback()` cho manual mode
- [x] Thêm 2 topics subscription
- [x] Mutex protection cho slot selection
- [x] Mapping Index 13-20 từ YAML

⚠️ **Cần kiểm tra**:
1. **Kiểm tra YAML**: Đảm bảo index 13-20 có định nghĩa cho 8 slot positions
2. **Kiểm tra DO[8]**: Đó có phải gripper không?
3. **Kiểm tra camera node**: Tên topic `/camera/ai/selected_slot` có khớp không?

---

## 💡 Mở Rộng

Nếu bạn muốn thêm feedback sensor:
```cpp
// Thêm subscription để nhận feedback từ camera
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr output_tray_full_sub_;

// Callback
void outputTrayFullCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // Xử lý khi khay full
    if (msg->data) {
        // Change tray
    }
}
```

---

**🚀 Sẵn sàng test!** Build project và test từ terminal.
