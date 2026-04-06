#!/bin/bash
# Script to convert robot_logic_node.cpp to dual camera mode
# This modifies the copied file robot_logic_node_dual.cpp

FILE="/home/pi/ros2_ws/src/robot_control_main/src/robot_logic_node_dual.cpp"

echo "Converting to dual camera mode..."

# 1. Change node name
sed -i 's/robot_logic_nova5/robot_logic_nova5_dual/g' "$FILE"

# 2. Remove camera_select publisher initialization (line ~282)
sed -i '/camera_select_pub_ = this->create_publisher<std_msgs::msg::Int32>.*camera_select/d' "$FILE"

# 3. Remove camera active ID subscription (line ~277-279)
sed -i '/camera_active_id_sub_ = this->create_subscription<std_msgs::msg::Int32>/,/cameraActiveIdCallback/d' "$FILE"

# 4. Comment out all switchAndWaitForCamera calls
sed -i 's/if (!switchAndWaitForCameraWithRetry/\/\/ DUAL MODE: No camera switching needed\n        \/\/ if (!switchAndWaitForCameraWithRetry/g' "$FILE"
sed -i 's/requestCameraSwitch(/\/\/ requestCameraSwitch(/g' "$FILE"

# 5. Update topic names to match dual camera launch
sed -i 's|/cam0Funai/yolo/bounding_boxes|/cam0/detections|g' "$FILE"
sed -i 's|/cam1Funai/yolo/bounding_boxes|/cam1/detections|g' "$FILE"

# 6. Add comment about dual mode at top
sed -i '1i // ================================================================' "$FILE"
sed -i '2i // DUAL CAMERA MODE - Processes both cameras simultaneously' "$FILE"
sed -i '3i // Original file: robot_logic_node.cpp (switch-based backup)' "$FILE"
sed -i '4i // ================================================================' "$FILE"

echo "✅ Conversion complete: $FILE"
echo "Summary of changes:"
echo "  - Node name: robot_logic_nova5_dual"
echo "  - Removed camera switching logic"
echo "  - Updated topics: /cam0/detections, /cam1/detections"
echo "  - Both cameras process simultaneously"
