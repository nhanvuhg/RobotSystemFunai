cmake_minimum_required(VERSION 3.5)
project(robot_control_main)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dobot_msgs_v3 REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(plc_msg REQUIRED)
find_package(snap7_driver REQUIRED)
find_package(OpenCV REQUIRED)

# Original switch-based robot logic (BACKUP)
add_executable(robot_logic_node src/robot_logic_node.cpp)
ament_target_dependencies(robot_logic_node rclcpp std_msgs dobot_msgs_v3 vision_msgs plc_msg snap7_driver OpenCV)

# NEW: Dual parallel camera mode
add_executable(robot_logic_node_dual src/robot_logic_node_dual.cpp)
ament_target_dependencies(robot_logic_node_dual rclcpp std_msgs dobot_msgs_v3 vision_msgs plc_msg snap7_driver OpenCV)

install(TARGETS
  robot_logic_node           # Original switch-based (backup)
  robot_logic_node_dual      # New dual parallel mode
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
