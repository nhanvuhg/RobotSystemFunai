cmake_minimum_required(VERSION 3.8)
project(plc_msg)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PLCRead.msg"
  "msg/PLCWrite.msg"
  "msg/PLCResponse.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
