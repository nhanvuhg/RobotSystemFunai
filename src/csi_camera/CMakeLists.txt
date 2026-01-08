cmake_minimum_required(VERSION 3.8)
project(csi_camera)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)        # ✅ ADDED - Required for Int32
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Create executable
add_executable(csi_camera_node src/csi_camera_node.cpp)

# Include directories
target_include_directories(csi_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# Link dependencies
ament_target_dependencies(csi_camera_node
  rclcpp
  sensor_msgs
  std_msgs          # ✅ ADDED - Required for Int32
  cv_bridge
)

target_link_libraries(csi_camera_node 
  ${OpenCV_LIBS}
)

# Install
install(TARGETS csi_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (optional)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py"
)

# Install config files (optional)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.yaml"
)

ament_package()