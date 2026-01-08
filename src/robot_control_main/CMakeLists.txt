cmake_minimum_required(VERSION 3.8)
project(robot_control_main)

# 1. TÌM GÓI (Thêm vision_msgs và geometry_msgs)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(dobot_msgs_v3 REQUIRED)
find_package(vision_msgs REQUIRED)    # <--- QUAN TRỌNG
find_package(geometry_msgs REQUIRED)  # <--- QUAN TRỌNG
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(robot_logic_node src/robot_logic_node.cpp)

target_include_directories(robot_logic_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# 2. LIÊN KẾT THƯ VIỆN (Thêm vào ament_target_dependencies)
ament_target_dependencies(robot_logic_node
  rclcpp
  std_msgs
  std_srvs
  dobot_msgs_v3
  vision_msgs    # <--- QUAN TRỌNG: Phải có ở đây
  geometry_msgs  # <--- QUAN TRỌNG
  cv_bridge
  OpenCV
)

# Link trực tiếp thư viện OpenCV (nếu cần)
target_link_libraries(robot_logic_node ${OpenCV_LIBS})

install(TARGETS
  robot_logic_node
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Copy to install directory
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/gripper_festo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()