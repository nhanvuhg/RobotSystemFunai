cmake_minimum_required(VERSION 3.8)
project(yolo_ros_hailort_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Tìm các gói phụ thuộc
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Threads REQUIRED)
find_package(HailoRT REQUIRED)     # Thư viện AI Hailo
find_package(OpenCV REQUIRED)      # Thư viện xử lý ảnh
find_package(cv_bridge REQUIRED)   # Cầu nối ROS-OpenCV
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# 2. Tạo thư viện Shared (Node Component)
ament_auto_add_library(yolo_ros_hailort_cpp SHARED
  src/yolo_ros_hailort_cpp.cpp
)

# 3. Liên kết các gói ROS (Ament tự động xử lý include path)
ament_target_dependencies(yolo_ros_hailort_cpp
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  vision_msgs
  OpenCV
)

# 4. Liên kết các thư viện ngoài ROS (HailoRT)
# LƯU Ý: Phải link vào 'yolo_ros_hailort_cpp', KHÔNG PHẢI 'csi_camera_node'
target_link_libraries(yolo_ros_hailort_cpp
  HailoRT::libhailort
  Threads::Threads
  ${OpenCV_LIBS}
)

# 5. Đăng ký Node Component (Để chạy bằng ros2 run)
rclcpp_components_register_node(
  yolo_ros_hailort_cpp
  PLUGIN "yolo_ros_hailort_cpp::YoloNode"
  EXECUTABLE yolo_ros_hailort_cpp_node
)

# 6. Cài đặt và đóng gói
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)