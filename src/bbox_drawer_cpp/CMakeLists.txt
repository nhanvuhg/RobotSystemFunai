cmake_minimum_required(VERSION 3.8)
project(bbox_drawer_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(overlay_bboxes_node src/overlay_bboxes_node.cpp)
ament_target_dependencies(overlay_bboxes_node
  rclcpp sensor_msgs vision_msgs image_transport message_filters OpenCV
)

install(TARGETS overlay_bboxes_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()