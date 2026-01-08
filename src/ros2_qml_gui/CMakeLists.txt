cmake_minimum_required(VERSION 3.5)
project(ros2_qml_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Qml Quick Charts QuickControls2 Widgets)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

set(HEADER_FILES
  include/ros2_qml_gui/cam_node.hpp
  include/ros2_qml_gui/cam_provider.hpp
)

set(SOURCES
  src/main.cpp
  src/cam_node.cpp
  src/cam_provider.cpp
)

qt5_add_resources(QML_RESOURCES qml.qrc)

add_executable(${PROJECT_NAME}
  ${SOURCES}
  ${HEADER_FILES}
  ${QML_RESOURCES}
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::Qml
  Qt5::Quick
  Qt5::Charts
  Qt5::QuickControls2
  Qt5::Widgets
  ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
