cmake_minimum_required(VERSION 3.10)
project(nova5_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(dobot_msgs_v3 REQUIRED)

# Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Gui Qml Quick Widgets QuickControls2)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

include_directories(include)

add_executable(nova5_gui_node
  src/main.cpp
  src/nova5_client.cpp
  resource/qml.qrc
  include/nova5_gui/nova5_client.hpp
)

target_link_libraries(nova5_gui_node
  Qt5::Core Qt5::Gui Qt5::Qml Qt5::Quick Qt5::Widgets Qt5::QuickControls2
  yaml-cpp
)

ament_target_dependencies(nova5_gui_node rclcpp dobot_msgs_v3)

install(TARGETS nova5_gui_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY qml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
