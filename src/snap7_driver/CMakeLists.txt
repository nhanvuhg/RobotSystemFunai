cmake_minimum_required(VERSION 3.8)
project(snap7_driver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(plc_msg REQUIRED)

set(SNAP7_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/snap7)
set(SNAP7_SYS_DIR ${SNAP7_BASE_DIR}/src/sys)
set(SNAP7_CORE_DIR ${SNAP7_BASE_DIR}/src/core)
set(SNAP7_LIB_DIR ${SNAP7_BASE_DIR}/src/lib)

include_directories(
  include
  ${SNAP7_SYS_DIR}
  ${SNAP7_CORE_DIR}
  ${SNAP7_LIB_DIR}
)

set(SNAP7_SOURCES
  ${SNAP7_SYS_DIR}/snap_msgsock.cpp
  ${SNAP7_SYS_DIR}/snap_sysutils.cpp
  ${SNAP7_SYS_DIR}/snap_tcpsrvr.cpp
  ${SNAP7_SYS_DIR}/snap_threads.cpp
  ${SNAP7_CORE_DIR}/s7_client.cpp
  ${SNAP7_CORE_DIR}/s7_isotcp.cpp
  ${SNAP7_CORE_DIR}/s7_partner.cpp
  ${SNAP7_CORE_DIR}/s7_peer.cpp
  ${SNAP7_CORE_DIR}/s7_server.cpp
  ${SNAP7_CORE_DIR}/s7_text.cpp
  ${SNAP7_CORE_DIR}/s7_micro_client.cpp
  ${SNAP7_LIB_DIR}/snap7_libmain.cpp
)

add_library(snap7 SHARED ${SNAP7_SOURCES})
target_link_libraries(snap7 pthread rt)

add_executable(snap7_node
  src/snap7_node.cpp
  src/snap7.cpp
)
ament_target_dependencies(snap7_node
  rclcpp
  std_msgs
  plc_msg
)
target_link_libraries(snap7_node snap7)

install(TARGETS snap7_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS snap7
  LIBRARY DESTINATION lib
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
