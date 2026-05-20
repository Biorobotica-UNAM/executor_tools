cmake_minimum_required(VERSION 3.8)
project(executor_tools)

# -----------------------------
# Dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(executor_interfaces REQUIRED)

find_package(hri_tools REQUIRED)
find_package(navigation_tools REQUIRED)
find_package(hardware_tools REQUIRED)
find_package(vision_tools REQUIRED)

# -----------------------------
# Library
# -----------------------------
add_library(${PROJECT_NAME}
  src/Say.cpp
  src/GoTo.cpp
  src/Check.cpp
  src/Point.cpp
  src/Align.cpp
  src/Give.cpp
  src/FindPerson.cpp
  src/FindObject.cpp
  src/Focus.cpp
  src/Receive.cpp
  src/Follow.cpp
  src/Take.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  geometry_msgs
  ament_index_cpp
  executor_interfaces
  hri_tools
  navigation_tools
  hardware_tools
  vision_tools
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

# -----------------------------
# Install
# -----------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# -----------------------------
# Export
# -----------------------------
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rclcpp_action
  ament_index_cpp
  executor_interfaces
  hri_tools
  navigation_tools
  hardware_tools
  vision_tools
)

ament_package()
