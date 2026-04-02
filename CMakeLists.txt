cmake_minimum_required(VERSION 3.8)
project(executor_tools)

# -----------------------------
# Compiler options
# -----------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Find dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(executor_interfaces REQUIRED)
find_package(supervisor_executor_interfaces REQUIRED)
find_package(talker_interfaces REQUIRED)
find_package(listener_interfaces REQUIRED)
find_package(known_locations_interfaces REQUIRED)
find_package(hri_tools REQUIRED)
find_package(navigation_tools REQUIRED)
find_package(hardware_tools REQUIRED)

# -----------------------------
# Library
# -----------------------------
add_library(${PROJECT_NAME}
  src/Executor.cpp
  src/Say.cpp
  src/Listen.cpp
  src/GoTo.cpp
  src/Check.cpp
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
  std_srvs
  ament_index_cpp
  executor_interfaces
  supervisor_executor_interfaces
  talker_interfaces
  listener_interfaces
  known_locations_interfaces
  hri_tools
  navigation_tools
  hardware_tools
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
  std_srvs
  executor_interfaces
  supervisor_executor_interfaces
  talker_interfaces
  listener_interfaces
  known_locations_interfaces
  hri_tools
  navigation_tools
  hardware_tools
  ament_index_cpp
)

ament_package()

