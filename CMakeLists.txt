cmake_minimum_required(VERSION 3.8)
project(executor_tools)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(executor_interfaces REQUIRED)
find_package(supervisor_executor_interfaces REQUIRED)
find_package(hri_tools REQUIRED)

add_library(${PROJECT_NAME}
  src/Executor.cpp
  src/Say.cpp
  src/Listen.cpp
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
  executor_interfaces
  supervisor_executor_interfaces
  hri_tools
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_srvs
  executor_interfaces
  supervisor_executor_interfaces
  hri_tools
)

ament_package()

