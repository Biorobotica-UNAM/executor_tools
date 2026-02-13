cmake_minimum_required(VERSION 3.8)
project(executor_tools)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(executor_interfaces REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/RequestAuthorization.cpp
  src/Say.cpp
  src/Listen.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  executor_interfaces
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
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

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_action executor_interfaces)

ament_package()

