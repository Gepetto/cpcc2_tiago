cmake_minimum_required(VERSION 3.5.0)
project(cpcc2_tiago)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  ros2_control
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(Boost REQUIRED)
find_package(crocoddyl REQUIRED)
find_package(pinocchio REQUIRED)
find_package(eigenpy REQUIRED)
include_directories(${crocoddyl_INCLUDE_DIRS})
include_directories(${pinocchio_INCLUDE_DIRS})


generate_parameter_library(
  cpcc2_tiago_parameters
  config/cpcc2_tiago_parameters.yaml
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)


## crocoddyl_controller
add_library(
  crocoddyl_controller
  SHARED
  src/crocoddyl_controller.cpp
  src/tiago_OCP_maker.cpp
  src/model_builder.cpp
  src/logger_OCP.cpp

)
target_include_directories(
  crocoddyl_controller
  PRIVATE
  include
)
ament_target_dependencies(
  crocoddyl_controller
  controller_interface
  hardware_interface
  realtime_tools
  pluginlib
  rclcpp
  rclcpp_lifecycle
  spdlog
  pinocchio
  crocoddyl
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(crocoddyl_controller /opt/openrobots/lib/libpinocchio.so)


pluginlib_export_plugin_description_file(controller_interface crocoddyl_controller.xml)

install(
  TARGETS
  crocoddyl_controller
  DESTINATION lib
)
install(
  DIRECTORY
    include
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  crocoddyl_controller
)

## pveg_chained_controller
add_library(
  pveg_chained_controller
  SHARED
  src/pveg_chained_controller.cpp
)
target_include_directories(
  pveg_chained_controller
  PRIVATE
  include
)
ament_target_dependencies(
  pveg_chained_controller
  controller_interface
  hardware_interface
  realtime_tools
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(controller_interface pveg_chained_controller.xml)

install(
  TARGETS
  pveg_chained_controller
  DESTINATION lib
)
install(
  DIRECTORY
    include
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  pveg_chained_controller
)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  interfaces
  lib
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(pveg_chained_controller
  cpcc2_tiago_parameters
)

target_link_libraries(crocoddyl_controller
  cpcc2_tiago_parameters
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
