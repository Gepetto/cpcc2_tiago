cmake_minimum_required(VERSION 3.5.0)
project(cpcc2_tiago)

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
  rosbag2_cpp
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(crocoddyl REQUIRED)



generate_parameter_library(
  cpcc2_tiago_parameters
  config/cpcc2_tiago_parameters.yaml
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

## python scripts
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/performance_evaluator.py
  scripts/gui_target_sender.py
  scripts/performance_plotter.py
  scripts/performance_full_plotter.py
  DESTINATION lib/${PROJECT_NAME}
)

## crocoddyl_controller
add_library(
  crocoddyl_controller
  SHARED
  src/crocoddyl_controller.cpp
  src/tiago_OCP.cpp
  src/tiago_OCP_costs.cpp
  src/model_builder.cpp

)

target_link_libraries(crocoddyl_controller crocoddyl::crocoddyl)

target_include_directories(
  crocoddyl_controller
  PRIVATE
  include
)
ament_target_dependencies(
  crocoddyl_controller

  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

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

## parallel_croc_solver

add_executable(parallel_croc_solver src/parallel_croc_solver.cpp)

target_include_directories(
  parallel_croc_solver
  PRIVATE
  include
)

install(
  TARGETS
  parallel_croc_solver
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(parallel_croc_solver crocoddyl_controller)

## pveg_chained_controller
add_library(
  pveg_chained_controller
  SHARED
  src/pveg_chained_controller.cpp
  src/model_builder.cpp
)
target_include_directories(
  pveg_chained_controller
  PRIVATE
  include
)
ament_target_dependencies(
  pveg_chained_controller
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(pveg_chained_controller crocoddyl::crocoddyl)

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
