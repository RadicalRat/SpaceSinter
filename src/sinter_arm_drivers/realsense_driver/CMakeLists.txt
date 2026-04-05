cmake_minimum_required(VERSION 3.8)
project(realsense_driver)

# Default settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# ------------------------------------------------------
# Optional: If you add custom hardware plugins later
# ------------------------------------------------------
# add_library(example_robot_hardware SHARED
#   src/example_robot_hardware.cpp
# )
#
# target_include_directories(example_robot_hardware PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
#
# ament_target_dependencies(example_robot_hardware
#   rclcpp
#   hardware_interface
#   pluginlib
# )
#
# pluginlib_export_plugin_description_file(hardware_interface example_robot_hardware.xml)

# ------------------------------------------------------
# Installation
# ------------------------------------------------------


# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install controller config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# ------------------------------------------------------
# Export
# ------------------------------------------------------

ament_package()
