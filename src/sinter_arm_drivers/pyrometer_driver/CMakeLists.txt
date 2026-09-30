cmake_minimum_required(VERSION 3.14)
project(pyrometer_driver)

# Require C++17 for structured bindings and other modern features.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ── Hardware plugin shared library ────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED src/pyrometer_driver.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface pyrometer_driver.xml)

# ── Broadcaster controller shared library ─────────────────────────────────────
add_library(pyrometer_broadcaster SHARED src/pyrometer_broadcaster.cpp)

target_include_directories(pyrometer_broadcaster
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(pyrometer_broadcaster PUBLIC cxx_std_17)

ament_target_dependencies(pyrometer_broadcaster
  controller_interface
  realtime_tools
  sensor_msgs
  std_msgs
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(controller_interface pyrometer_broadcaster.xml)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME} pyrometer_broadcaster
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(FILES pyrometer_driver.xml pyrometer_broadcaster.xml DESTINATION share/${PROJECT_NAME})

# Install udev rules to share so users can copy them to /etc/udev/rules.d/
install(FILES udev/99-pyrometer.rules DESTINATION share/${PROJECT_NAME}/udev)

# Install URDF/xacro, controller config, and launch files.
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
