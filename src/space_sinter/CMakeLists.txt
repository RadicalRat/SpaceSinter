cmake_minimum_required(VERSION 3.16.3)
project(space_sinter)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    objectives
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# Add the library
add_library(space_sinter_lib SHARED src/get_solar_pointing.cpp)

# Link dependencies (MoveIt Pro, CSPICE, Eigen, etc.)
target_link_libraries(space_sinter_lib
  PRIVATE
    moveit_studio_behavior_interface::moveit_studio_behavior_interface
    cspice
    Eigen3::Eigen
)

# Crucial: This makes the plugin discoverable
pluginlib_export_plugin_description_file(moveit_studio_behavior_interface behaviors.xml)

install(TARGETS space_sinter_lib
  DESTINATION lib/${PROJECT_NAME})