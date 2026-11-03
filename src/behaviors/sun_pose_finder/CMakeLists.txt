cmake_minimum_required(VERSION 3.22)
project(sun_pose_finder CXX)

find_package(moveit_pro_package REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

moveit_pro_package()


set(THIS_PACKAGE_INCLUDE_DEPENDS moveit_pro_behavior_interface pluginlib cv_bridge sensor_msgs geometry_msgs nav_msgs std_msgs tf2_eigen tf2_ros)
foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(
  sun_pose_finder
  SHARED
  src/sun_pose_finder.cpp
  src/compute_sun_ephemeris.cpp
  src/fit_plane_raster_path.cpp
  src/register_behaviors.cpp)

target_include_directories(
  sun_pose_finder
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
         ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(sun_pose_finder
  cv_bridge
)

ament_target_dependencies(sun_pose_finder
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS sun_pose_finder
  EXPORT sun_pose_finderTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

target_link_libraries(sun_pose_finder
  ${OpenCV_LIBS}
  ${cv_bridge_LIBS})

# Standalone sun ephemeris publisher, launched with the drivers so it runs whether or not an objective does.
add_executable(sun_ephemeris_node src/sun_ephemeris_node.cpp)
target_include_directories(sun_ephemeris_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(sun_ephemeris_node rclcpp geometry_msgs std_msgs tf2_eigen tf2_ros)
install(TARGETS sun_ephemeris_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  moveit_pro_behavior_test(sun_pose_finder)
endif()

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_pro_behavior_interface package.
pluginlib_export_plugin_description_file(
  moveit_pro_behavior_interface sun_pose_finder_plugin_description.xml)

ament_export_targets(sun_pose_finderTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()