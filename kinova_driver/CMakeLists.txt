cmake_minimum_required(VERSION 3.8)
project(kinova_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN LISTS THIS_PACKAGE_INCLUDE_DEPENDS)
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  kinova_driver 
  SHARED
  src/kinova_arm.cpp
)

target_include_directories(kinova_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(kinova_driver
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS kinova_driver
  EXPORT export_kinova_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include
  DESTINATION include
)

install(DIRECTORY
  src
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()