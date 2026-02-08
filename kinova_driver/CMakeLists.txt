cmake_minimum_required(VERSION 3.8)
project(kinova_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN LISTS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Minimal and explicit build/install for kinova_driver
add_library(kinova_driver SHARED src/kinova_arm.cpp)

target_compile_features(kinova_driver PUBLIC cxx_std_17)

# Use the package's include/ directory so headers like
# #include "kinova_ros/kinova_arm.hpp" resolve during build.
target_include_directories(kinova_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(kinova_driver
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export plugin description for pluginlib
pluginlib_export_plugin_description_file(hardware_interface kinova_driver.xml)

# Install plugin descriptor so pluginlib can find your SystemInterface at runtime
install(FILES kinova_driver.xml DESTINATION share/${PROJECT_NAME})

# Install headers and library
install(DIRECTORY include/ DESTINATION include)


install(TARGETS kinova_driver
  EXPORT export_kinova_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()