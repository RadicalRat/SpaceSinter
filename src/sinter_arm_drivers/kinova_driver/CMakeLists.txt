cmake_minimum_required(VERSION 3.14)
project(kinova_driver)

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
find_package(rclcpp_lifecycle REQUIRED)

# ── Kinova vendor library directory ──────────────────────────────────────────
# Select the GNU multiarch triplet sub-directory that matches the build host.
set(KINOVA_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu")
if(NOT EXISTS "${KINOVA_LIB_DIR}/USBCommandLayerUbuntu.so")
  # Warn rather than fail so the rest of the workspace still builds on e.g.
  # aarch64 dev machines; the plugin then fails loudly in on_init().
  message(WARNING
    "[kinova_driver] No Kinova SDK libraries for ${CMAKE_SYSTEM_PROCESSOR} "
    "(expected ${KINOVA_LIB_DIR}). The hardware plugin will not be usable.")
endif()

# The driver dlopen()s these at runtime — never link against them, that would
# add spurious transitive deps.  They are installed next to the plugin, which
# is where the driver looks for them.
file(GLOB KINOVA_SO_FILES "${KINOVA_LIB_DIR}/*.so")

# ── Plugin shared library ─────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/kinova_sdk.cpp
  src/kinova_system_hardware.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# Required by the Kinova SDK headers.
target_compile_definitions(${PROJECT_NAME} PUBLIC _OS_UNIX=1)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} ${CMAKE_DL_LIBS} pthread)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface kinova_driver.xml)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES ${KINOVA_SO_FILES} DESTINATION lib)
install(FILES udev/10-kinova-arm.rules DESTINATION share/${PROJECT_NAME}/udev)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_joint_math test/test_joint_math.cpp)
  target_include_directories(test_joint_math PRIVATE include)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
