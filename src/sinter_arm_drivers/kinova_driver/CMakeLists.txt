cmake_minimum_required(VERSION 3.14)
project(kinova_driver)

# Require C++17 for structured bindings and other modern features.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required by the Kinova SDK headers.
add_definitions(-D_OS_UNIX=1)

# ── ROS dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# ── Kinova vendor library directory ──────────────────────────────────────────
# Select the GNU multiarch triplet sub-directory that matches the build host.
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(_arch_primary "aarch64-linux-gnu")
  set(_arch_fallback "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(_arch_primary "x86_64-linux-gnu")
  set(_arch_fallback "x86_64")
else()
  set(_arch_primary "${CMAKE_SYSTEM_PROCESSOR}-linux-gnu")
  set(_arch_fallback "${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/${_arch_primary}")
  set(KINOVA_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/${_arch_primary}")
else()
  set(KINOVA_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/${_arch_fallback}")
endif()

# Collect all vendor .so files for installation.
# The driver uses dlopen() at runtime — no compile-time linking against these
# vendor libraries is needed or desired; that would add spurious transitive deps.
file(GLOB KINOVA_SO_FILES "${KINOVA_LIB_DIR}/*.so*")
message(STATUS "[kinova_driver] Vendor libs to install: ${KINOVA_SO_FILES}")

# ── Plugin shared library ─────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED src/kinova_arm.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Link only dl (dlopen/dlsym/dladdr) and pthread.
# The Kinova vendor .so files are opened at runtime — do NOT add them as
# link-time dependencies.
target_link_libraries(${PROJECT_NAME} dl pthread)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface kinova_driver.xml)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(FILES kinova_driver.xml DESTINATION share/${PROJECT_NAME})

# Install udev rules to share so users can copy them to /etc/udev/rules.d/
install(FILES udev/10-kinova-arm.rules DESTINATION share/${PROJECT_NAME}/udev)

# Install vendor .so files into the same lib directory as the plugin so that
# the dladdr-derived path used by on_init() to load them at runtime is correct.
if(KINOVA_SO_FILES)
  install(FILES ${KINOVA_SO_FILES} DESTINATION lib)
endif()

ament_package()

