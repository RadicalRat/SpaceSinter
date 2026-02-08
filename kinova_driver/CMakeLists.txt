cmake_minimum_required(VERSION 3.14)
project(kinova_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-D_OS_UNIX=1)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# detect architecture-specific lib dir inside package repo
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(_arch_dir "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(_arch_dir "x86_64")
else()
  set(_arch_dir "${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(KINOVA_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/${_arch_dir}")

# collect absolute paths to vendor .so files (if present)
set(KINOVA_SO_FILES "")
if(EXISTS "${KINOVA_LIB_DIR}")
  file(GLOB KINOVA_SO_GLOB "${KINOVA_LIB_DIR}/*.so*")
  foreach(_f IN LISTS KINOVA_SO_GLOB)
    get_filename_component(_abs "${_f}" REALPATH)
    list(APPEND KINOVA_SO_FILES "${_abs}")
  endforeach()
endif()
message(STATUS "Kinova vendor libs: ${KINOVA_SO_FILES}")

# Build driver
add_library(${PROJECT_NAME} SHARED src/kinova_arm.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)

# Create IMPORTED targets for vendor .so files and link them (absolute paths)
set(KINOVA_IMPORTED_LIBS "")
if(KINOVA_SO_FILES)
  foreach(_lib IN LISTS KINOVA_SO_FILES)
    get_filename_component(_lib_name "${_lib}" NAME_WE)
    string(REPLACE " " "_" _lib_name_safe "${_lib_name}")
    set(_imp_target "kinova_vendor_${_lib_name_safe}")
    if(NOT TARGET ${_imp_target})
      add_library(${_imp_target} SHARED IMPORTED)
      set_target_properties(${_imp_target} PROPERTIES IMPORTED_LOCATION "${_lib}")
    endif()
    list(APPEND KINOVA_IMPORTED_LIBS ${_imp_target})
  endforeach()
endif()

# link the imported targets and pthread (plain signature)
if(KINOVA_IMPORTED_LIBS)
  target_link_libraries(${PROJECT_NAME} ${KINOVA_IMPORTED_LIBS})
endif()
target_link_libraries(${PROJECT_NAME} pthread)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)

# ensure installed driver finds co-installed vendor libs
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH ON
)

pluginlib_export_plugin_description_file(hardware_interface kinova_driver.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(FILES kinova_driver.xml DESTINATION share/${PROJECT_NAME})

if(KINOVA_SO_FILES)
  install(FILES ${KINOVA_SO_FILES} DESTINATION lib OPTIONAL)
endif()

ament_package()
