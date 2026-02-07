# cmake_minimum_required(VERSION 3.8)
# project(kinova_driver)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# set(THIS_PACKAGE_INCLUDE_DEPENDS
#   hardware_interface
#   pluginlib
#   rclcpp
#   rclcpp_lifecycle
# )

# find_package(ament_cmake REQUIRED)
# foreach(Dependency IN LISTS THIS_PACKAGE_INCLUDE_DEPENDS)
#   find_package(${Dependency} REQUIRED)
# endforeach()

# add_library(
#   kinova_driver 
#   SHARED
#   src/kinova_arm.cpp
# )

# target_include_directories(kinova_driver PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# ament_target_dependencies(kinova_driver
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )

# install(TARGETS kinova_driver
#   EXPORT export_kinova_driver
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(DIRECTORY include
#   DESTINATION include
# )

# install(DIRECTORY
#   src
#   DESTINATION share/${PROJECT_NAME}/
# )

# ament_package()



cmake_minimum_required(VERSION 3.8)
project(kinova_driver LANGUAGES CXX)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Dependencies
find_package(ament_cmake REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

foreach(dep IN LISTS THIS_PACKAGE_INCLUDE_DEPENDS)
  find_package(${dep} REQUIRED)
endforeach()

# Library
add_library(kinova_driver SHARED
  src/kinova_arm.cpp
)

target_include_directories(kinova_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(kinova_driver
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# -- Kinova SDK linking (automatic selection of arch subdir) --
# Adjust names below if the actual .so basenames differ
set(KINOVA_USB_BASENAME "Kinova.API.USBCommandLayerUbuntu")
set(KINOVA_ETH_BASENAME "Kinova.API.EthCommandLayerUbuntu")

# prefer explicit arch directory under package (x86_64-linux-gnu, i386-linux-gnu, ...)
if(NOT DEFINED KINOVA_LIB_DIR)
  # try common cpu identifier, fallback to x86_64
  if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(_arch_dir "x86_64-linux-gnu")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i.86")
    set(_arch_dir "i386-linux-gnu")
  else()
    set(_arch_dir "${CMAKE_SYSTEM_PROCESSOR}-linux-gnu")
  endif()
  set(KINOVA_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/${_arch_dir}")
endif()

if(EXISTS "${KINOVA_LIB_DIR}")
  find_library(KINOVA_USB_LIB NAMES ${KINOVA_USB_BASENAME}
    PATHS ${KINOVA_LIB_DIR} NO_DEFAULT_PATH)
  find_library(KINOVA_ETH_LIB NAMES ${KINOVA_ETH_BASENAME}
    PATHS ${KINOVA_LIB_DIR} NO_DEFAULT_PATH)

  if(KINOVA_USB_LIB)
    message(STATUS "Found Kinova USB lib: ${KINOVA_USB_LIB}")
    target_link_libraries(kinova_driver PUBLIC ${KINOVA_USB_LIB})
  else()
    message(STATUS "Kinova USB lib not found in ${KINOVA_LIB_DIR}")
  endif()

  if(KINOVA_ETH_LIB)
    message(STATUS "Found Kinova ETH lib: ${KINOVA_ETH_LIB}")
    target_link_libraries(kinova_driver PUBLIC ${KINOVA_ETH_LIB})
  else()
    message(STATUS "Kinova ETH lib not found in ${KINOVA_LIB_DIR}")
  endif()

  # Install any .so files from that arch dir into the package lib so runtime finds them
  file(GLOB KINOVA_SO_FILES "${KINOVA_LIB_DIR}/*.so*")
  if(KINOVA_SO_FILES)
    install(FILES ${KINOVA_SO_FILES}
      DESTINATION lib
      OPTIONAL)
  endif()

  # Set RPATH so installed kinova_driver can find libs in the same install lib dir
  # $ORIGIN points to <install_prefix>/lib (when library lives in lib)
  set_target_properties(kinova_driver PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH ON)
else()
  message(STATUS "Kinova lib directory ${KINOVA_LIB_DIR} does not exist; skipping SDK linking")
endif()

# Installation
install(TARGETS kinova_driver
  EXPORT export_kinova_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Install plugin descriptor so pluginlib can find your SystemInterface
# (your repo already contains kinova_driver.xml at package root)
install(FILES kinova_driver.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install extra resources (examples, configs)
install(DIRECTORY kinova_ex
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()