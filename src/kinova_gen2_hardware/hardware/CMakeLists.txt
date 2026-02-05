cmake_minimum_required(VERSION 3.28)
project(EXAMPLE)

# ...existing code...
add_executable( get_ang
  src/kinova_get_ang.cpp
)

target_include_directories(get_ang PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(get_ang PRIVATE cxx_std_11)

# dlopen/dlsym needs libdl
target_link_libraries(get_ang PRIVATE dl)

# ...existing code...