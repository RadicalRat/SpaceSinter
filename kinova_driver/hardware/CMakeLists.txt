cmake_minimum_required(VERSION 3.28)
project(EXAMPLE)


#EXE for ang get
add_executable( get_ang
  src/kinova_get_ang.cpp
)

target_include_directories(get_ang PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(get_ang PRIVATE cxx_std_11)

# dlopen/dlsym needs libdl
target_link_libraries(get_ang PRIVATE dl)



#EXE for ang control
add_executable( ang_control
  src/kinova_ang_control.cpp
)

target_include_directories(ang_control PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(ang_control PRIVATE cxx_std_11)

# dlopen/dlsym needs libdl
target_link_libraries(ang_control PRIVATE dl)