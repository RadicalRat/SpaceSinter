cmake_minimum_required(VERSION 3.22)
project(sinter_robot_hw_testing)

find_package(ament_cmake REQUIRED)

# Install configuration files
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
