cmake_minimum_required(VERSION 3.22)
project(sinter_robot_mock)

find_package(ament_cmake REQUIRED)

# Install configuration files
install(DIRECTORY config launch objectives urdf waypoints DESTINATION share/${PROJECT_NAME}/)

ament_package()
