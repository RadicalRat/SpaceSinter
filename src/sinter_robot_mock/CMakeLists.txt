cmake_minimum_required(VERSION 3.22)
project(sinter_robot_mock)

find_package(ament_cmake REQUIRED)

# Install configuration files
install(DIRECTORY config launch objectives urdf waypoints DESTINATION share/${PROJECT_NAME}/)

# Simulation nodes
install(PROGRAMS scripts/sim_sun_camera.py DESTINATION lib/${PROJECT_NAME})

ament_package()
