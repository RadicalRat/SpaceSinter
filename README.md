# Table of Contents
- [Table of Contents](#table-of-contents)
- [Purpose](#purpose)
- [FileSystem](#filesystem)
- [Moveit! Pro](#moveit-pro)
- [ROS2 Hardware Drivers](#ros2-hardware-drivers)
- [How to use the SpaceSinter Stack](#how-to-use-the-spacesinter-stack)


# Purpose
This code stack was developed for the SolarSinter senior design team at the Colorado School of Mines. This project was the development of a road building system to concentrate solar energy to "melt" lunar regolith into glassy roads as a possible avenue for lunar infrastructure. This system utilizes a Kinova Jaco Gen2 arm for the deployment and manuvering of the fresnel lens. Software was created to connect this arm to ROS2 MoveIt! Pro, in order to have precise control of the arm and planned behaviors.

# FileSystem
* ```kinova_driver``` : This directory contains the ROS2 hardware drivers for the kinova Gen2 Jaco arm to expose state and command interfaces. 


# Moveit! Pro


# ROS2 Hardware Drivers
* The command interfaces exposed are:
    * ```velocity```: Exposed control of angular velocity for joints 1-6, expressed in rad/s.
  * The state interfaces exposed are:
    * ```position```: Exposed angular position for joints 1-6, expressed in rad.
    * ```velocity```: Exposed angular velocity for joints 1-6, expressed in rad.
    * ```effort```: Exposed joint effort for joints 1-6, expressed in N*m.


# How to use the SpaceSinter Stack
