- [Purpose](#purpose)
- [Requirements](#requirements)
- [File System](#file-system)
- [*Moveit! Pro*](#moveit-pro)
- [ROS2 Hardware Drivers](#ros2-hardware-drivers)
- [How to use the SpaceSinter Stack](#how-to-use-the-spacesinter-stack)


# Purpose
This code stack was developed for the SolarSinter senior design team at the Colorado School of Mines. This project was the development of a road building system to concentrate solar energy to "melt" lunar regolith into glassy roads as a possible avenue for lunar infrastructure. This system utilizes a Kinova Jaco Gen2 arm for the deployment and manuvering of the fresnel lens. Software was created to connect this arm to ROS2 *MoveIt! Pro*, in order to have precise control of the arm and planned behaviors.

# Requirements
This software stack runs on Ubuntu 24.04 with ROS2 Jazzy. Due to the degredation of the [kinova-ros](https://github.com/Kinovarobotics/kinova-ros) drivers, meant for ROS1, limited capabilities are available. Furthermore, this stack is not intended to work outside of the Ubuntu ecosystem, as several libraries used do not have native Windows counterparts.

MoveIt! Pro is required for pathplanning and behavior-tree management. Install directions can be found [here](https://docs.picknik.ai/software_installation/). This software stack handles the ROS2 ecosystem via docker container.

# File System
* ```kinova_driver``` : This directory contains the ROS2 hardware drivers for the kinova Gen2 Jaco arm. This directory also contains all relevant kinova API libraries used to control the arm.
* ```kinova_moveit```: This directory contains the moveit configuration packages for use with *MoveIt! Pro*.
  * ```config```: This directory contains all of the configuration yamls for drivers, and moveit algorithms. Also contains robot srdf files and the ros2_control srdf
  * ```description```: This directory contains all of the mesh files, robot urdfs, and mujuco configuration files. Also contains all of the world simulation files.
  * ```launch```: This directory contains all launch files for *MoveIt! Pro*.
  * ```objectives```: This directory contains any robot behaviors for *MoveIt! Pro*.
  * ```test```: This directory contains any python unit tests for this package.
  * ```waypoints```: This directory contains key waypoints for the SpaceSinter System.

# *Moveit! Pro*


# ROS2 Hardware Drivers
* The command interfaces exposed are:
    * ```velocity```: Exposed control of angular velocity for joints 1-6, expressed in ```rad/s```.
  * The state interfaces exposed are:
    * ```position```: Exposed angular position for joints 1-6, expressed in ```rad```.
    * ```velocity```: Exposed angular velocity for joints 1-6, expressed in ```rad/s```.
    * ```effort```: Exposed joint effort for joints 1-6, expressed in ```N*m```.


# How to use the SpaceSinter Stack
