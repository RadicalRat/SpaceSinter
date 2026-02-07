# SpaceSinter: System for Solar Sintering of Lunar Regolith <!-- omit in toc -->

## Table of Contents <!-- omit in toc -->
- [Purpose](#purpose)
- [Requirements](#requirements)
- [File System](#file-system)
- [Hardware Setup](#hardware-setup)
- [*MoveIt Pro*](#moveit-pro)
- [ROS2 Hardware Drivers](#ros2-hardware-drivers)
- [Useage](#useage)


## Purpose
This code stack was developed for the **SolarSinter** senior design team at the Colorado School of Mines. The project goal was the development of a road-building system to concentrate solar energy and "sinter" (melt) lunar regolith into glassy roads for lunar infrastructure.

This system utilizes a **Kinova Jaco Gen2 (6-DOF)** arm for the deployment and maneuvering of a large Fresnel lens. Custom software was developed to bridge this legacy hardware with **ROS 2 Jazzy** and **MoveIt Pro**, enabling precise path planning, obstacle avoidance, and behavior-tree-based autonomy.

## Requirements
* **OS:** Ubuntu 24.04 (Noble Numbat)
* **Middleware:** ROS 2 Jazzy Jalisco
* **Software:** MoveIt Pro (Release 2.0+)
* **Hardware:** Kinova Jaco Gen2 6-DOF arm

**Note:** Due to the obsolescence of the original ROS 1 [kinova-ros](https://github.com/Kinovarobotics/kinova-ros) drivers, this repository contains a custom hardware interface rewritten for `ros2_control`. This stack is strictly for Linux/Docker environments and does not support Windows.

## File System
* `kinova_driver` : This directory contains the ROS2 hardware drivers for the kinova Gen2 Jaco arm. This directory also contains all relevant kinova API libraries used to control the arm.
* `kinova_moveit`: This directory contains the moveit configuration packages for use with *MoveIt Pro*.
  * `config`: This directory contains all of the configuration yamls for drivers, and moveit algorithms. Also contains robot srdf files and the ros2_control srdf
  * `description`: This directory contains all of the mesh files, robot urdfs, and mujuco configuration files. Also contains all of the world simulation files.
  * `launch`: This directory contains all launch files for *MoveIt Pro*.
  * `objectives`: This directory contains any robot behaviors for *MoveIt Pro*.
  * `test`: This directory contains any python unit tests for this package.
  * `waypoints`: This directory contains key waypoints for the SpaceSinter System.

## Hardware Setup
To allow ROS 2 to communicate with the Kinova arm via USB without `sudo`, you must install udev rules.

1.  Create a file at `/etc/udev/rules.d/99-kinova.rules`:
    ```bash
    SUBSYSTEM=="usb", ATTRS{idVendor}=="2032", ATTRS{idProduct}=="0005", MODE="0666"
    ```
2.  Reload the rules:
    ```bash
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```

**need to verify**


## *MoveIt Pro*
This repository is designed to run inside the standard MoveIt Pro Docker environment. 

* **Objectives:** The core logic is defined in `kinova_moveit/objectives`. These behavior trees handle the sintering pattern (e.g., raster scans).
* **Configuration:** The `spacesinter_site_config` package bundles the robot description and the site map (if applicable).

To install MoveIt Pro, refer to the [official documentation](https://docs.picknik.ai/software_installation/).



## ROS2 Hardware Drivers
* The command interfaces exposed are:
    * ```velocity```: Exposed control of angular velocity for joints 1-6, expressed in ```rad/s```.
  * The state interfaces exposed are:
    * ```position```: Exposed angular position for joints 1-6, expressed in ```rad```.
    * ```velocity```: Exposed angular velocity for joints 1-6, expressed in ```rad/s```.
    * ```effort```: Exposed joint effort for joints 1-6, expressed in ```N*m```.


## Useage
