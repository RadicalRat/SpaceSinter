// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KINOVA_SYSTEM_HPP_
#define KINOVA_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"


// kinova includes
#include "kinova_api/KinovaTypes.h"

// MAY NEED TO FIX FOR ARM SYSTEMS: check for rpi3 library build on github
#include "kinova_api/Kinova.API.USBCommLayerUbuntu.h"
#include "kinova_api/Kinova.API.USBCommandLayerUbuntu.h"


namespace kinova_driver
{
class KinovaSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KinovaSystemHardware)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // TODO: add parameters for hardware interface
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_efforts_;
    std::vector<double> hw_commands_; //Velocity commands

    //A handle to the API.
    void * commandLayer_handle;

    //Function pointers to the functions we need
    int(*MyInitAPI)();
    int(*MyCloseAPI)();
    int(*MySendBasicTrajectory)(TrajectoryPoint command);
    int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
    int(*MySetActiveDevice)(KinovaDevice device);
    int(*MyGetAngularPosition)(AngularPosition &);
    int (*MyGetAngularVelocity)(AngularPosition &Response);
    int(*MyGetAngularForce)(AngularPosition &Response);


    static constexpr double TO_RAD = M_PI / 180.0;
    static constexpr double TO_DEG = 180.0 / M_PI;

    // Standard conversion for POSITIONS
    double degreesToRadians(double degrees) const {
        return degrees * TO_RAD;
    }

    double radiansToDegrees(double radians) const {
        return radians * TO_DEG;
    }

    // Special conversion for VELOCITIES
    double kinovaVelocityToRos(double kinova_deg_per_sec) const {
    // Kinova Gen2 API often reports negative velocity as (360 - val)
    // e.g. -10 deg/s might come in as 350.
        if (kinova_deg_per_sec > 180.0) {
            kinova_deg_per_sec -= 360.0;
        }
        return kinova_deg_per_sec * TO_RAD;
    }

    // converts to degrees
    double rosVelocityToKinova(double ros_rad_per_sec) const {
        return ros_rad_per_sec * TO_DEG;
    }
};

}  // namespace kinova_driver

#endif 