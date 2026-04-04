// Copyright 2024 Space Sinter Team, Colorado School of Mines
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

// kinova_arm.hpp — ros2_control hardware plugin for Kinova Gen2 j2n6s300.
// State:   position (rad), velocity (rad/s), effort (N·m gravity-free)
// Command: position (rad), velocity (rad/s)

#ifndef KINOVA_SYSTEM_HPP_
#define KINOVA_SYSTEM_HPP_

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "kinova_api/KinovaTypes.h"
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

  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_positions_;   // rad
  std::vector<double> hw_velocities_;  // rad/s
  std::vector<double> hw_efforts_;     // N·m (gravity-free)
  std::vector<double> hw_vel_cmds_;       // rad/s  — velocity command interface
  std::vector<double> hw_pos_cmds_;       // rad    — position command interface
  std::vector<double> hw_pos_cmds_prev_;  // previous cycle's pos cmds (for velocity feedforward)
  std::vector<double> pid_integral_;      // rad·s  — error integral for I term
  std::vector<int>    static_hold_count_; // consecutive write() cycles with |sp_vel|<thresh

  void * commLayer_handle_{nullptr};
  void * commandLayer_handle_{nullptr};

  int (*MyInitAPI)()                                    {nullptr};
  int (*MyCloseAPI)()                                   {nullptr};
  int (*MyGetDevices)(KinovaDevice[], int &)            {nullptr};
  int (*MySetActiveDevice)(KinovaDevice)                {nullptr};
  int (*MyMoveHome)()                                   {nullptr};
  int (*MySendBasicTrajectory)(TrajectoryPoint)         {nullptr};
  int (*MyGetAngularPosition)(AngularPosition &)        {nullptr};
  int (*MyGetAngularVelocity)(AngularPosition &)        {nullptr};
  int (*MyGetAngularForceGravityFree)(AngularPosition &){nullptr};
  int (*MyStartControlAPI)()                            {nullptr};
  int (*MyStopControlAPI)()                             {nullptr};
  int (*MySetAngularControl)()                          {nullptr};
  int (*MyRefresDevicesList)()                          {nullptr};
  int (*MyEraseAllTrajectories)()                       {nullptr};

  bool     velocity_commands_active_{false};
  bool     position_commands_active_{false};
  uint64_t write_log_counter_{0};
  unsigned read_iter_{0};

  static constexpr double DEG_TO_RAD = M_PI / 180.0;
  static constexpr double RAD_TO_DEG = 180.0 / M_PI;

  // Kinova velocity 2's-complement: 181..360 maps to negative deg/s.
  static double kinVelToRad(float deg) {
    double d = static_cast<double>(deg);
    if (d > 180.0) d -= 360.0;
    return d * DEG_TO_RAD;
  }
};

}  // namespace kinova_driver

#endif  // KINOVA_SYSTEM_HPP_