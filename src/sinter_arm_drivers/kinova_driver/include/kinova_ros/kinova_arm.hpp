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

/*
 * kinova_arm.hpp
 *
 * ros2_control hardware plugin for the Kinova Gen2 j2n6s300 6-DOF arm.
 *
 * State interfaces   : position (rad), velocity (rad/s), effort (N·m, gravity-free)
 * Command interfaces : position (rad) and velocity (rad/s)
 *
 * Supports: joint_trajectory_admittance_controller (position mode),
 *           joint_velocity_controller (velocity mode)
 * Target control rate: 200 Hz
 *
 * Threading:
 *   A SCHED_OTHER background thread (poller_loop) performs all Kinova USB I/O.
 *   read() copies shadow state under a mutex — no USB calls on the RT thread.
 *   write() velocity path: SendBasicTrajectory(ANGULAR_VELOCITY) — matches the
 *   official SDK example kinova_ang_control.cpp (usleep(5000) between calls).
 *   write() position path: EraseAllTrajectories + SendAdvanceTrajectory(ANGULAR_POSITION).
 */

#ifndef KINOVA_SYSTEM_HPP_
#define KINOVA_SYSTEM_HPP_

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <cmath>

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

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
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
  // ── State buffers (exposed as state interfaces, written by read()) ─────────
  std::vector<double> hw_positions_;   // rad
  std::vector<double> hw_velocities_;  // rad/s
  std::vector<double> hw_efforts_;     // N·m  (gravity-free torque)

  // ── Command buffers (exposed as command interfaces, written by controllers) ─
  std::vector<double> hw_commands_;           // rad/s  (velocity)
  std::vector<double> hw_position_commands_;  // rad    (position)

  // ── Kinova SDK library handles ─────────────────────────────────────────────
  void * commLayer_handle_{nullptr};
  void * commandLayer_handle_{nullptr};

  // ── Kinova API function pointers ───────────────────────────────────────────
  // Mirrors the function-pointer pattern used in the official example scripts
  // (kinova_ang_control.cpp, kinova_get_ang.cpp, kinova_force_control.cpp).
  int (*MyInitAPI)()                                               {nullptr};
  int (*MyCloseAPI)()                                              {nullptr};
  int (*MyGetDevices)(KinovaDevice[], int &)                       {nullptr};
  int (*MySetActiveDevice)(KinovaDevice)                           {nullptr};
  int (*MyMoveHome)()                                              {nullptr};
  int (*MyInitFingers)()                                           {nullptr};
  int (*MySendBasicTrajectory)(TrajectoryPoint)                    {nullptr};
  int (*MySendAdvanceTrajectory)(TrajectoryPoint)                 {nullptr};
  int (*MyGetAngularPosition)(AngularPosition &)                   {nullptr};
  int (*MyGetAngularVelocity)(AngularPosition &)                   {nullptr};
  int (*MyGetAngularCommand)(AngularPosition &)                    {nullptr};
  int (*MyGetAngularForce)(AngularPosition &)                      {nullptr};
  int (*MyGetAngularForceGravityFree)(AngularPosition &)           {nullptr};
  int (*MyStartControlAPI)()                                       {nullptr};
  int (*MyStopControlAPI)()                                        {nullptr};
  int (*MySetAngularControl)()                                     {nullptr};
  int (*MyRefresDevicesList)()                                     {nullptr};  int (*MyEraseAllTrajectories)()                                    {nullptr};
  // ── Controller-mode guard ──────────────────────────────────────────────────
  // write() only sends USB commands after a controller has claimed the command
  // interfaces via perform_command_mode_switch().  This prevents the RT thread
  // from filling the arm's trajectory FIFO during the JSB-only startup phase.
  bool velocity_commands_active_{false};
  bool position_commands_active_{false};

  // ── Background USB polling thread ─────────────────────────────────────────
  // All Kinova GetAngular* calls live here (SCHED_OTHER), keeping the vendor
  // libUSB userspace polling loop off the SCHED_FIFO RT control thread.
  std::thread       poller_thread_;
  std::mutex        state_mutex_;
  std::atomic<bool> poller_running_{false};
  AngularPosition   pos_shadow_{};
  AngularPosition   vel_shadow_{};
  AngularPosition   torq_shadow_{};
  void poller_loop();

  // ── Unit-conversion constants ──────────────────────────────────────────────
  static constexpr double DEG_TO_RAD = M_PI / 180.0;
  static constexpr double RAD_TO_DEG = 180.0 / M_PI;
};

}  // namespace kinova_driver

#endif  // KINOVA_SYSTEM_HPP_
