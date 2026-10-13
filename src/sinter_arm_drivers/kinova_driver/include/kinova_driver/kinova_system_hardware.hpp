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

// kinova_system_hardware.hpp — ros2_control hardware plugin for the Kinova
// Gen2 j2n6s300.
//
// State interfaces:   position (rad), velocity (rad/s), effort (N·m, gravity-free)
// Command interfaces: position (rad), velocity (rad/s) — mutually exclusive
//
// All USB I/O runs on a dedicated background thread so read()/write() never
// block the controller_manager loop (SDK calls can stall for 500 ms+).  The
// two sides exchange plain structs under a mutex.
//
// Position commands are tracked by a velocity-based P-controller on the I/O
// thread, streamed as ANGULAR_VELOCITY.  Velocity commands pass straight
// through.  Both are clamped to max_joint_velocity_deg.
//
// Safety behaviour:
//   * Command watchdog: if write() stops arriving for command_timeout_ms the
//     I/O thread streams zero velocity until commands resume.
//   * NaN commands are never sent; the arm holds instead.
//   * After max_consecutive_io_errors failed SDK calls in a row the I/O thread
//     stops the arm and latches a fault; read() then returns ERROR so the
//     controller_manager deactivates the hardware.
//
// Hardware parameters (all optional, set in the ros2_control <hardware> tag):
//   serial_number              ""    pick this arm; empty = first arm found
//   position_kp                5.0   (rad/s) per rad of position error
//   max_joint_velocity_deg     100   clamp on every streamed joint velocity
//   io_rate_hz                 100   I/O thread loop rate
//   effort_decimation          10    read effort every Nth I/O cycle
//   command_timeout_ms         100   watchdog timeout for write()
//   max_consecutive_io_errors  10    SDK failures in a row before faulting
//   device_discovery_timeout_s 3.0   how long on_configure waits for the arm

#ifndef KINOVA_DRIVER__KINOVA_SYSTEM_HARDWARE_HPP_
#define KINOVA_DRIVER__KINOVA_SYSTEM_HARDWARE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "kinova_driver/kinova_sdk.hpp"

namespace kinova_driver
{

using JointArray = std::array<double, kNumJoints>;

class KinovaSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KinovaSystemHardware)

  ~KinovaSystemHardware() override;

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
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
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
  enum class ControlMode : uint8_t { kIdle, kPosition, kVelocity };

  struct Params
  {
    std::string serial_number;
    double position_kp{5.0};
    double max_joint_velocity_deg{100.0};
    double io_rate_hz{100.0};
    unsigned effort_decimation{10};
    std::chrono::milliseconds command_timeout{100};
    unsigned max_consecutive_io_errors{10};
    double device_discovery_timeout_s{3.0};
  };

  /// I/O thread → controller_manager thread.
  struct SensorFrame
  {
    JointArray positions{};   ///< rad; continuous joints in (−π, π]
    JointArray velocities{};  ///< rad/s
    JointArray efforts{};     ///< N·m gravity-free
  };

  /// controller_manager thread → I/O thread.
  struct CommandFrame
  {
    JointArray positions{};
    JointArray velocities{};
    ControlMode mode{ControlMode::kIdle};
    std::chrono::steady_clock::time_point stamp{};  ///< when write() produced it
    bool erase_trajectories{false};                 ///< one-shot, set on mode switch
  };

  // ── Lifecycle helpers ────────────────────────────────────────────────────
  bool parseParams();
  bool validateJoints() const;
  bool connectToArm();
  bool readPositionsBlocking(JointArray & out) const;
  void startIoThread();
  void stopIoThread();
  void closeApi();

  /// Resolve the mode that would result from applying the given switch.
  /// Returns false if the switch is invalid (e.g. position + velocity at once).
  bool resolveMode(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces,
    ControlMode & mode) const;
  bool ownsInterface(const std::string & full_name, const std::string & type) const;

  // ── I/O thread ───────────────────────────────────────────────────────────
  void ioLoop();
  /// Compute the velocity to stream this cycle from the latest command/state.
  JointArrayF computeVelocityCommand(
    const CommandFrame & cmd, const SensorFrame & state,
    std::chrono::steady_clock::time_point now);

  Params params_;
  std::unique_ptr<KinovaSdk> sdk_;
  bool api_initialized_{false};
  bool control_api_started_{false};

  // ros2_control-facing buffers (controller_manager thread only).
  JointArray hw_positions_{};
  JointArray hw_velocities_{};
  JointArray hw_efforts_{};
  JointArray hw_pos_cmds_{};
  JointArray hw_vel_cmds_{};
  ControlMode mode_{ControlMode::kIdle};
  bool pending_erase_{false};

  // Shared with the I/O thread, guarded by io_mutex_.
  std::mutex io_mutex_;
  SensorFrame shared_sensors_;
  CommandFrame shared_commands_;

  std::thread io_thread_;
  std::atomic<bool> io_running_{false};
  std::atomic<bool> io_fault_{false};

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};  ///< for throttled logging
};

}  // namespace kinova_driver

#endif  // KINOVA_DRIVER__KINOVA_SYSTEM_HARDWARE_HPP_
