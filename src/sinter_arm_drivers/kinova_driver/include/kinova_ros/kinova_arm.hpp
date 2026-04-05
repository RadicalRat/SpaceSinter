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
//
// State interfaces:   position (rad), velocity (rad/s), effort (N·m gravity-free)
// Command interfaces: position (rad), velocity (rad/s)
//
// All USB I/O runs on a dedicated background thread so that read()/write()
// never block the ros2_control update loop (USB calls can stall for 500 ms+).
// Position control uses a velocity-based P-controller (Kp=5, max 100 deg/s)
// running at ~100 Hz on the background thread.  All math in unwrapped
// rad-space — avoids Kinova firmware's 0°/360° wrapping issues.

#ifndef KINOVA_SYSTEM_HPP_
#define KINOVA_SYSTEM_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
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
  // ── Sensor state (exported as StateInterfaces) ────────────────────────────
  std::vector<double> hw_positions_;   ///< rad — continuously unwrapped; seeded to (−π,π] at activation
  std::vector<double> hw_velocities_;  ///< rad/s — ×2 corrected (firmware reports half actual)
  std::vector<double> hw_efforts_;     ///< N·m gravity-free; sampled every 10th cycle to reduce USB load

  // ── Command interfaces ───────────────────────────────────────────────────
  std::vector<double> hw_pos_cmds_;             ///< rad — written by joint_trajectory_controller
  std::vector<double> hw_vel_cmds_;             ///< rad/s — written by velocity controller

  // ── Background USB I/O thread ──────────────────────────────────────────
  //  All Kinova USB SDK calls happen exclusively on this thread.  read()
  //  and write() only exchange data with the thread through mutex-protected
  //  buffers, so the ros2_control update loop never stalls on USB.
  std::thread usb_thread_;
  std::atomic<bool> usb_thread_running_{false};
  mutable std::mutex io_mutex_;                 ///< guards all bg_* members

  // Sensor data (bg thread → main thread via read(), mutex-protected)
  std::vector<double> bg_positions_;
  std::vector<double> bg_velocities_;
  std::vector<double> bg_efforts_;

  // Command data (main thread → bg thread via write(), mutex-protected)
  std::vector<double> bg_pos_cmds_;
  std::vector<double> bg_vel_cmds_;
  bool bg_pos_active_{false};
  bool bg_vel_active_{false};
  bool bg_erase_trajectories_{false};           ///< one-shot flag set by perform_command_mode_switch

  /// USB timing diagnostics (bg thread → main thread, mutex-protected).
  struct UsbDiagnostics {
    double read_pos_ms{0.0};                    ///< last GetAngularPosition duration
    double read_vel_ms{0.0};                    ///< last GetAngularVelocity duration
    double read_eff_ms{0.0};                    ///< last GetAngularForceGravityFree duration
    double write_ms{0.0};                       ///< last command-send duration
    double cycle_ms{0.0};                       ///< full bg-loop iteration time
    uint64_t cycle_count{0};                    ///< total bg iterations since activation
    uint64_t slow_cycles{0};                    ///< iterations that took > 50 ms
  };
  UsbDiagnostics bg_diag_;

  void usb_io_thread_func();

  // ── USB vendor library handles ────────────────────────────────────────────
  void * commLayer_handle_{nullptr};
  void * commandLayer_handle_{nullptr};

  // ── Kinova SDK function pointers (resolved via dlsym in on_init) ─────────
  int (*MyInitAPI)()                                     {nullptr};
  int (*MyCloseAPI)()                                    {nullptr};
  int (*MyGetDevices)(KinovaDevice[], int &)             {nullptr};
  int (*MySetActiveDevice)(KinovaDevice)                 {nullptr};
  int (*MyMoveHome)()                                    {nullptr};
  int (*MySendBasicTrajectory)(TrajectoryPoint)          {nullptr};
  int (*MySendAdvanceTrajectory)(TrajectoryPoint)        {nullptr};
  int (*MyGetGlobalTrajectoryInfo)(TrajectoryFIFO &)     {nullptr};
  int (*MyGetAngularPosition)(AngularPosition &)         {nullptr};
  int (*MyGetAngularVelocity)(AngularPosition &)         {nullptr};
  int (*MyGetAngularForceGravityFree)(AngularPosition &) {nullptr};
  int (*MyStartControlAPI)()                             {nullptr};
  int (*MyStopControlAPI)()                              {nullptr};
  int (*MySetAngularControl)()                           {nullptr};
  int (*MyRefresDevicesList)()                           {nullptr};
  int (*MyEraseAllTrajectories)()                        {nullptr};

  // ── Active controller flags ───────────────────────────────────────────────
  bool pos_active_{false};  ///< position command interface is claimed
  bool vel_active_{false};  ///< velocity command interface is claimed

  // ── Bookkeeping ───────────────────────────────────────────────────────────
  uint64_t read_log_counter_{0};                ///< cycle counter for periodic diagnostic log in read()

  // ── Unit conversion ───────────────────────────────────────────────────────
  static constexpr double DEG_TO_RAD = M_PI / 180.0;
  static constexpr double RAD_TO_DEG = 180.0 / M_PI;

  /// Kinova 2's-complement velocity encoding: raw values 181–360 represent −179–0 deg/s.
  static double kinVelToRad(float raw)
  {
    double d = static_cast<double>(raw);
    if (d > 180.0) d -= 360.0;
    return d * DEG_TO_RAD;
  }

  /// Convert a joint angle (rad) to Kinova firmware degrees in [0°, 360°).
  /// The Kinova API requires actuator positions in [0, 360) — negative values
  /// cause the firmware to drive joints in the wrong direction or to 0°.
  ///
  /// Edge-case fix: fmod(-ε, 360) → small negative → +360 → rounds to 360.0f
  /// in float32.  The Kinova firmware interprets 360.0f as a full revolution
  /// from 0°, not as 0°.  The final clamp maps 360.0f back to 0.0f.
  static float radToKinovaDeg(double rad)
  {
    double deg = std::fmod(rad * (180.0 / M_PI), 360.0);
    if (deg < 0.0) deg += 360.0;
    const float result = static_cast<float>(deg);
    return result < 360.0f ? result : 0.0f;
  }
};

}  // namespace kinova_driver

#endif  // KINOVA_SYSTEM_HPP_