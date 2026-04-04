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

// kinova_arm.cpp — ros2_control hardware plugin for Kinova Gen2 j2n6s300.
// Author: Cameron Hinkle, Colorado School of Mines
//
// read():  USB calls on the control thread. pos+vel every cycle,
//          effort every 10th (~20 Hz at 200 Hz loop). No background thread.
// write(): Position — PID-controller + velocity feedforward → ANGULAR_VELOCITY.
//          Velocity — direct rad/s → deg/s → ANGULAR_VELOCITY.
//
// USB setup (host, run once before first use):
//   sudo cp <pkg>/udev/10-kinova-arm.rules /etc/udev/rules.d/
//   sudo udevadm control --reload-rules && sudo udevadm trigger
//
// Firmware quirks preserved from kinova-ros upstream:
//   GetAngularVelocity returns half the actual value — ×2 applied in read().
//   Continuous joint positions wrap at 360°/0° — 2π unwrap applied in read().

#include "kinova_ros/kinova_arm.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <dlfcn.h>
#include <libgen.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <thread>
#include <vector>

extern "C" void kinova_driver__dladdr_anchor() {}

namespace kinova_driver
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("KinovaSystemHardware");

#define LOAD_SYM(h, ptr, sig, name)                                                \
  ptr = reinterpret_cast<sig>(dlsym((h), (name)));                                 \
  if (!(ptr)) {                                                                     \
    RCLCPP_ERROR(LOGGER, "Symbol '%s' not found: %s", (name), dlerror());          \
    return hardware_interface::CallbackReturn::ERROR;                               \
  }

// ---------------------------------------------------------------------------
// on_init
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KinovaSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  // Find the directory this plugin .so lives in so we can load vendor libs.
  Dl_info dl_info;
  if (dladdr(reinterpret_cast<void *>(kinova_driver__dladdr_anchor), &dl_info) == 0) {
    RCLCPP_ERROR(LOGGER, "dladdr failed — cannot find plugin path");
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::string plugin_path(dl_info.dli_fname);
  std::vector<char> buf(plugin_path.begin(), plugin_path.end());
  buf.push_back('\0');
  const std::string lib_dir = dirname(buf.data());

  // Prepend lib_dir so the command layer's internal dlopen("USBCommLayerUbuntu.so")
  // finds our vendored copy rather than a system copy (or nothing).
  const char * existing = getenv("LD_LIBRARY_PATH");
  setenv("LD_LIBRARY_PATH",
    (lib_dir + (existing ? (":" + std::string(existing)) : "")).c_str(), 1);

  // Load comm layer first (RTLD_GLOBAL) so its inode is resident when the
  // command layer's internal short-name dlopen fires — prevents a double-load
  // that would leave the SDK's commLayer_Handle pointer NULL.
  commLayer_handle_ = dlopen(
    (lib_dir + "/USBCommLayerUbuntu.so").c_str(), RTLD_NOW | RTLD_GLOBAL);
  if (!commLayer_handle_) {
    RCLCPP_ERROR(LOGGER, "dlopen USBCommLayerUbuntu.so: %s", dlerror());
    return hardware_interface::CallbackReturn::ERROR;
  }

  commandLayer_handle_ = dlopen(
    (lib_dir + "/USBCommandLayerUbuntu.so").c_str(), RTLD_NOW | RTLD_GLOBAL);
  if (!commandLayer_handle_) {
    RCLCPP_ERROR(LOGGER, "dlopen USBCommandLayerUbuntu.so: %s", dlerror());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Resolve symbols from commandLayer_handle_ explicitly — NOT RTLD_DEFAULT —
  // because both .so files export same-named symbols with different semantics.
  dlerror();
  LOAD_SYM(commandLayer_handle_, MyInitAPI,               int(*)(),                      "InitAPI")
  LOAD_SYM(commandLayer_handle_, MyCloseAPI,              int(*)(),                      "CloseAPI")
  LOAD_SYM(commandLayer_handle_, MyGetDevices,            int(*)(KinovaDevice[], int &), "GetDevices")
  LOAD_SYM(commandLayer_handle_, MySetActiveDevice,       int(*)(KinovaDevice),          "SetActiveDevice")
  LOAD_SYM(commandLayer_handle_, MyMoveHome,              int(*)(),                      "MoveHome")
  LOAD_SYM(commandLayer_handle_, MySendBasicTrajectory,   int(*)(TrajectoryPoint),       "SendBasicTrajectory")
  LOAD_SYM(commandLayer_handle_, MyGetAngularPosition,    int(*)(AngularPosition &),     "GetAngularPosition")
  LOAD_SYM(commandLayer_handle_, MyGetAngularVelocity,    int(*)(AngularPosition &),     "GetAngularVelocity")
  LOAD_SYM(commandLayer_handle_, MyGetAngularForceGravityFree, int(*)(AngularPosition &),"GetAngularForceGravityFree")
  LOAD_SYM(commandLayer_handle_, MyStartControlAPI,       int(*)(),                      "StartControlAPI")
  LOAD_SYM(commandLayer_handle_, MyStopControlAPI,        int(*)(),                      "StopControlAPI")
  LOAD_SYM(commandLayer_handle_, MySetAngularControl,     int(*)(),                      "SetAngularControl")
  LOAD_SYM(commandLayer_handle_, MyRefresDevicesList,     int(*)(),                      "RefresDevicesList")
  LOAD_SYM(commandLayer_handle_, MyEraseAllTrajectories,  int(*)(),                      "EraseAllTrajectories")
#undef LOAD_SYM

  const size_t n = info_.joints.size();
  hw_positions_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.assign(n, 0.0);
  hw_efforts_.assign(n, 0.0);
  hw_vel_cmds_.assign(n, 0.0);
  hw_pos_cmds_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_pos_cmds_prev_.assign(n, std::numeric_limits<double>::quiet_NaN());
  pid_integral_.assign(n, 0.0);
  static_hold_count_.assign(n, 0);

  RCLCPP_INFO(LOGGER, "Initialized (%zu joints).", n);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// export interfaces
// ---------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
KinovaSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   &hw_efforts_[i]);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface>
KinovaSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_pos_cmds_[i]);
    ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_cmds_[i]);
  }
  return ci;
}

// ---------------------------------------------------------------------------
// on_configure — connect to arm over USB
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KinovaSystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  int result = MyInitAPI();
  RCLCPP_INFO(LOGGER, "InitAPI result: %d", result);
  if (result != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(LOGGER,
      "InitAPI failed (%d). Check USB cable and udev rule:\n"
      "  sudo cp <pkg>/udev/10-kinova-arm.rules /etc/udev/rules.d/\n"
      "  sudo udevadm control --reload-rules && sudo udevadm trigger", result);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Poll up to 3 s for the USB background thread to enumerate the device.
  KinovaDevice list[MAX_KINOVA_DEVICE];
  int count = 0;
  for (int i = 0; i < 30 && count == 0; ++i) {
    MyRefresDevicesList();
    count = MyGetDevices(list, result);
    if (count == 0) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (count == 0) {
    RCLCPP_ERROR(LOGGER, "No Kinova device found after 3 s.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(LOGGER, "Found arm: serial='%s' model='%s'",
    list[0].SerialNumber, list[0].Model);
  MySetActiveDevice(list[0]);

  // All-zero position check: device enumerated but USB data transfers failing
  // (typical symptom of missing udev rule — libusb can open but not read).
  AngularPosition pos{};
  MyGetAngularPosition(pos);
  if (pos.Actuators.Actuator1 == 0.0f && pos.Actuators.Actuator2 == 0.0f &&
      pos.Actuators.Actuator3 == 0.0f && pos.Actuators.Actuator4 == 0.0f &&
      pos.Actuators.Actuator5 == 0.0f && pos.Actuators.Actuator6 == 0.0f) {
    RCLCPP_ERROR(LOGGER,
      "All joint positions are 0 — USB data transfers silently failing.\n"
      "Install the udev rule on the host and replug the USB cable.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(LOGGER, "USB OK: pos=[%.1f %.1f %.1f %.1f %.1f %.1f] deg",
    pos.Actuators.Actuator1, pos.Actuators.Actuator2, pos.Actuators.Actuator3,
    pos.Actuators.Actuator4, pos.Actuators.Actuator5, pos.Actuators.Actuator6);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_activate — start control API, home arm, seed state
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KinovaSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  MyStartControlAPI();
  MySetAngularControl();

  // Seed hw_positions_ from a live read so the first read() unwrap delta ≈ 0.
  // Continuous joints (1,4,5,6) are normalized to (-π, π] immediately so that
  // the seed matches the convention read() will maintain going forward.
  AngularPosition init{};
  MyGetAngularPosition(init);
  const float raw[6] = {
    init.Actuators.Actuator1, init.Actuators.Actuator2, init.Actuators.Actuator3,
    init.Actuators.Actuator4, init.Actuators.Actuator5, init.Actuators.Actuator6};
  constexpr bool NORMALIZE[6] = {true, false, false, true, true, true};
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    double p = static_cast<double>(raw[i]) * DEG_TO_RAD;
    if (NORMALIZE[i]) {
      while (p >  M_PI) p -= 2.0 * M_PI;
      while (p <= -M_PI) p += 2.0 * M_PI;
    }
    hw_positions_[i] = p;
  }
  RCLCPP_INFO(LOGGER, "Seeded pos (deg): [%.1f %.1f %.1f %.1f %.1f %.1f]",
    raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
  RCLCPP_INFO(LOGGER, "Seeded pos (rad, normalized): [%.3f %.3f %.3f %.3f %.3f %.3f]",
    hw_positions_[0], hw_positions_[1], hw_positions_[2],
    hw_positions_[3], hw_positions_[4], hw_positions_[5]);

  velocity_commands_active_ = false;
  position_commands_active_ = false;
  write_log_counter_ = 0;
  read_iter_ = 0;
  hw_pos_cmds_prev_.assign(hw_pos_cmds_.size(), std::numeric_limits<double>::quiet_NaN());
  pid_integral_.assign(hw_pos_cmds_.size(), 0.0);
  static_hold_count_.assign(hw_pos_cmds_.size(), 0);

  RCLCPP_INFO(LOGGER, "Hardware activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate / on_cleanup
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KinovaSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  velocity_commands_active_ = false;
  position_commands_active_ = false;
  MyEraseAllTrajectories();
  MyStopControlAPI();
  RCLCPP_INFO(LOGGER, "Hardware deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KinovaSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  MyCloseAPI();
  if (commandLayer_handle_) { dlclose(commandLayer_handle_); commandLayer_handle_ = nullptr; }
  if (commLayer_handle_)    { dlclose(commLayer_handle_);    commLayer_handle_    = nullptr; }
  RCLCPP_INFO(LOGGER, "API closed.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// prepare / perform command mode switch
// ---------------------------------------------------------------------------
hardware_interface::return_type KinovaSystemHardware::prepare_command_mode_switch(
  const std::vector<std::string> &, const std::vector<std::string> &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KinovaSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Process stops first so a simultaneous stop+start leaves the start flag set.
  for (const auto & iface : stop_interfaces) {
    if (iface.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos)
      velocity_commands_active_ = false;
    if (iface.find(hardware_interface::HW_IF_POSITION) != std::string::npos)
      position_commands_active_ = false;
  }
  for (const auto & iface : start_interfaces) {
    if (iface.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos)
      velocity_commands_active_ = true;
    if (iface.find(hardware_interface::HW_IF_POSITION) != std::string::npos) {
      position_commands_active_ = true;
      pid_integral_.assign(pid_integral_.size(), 0.0);
      static_hold_count_.assign(static_hold_count_.size(), 0);
    }
  }
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// read — USB calls directly on the control thread.
//   Position: raw deg → rad, 2π unwrap for continuous joints, then normalize
//     continuous joints to (-π, π] to match ROS convention and stay consistent
//     with MoveIt waypoints. Revolute joints (J2, J3) have limits inside [0, 2π)
//     so they are left un-normalized.
//   Velocity: ×2 (firmware halves value), 2's-complement for sign.
//   Effort:   gravity-free torque, every 10th cycle to reduce USB load.
// ---------------------------------------------------------------------------
hardware_interface::return_type KinovaSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  AngularPosition pos{}, vel{};
  MyGetAngularPosition(pos);
  MyGetAngularVelocity(vel);

  const float rp[6] = {
    pos.Actuators.Actuator1, pos.Actuators.Actuator2, pos.Actuators.Actuator3,
    pos.Actuators.Actuator4, pos.Actuators.Actuator5, pos.Actuators.Actuator6};
  const float rv[6] = {
    vel.Actuators.Actuator1, vel.Actuators.Actuator2, vel.Actuators.Actuator3,
    vel.Actuators.Actuator4, vel.Actuators.Actuator5, vel.Actuators.Actuator6};

  // Joints 1,4,5,6 are continuous — normalize to (-π, π] so MoveIt waypoints
  // stored in standard ROS convention always match.
  // Joints 2 and 3 are revolute with limits in [0, 2π); leave them as-is.
  constexpr bool NORMALIZE[6] = {true, false, false, true, true, true};

  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    double new_pos = static_cast<double>(rp[i]) * DEG_TO_RAD;
    if (std::isnan(hw_positions_[i])) {
      // First read after activation: normalize continuous joints to (−π, π]
      // so the initial reported position matches the planner's stored convention.
      if (NORMALIZE[i]) {
        while (new_pos >  M_PI) new_pos -= 2.0 * M_PI;
        while (new_pos <= -M_PI) new_pos += 2.0 * M_PI;
      }
    } else {
      // Subsequent reads: preserve continuity across the ±π boundary.
      // The anti-jump correction absorbs SDK wrap-arounds (e.g. 359°→1°)
      // without forcing the value back into (−π, π].  Re-normalising on every
      // cycle would produce a 2π step in reported position each time a continuous
      // joint crosses ±π, which the trajectory controller's path-tolerance check
      // treats as a ~6.28 rad deviation and aborts the trajectory.
      const double delta = new_pos - hw_positions_[i];
      if      (delta >  M_PI) new_pos -= 2.0 * M_PI;
      else if (delta < -M_PI) new_pos += 2.0 * M_PI;
    }
    hw_positions_[i]  = new_pos;
    hw_velocities_[i] = 2.0 * kinVelToRad(rv[i]);
  }

  if (++read_iter_ % 10 == 0) {
    AngularPosition torq{};
    MyGetAngularForceGravityFree(torq);
    hw_efforts_[0] = torq.Actuators.Actuator1;
    hw_efforts_[1] = torq.Actuators.Actuator2;
    hw_efforts_[2] = torq.Actuators.Actuator3;
    hw_efforts_[3] = torq.Actuators.Actuator4;
    hw_efforts_[4] = torq.Actuators.Actuator5;
    hw_efforts_[5] = torq.Actuators.Actuator6;
  }

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// write — PID-controller + velocity FF (position mode), or direct passthrough (velocity mode).
//   Position: vel_cmd = clamp(ff,±MAX_FF) + clamp(Kp×err,±MAX_P) + clamp(Ki×∫err,±MAX_I)
//                       + Kd×(sp_vel - actual_vel)
//
//     FF_GAIN=0.5   — half trajectory velocity as feedforward. Full ff caused decel
//       overshoot; at 0.5, MAX_P=25 wins during the decelerating phase.
//     MAX_FF=30 deg/s  — hard cap on the ff term.
//     MAX_P=25 deg/s   — P correction cap; prevents violent post-abort oscillation.
//     Kp=6 s⁻¹   — handles steady-state tracking lag.
//     Ki=0.5 s⁻² — small integral to eliminate persistent steady-state offset.
//     MAX_I=10 deg/s — anti-windup clamp on the integral contribution.
//     Kd=0.6 s    — D is derivative-of-error (sp_vel − actual_vel); damps oscillation
//       at static setpoints (sp_vel=0 → D=−Kd×vel) without opposing FF during
//       trajectory tracking (sp_vel≈actual_vel → D≈0).
//   Position error for continuous joints (1,4,5,6) is wrapped to (−π, π] to
//     prevent the 2π phantom error when the measured position crosses the ±π
//     boundary (e.g. cmd=+π, pos=−π → true err≈0, naïve subtraction = 2π).
//   Static-hold deadband (STATIC_DEADBAND_RAD=0.010 rad, ~0.57°):
//     When the setpoint is not moving (|sp_vel|<0.01 rad/s) and |err|<deadband,
//     P is zeroed and I accumulation pauses. This breaks the relay/bang-bang limit
//     cycle that would otherwise form from P alone at a fixed setpoint. D remains
//     active inside the deadband to damp any residual arm velocity.
//   Velocity: rad/s → deg/s → ANGULAR_VELOCITY.
//   Position path logs every 20 cycles (~0.1 s at 200 Hz) for debugging.
// ---------------------------------------------------------------------------
hardware_interface::return_type KinovaSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (position_commands_active_) {
    for (const auto & v : hw_pos_cmds_)
      if (std::isnan(v)) return hardware_interface::return_type::OK;

    constexpr double KP           = 6.0;   // s⁻¹
    constexpr double KI           = 0.2;   // s⁻²  — eliminates steady-state offset; kept low to limit windup
    // KD must damp oscillations without saturating the output cap (which causes relay oscillation).
    // At peak oscillation velocity ~40 deg/s: D = 1.0×40 = 40 deg/s > P_max(25)+I_max(10) = 35 deg/s.
    // Net braking of 5 deg/s → oscillation damps within 2-3 cycles.
    // KD=3.0 saturated at 3.0×20=60 on every cycle → bang-bang → divergence (do not raise above ~1.2).
    constexpr double KD           = 1.0;   // s    — derivative-of-error damping
    constexpr double FF_GAIN      = 0.5;   // half trajectory velocity — prevents decel overshoot
    constexpr double MAX_FF_DEG_S = 30.0;  // trajectory velocity cap (~Gen2 firmware limit)
    constexpr double MAX_P_DEG_S        = 25.0;  // P correction cap during trajectory tracking
    constexpr double MAX_P_STATIC_DEG_S  =  5.0;  // reduced P cap during static hold — limits equilibrium
                                                   //   approach speed to ~5 deg/s so the arm doesn't coast
                                                   //   through the target on trajectory end. With KD=1.0, the
                                                   //   P/D balance point is MAX_P/KD_DEG = 5/57.3 = 0.087 r/s.
                                                   //   At 200ms firmware lag, overshoot ≤ 5×0.2 = 1° < deadband.
    constexpr double MAX_I_DEG_S        = 10.0;  // integral cap during trajectory tracking (anti-windup)
    constexpr double MAX_I_STATIC_DEG_S =  5.0;  // reduced integral cap during static hold
    constexpr double MAX_VEL_DEG_S = 80.0; // global output cap
    // Deadband: when the setpoint is stationary and error is small, zero P and pause
    // I accumulation to prevent the relay/bang-bang limit cycle that forms at static holds.
    // D remains active inside the deadband to damp any residual arm velocity.
    constexpr double STATIC_VEL_THRESH_RAD_S = 0.01;  // sp_vel below this → static hold
    constexpr double STATIC_DEADBAND_RAD     = 0.030; // ~1.7° — P+I cutoff at static hold; wide enough to absorb arm chatter
    constexpr double KP_TO_DEG    = KP * RAD_TO_DEG;
    constexpr double KI_TO_DEG    = KI * RAD_TO_DEG;
    constexpr double KD_TO_DEG    = KD * RAD_TO_DEG;

    const bool NORMALIZE[6] = {true, false, false, true, true, true};

    const double period_sec = period.seconds();

    double errs[6], vels[6];
    for (size_t i = 0; i < hw_pos_cmds_.size(); ++i) {
      // Setpoint velocity (rad/s) — shared by FF and D terms.
      double sp_vel = 0.0;
      if (period_sec > 1e-6 && !std::isnan(hw_pos_cmds_prev_[i]))
        sp_vel = (hw_pos_cmds_[i] - hw_pos_cmds_prev_[i]) / period_sec;
      hw_pos_cmds_prev_[i] = hw_pos_cmds_[i];

      // FF: half the setpoint velocity — prevents decel-phase overshoot.
      double ff = FF_GAIN * sp_vel * RAD_TO_DEG;
      if (ff >  MAX_FF_DEG_S) ff =  MAX_FF_DEG_S;
      if (ff < -MAX_FF_DEG_S) ff = -MAX_FF_DEG_S;

      // Raw error — wrap to (−π, π] for continuous joints to prevent the 2π
      // phantom error when the measured position crosses the ±π boundary.
      double raw_err = hw_pos_cmds_[i] - hw_positions_[i];
      if (NORMALIZE[i]) {
        while (raw_err >  M_PI) raw_err -= 2.0 * M_PI;
        while (raw_err <= -M_PI) raw_err += 2.0 * M_PI;
      }
      errs[i] = raw_err;
      // Static-hold detection: require 20 consecutive cycles with |sp_vel|<thresh
      // (~0.1 s at 200 Hz) before treating the setpoint as stationary.
      // A single cycle with sp_vel=0 (e.g. duplicate cmd tick during trajectory)
      // does NOT count as a static hold and must not reset the integral.
      constexpr int STATIC_HOLD_CYCLES = 20;
      if (std::abs(sp_vel) < STATIC_VEL_THRESH_RAD_S) {
        ++static_hold_count_[i];
      } else {
        static_hold_count_[i] = 0;
      }
      const bool at_static = (static_hold_count_[i] >= STATIC_HOLD_CYCLES);
      // On the first cycle that crosses the static-hold threshold, reset the integral
      // so trajectory-tracking windup cannot drive a post-arrival overshoot.
      if (static_hold_count_[i] == STATIC_HOLD_CYCLES) {
        pid_integral_[i] = 0.0;
      }
      double err = (at_static && std::abs(raw_err) < STATIC_DEADBAND_RAD) ? 0.0 : raw_err;

      // P term — cap is lower during static hold to limit approach speed and prevent overshoot.
      const double max_p_eff = at_static ? MAX_P_STATIC_DEG_S : MAX_P_DEG_S;
      double p = err * KP_TO_DEG;
      if (p >  max_p_eff) p =  max_p_eff;
      if (p < -max_p_eff) p = -max_p_eff;

      // I term — pauses accumulation inside deadband; cap also reduced during static hold.
      const double max_i_eff = at_static ? MAX_I_STATIC_DEG_S : MAX_I_DEG_S;
      if (period_sec > 1e-6)
        pid_integral_[i] += err * period_sec;
      double iterm = pid_integral_[i] * KI_TO_DEG;
      if (iterm >  max_i_eff) { iterm =  max_i_eff; pid_integral_[i] =  max_i_eff / KI_TO_DEG; }
      if (iterm < -max_i_eff) { iterm = -max_i_eff; pid_integral_[i] = -max_i_eff / KI_TO_DEG; }

      // D term: derivative of error (sp_vel − actual_vel) → damps oscillation
      // at static setpoints without opposing FF during trajectory tracking.
      // Always active — provides velocity braking even inside the position deadband.
      // KD=3.0 gives ζ≈0.61 with KP=6 — sufficient to kill the ±0.13 rad limit cycle.
      double d = KD_TO_DEG * (sp_vel - hw_velocities_[i]);

      double vel_cmd = ff + p + iterm + d;
      if (vel_cmd >  MAX_VEL_DEG_S) vel_cmd =  MAX_VEL_DEG_S;
      if (vel_cmd < -MAX_VEL_DEG_S) vel_cmd = -MAX_VEL_DEG_S;
      vels[i] = vel_cmd;
    }

    TrajectoryPoint cmd;
    cmd.InitStruct();
    cmd.Position.Type                = ANGULAR_VELOCITY;
    cmd.Position.Actuators.Actuator1 = static_cast<float>(vels[0]);
    cmd.Position.Actuators.Actuator2 = static_cast<float>(vels[1]);
    cmd.Position.Actuators.Actuator3 = static_cast<float>(vels[2]);
    cmd.Position.Actuators.Actuator4 = static_cast<float>(vels[3]);
    cmd.Position.Actuators.Actuator5 = static_cast<float>(vels[4]);
    cmd.Position.Actuators.Actuator6 = static_cast<float>(vels[5]);
    cmd.Position.Fingers.Finger1     = 0.0f;
    cmd.Position.Fingers.Finger2     = 0.0f;
    cmd.Position.Fingers.Finger3     = 0.0f;
    MySendBasicTrajectory(cmd);

    if (write_log_counter_++ % 20 == 0) {
      RCLCPP_INFO(LOGGER,
        "[pos] cmd=[%.3f %.3f %.3f %.3f %.3f %.3f] "
        "pos=[%.3f %.3f %.3f %.3f %.3f %.3f] "
        "err=[%.3f %.3f %.3f %.3f %.3f %.3f] "
        "vel=[%.1f %.1f %.1f %.1f %.1f %.1f] deg/s",
        hw_pos_cmds_[0], hw_pos_cmds_[1], hw_pos_cmds_[2],
        hw_pos_cmds_[3], hw_pos_cmds_[4], hw_pos_cmds_[5],
        hw_positions_[0], hw_positions_[1], hw_positions_[2],
        hw_positions_[3], hw_positions_[4], hw_positions_[5],
        errs[0], errs[1], errs[2], errs[3], errs[4], errs[5],
        vels[0], vels[1], vels[2], vels[3], vels[4], vels[5]);
    }
    return hardware_interface::return_type::OK;
  }

  if (!velocity_commands_active_)
    return hardware_interface::return_type::OK;

  TrajectoryPoint cmd;
  cmd.InitStruct();
  cmd.Position.Type                = ANGULAR_VELOCITY;
  cmd.Position.Actuators.Actuator1 = static_cast<float>(hw_vel_cmds_[0] * RAD_TO_DEG);
  cmd.Position.Actuators.Actuator2 = static_cast<float>(hw_vel_cmds_[1] * RAD_TO_DEG);
  cmd.Position.Actuators.Actuator3 = static_cast<float>(hw_vel_cmds_[2] * RAD_TO_DEG);
  cmd.Position.Actuators.Actuator4 = static_cast<float>(hw_vel_cmds_[3] * RAD_TO_DEG);
  cmd.Position.Actuators.Actuator5 = static_cast<float>(hw_vel_cmds_[4] * RAD_TO_DEG);
  cmd.Position.Actuators.Actuator6 = static_cast<float>(hw_vel_cmds_[5] * RAD_TO_DEG);
  cmd.Position.Fingers.Finger1     = 0.0f;
  cmd.Position.Fingers.Finger2     = 0.0f;
  cmd.Position.Fingers.Finger3     = 0.0f;
  MySendBasicTrajectory(cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace kinova_driver

PLUGINLIB_EXPORT_CLASS(
  kinova_driver::KinovaSystemHardware,
  hardware_interface::SystemInterface)

