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
// ── Architecture ──────────────────────────────────────────────────────────────
// All Kinova USB SDK calls run on a dedicated background thread
// (usb_io_thread_func) so that the ros2_control read()/write() callbacks
// never block on USB I/O.  The main thread exchanges data with the USB
// thread via mutex-protected buffers (bg_positions_, bg_pos_cmds_, etc.).
//
// ── Position control ──────────────────────────────────────────────────────────
// Velocity-based P-controller running at ~100 Hz on the USB thread.
//   vel_cmd = clamp(Kp * (cmd_rad − pos_rad), ±MAX_VEL)
// All math in unwrapped rad-space → no 0°/360° wrap issues.
// Sends ANGULAR_VELOCITY via SendBasicTrajectory (streaming, no FIFO).
// Kp=5.0 (rad/s per rad), MAX_VEL=100 deg/s.
//
// ── Velocity control ──────────────────────────────────────────────────────────
// Direct passthrough to ANGULAR_VELOCITY via SendBasicTrajectory.
//
// ── Position unwrap ───────────────────────────────────────────────────────────
// Joints 1,4,5,6 are continuous (can wrap 360°→0°).  The USB thread applies
// delta-only unwrap so the reported position is monotonically continuous.
// Joints 2,3 are revolute with limits inside [0, 2π); they are not unwrapped.
//   USB thread: continuous joints are delta-unwrapped (can go outside (−π, π]).
//   read(): continuous joints are always renormalised to (−π, π] so that
//   hw_positions_ matches MoveIt's representation at trajectory-start time.
//   JTC angle_wraparound:true handles the ±π crossing case during execution.
//
// ── Firmware quirks ───────────────────────────────────────────────────────────
//   GetAngularVelocity returns half the actual value → ×2 correction.
//   Velocity encoding: 181–360 deg/s represents −179–0 deg/s (2's complement).
//
// ── USB setup (host, run once before first use) ───────────────────────────────
//   sudo cp <pkg>/udev/10-kinova-arm.rules /etc/udev/rules.d/
//   sudo udevadm control --reload-rules && sudo udevadm trigger

#include "kinova_ros/kinova_arm.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <cinttypes>
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

// ─────────────────────────────────────────────────────────────────────────────
// on_init — dlopen vendor libs, resolve SDK symbols, size state/command arrays
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn KinovaSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  // Resolve this plugin's own .so path so we can load the vendor libs from the
  // same directory, regardless of where the workspace is installed.
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
  LOAD_SYM(commandLayer_handle_, MySendBasicTrajectory,        int(*)(TrajectoryPoint),       "SendBasicTrajectory")
  LOAD_SYM(commandLayer_handle_, MySendAdvanceTrajectory,       int(*)(TrajectoryPoint),       "SendAdvanceTrajectory")
  LOAD_SYM(commandLayer_handle_, MyGetGlobalTrajectoryInfo,     int(*)(TrajectoryFIFO &),      "GetGlobalTrajectoryInfo")
  LOAD_SYM(commandLayer_handle_, MyGetAngularPosition,          int(*)(AngularPosition &),     "GetAngularPosition")
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

  bg_positions_.assign(n, std::numeric_limits<double>::quiet_NaN());
  bg_velocities_.assign(n, 0.0);
  bg_efforts_.assign(n, 0.0);
  bg_pos_cmds_.assign(n, std::numeric_limits<double>::quiet_NaN());
  bg_vel_cmds_.assign(n, 0.0);

  RCLCPP_INFO(LOGGER, "Initialized (%zu joints).", n);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_state_interfaces / export_command_interfaces
// ─────────────────────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────────────
// on_configure — connect to arm over USB
// ─────────────────────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────────────
// on_activate — start control API, seed position state from live read
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn KinovaSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  MyStartControlAPI();
  MySetAngularControl();

  // Seed hw_positions_ from a live read so the first read() unwrap delta ≈ 0.
  // Continuous joints (1,4,5,6) are normalised to (−π, π] immediately so the
  // seed matches the convention that read() will maintain going forward.
  AngularPosition init{};
  MyGetAngularPosition(init);
  const float raw[6] = {
    init.Actuators.Actuator1, init.Actuators.Actuator2, init.Actuators.Actuator3,
    init.Actuators.Actuator4, init.Actuators.Actuator5, init.Actuators.Actuator6};
  constexpr bool NORMALIZE_SEED[6] = {true, false, false, true, true, true};
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    double p = static_cast<double>(raw[i]) * DEG_TO_RAD;
    if (NORMALIZE_SEED[i]) {
      while (p >  M_PI) p -= 2.0 * M_PI;
      while (p <= -M_PI) p += 2.0 * M_PI;
    }
    hw_positions_[i] = p;
  }
  RCLCPP_INFO(LOGGER, "Seeded pos (deg): [%.1f %.1f %.1f %.1f %.1f %.1f]",
    raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
  RCLCPP_INFO(LOGGER, "Seeded pos (rad, normalised): [%.3f %.3f %.3f %.3f %.3f %.3f]",
    hw_positions_[0], hw_positions_[1], hw_positions_[2],
    hw_positions_[3], hw_positions_[4], hw_positions_[5]);

  pos_active_ = false;
  vel_active_ = false;
  read_log_counter_ = 0;

  // Seed background thread buffers with current position.
  {
    std::lock_guard<std::mutex> lk(io_mutex_);
    bg_positions_ = hw_positions_;
    bg_velocities_.assign(bg_velocities_.size(), 0.0);
    bg_efforts_.assign(bg_efforts_.size(), 0.0);
    bg_pos_cmds_.assign(bg_pos_cmds_.size(), std::numeric_limits<double>::quiet_NaN());
    bg_vel_cmds_.assign(bg_vel_cmds_.size(), 0.0);
    bg_pos_active_ = false;
    bg_vel_active_ = false;
    bg_erase_trajectories_ = false;
    bg_diag_ = UsbDiagnostics{};
  }

  // Start the background USB I/O thread.
  usb_thread_running_.store(true, std::memory_order_release);
  usb_thread_ = std::thread(&KinovaSystemHardware::usb_io_thread_func, this);

  RCLCPP_INFO(LOGGER, "Hardware activated — USB I/O thread started.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_deactivate / on_cleanup
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn KinovaSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // Stop the background USB I/O thread first.
  usb_thread_running_.store(false, std::memory_order_release);
  if (usb_thread_.joinable()) {
    usb_thread_.join();
  }

  pos_active_ = false;
  vel_active_ = false;
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

// ─────────────────────────────────────────────────────────────────────────────
// prepare / perform command mode switch
// ─────────────────────────────────────────────────────────────────────────────
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
      vel_active_ = false;
    if (iface.find(hardware_interface::HW_IF_POSITION) != std::string::npos)
      pos_active_ = false;
  }
  for (const auto & iface : start_interfaces) {
    if (iface.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos)
      vel_active_ = true;
    if (iface.find(hardware_interface::HW_IF_POSITION) != std::string::npos) {
      pos_active_ = true;
      // Tell the bg thread to flush stale waypoints from the arm's FIFO.
      std::lock_guard<std::mutex> lk(io_mutex_);
      bg_erase_trajectories_ = true;
    }
  }
  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// read — non-blocking: copies latest sensor data from background USB thread
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type KinovaSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Continuous joints are always normalised to (−π, π] so that hw_positions_
  // stays in the same representation that MoveIt uses when building trajectories.
  // JTC's angle_wraparound parameter handles the ±π crossing case during motion.
  static constexpr bool CONTINUOUS[6] = {true, false, false, true, true, true};

  UsbDiagnostics diag;
  {
    std::lock_guard<std::mutex> lk(io_mutex_);
    for (size_t i = 0; i < hw_positions_.size(); ++i) {
      double p = bg_positions_[i];
      if (CONTINUOUS[i]) {
        while (p >  M_PI) p -= 2.0 * M_PI;
        while (p <= -M_PI) p += 2.0 * M_PI;
      }
      hw_positions_[i]  = p;
      hw_velocities_[i] = bg_velocities_[i];
      hw_efforts_[i]    = bg_efforts_[i];
    }
    diag = bg_diag_;
  }

  ++read_log_counter_;

  // Every-cycle tracking log when position mode is active (~2 Hz = every 50th cycle).
  if (pos_active_ && (read_log_counter_ % 50 == 0)) {
    RCLCPP_INFO(LOGGER,
      "[read:TRACK] "
      "pos=[%.4f %.4f %.4f %.4f %.4f %.4f] "
      "cmd=[%.4f %.4f %.4f %.4f %.4f %.4f] "
      "err=[%.4f %.4f %.4f %.4f %.4f %.4f]",
      hw_positions_[0], hw_positions_[1], hw_positions_[2],
      hw_positions_[3], hw_positions_[4], hw_positions_[5],
      hw_pos_cmds_[0], hw_pos_cmds_[1], hw_pos_cmds_[2],
      hw_pos_cmds_[3], hw_pos_cmds_[4], hw_pos_cmds_[5],
      hw_pos_cmds_[0] - hw_positions_[0], hw_pos_cmds_[1] - hw_positions_[1],
      hw_pos_cmds_[2] - hw_positions_[2], hw_pos_cmds_[3] - hw_positions_[3],
      hw_pos_cmds_[4] - hw_positions_[4], hw_pos_cmds_[5] - hw_positions_[5]);
  }

  // Periodic full diagnostic log (~every 5 s at 100 Hz update rate).
  if (read_log_counter_ % 500 == 0) {
    RCLCPP_INFO(LOGGER,
      "[read] pos=[%.3f %.3f %.3f %.3f %.3f %.3f] "
      "vel=[%.3f %.3f %.3f %.3f %.3f %.3f] "
      "eff=[%.2f %.2f %.2f %.2f %.2f %.2f] | "
      "usb cycle=%.1fms pos=%.1fms vel=%.1fms eff=%.1fms wr=%.1fms "
      "n=%" PRIu64 " slow=%" PRIu64,
      hw_positions_[0], hw_positions_[1], hw_positions_[2],
      hw_positions_[3], hw_positions_[4], hw_positions_[5],
      hw_velocities_[0], hw_velocities_[1], hw_velocities_[2],
      hw_velocities_[3], hw_velocities_[4], hw_velocities_[5],
      hw_efforts_[0], hw_efforts_[1], hw_efforts_[2],
      hw_efforts_[3], hw_efforts_[4], hw_efforts_[5],
      diag.cycle_ms, diag.read_pos_ms, diag.read_vel_ms,
      diag.read_eff_ms, diag.write_ms,
      diag.cycle_count, diag.slow_cycles);
  }

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// write — non-blocking: copies commands into background USB thread buffers
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type KinovaSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lk(io_mutex_);
  bg_pos_cmds_  = hw_pos_cmds_;
  bg_vel_cmds_  = hw_vel_cmds_;
  bg_pos_active_ = pos_active_;
  bg_vel_active_ = vel_active_;
  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// usb_io_thread_func — dedicated thread for all Kinova USB SDK communication
//
// Runs at best-effort ~100 Hz.  Each iteration:
//   1. Reads position and velocity from USB (always).
//   2. Reads effort every 10th cycle (reduce USB load).
//   3. Exchanges data with main thread under mutex.
//   4. Sends the appropriate command (ANGULAR_VELOCITY for pos P-ctrl / vel passthrough / idle).
//
// Position unwrap:
//   Continuous joints (1,4,5,6): delta-only 2π correction.
//   Revolute joints (2,3): left as raw deg→rad.
//   First read after activation: continuous joints normalised to (−π, π].
//
// Position commands:
//   Velocity-based P-controller: vel = clamp(Kp * error, ±MAX_VEL).
//   All math in unwrapped rad-space — no 0°/360° wrapping issues.
//   Sends ANGULAR_VELOCITY via SendBasicTrajectory (streaming, no FIFO).
//
// Velocity commands:
//   ANGULAR_VELOCITY passthrough via SendBasicTrajectory.
// ─────────────────────────────────────────────────────────────────────────────
void KinovaSystemHardware::usb_io_thread_func()
{
  RCLCPP_INFO(LOGGER, "[usb_thread] Started.");

  const size_t n = 6;  // j2n6s300 = 6 DOF

  // Thread-local sensor buffers — only this thread writes to these.
  std::vector<double> local_positions(n, 0.0);
  std::vector<double> local_velocities(n, 0.0);
  std::vector<double> local_efforts(n, 0.0);

  // Seed local positions from the values set in on_activate().
  {
    std::lock_guard<std::mutex> lk(io_mutex_);
    local_positions = bg_positions_;
  }

  // Thread-local command buffers.
  std::vector<double> local_pos_cmds(n, std::numeric_limits<double>::quiet_NaN());
  std::vector<double> local_vel_cmds(n, 0.0);
  bool local_pos_active = false;
  bool local_vel_active = false;

  unsigned effort_iter = 0;
  unsigned write_counter = 0;

  // Joints 1,4,5,6 are continuous — wrap-tracked.
  // Joints 2,3 are revolute with limits in [0, 2π) — left as-is.
  static constexpr bool WRAP[6] = {true, false, false, true, true, true};

  while (usb_thread_running_.load(std::memory_order_acquire)) {
    const auto t_cycle = std::chrono::steady_clock::now();

    // ── 1. Read position ────────────────────────────────────────────────────
    AngularPosition pos{}, vel{};
    auto t0 = std::chrono::steady_clock::now();
    MyGetAngularPosition(pos);
    auto t1 = std::chrono::steady_clock::now();
    const double ms_pos = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // ── 2. Read velocity ────────────────────────────────────────────────────
    t0 = std::chrono::steady_clock::now();
    MyGetAngularVelocity(vel);
    t1 = std::chrono::steady_clock::now();
    const double ms_vel = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Process raw sensor data.
    const float rp[6] = {
      pos.Actuators.Actuator1, pos.Actuators.Actuator2, pos.Actuators.Actuator3,
      pos.Actuators.Actuator4, pos.Actuators.Actuator5, pos.Actuators.Actuator6};
    const float rv[6] = {
      vel.Actuators.Actuator1, vel.Actuators.Actuator2, vel.Actuators.Actuator3,
      vel.Actuators.Actuator4, vel.Actuators.Actuator5, vel.Actuators.Actuator6};

    for (size_t i = 0; i < n; ++i) {
      // Position unwrap.
      double new_pos = static_cast<double>(rp[i]) * DEG_TO_RAD;
      if (std::isnan(local_positions[i])) {
        // First read — normalise continuous joints to (−π, π].
        if (WRAP[i]) {
          while (new_pos >  M_PI) new_pos -= 2.0 * M_PI;
          while (new_pos <= -M_PI) new_pos += 2.0 * M_PI;
        }
      } else {
        // Subsequent reads — delta-only correction (no re-normalise).
        const double delta = new_pos - local_positions[i];
        if      (delta >  M_PI) new_pos -= 2.0 * M_PI;
        else if (delta < -M_PI) new_pos += 2.0 * M_PI;
      }
      local_positions[i] = new_pos;

      // Velocity: ×2 correction (firmware reports half actual value).
      local_velocities[i] = 2.0 * kinVelToRad(rv[i]);
    }

    // ── 3. Read effort (every 10th cycle) ───────────────────────────────────
    double ms_eff = 0.0;
    if (++effort_iter % 10 == 0) {
      AngularPosition torq{};
      t0 = std::chrono::steady_clock::now();
      MyGetAngularForceGravityFree(torq);
      t1 = std::chrono::steady_clock::now();
      ms_eff = std::chrono::duration<double, std::milli>(t1 - t0).count();
      local_efforts[0] = torq.Actuators.Actuator1;
      local_efforts[1] = torq.Actuators.Actuator2;
      local_efforts[2] = torq.Actuators.Actuator3;
      local_efforts[3] = torq.Actuators.Actuator4;
      local_efforts[4] = torq.Actuators.Actuator5;
      local_efforts[5] = torq.Actuators.Actuator6;
    }

    // ── 4. Exchange data with main thread ───────────────────────────────────
    bool do_erase = false;
    {
      std::lock_guard<std::mutex> lk(io_mutex_);
      // Push sensor data.
      bg_positions_  = local_positions;
      bg_velocities_ = local_velocities;
      bg_efforts_    = local_efforts;
      // Pull command data.
      local_pos_cmds = bg_pos_cmds_;
      local_vel_cmds = bg_vel_cmds_;
      local_pos_active = bg_pos_active_;
      local_vel_active = bg_vel_active_;
      do_erase = bg_erase_trajectories_;
      bg_erase_trajectories_ = false;
    }

    // ── 5. Handle trajectory erase request (position mode switch) ───────────
    if (do_erase) {
      MyEraseAllTrajectories();
      RCLCPP_INFO(LOGGER, "[usb_thread] Trajectory FIFO erased (mode switch).");
    }

    // ── 6. Send commands ────────────────────────────────────────────────────
    const auto t_wr0 = std::chrono::steady_clock::now();

    if (local_pos_active) {
      // Check that the controller has written a full set of commands.
      bool all_valid = true;
      for (const auto & v : local_pos_cmds) {
        if (std::isnan(v)) { all_valid = false; break; }
      }

      if (all_valid) {
        ++write_counter;

        // ── P-controller: position error → velocity command ─────────────────
        // Works in unwrapped rad-space so continuous joints crossing 0°/360°
        // are handled naturally — no degree-space wrapping issues.
        // Sends ANGULAR_VELOCITY every cycle (~100 Hz) — streaming, no FIFO.
        static constexpr double KP = 5.0;            // (rad/s) per rad error
        static constexpr float MAX_VEL_DEG = 100.0f; // deg/s clamp per joint

        double errors[6];
        float vel_deg[6];
        for (size_t i = 0; i < n; ++i) {
          // Commands from JTAC are in (−π, π] (MoveIt-normalised).
          // local_positions are unwrapped (can drift far from that range).
          // Bring the command into the unwrapped frame so the error is small
          // and the P-controller follows the shortest path faithfully.
          double cmd_adj = local_pos_cmds[i];
          if (WRAP[i]) {
            double d = cmd_adj - local_positions[i];
            while (d >  M_PI) { d -= 2.0 * M_PI; cmd_adj -= 2.0 * M_PI; }
            while (d < -M_PI) { d += 2.0 * M_PI; cmd_adj += 2.0 * M_PI; }
          }
          errors[i] = cmd_adj - local_positions[i];
          float vd = static_cast<float>(KP * errors[i] * RAD_TO_DEG);
          if (vd >  MAX_VEL_DEG) vd =  MAX_VEL_DEG;
          if (vd < -MAX_VEL_DEG) vd = -MAX_VEL_DEG;
          vel_deg[i] = vd;
        }

        TrajectoryPoint cmd;
        cmd.InitStruct();
        cmd.Position.Type                = ANGULAR_VELOCITY;
        cmd.Position.Actuators.Actuator1 = vel_deg[0];
        cmd.Position.Actuators.Actuator2 = vel_deg[1];
        cmd.Position.Actuators.Actuator3 = vel_deg[2];
        cmd.Position.Actuators.Actuator4 = vel_deg[3];
        cmd.Position.Actuators.Actuator5 = vel_deg[4];
        cmd.Position.Actuators.Actuator6 = vel_deg[5];
        cmd.Position.Fingers.Finger1     = 0.0f;
        cmd.Position.Fingers.Finger2     = 0.0f;
        cmd.Position.Fingers.Finger3     = 0.0f;
        MySendBasicTrajectory(cmd);

        // Verbose log every 50th cycle (~2 Hz).
        if (write_counter % 50 == 0) {
          RCLCPP_INFO(LOGGER,
            "[usb_thread:POS] P-ctrl "
            "err=[%.4f %.4f %.4f %.4f %.4f %.4f] "
            "vel_cmd=[%.1f %.1f %.1f %.1f %.1f %.1f]dps "
            "cmd=[%.4f %.4f %.4f %.4f %.4f %.4f] "
            "pos=[%.4f %.4f %.4f %.4f %.4f %.4f]",
            errors[0], errors[1], errors[2],
            errors[3], errors[4], errors[5],
            vel_deg[0], vel_deg[1], vel_deg[2],
            vel_deg[3], vel_deg[4], vel_deg[5],
            local_pos_cmds[0], local_pos_cmds[1], local_pos_cmds[2],
            local_pos_cmds[3], local_pos_cmds[4], local_pos_cmds[5],
            local_positions[0], local_positions[1], local_positions[2],
            local_positions[3], local_positions[4], local_positions[5]);
        }
      } else {
        RCLCPP_INFO_THROTTLE(LOGGER, *rclcpp::Clock::make_shared(), 2000,
          "[usb_thread:POS] Waiting for valid commands (have NaN)");
      }
    } else if (local_vel_active) {
      TrajectoryPoint cmd;
      cmd.InitStruct();
      cmd.Position.Type                = ANGULAR_VELOCITY;
      cmd.Position.Actuators.Actuator1 = static_cast<float>(local_vel_cmds[0] * RAD_TO_DEG);
      cmd.Position.Actuators.Actuator2 = static_cast<float>(local_vel_cmds[1] * RAD_TO_DEG);
      cmd.Position.Actuators.Actuator3 = static_cast<float>(local_vel_cmds[2] * RAD_TO_DEG);
      cmd.Position.Actuators.Actuator4 = static_cast<float>(local_vel_cmds[3] * RAD_TO_DEG);
      cmd.Position.Actuators.Actuator5 = static_cast<float>(local_vel_cmds[4] * RAD_TO_DEG);
      cmd.Position.Actuators.Actuator6 = static_cast<float>(local_vel_cmds[5] * RAD_TO_DEG);
      cmd.Position.Fingers.Finger1     = 0.0f;
      cmd.Position.Fingers.Finger2     = 0.0f;
      cmd.Position.Fingers.Finger3     = 0.0f;
      MySendBasicTrajectory(cmd);
    } else {
      // No controller active — send zero velocity to hold arm stationary.
      TrajectoryPoint idle;
      idle.InitStruct();
      idle.Position.Type = ANGULAR_VELOCITY;
      MySendBasicTrajectory(idle);
    }

    const auto t_wr1 = std::chrono::steady_clock::now();
    const double ms_wr = std::chrono::duration<double, std::milli>(t_wr1 - t_wr0).count();

    // ── 7. Update diagnostics ───────────────────────────────────────────────
    const auto t_end = std::chrono::steady_clock::now();
    const double ms_cycle = std::chrono::duration<double, std::milli>(t_end - t_cycle).count();

    {
      std::lock_guard<std::mutex> lk(io_mutex_);
      bg_diag_.read_pos_ms = ms_pos;
      bg_diag_.read_vel_ms = ms_vel;
      bg_diag_.read_eff_ms = ms_eff;
      bg_diag_.write_ms    = ms_wr;
      bg_diag_.cycle_ms    = ms_cycle;
      bg_diag_.cycle_count++;
      if (ms_cycle > 50.0) bg_diag_.slow_cycles++;
    }

    // Warn on very slow cycles (> 100 ms = 10× target period).
    if (ms_cycle > 100.0) {
      RCLCPP_WARN(LOGGER,
        "[usb_thread] SLOW cycle: %.1f ms "
        "(pos=%.1f vel=%.1f eff=%.1f wr=%.1f)",
        ms_cycle, ms_pos, ms_vel, ms_eff, ms_wr);
    }

    // Periodic state log from the USB thread (~every 5 s).
    if (bg_diag_.cycle_count % 500 == 0) {
      RCLCPP_INFO(LOGGER,
        "[usb_thread] pos(deg)=[%.1f %.1f %.1f %.1f %.1f %.1f] "
        "cmd=[%.3f %.3f %.3f %.3f %.3f %.3f] "
        "mode=%s cycle=%.1fms n=%" PRIu64 " slow=%" PRIu64,
        rp[0], rp[1], rp[2], rp[3], rp[4], rp[5],
        local_pos_cmds[0], local_pos_cmds[1], local_pos_cmds[2],
        local_pos_cmds[3], local_pos_cmds[4], local_pos_cmds[5],
        local_pos_active ? "POS" : (local_vel_active ? "VEL" : "IDLE"),
        ms_cycle, bg_diag_.cycle_count, bg_diag_.slow_cycles);
    }

    // Target ~100 Hz.  Sleep if we finished early.
    static constexpr auto TARGET_PERIOD = std::chrono::milliseconds(10);
    const auto elapsed = std::chrono::steady_clock::now() - t_cycle;
    if (elapsed < TARGET_PERIOD) {
      std::this_thread::sleep_for(TARGET_PERIOD - elapsed);
    }
  }

  // Send zero velocity before exiting so the arm doesn't drift.
  TrajectoryPoint idle;
  idle.InitStruct();
  idle.Position.Type = ANGULAR_VELOCITY;
  MySendBasicTrajectory(idle);

  RCLCPP_INFO(LOGGER, "[usb_thread] Stopped (%" PRIu64 " cycles, %" PRIu64 " slow).",
    bg_diag_.cycle_count, bg_diag_.slow_cycles);
}

}  // namespace kinova_driver

PLUGINLIB_EXPORT_CLASS(
  kinova_driver::KinovaSystemHardware,
  hardware_interface::SystemInterface)
