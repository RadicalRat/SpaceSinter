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

// kinova_system_hardware.cpp — ros2_control hardware plugin for Kinova Gen2 j2n6s300.
// Author: Cameron Hinkle, Colorado School of Mines
//
// See kinova_system_hardware.hpp for the architecture and parameter list.
//
// ── Joint conventions ─────────────────────────────────────────────────────────
// URDF joint order must match actuator order (joint_1 → Actuator1, ...).
// Joints 1,4,5,6 are continuous: reported in (−π, π] to match MoveIt, and the
// P-controller takes the shortest path around the circle (JTC must use
// angle_wraparound: true).  Joints 2,3 are revolute with limits inside
// (0, 2π) and are reported in the firmware's native range.
//
// ── USB setup (host, run once before first use) ───────────────────────────────
//   sudo cp <pkg>/udev/10-kinova-arm.rules /etc/udev/rules.d/
//   sudo udevadm control --reload-rules && sudo udevadm trigger

#include "kinova_driver/kinova_system_hardware.hpp"

#include <dlfcn.h>
#include <libgen.h>

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

#include "kinova_driver/joint_math.hpp"

namespace kinova_driver
{

namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("KinovaSystemHardware");

/// Continuous (unlimited) joints of the j2n6s300, in actuator order.
constexpr std::array<bool, kNumJoints> kContinuous = {true, false, false, true, true, true};

/// Any object inside this .so; dladdr() on it yields the plugin's install path.
const int kPluginAnchor = 0;

constexpr const char * kUdevHint =
  "Check the USB cable and that the udev rule is installed on the host:\n"
  "  sudo cp <kinova_driver share>/udev/10-kinova-arm.rules /etc/udev/rules.d/\n"
  "  sudo udevadm control --reload-rules && sudo udevadm trigger";

bool pluginLibDir(std::string & dir)
{
  Dl_info info;
  if (dladdr(&kPluginAnchor, &info) == 0 || info.dli_fname == nullptr) {
    return false;
  }
  std::string path(info.dli_fname);
  dir = dirname(path.data());
  return true;
}

bool allZero(const JointArrayF & a)
{
  return std::all_of(a.begin(), a.end(), [](float v) {return v == 0.0f;});
}

bool allFinite(const JointArray & a)
{
  return std::all_of(a.begin(), a.end(), [](double v) {return std::isfinite(v);});
}

double msSince(std::chrono::steady_clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

const char * modeName(int mode)
{
  static const char * names[] = {"IDLE", "POSITION", "VELOCITY"};
  return names[mode];
}

}  // namespace

KinovaSystemHardware::~KinovaSystemHardware()
{
  // controller_manager may destroy us without walking the lifecycle down
  // (e.g. on a crash in another component).  A joinable std::thread in a
  // destructor calls std::terminate, so always stop it here.
  stopIoThread();
  closeApi();
}

// ─────────────────────────────────────────────────────────────────────────────
// on_init — validate URDF, parse parameters, load the vendor SDK
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn KinovaSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!validateJoints() || !parseParams()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // The vendor .so files are installed next to this plugin.
  std::string lib_dir;
  if (!pluginLibDir(lib_dir)) {
    RCLCPP_ERROR(kLogger, "dladdr failed — cannot locate the plugin install directory.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::string error;
  sdk_ = KinovaSdk::load(lib_dir, error);
  if (!sdk_) {
    RCLCPP_ERROR(kLogger, "Failed to load Kinova SDK from '%s': %s", lib_dir.c_str(), error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  constexpr double nan = std::numeric_limits<double>::quiet_NaN();
  hw_positions_.fill(nan);
  hw_velocities_.fill(0.0);
  hw_efforts_.fill(0.0);
  hw_pos_cmds_.fill(nan);
  hw_vel_cmds_.fill(0.0);

  RCLCPP_INFO(kLogger, "Initialized: Kp=%.2f, max vel=%.1f deg/s, I/O rate=%.0f Hz, "
    "watchdog=%ld ms.", params_.position_kp, params_.max_joint_velocity_deg,
    params_.io_rate_hz, static_cast<long>(params_.command_timeout.count()));
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool KinovaSystemHardware::validateJoints() const
{
  if (info_.joints.size() != kNumJoints) {
    RCLCPP_ERROR(kLogger, "Expected %zu joints in the ros2_control tag, got %zu.",
      kNumJoints, info_.joints.size());
    return false;
  }
  auto has = [](const std::vector<hardware_interface::InterfaceInfo> & ifaces,
      const std::string & name) {
      return std::any_of(ifaces.begin(), ifaces.end(),
               [&](const auto & i) {return i.name == name;});
    };
  for (const auto & joint : info_.joints) {
    for (const char * type : {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY}) {
      if (!has(joint.command_interfaces, type)) {
        RCLCPP_ERROR(kLogger, "Joint '%s' is missing command interface '%s'.",
          joint.name.c_str(), type);
        return false;
      }
    }
    for (const char * type : {hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT})
    {
      if (!has(joint.state_interfaces, type)) {
        RCLCPP_ERROR(kLogger, "Joint '%s' is missing state interface '%s'.",
          joint.name.c_str(), type);
        return false;
      }
    }
  }
  return true;
}

bool KinovaSystemHardware::parseParams()
{
  const auto & hp = info_.hardware_parameters;
  auto get = [&](const std::string & key, double & out) {
      const auto it = hp.find(key);
      if (it == hp.end()) {
        return true;  // keep default
      }
      try {
        out = std::stod(it->second);
      } catch (const std::exception &) {
        RCLCPP_ERROR(kLogger, "Parameter '%s' = '%s' is not a number.",
          key.c_str(), it->second.c_str());
        return false;
      }
      if (!(out > 0.0)) {
        RCLCPP_ERROR(kLogger, "Parameter '%s' must be > 0 (got %s).",
          key.c_str(), it->second.c_str());
        return false;
      }
      return true;
    };

  if (const auto it = hp.find("serial_number"); it != hp.end()) {
    params_.serial_number = it->second;
  }

  double effort_decimation = params_.effort_decimation;
  double timeout_ms = static_cast<double>(params_.command_timeout.count());
  double max_errors = params_.max_consecutive_io_errors;
  const bool ok =
    get("position_kp", params_.position_kp) &&
    get("max_joint_velocity_deg", params_.max_joint_velocity_deg) &&
    get("io_rate_hz", params_.io_rate_hz) &&
    get("effort_decimation", effort_decimation) &&
    get("command_timeout_ms", timeout_ms) &&
    get("max_consecutive_io_errors", max_errors) &&
    get("device_discovery_timeout_s", params_.device_discovery_timeout_s);
  if (!ok) {
    return false;
  }
  params_.effort_decimation = static_cast<unsigned>(std::max(1.0, std::round(effort_decimation)));
  params_.command_timeout = std::chrono::milliseconds(static_cast<int64_t>(timeout_ms));
  params_.max_consecutive_io_errors = static_cast<unsigned>(std::max(1.0, std::round(max_errors)));
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Interface export
// ─────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
KinovaSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i = 0; i < kNumJoints; ++i) {
    const auto & name = info_.joints[i].name;
    si.emplace_back(name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    si.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    si.emplace_back(name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface>
KinovaSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i = 0; i < kNumJoints; ++i) {
    const auto & name = info_.joints[i].name;
    ci.emplace_back(name, hardware_interface::HW_IF_POSITION, &hw_pos_cmds_[i]);
    ci.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_vel_cmds_[i]);
  }
  return ci;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_configure — open the SDK and select the arm
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn KinovaSystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!connectToArm()) {
    closeApi();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool KinovaSystemHardware::connectToArm()
{
  const int init_rc = sdk_->initApi();
  if (!KinovaSdk::ok(init_rc)) {
    RCLCPP_ERROR(kLogger, "InitAPI failed (code %d). %s", init_rc, kUdevHint);
    return false;
  }
  api_initialized_ = true;

  // The SDK enumerates on its own background thread; poll until the arm we
  // want shows up.
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(params_.device_discovery_timeout_s);
  std::vector<KinovaDevice> devices;
  const KinovaDevice * selected = nullptr;
  while (selected == nullptr && std::chrono::steady_clock::now() < deadline) {
    sdk_->getDevices(devices);
    for (const auto & d : devices) {
      if (params_.serial_number.empty() ||
        std::strncmp(d.SerialNumber, params_.serial_number.c_str(), SERIAL_LENGTH) == 0)
      {
        selected = &d;
        break;
      }
    }
    if (selected == nullptr) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  if (selected == nullptr) {
    std::ostringstream found;
    for (const auto & d : devices) {
      found << " '" << d.SerialNumber << "'";
    }
    RCLCPP_ERROR(kLogger, "No Kinova arm%s%s found after %.1f s (found:%s). %s",
      params_.serial_number.empty() ? "" : " with serial ",
      params_.serial_number.c_str(), params_.device_discovery_timeout_s,
      devices.empty() ? " none" : found.str().c_str(), kUdevHint);
    return false;
  }
  if (devices.size() > 1 && params_.serial_number.empty()) {
    RCLCPP_WARN(kLogger, "%zu Kinova arms found; using the first. Set the 'serial_number' "
      "hardware parameter to choose one explicitly.", devices.size());
  }

  const int active_rc = sdk_->setActiveDevice(*selected);
  if (!KinovaSdk::ok(active_rc)) {
    RCLCPP_ERROR(kLogger, "SetActiveDevice('%s') failed (code %d).",
      selected->SerialNumber, active_rc);
    return false;
  }
  RCLCPP_INFO(kLogger, "Connected to arm: serial='%s' model='%s' firmware=%d.%d.%d",
    selected->SerialNumber, selected->Model,
    selected->VersionMajor, selected->VersionMinor, selected->VersionRelease);

  JointArray pos;
  if (!readPositionsBlocking(pos)) {
    return false;
  }
  RCLCPP_INFO(kLogger, "USB OK: pos=[%.3f %.3f %.3f %.3f %.3f %.3f] rad",
    pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
  return true;
}

bool KinovaSystemHardware::readPositionsBlocking(JointArray & out) const
{
  JointArrayF raw{};
  const int rc = sdk_->getJointPositionsDeg(raw);
  if (!KinovaSdk::ok(rc)) {
    RCLCPP_ERROR(kLogger, "GetAngularPosition failed (code %d). %s", rc, kUdevHint);
    return false;
  }
  // Device enumerated but every reading is exactly 0: libusb can open the
  // device but transfers silently fail — the classic missing-udev-rule symptom.
  if (allZero(raw)) {
    RCLCPP_ERROR(kLogger, "All joint positions read as 0 — USB transfers are failing. %s",
      kUdevHint);
    return false;
  }
  for (size_t i = 0; i < kNumJoints; ++i) {
    out[i] = kinovaPositionToRad(raw[i], kContinuous[i]);
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_activate / on_deactivate
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn KinovaSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  int rc = sdk_->startControlApi();
  if (!KinovaSdk::ok(rc)) {
    RCLCPP_ERROR(kLogger, "StartControlAPI failed (code %d).", rc);
    return hardware_interface::CallbackReturn::ERROR;
  }
  control_api_started_ = true;

  rc = sdk_->setAngularControl();
  if (!KinovaSdk::ok(rc)) {
    RCLCPP_ERROR(kLogger, "SetAngularControl failed (code %d).", rc);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Seed state from a live read so controllers activate against real values,
  // and seed the position command to "hold here".
  if (!readPositionsBlocking(hw_positions_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  hw_velocities_.fill(0.0);
  hw_efforts_.fill(0.0);
  hw_pos_cmds_ = hw_positions_;
  hw_vel_cmds_.fill(0.0);
  mode_ = ControlMode::kIdle;
  pending_erase_ = false;

  {
    std::lock_guard<std::mutex> lk(io_mutex_);
    shared_sensors_ = SensorFrame{hw_positions_, hw_velocities_, hw_efforts_};
    shared_commands_ = CommandFrame{};
    shared_commands_.positions = hw_pos_cmds_;
    shared_commands_.stamp = std::chrono::steady_clock::now();
  }

  startIoThread();
  RCLCPP_INFO(kLogger, "Activated at pos=[%.3f %.3f %.3f %.3f %.3f %.3f] rad.",
    hw_positions_[0], hw_positions_[1], hw_positions_[2],
    hw_positions_[3], hw_positions_[4], hw_positions_[5]);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KinovaSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  stopIoThread();
  mode_ = ControlMode::kIdle;
  if (control_api_started_) {
    sdk_->eraseAllTrajectories();
    sdk_->stopControlApi();
    control_api_started_ = false;
  }
  RCLCPP_INFO(kLogger, "Deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_cleanup / on_shutdown / on_error
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn KinovaSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  closeApi();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KinovaSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  on_deactivate(state);
  closeApi();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KinovaSystemHardware::on_error(
  const rclcpp_lifecycle::State & state)
{
  RCLCPP_ERROR(kLogger, "Hardware error — stopping arm and closing the SDK.");
  // SUCCESS returns the component to UNCONFIGURED, from which it can be
  // reconfigured; leave nothing open behind us.
  on_deactivate(state);
  closeApi();
  io_fault_.store(false);
  return hardware_interface::CallbackReturn::SUCCESS;
}

void KinovaSystemHardware::closeApi()
{
  if (sdk_ && api_initialized_) {
    sdk_->closeApi();
    api_initialized_ = false;
    RCLCPP_INFO(kLogger, "SDK closed.");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Command mode switching
//
// Position and velocity are mutually exclusive across the whole arm: the I/O
// thread streams one velocity vector per cycle derived from exactly one of
// them.
// ─────────────────────────────────────────────────────────────────────────────
bool KinovaSystemHardware::ownsInterface(
  const std::string & full_name, const std::string & type) const
{
  return std::any_of(info_.joints.begin(), info_.joints.end(),
           [&](const auto & j) {return full_name == j.name + "/" + type;});
}

bool KinovaSystemHardware::resolveMode(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces,
  ControlMode & mode) const
{
  using hardware_interface::HW_IF_POSITION;
  using hardware_interface::HW_IF_VELOCITY;

  mode = mode_;
  for (const auto & iface : stop_interfaces) {
    if ((mode == ControlMode::kPosition && ownsInterface(iface, HW_IF_POSITION)) ||
      (mode == ControlMode::kVelocity && ownsInterface(iface, HW_IF_VELOCITY)))
    {
      mode = ControlMode::kIdle;
    }
  }

  bool start_pos = false;
  bool start_vel = false;
  for (const auto & iface : start_interfaces) {
    start_pos |= ownsInterface(iface, HW_IF_POSITION);
    start_vel |= ownsInterface(iface, HW_IF_VELOCITY);
  }
  if (start_pos && start_vel) {
    // A single controller (e.g. a trajectory controller with position +
    // velocity command interfaces) may claim both.  Track position; its
    // velocity commands are feed-forward we don't use.
    RCLCPP_WARN(kLogger, "Position and velocity claimed together — using position control, "
      "velocity commands will be ignored.");
    start_vel = false;
  }
  if ((start_pos || start_vel) && mode != ControlMode::kIdle) {
    RCLCPP_ERROR(kLogger, "Cannot start %s control while %s control is still active.",
      start_pos ? "position" : "velocity", modeName(static_cast<int>(mode)));
    return false;
  }
  if (start_pos) {
    mode = ControlMode::kPosition;
  } else if (start_vel) {
    mode = ControlMode::kVelocity;
  }
  return true;
}

hardware_interface::return_type KinovaSystemHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  ControlMode mode;
  return resolveMode(start_interfaces, stop_interfaces, mode) ?
         hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type KinovaSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  ControlMode mode;
  if (!resolveMode(start_interfaces, stop_interfaces, mode)) {
    return hardware_interface::return_type::ERROR;
  }
  if (mode == mode_) {
    return hardware_interface::return_type::OK;
  }

  // Start every new mode from "hold still" so a controller that is slow to
  // write its first command can't inherit a stale target.
  if (mode == ControlMode::kPosition) {
    hw_pos_cmds_ = hw_positions_;
  } else if (mode == ControlMode::kVelocity) {
    hw_vel_cmds_.fill(0.0);
  }
  RCLCPP_INFO(kLogger, "Control mode %s → %s.",
    modeName(static_cast<int>(mode_)), modeName(static_cast<int>(mode)));
  mode_ = mode;
  pending_erase_ = true;  // flush anything queued in the arm's trajectory FIFO
  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// read / write — non-blocking exchange with the I/O thread
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type KinovaSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (io_fault_.load(std::memory_order_acquire)) {
    RCLCPP_ERROR_THROTTLE(kLogger, steady_clock_, 1000,
      "USB I/O thread faulted — reporting hardware error.");
    return hardware_interface::return_type::ERROR;
  }
  std::lock_guard<std::mutex> lk(io_mutex_);
  hw_positions_ = shared_sensors_.positions;
  hw_velocities_ = shared_sensors_.velocities;
  hw_efforts_ = shared_sensors_.efforts;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KinovaSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lk(io_mutex_);
  shared_commands_.positions = hw_pos_cmds_;
  shared_commands_.velocities = hw_vel_cmds_;
  shared_commands_.mode = mode_;
  shared_commands_.stamp = std::chrono::steady_clock::now();
  shared_commands_.erase_trajectories |= pending_erase_;
  pending_erase_ = false;
  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// I/O thread
// ─────────────────────────────────────────────────────────────────────────────
void KinovaSystemHardware::startIoThread()
{
  stopIoThread();
  io_fault_.store(false, std::memory_order_release);
  io_running_.store(true, std::memory_order_release);
  io_thread_ = std::thread(&KinovaSystemHardware::ioLoop, this);
}

void KinovaSystemHardware::stopIoThread()
{
  io_running_.store(false, std::memory_order_release);
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

JointArrayF KinovaSystemHardware::computeVelocityCommand(
  const CommandFrame & cmd, const SensorFrame & state,
  std::chrono::steady_clock::time_point now)
{
  JointArrayF vel_deg{};  // zero = hold
  if (cmd.mode == ControlMode::kIdle) {
    return vel_deg;
  }
  if (now - cmd.stamp > params_.command_timeout) {
    RCLCPP_WARN_THROTTLE(kLogger, steady_clock_, 1000,
      "No command from controller_manager for %.0f ms (watchdog %ld ms) — holding.",
      std::chrono::duration<double, std::milli>(now - cmd.stamp).count(),
      static_cast<long>(params_.command_timeout.count()));
    return vel_deg;
  }

  const double max_vel = params_.max_joint_velocity_deg;
  if (cmd.mode == ControlMode::kPosition) {
    if (!allFinite(cmd.positions)) {
      RCLCPP_WARN_THROTTLE(kLogger, steady_clock_, 2000,
        "Position command contains NaN/inf — holding.");
      return vel_deg;
    }
    JointArray err{};
    for (size_t i = 0; i < kNumJoints; ++i) {
      err[i] = positionError(cmd.positions[i], state.positions[i], kContinuous[i]);
      vel_deg[i] = positionErrorToVelocityDeg(err[i], params_.position_kp, max_vel);
    }
    RCLCPP_DEBUG_THROTTLE(kLogger, steady_clock_, 500,
      "[pos] err=[%.4f %.4f %.4f %.4f %.4f %.4f] vel=[%.1f %.1f %.1f %.1f %.1f %.1f] deg/s",
      err[0], err[1], err[2], err[3], err[4], err[5],
      vel_deg[0], vel_deg[1], vel_deg[2], vel_deg[3], vel_deg[4], vel_deg[5]);
  } else {
    if (!allFinite(cmd.velocities)) {
      RCLCPP_WARN_THROTTLE(kLogger, steady_clock_, 2000,
        "Velocity command contains NaN/inf — holding.");
      return vel_deg;
    }
    for (size_t i = 0; i < kNumJoints; ++i) {
      vel_deg[i] = velocityCommandToDeg(cmd.velocities[i], max_vel);
    }
  }
  return vel_deg;
}

void KinovaSystemHardware::ioLoop()
{
  using Clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / params_.io_rate_hz));
  const double period_ms = 1000.0 / params_.io_rate_hz;

  SensorFrame state;
  {
    std::lock_guard<std::mutex> lk(io_mutex_);
    state = shared_sensors_;
  }

  JointArrayF raw{};
  unsigned consecutive_errors = 0;
  uint64_t cycles = 0;
  uint64_t slow_cycles = 0;
  ControlMode last_mode = ControlMode::kIdle;
  auto next_wakeup = Clock::now();

  RCLCPP_INFO(kLogger, "[io] Started at %.0f Hz.", params_.io_rate_hz);

  while (io_running_.load(std::memory_order_acquire)) {
    const auto t_cycle = Clock::now();
    bool cycle_ok = true;
    int last_rc = NO_ERROR_KINOVA;

    // 1. Sensors.  On a failed read the previous values are kept: a single
    //    dropped sample is harmless, and sustained failure trips the fault
    //    below.  Never feed a zero-initialised struct forward as real data.
    if (int rc = sdk_->getJointPositionsDeg(raw); KinovaSdk::ok(rc)) {
      for (size_t i = 0; i < kNumJoints; ++i) {
        state.positions[i] = kinovaPositionToRad(raw[i], kContinuous[i]);
      }
    } else {
      cycle_ok = false;
      last_rc = rc;
    }
    if (int rc = sdk_->getJointVelocitiesRaw(raw); KinovaSdk::ok(rc)) {
      for (size_t i = 0; i < kNumJoints; ++i) {
        state.velocities[i] = kinovaVelocityToRad(raw[i]);
      }
    } else {
      cycle_ok = false;
      last_rc = rc;
    }
    if (cycles % params_.effort_decimation == 0) {
      if (int rc = sdk_->getJointTorquesGravityFree(raw); KinovaSdk::ok(rc)) {
        std::copy(raw.begin(), raw.end(), state.efforts.begin());
      } else {
        cycle_ok = false;
        last_rc = rc;
      }
    }

    // 2. Exchange with the controller_manager thread.
    CommandFrame cmd;
    {
      std::lock_guard<std::mutex> lk(io_mutex_);
      shared_sensors_ = state;
      cmd = shared_commands_;
      shared_commands_.erase_trajectories = false;
    }

    // 3. Commands.
    if (cmd.erase_trajectories) {
      sdk_->eraseAllTrajectories();
    }
    if (cmd.mode != last_mode) {
      RCLCPP_INFO(kLogger, "[io] Streaming mode: %s.", modeName(static_cast<int>(cmd.mode)));
      last_mode = cmd.mode;
    }
    if (int rc = sdk_->sendJointVelocitiesDeg(computeVelocityCommand(cmd, state, t_cycle));
      !KinovaSdk::ok(rc))
    {
      cycle_ok = false;
      last_rc = rc;
    }

    // 4. Fault handling.
    if (cycle_ok) {
      consecutive_errors = 0;
    } else if (++consecutive_errors >= params_.max_consecutive_io_errors) {
      RCLCPP_ERROR(kLogger, "[io] %u consecutive USB errors (last code %d) — stopping arm "
        "and faulting.", consecutive_errors, last_rc);
      io_fault_.store(true, std::memory_order_release);
      break;
    } else {
      RCLCPP_WARN_THROTTLE(kLogger, steady_clock_, 1000,
        "[io] USB call failed (code %d), %u in a row.", last_rc, consecutive_errors);
    }

    // 5. Timing.
    const double cycle_ms = msSince(t_cycle);
    ++cycles;
    if (cycle_ms > 5.0 * period_ms) {
      ++slow_cycles;
      RCLCPP_WARN_THROTTLE(kLogger, steady_clock_, 1000,
        "[io] Slow cycle: %.1f ms (target %.1f ms).", cycle_ms, period_ms);
    }
    RCLCPP_INFO_THROTTLE(kLogger, steady_clock_, 5000,
      "[io] mode=%s pos=[%.3f %.3f %.3f %.3f %.3f %.3f] rad cycle=%.1f ms "
      "n=%" PRIu64 " slow=%" PRIu64,
      modeName(static_cast<int>(cmd.mode)),
      state.positions[0], state.positions[1], state.positions[2],
      state.positions[3], state.positions[4], state.positions[5],
      cycle_ms, cycles, slow_cycles);

    // Fixed-rate schedule; if we overran, restart from now instead of bursting.
    next_wakeup += period;
    const auto now = Clock::now();
    if (next_wakeup < now) {
      next_wakeup = now;
    }
    std::this_thread::sleep_until(next_wakeup);
  }

  // Best effort: leave the arm commanded to zero velocity.
  sdk_->sendJointVelocitiesDeg(JointArrayF{});
  RCLCPP_INFO(kLogger, "[io] Stopped after %" PRIu64 " cycles (%" PRIu64 " slow).",
    cycles, slow_cycles);
}

}  // namespace kinova_driver

PLUGINLIB_EXPORT_CLASS(
  kinova_driver::KinovaSystemHardware,
  hardware_interface::SystemInterface)
