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
 * kinova_arm.cpp
 *
 * Author: Cameron Hinkle, Colorado School of Mines
 *
 * ros2_control hardware plugin for the Kinova Gen2 j2n6s300 6-DOF arm.
 *
 * API usage follows the official Kinova SDK example scripts:
 *   kinova_ang_control.cpp   — ANGULAR_VELOCITY streaming via SendBasicTrajectory
 *   kinova_get_ang.cpp       — GetAngularPosition / GetAngularVelocity (note: ×2 correction needed)
 *   kinova_force_control.cpp — GetAngularForceGravityFree
 *
 * Velocity readback correction:
 *   GetAngularVelocity returns half the actual velocity (firmware quirk documented
 *   in kinova_comm.cpp line 623: "velocities reported back by firmware seem to be
 *   half of actual value").  All velocity reads are multiplied by 2.0.
 *
 * Threading model:
 *   A SCHED_OTHER background thread (poller_loop) runs all USB GetAngular* calls at
 *   roughly 100 Hz (two USB reads per 5 ms polled cycle).  read() copies the shadow
 *   state under a mutex — zero USB I/O on the SCHED_FIFO RT thread.
 *   write() calls SendBasicTrajectory which enqueues to the arm's internal FIFO and
 *   returns in microseconds, safe for the 200 Hz control loop.
 */

#include "kinova_ros/kinova_arm.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <dlfcn.h>
#include <libgen.h>
#include <stdlib.h>
#include <chrono>
#include <limits>
#include <string>
#include <thread>
#include <vector>

// Anchor symbol: dladdr() uses this to locate the path of this .so at runtime.
extern "C" void kinova_driver__dladdr_anchor() {}

namespace kinova_driver
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("KinovaSystemHardware");

// ---------------------------------------------------------------------------
// on_init
//   Load vendor libraries via dlopen and resolve all API function pointers.
//
//   The dlopen strategy follows the official example scripts (single RTLD_GLOBAL
//   load of the command layer), adapted for the ROS plugin context where the
//   vendor .so files live in the install/<pkg>/lib directory which is NOT on
//   the system's default ldconfig/LD_LIBRARY_PATH, but IS on the ROS sourced
//   LD_LIBRARY_PATH.  We use dladdr to find our own install dir and prepend it
//   to LD_LIBRARY_PATH so the command layer's internal
//     dlopen("USBCommLayerUbuntu.so")   ← short name
//   can locate the vendor comm .so.  We also preload the comm layer by full path
//   first so the internal call returns the already-loaded inode.
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KinovaSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Find the directory where this plugin .so was installed.
  Dl_info dl_info;
  if (dladdr(reinterpret_cast<void *>(kinova_driver__dladdr_anchor), &dl_info) == 0) {
    RCLCPP_ERROR(LOGGER, "dladdr failed — cannot locate plugin library path");
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::string plugin_path(dl_info.dli_fname);
  std::vector<char> path_buf(plugin_path.begin(), plugin_path.end());
  path_buf.push_back('\0');
  const std::string lib_dir = dirname(path_buf.data());
  RCLCPP_INFO(LOGGER, "Kinova vendor libs expected in: %s", lib_dir.c_str());

  // Prepend lib_dir to LD_LIBRARY_PATH so the command layer's internal short-name
  // dlopen("USBCommLayerUbuntu.so") finds the vendor copy in our install dir.
  {
    const char * existing = getenv("LD_LIBRARY_PATH");
    const std::string new_path =
      lib_dir + (existing ? (std::string(":") + existing) : std::string());
    setenv("LD_LIBRARY_PATH", new_path.c_str(), /*overwrite=*/1);
  }

  // Pre-load the comm layer by full path with RTLD_GLOBAL.  When InitAPI's
  // internal dlopen("USBCommLayerUbuntu.so") fires, glibc matches the same
  // inode already in memory and returns the existing handle, preventing a
  // duplicate load which would leave the internal commLayer_Handle pointer NULL.
  const std::string comm_path = lib_dir + "/USBCommLayerUbuntu.so";
  commLayer_handle_ = dlopen(comm_path.c_str(), RTLD_NOW | RTLD_GLOBAL);
  if (!commLayer_handle_) {
    RCLCPP_ERROR(LOGGER, "dlopen comm layer failed (%s): %s", comm_path.c_str(), dlerror());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Load the command layer with RTLD_GLOBAL — matches the example scripts.
  const std::string cmd_path = lib_dir + "/USBCommandLayerUbuntu.so";
  commandLayer_handle_ = dlopen(cmd_path.c_str(), RTLD_NOW | RTLD_GLOBAL);
  if (!commandLayer_handle_) {
    RCLCPP_ERROR(LOGGER, "dlopen command layer failed (%s): %s", cmd_path.c_str(), dlerror());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(LOGGER, "Kinova SDK libraries loaded from %s", lib_dir.c_str());

  // Resolve API symbols from the command layer handle explicitly.
  // Do NOT use RTLD_DEFAULT — both .so files export identically-named symbols
  // (GetDevices, SetActiveDevice, …) with completely different semantics.
  // Since the comm layer was loaded with RTLD_GLOBAL, RTLD_DEFAULT would
  // resolve to the low-level USB comm variants, not the robot API functions.
  dlerror();
#define LOAD_SYM(ptr, sig, name) \
  ptr = reinterpret_cast<sig>(dlsym(commandLayer_handle_, (name))); \
  if (!(ptr)) { \
    RCLCPP_ERROR(LOGGER, "Symbol '%s' not found in command layer: %s", (name), dlerror()); \
    return hardware_interface::CallbackReturn::ERROR; \
  }

  LOAD_SYM(MyInitAPI,                  int(*)(),                       "InitAPI")
  LOAD_SYM(MyCloseAPI,                 int(*)(),                       "CloseAPI")
  LOAD_SYM(MyGetDevices,               int(*)(KinovaDevice[], int &),  "GetDevices")
  LOAD_SYM(MySetActiveDevice,          int(*)(KinovaDevice),           "SetActiveDevice")
  LOAD_SYM(MyMoveHome,                 int(*)(),                       "MoveHome")
  LOAD_SYM(MyInitFingers,              int(*)(),                       "InitFingers")
  LOAD_SYM(MySendBasicTrajectory,      int(*)(TrajectoryPoint),        "SendBasicTrajectory")
  LOAD_SYM(MySendAdvanceTrajectory,    int(*)(TrajectoryPoint),        "SendAdvanceTrajectory")
  LOAD_SYM(MyGetAngularPosition,       int(*)(AngularPosition &),      "GetAngularPosition")
  LOAD_SYM(MyGetAngularVelocity,       int(*)(AngularPosition &),      "GetAngularVelocity")
  LOAD_SYM(MyGetAngularCommand,        int(*)(AngularPosition &),      "GetAngularCommand")
  LOAD_SYM(MyGetAngularForce,          int(*)(AngularPosition &),      "GetAngularForce")
  LOAD_SYM(MyGetAngularForceGravityFree, int(*)(AngularPosition &),    "GetAngularForceGravityFree")
  LOAD_SYM(MyStartControlAPI,          int(*)(),                       "StartControlAPI")
  LOAD_SYM(MyStopControlAPI,           int(*)(),                       "StopControlAPI")
  LOAD_SYM(MySetAngularControl,        int(*)(),                       "SetAngularControl")
  LOAD_SYM(MyRefresDevicesList,        int(*)(),                       "RefresDevicesList")
  LOAD_SYM(MyEraseAllTrajectories,     int(*)(),                       "EraseAllTrajectories")

#undef LOAD_SYM

  const size_t n = info_.joints.size();
  hw_positions_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.assign(n, 0.0);
  hw_commands_.assign(n, 0.0);
  hw_position_commands_.assign(n, std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(LOGGER, "I N I T I A L I Z A T I O N   C O M P L E T E D  (%zu joints)", n);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// export_state_interfaces / export_command_interfaces
// ---------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
KinovaSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.reserve(info_.joints.size() * 3);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,  &hw_positions_[i]);
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,  &hw_velocities_[i]);
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,    &hw_efforts_[i]);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface>
KinovaSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.reserve(info_.joints.size() * 2);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]);
    ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return ci;
}

// ---------------------------------------------------------------------------
// on_configure
//   Initialise the Kinova SDK and wait for USB device enumeration.
//   Mirrors the init sequence from the official example scripts:
//     result = MyInitAPI();
//     devicesCount = MyGetDevices(list, result);
//     MySetActiveDevice(list[i]);
//
//   IMPORTANT — USB permissions:
//   The Kinova Gen2 USB device defaults to root-only access.  The udev rule
//   `10-kinova-arm.rules` must be installed on the HOST machine so that
//   non-root processes (and Docker containers with /dev:/dev) can open it:
//
//     SUBSYSTEM=="usb", ATTR{idVendor}=="22cd", MODE:="666"
//
//   Install steps (host, run once):
//     sudo cp <pkg>/udev/10-kinova-arm.rules /etc/udev/rules.d/
//     sudo udevadm control --reload-rules && sudo udevadm trigger
//
//   Without this rule, InitAPI() returns NO_ERROR_KINOVA (USB enumeration
//   succeeds) but GetAngular* calls return all zeros silently because libusb
//   cannot actually open the device handle.
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KinovaSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  int result = MyInitAPI();
  RCLCPP_INFO(LOGGER, "InitAPI result: %d  (NO_ERROR_KINOVA = %d)", result, NO_ERROR_KINOVA);

  if (result != NO_ERROR_KINOVA) {
    RCLCPP_ERROR(LOGGER,
      "InitAPI FAILED with code %d.  The USB comm layer could not be initialised.\n"
      "Most likely cause: missing udev rule. Install it on the HOST machine:\n"
      "  sudo cp <kinova_driver_pkg>/udev/10-kinova-arm.rules /etc/udev/rules.d/\n"
      "  sudo udevadm control --reload-rules && sudo udevadm trigger\n"
      "Then replug the USB cable and restart.",
      result);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // The USB background thread launched by InitAPI may take up to ~300 ms to
  // enumerate devices.  Poll RefresDevicesList + GetDevices until an arm
  // appears (max 3 s), matching the robustness pattern in kinova-ros upstream.
  KinovaDevice list[MAX_KINOVA_DEVICE];
  int count = 0;
  for (int attempt = 0; attempt < 30; ++attempt) {
    MyRefresDevicesList();
    count = MyGetDevices(list, result);
    RCLCPP_INFO(LOGGER, "  GetDevices attempt %2d: count=%d api_result=%d", attempt, count, result);
    if (count > 0) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (count <= 0) {
    RCLCPP_ERROR(LOGGER,
      "No Kinova device found on USB bus after 3 s.\n"
      "Check: USB cable connected, arm powered on, udev rule installed (see above).");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(LOGGER, "Found arm on USB bus — serial='%s'  model='%s'  type=%d  fw=%d.%d.%d",
    list[0].SerialNumber, list[0].Model, list[0].DeviceType,
    list[0].VersionMajor, list[0].VersionMinor, list[0].VersionRelease);

  MySetActiveDevice(list[0]);

  // ── USB data-flow sanity check ────────────────────────────────────────────
  // The Kinova home position is physically non-zero for every joint.  If all
  // six values come back as exactly 0.0, libusb opened the device entry but
  // cannot actually read from it (most likely: USB permission denied).
  // This is the clearest early-warning of the "all-zero states" symptom.
  AngularPosition test_pos{};
  MyGetAngularPosition(test_pos);

  const bool all_zero =
    (test_pos.Actuators.Actuator1 == 0.0f && test_pos.Actuators.Actuator2 == 0.0f &&
     test_pos.Actuators.Actuator3 == 0.0f && test_pos.Actuators.Actuator4 == 0.0f &&
     test_pos.Actuators.Actuator5 == 0.0f && test_pos.Actuators.Actuator6 == 0.0f);

  if (all_zero) {
    RCLCPP_ERROR(LOGGER,
      "GetAngularPosition returned all zeros immediately after SetActiveDevice.\n"
      "This means USB communication is broken even though the device was enumerated.\n"
      "\n"
      "ROOT CAUSE: missing udev permissions rule for the Kinova arm USB device.\n"
      "\n"
      "Fix (on the HOST machine — not inside Docker):\n"
      "  1. Create /etc/udev/rules.d/10-kinova-arm.rules with:\n"
      "       SUBSYSTEM==\"usb\", ATTR{idVendor}==\"22cd\", MODE:=\"666\"\n"
      "  2. sudo udevadm control --reload-rules\n"
      "  3. sudo udevadm trigger\n"
      "  4. Replug the USB cable, then restart MoveIt Pro.\n"
      "\n"
      "Alternative (Docker only): add 'privileged: true' to the robot service in\n"
      "docker-compose.yaml (less secure, but bypasses udev entirely).");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(LOGGER,
    "USB data verified: pos=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f] deg — looks good.",
    test_pos.Actuators.Actuator1, test_pos.Actuators.Actuator2, test_pos.Actuators.Actuator3,
    test_pos.Actuators.Actuator4, test_pos.Actuators.Actuator5, test_pos.Actuators.Actuator6);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_activate
//   Apply the standard kick-start sequence used in kinova_comm.cpp:
//     StartControlAPI → StopControlAPI → StartControlAPI
//   This clears any stale trajectory buffer and ensures the controller is in
//   a known ready state on both cold-boot and re-activation after suspend.
//   Then switch to angular (joint-space) control and launch the poller thread.
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KinovaSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  MyStartControlAPI();
  MyStopControlAPI();
  int result = MyStartControlAPI();
  RCLCPP_INFO(LOGGER, "StartControlAPI (kick-start) result: %d", result);

  MySetAngularControl();
  velocity_commands_active_ = false;
  position_commands_active_ = false;

  // Reset position shadows to NaN so the first read() populates them from
  // raw kinDegToRad (no stale wrapped/accumulated offset from a prior session).
  std::fill(hw_positions_.begin(), hw_positions_.end(),
            std::numeric_limits<double>::quiet_NaN());

  // Start the background USB polling thread (SCHED_OTHER).
  // Shadow buffers are zero-initialised; the poller fills them within ~10 ms.
  poller_running_.store(true);
  poller_thread_ = std::thread(&KinovaSystemHardware::poller_loop, this);

  RCLCPP_INFO(LOGGER, "Hardware activated — USB poller started.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate
//   Stop accepting velocity commands, shut down the poller, then stop the API.
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KinovaSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  velocity_commands_active_ = false;
  position_commands_active_ = false;

  poller_running_.store(false);
  if (poller_thread_.joinable()) {
    poller_thread_.join();
  }

  // Flush the trajectory FIFO so any queued position waypoints (especially
  // from an aborted trajectory) do not replay on the next activation.
  // kinova_comm::stopAPI() does the same: eraseAllTrajectories() then stopControlAPI().
  MyEraseAllTrajectories();
  MyStopControlAPI();
  RCLCPP_INFO(LOGGER, "Hardware deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_cleanup
//   Close the SDK and release the dlopen handles.
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KinovaSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  MyCloseAPI();

  if (commandLayer_handle_) {
    dlclose(commandLayer_handle_);
    commandLayer_handle_ = nullptr;
  }
  if (commLayer_handle_) {
    dlclose(commLayer_handle_);
    commLayer_handle_ = nullptr;
  }

  RCLCPP_INFO(LOGGER, "Kinova API closed and libraries unloaded.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// poller_loop
//   Runs in a dedicated SCHED_OTHER thread.  Performs all USB state reads and
//   stores results in shadow buffers protected by state_mutex_.  This keeps
//   the Kinova libUSB userspace polling loop entirely off the SCHED_FIFO RT
//   control thread, preventing USB blocking from causing RT deadline misses.
//
//   Rate target: ~100 Hz (3 USB reads ≈ 3–9 ms each cycle + 5 ms sleep).
//   Torque is read every 4th iteration (~25 Hz), since effort changes slowly
//   compared to position/velocity and this reduces USB bus load.
// ---------------------------------------------------------------------------
void KinovaSystemHardware::poller_loop()
{
  int iter = 0;
  while (poller_running_.load()) {
    AngularPosition pos, vel, torq;

    MyGetAngularPosition(pos);
    MyGetAngularVelocity(vel);

    // Read torque every 4th cycle to reduce USB bus utilisation.
    if (iter % 4 == 0) {
      MyGetAngularForceGravityFree(torq);
    }

    if (iter == 0 || iter % 100 == 0) {
      RCLCPP_INFO(
        LOGGER,
        "Poller [%d] pos(deg)=[%.1f %.1f %.1f %.1f %.1f %.1f]  vel=[%.2f %.2f %.2f %.2f %.2f %.2f]",
        iter,
        pos.Actuators.Actuator1, pos.Actuators.Actuator2, pos.Actuators.Actuator3,
        pos.Actuators.Actuator4, pos.Actuators.Actuator5, pos.Actuators.Actuator6,
        vel.Actuators.Actuator1, vel.Actuators.Actuator2, vel.Actuators.Actuator3,
        vel.Actuators.Actuator4, vel.Actuators.Actuator5, vel.Actuators.Actuator6);
    }

    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      pos_shadow_ = pos;
      vel_shadow_ = vel;
      if (iter % 4 == 0) {
        torq_shadow_ = torq;
      }
    }

    ++iter;
    // 5 ms sleep + ~6 ms for two USB reads ≈ ~90 Hz effective poll rate.
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  RCLCPP_INFO(LOGGER, "USB poller thread exiting.");
}

// ---------------------------------------------------------------------------
// prepare_command_mode_switch / perform_command_mode_switch
//   Track which controllers are active so write() can gate USB commands.
// ---------------------------------------------------------------------------
hardware_interface::return_type KinovaSystemHardware::prepare_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KinovaSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Process stops first, then starts.  If both contain the same interface
  // (e.g. velocity_force_controller stops while JTAC starts), the start
  // result wins, which is the correct behaviour.
  for (const auto & iface : stop_interfaces) {
    if (iface.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos) {
      velocity_commands_active_ = false;
    }
    if (iface.find(hardware_interface::HW_IF_POSITION) != std::string::npos) {
      position_commands_active_ = false;
    }
  }
  for (const auto & iface : start_interfaces) {
    if (iface.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos) {
      velocity_commands_active_ = true;
    }
    if (iface.find(hardware_interface::HW_IF_POSITION) != std::string::npos) {
      position_commands_active_ = true;
    }
  }
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// kinDegToRad / radToKinDeg
//   Paired conversion functions for the Kinova 2's-complement degree convention.
//
//   The Kinova Gen2 API uses 0..360° for all angular values:
//     0..180   → positive (same as standard)
//     360..181 → negative  (e.g. 240° means -120°)
//
//   kinDegToRad: SDK → ROS  (used in read())
//     NOTE: this function has a ±2π discontinuity at exactly 180°/181°.
//     read() applies per-joint unwrapping to remove it for position feedback.
//     It is safe to use without unwrapping for velocities (very small values).
//
//   radToKinDeg: ROS → SDK  (used in write() position mode)
//     Handles accumulated/unwrapped angles (e.g. 7.0 rad → 401° → 41° mod 360).
// ---------------------------------------------------------------------------
static inline double kinDegToRad(float deg)
{
  double d = static_cast<double>(deg);
  if (d > 180.0) {
    d -= 360.0;
  }
  return d * (M_PI / 180.0);
}

static inline float radToKinDeg(double rad)
{
  // Use fmod so accumulated/unwrapped angles (> π or < -π) map correctly
  // into the Kinova 0-360° convention without truncation.
  double deg = std::fmod(rad * (180.0 / M_PI), 360.0);
  if (deg < 0.0) {
    deg += 360.0;
  }
  return static_cast<float>(deg);
}


//   vectors exposed as state interfaces.  This is a pure memory operation —
//   no USB I/O — so it always returns in nanoseconds on the RT thread.
//
//   Unit conversions match the example scripts:
//     Kinova API returns degrees / (degrees per second) / (N·m).
//     ROS uses radians / (radians per second) / (N·m).
// ---------------------------------------------------------------------------
hardware_interface::return_type KinovaSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  AngularPosition pos, vel, torq;
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    pos  = pos_shadow_;
    vel  = vel_shadow_;
    torq = torq_shadow_;
  }

  // ── Position feedback — with 180° discontinuity unwrapping ────────────────
  // kinDegToRad() has a ±2π jump at exactly 180°/181° (Kinova 2-complement).
  // If a continuous joint (1, 4, 5, 6) crosses through 180°, the raw value
  // would jump by ~6.28 rad.  JTAC would see a huge position error and fire
  // a massive corrective velocity in the wrong direction.
  //
  // Fix: compare each new reading to the previous value.  If the delta
  // exceeds ±π the joint just crossed the discontinuity — correct by ∓2π.
  // Consecutive reads never differ by more than ~0.01 rad at the arm's top
  // speed (~1 rad/s × 10 ms poll period), so the threshold is unambiguous.
  const float raw_pos[6] = {
    pos.Actuators.Actuator1, pos.Actuators.Actuator2, pos.Actuators.Actuator3,
    pos.Actuators.Actuator4, pos.Actuators.Actuator5, pos.Actuators.Actuator6};
  const float raw_vel[6] = {
    vel.Actuators.Actuator1, vel.Actuators.Actuator2, vel.Actuators.Actuator3,
    vel.Actuators.Actuator4, vel.Actuators.Actuator5, vel.Actuators.Actuator6};

  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    double new_pos = kinDegToRad(raw_pos[i]);
    if (!std::isnan(hw_positions_[i])) {
      const double delta = new_pos - hw_positions_[i];
      if      (delta >  M_PI) new_pos -= 2.0 * M_PI;
      else if (delta < -M_PI) new_pos += 2.0 * M_PI;
    }
    hw_positions_[i] = new_pos;

    // Kinova firmware returns GetAngularVelocity at half the actual value.
    // kinova_comm.cpp line 623: "velocities reported back by firmware
    // seem to be half of actual value".
    // No unwrapping needed: velocities are small and never approach ±π rad/s.
    hw_velocities_[i] = 2.0 * kinDegToRad(raw_vel[i]);
  }

  // Gravity-free torque — units already N·m from the SDK.
  hw_efforts_[0] = torq.Actuators.Actuator1;
  hw_efforts_[1] = torq.Actuators.Actuator2;
  hw_efforts_[2] = torq.Actuators.Actuator3;
  hw_efforts_[3] = torq.Actuators.Actuator4;
  hw_efforts_[4] = torq.Actuators.Actuator5;
  hw_efforts_[5] = torq.Actuators.Actuator6;

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// write
//   Send angular position or velocity commands to the arm via SendBasicTrajectory.
//
//   Position mode (JTAC with command_interface_type: position):
//     pointToSend.Position.Type = ANGULAR_POSITION;
//     pointToSend.Position.Actuators.Actuator1 = <deg>;
//     EraseAllTrajectories() is called first to flush the FIFO so each write
//     imposes a fresh setpoint instead of queuing.  At 200 Hz the arm always
//     moves toward the latest command.  Without the flush, the 2000-point FIFO
//     would accumulate and the arm would lag behind the commanded trajectory.
//
//   Velocity mode (joint_velocity_controller):
//     pointToSend.Position.Type = ANGULAR_VELOCITY;
//     Mirror of the streaming loop in kinova_ang_control.cpp.
//
//   Both variants are gated by their respective *_commands_active_ flags set
//   in perform_command_mode_switch().  NaN position commands (before JTAC
//   initialises the interface) are silently skipped.
// ---------------------------------------------------------------------------
hardware_interface::return_type KinovaSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (position_commands_active_) {
    // Guard against NaN (hw_position_commands_ initialised to NaN in on_init;
    // JTAC writes valid values before any motion starts).
    for (const auto & v : hw_position_commands_) {
      if (std::isnan(v)) {
        return hardware_interface::return_type::OK;
      }
    }

    // Flush the FIFO so this point is executed immediately, not appended after
    // potentially hundreds of previously queued waypoints.
    MyEraseAllTrajectories();

    TrajectoryPoint cmd;
    cmd.InitStruct();
    cmd.Position.Type = ANGULAR_POSITION;

    // Convert ROS rad → Kinova degrees (0–360 convention).
    // ROS uses wrapped angles (-π to +π); Kinova expects 0–360.
    // radToKinDeg() is the inverse of kinDegToRad() applied in read().
    cmd.Position.Actuators.Actuator1 = radToKinDeg(hw_position_commands_[0]);
    cmd.Position.Actuators.Actuator2 = radToKinDeg(hw_position_commands_[1]);
    cmd.Position.Actuators.Actuator3 = radToKinDeg(hw_position_commands_[2]);
    cmd.Position.Actuators.Actuator4 = radToKinDeg(hw_position_commands_[3]);
    cmd.Position.Actuators.Actuator5 = radToKinDeg(hw_position_commands_[4]);
    cmd.Position.Actuators.Actuator6 = radToKinDeg(hw_position_commands_[5]);

    cmd.Position.Fingers.Finger1 = 0.0f;
    cmd.Position.Fingers.Finger2 = 0.0f;
    cmd.Position.Fingers.Finger3 = 0.0f;

    MySendAdvanceTrajectory(cmd);
    return hardware_interface::return_type::OK;
  }

  if (!velocity_commands_active_) {
    return hardware_interface::return_type::OK;
  }

  TrajectoryPoint cmd;
  cmd.InitStruct();
  cmd.Position.Type = ANGULAR_VELOCITY;

  // Convert from ROS rad/s → Kinova deg/s (example pattern).
  cmd.Position.Actuators.Actuator1 = static_cast<float>(hw_commands_[0] * RAD_TO_DEG);
  cmd.Position.Actuators.Actuator2 = static_cast<float>(hw_commands_[1] * RAD_TO_DEG);
  cmd.Position.Actuators.Actuator3 = static_cast<float>(hw_commands_[2] * RAD_TO_DEG);
  cmd.Position.Actuators.Actuator4 = static_cast<float>(hw_commands_[3] * RAD_TO_DEG);
  cmd.Position.Actuators.Actuator5 = static_cast<float>(hw_commands_[4] * RAD_TO_DEG);
  cmd.Position.Actuators.Actuator6 = static_cast<float>(hw_commands_[5] * RAD_TO_DEG);

  // Fingers not controlled by this driver; keep at zero.
  cmd.Position.Fingers.Finger1 = 0.0f;
  cmd.Position.Fingers.Finger2 = 0.0f;
  cmd.Position.Fingers.Finger3 = 0.0f;

  // For ANGULAR_VELOCITY streaming at 200 Hz, use SendBasicTrajectory — this
  // is exactly what the official SDK example kinova_ang_control.cpp does:
  //   for (int i = 0; i < 300; i++) { MySendBasicTrajectory(pt); usleep(5000); }
  // Do NOT call EraseAllTrajectories here; streaming velocity does not use
  // the trajectory FIFO the same way position waypoints do, and erasing every
  // cycle adds an extra blocking USB call in the RT write() path.
  MySendBasicTrajectory(cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace kinova_driver

PLUGINLIB_EXPORT_CLASS(
  kinova_driver::KinovaSystemHardware,
  hardware_interface::SystemInterface)

