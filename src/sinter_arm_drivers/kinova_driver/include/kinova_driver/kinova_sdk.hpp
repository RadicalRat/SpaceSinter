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

// kinova_sdk.hpp — RAII wrapper around the closed-source Kinova Gen2 USB SDK.
//
// The SDK ships as two shared objects that are loaded with dlopen() at runtime
// (never linked), so this class owns the library handles and the resolved
// function pointers.  The libraries stay loaded for the lifetime of the
// object; destroying it unloads them.

#ifndef KINOVA_DRIVER__KINOVA_SDK_HPP_
#define KINOVA_DRIVER__KINOVA_SDK_HPP_

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "kinova_api/KinovaTypes.h"
#include "kinova_api/Kinova.API.USBCommLayerUbuntu.h"

namespace kinova_driver
{

/// The j2n6s300 has six arm actuators; the SDK structs are fixed to this.
constexpr std::size_t kNumJoints = 6;

using JointArrayF = std::array<float, kNumJoints>;

/// Copy the six actuator fields of an SDK struct into an array.
inline JointArrayF toArray(const AngularInfo & a)
{
  return {a.Actuator1, a.Actuator2, a.Actuator3, a.Actuator4, a.Actuator5, a.Actuator6};
}

class KinovaSdk
{
public:
  /// Load the USB comm/command layers from `lib_dir`.  Returns nullptr and
  /// fills `error` on failure.
  static std::unique_ptr<KinovaSdk> load(const std::string & lib_dir, std::string & error);

  ~KinovaSdk();
  KinovaSdk(const KinovaSdk &) = delete;
  KinovaSdk & operator=(const KinovaSdk &) = delete;

  /// True when an SDK return code means success.
  static bool ok(int code) {return code == NO_ERROR_KINOVA;}

  // All methods return the raw SDK error code (NO_ERROR_KINOVA on success).
  int initApi() const {return init_api_();}
  int closeApi() const {return close_api_();}
  int startControlApi() const {return start_control_api_();}
  int stopControlApi() const {return stop_control_api_();}
  int setAngularControl() const {return set_angular_control_();}
  int eraseAllTrajectories() const {return erase_all_trajectories_();}
  int setActiveDevice(const KinovaDevice & device) const {return set_active_device_(device);}

  /// Refresh and list all Kinova devices on the USB bus.
  int getDevices(std::vector<KinovaDevice> & devices) const;

  int getJointPositionsDeg(JointArrayF & out) const;
  int getJointVelocitiesRaw(JointArrayF & out) const;
  int getJointTorquesGravityFree(JointArrayF & out) const;

  /// Stream an ANGULAR_VELOCITY command (deg/s per joint, fingers held at 0).
  int sendJointVelocitiesDeg(const JointArrayF & vel_deg) const;

private:
  KinovaSdk() = default;

  void * comm_layer_{nullptr};
  void * command_layer_{nullptr};

  int (*init_api_)() {nullptr};
  int (*close_api_)() {nullptr};
  int (*get_devices_)(KinovaDevice[], int &) {nullptr};
  int (*set_active_device_)(KinovaDevice) {nullptr};
  int (*refresh_devices_list_)() {nullptr};
  int (*start_control_api_)() {nullptr};
  int (*stop_control_api_)() {nullptr};
  int (*set_angular_control_)() {nullptr};
  int (*erase_all_trajectories_)() {nullptr};
  int (*send_basic_trajectory_)(TrajectoryPoint) {nullptr};
  int (*get_angular_position_)(AngularPosition &) {nullptr};
  int (*get_angular_velocity_)(AngularPosition &) {nullptr};
  int (*get_angular_force_gravity_free_)(AngularPosition &) {nullptr};
};

}  // namespace kinova_driver

#endif  // KINOVA_DRIVER__KINOVA_SDK_HPP_
