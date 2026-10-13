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

#include "kinova_driver/kinova_sdk.hpp"

#include <dlfcn.h>

#include <algorithm>

namespace kinova_driver
{

namespace
{

// Resolve `name` from `handle` into `fn`.  Returns false and fills `error` if
// the symbol is missing.
template<typename FnPtr>
bool resolve(void * handle, const char * name, FnPtr & fn, std::string & error)
{
  dlerror();
  fn = reinterpret_cast<FnPtr>(dlsym(handle, name));
  if (fn == nullptr) {
    const char * why = dlerror();
    error = std::string("symbol '") + name + "' not found: " + (why ? why : "null");
    return false;
  }
  return true;
}

}  // namespace

std::unique_ptr<KinovaSdk> KinovaSdk::load(const std::string & lib_dir, std::string & error)
{
  std::unique_ptr<KinovaSdk> sdk(new KinovaSdk());

  // The command layer internally calls dlopen("USBCommLayerUbuntu.so") by
  // short name.  Loading the comm layer first with RTLD_GLOBAL from our
  // vendored copy means glibc recognises that file as already resident and
  // reuses it instead of loading a second copy (which would leave the SDK's
  // internal comm-layer handle NULL).  The short name itself is resolved via
  // the LD_LIBRARY_PATH that sourcing the workspace sets up.
  const std::string comm_path = lib_dir + "/USBCommLayerUbuntu.so";
  sdk->comm_layer_ = dlopen(comm_path.c_str(), RTLD_NOW | RTLD_GLOBAL);
  if (sdk->comm_layer_ == nullptr) {
    error = "dlopen " + comm_path + ": " + dlerror();
    return nullptr;
  }

  const std::string command_path = lib_dir + "/USBCommandLayerUbuntu.so";
  sdk->command_layer_ = dlopen(command_path.c_str(), RTLD_NOW | RTLD_GLOBAL);
  if (sdk->command_layer_ == nullptr) {
    error = "dlopen " + command_path + ": " + dlerror();
    return nullptr;
  }

  // Resolve from the command layer handle explicitly (not RTLD_DEFAULT):
  // both libraries export same-named symbols with different semantics.
  void * h = sdk->command_layer_;
  const bool all_resolved =
    resolve(h, "InitAPI", sdk->init_api_, error) &&
    resolve(h, "CloseAPI", sdk->close_api_, error) &&
    resolve(h, "GetDevices", sdk->get_devices_, error) &&
    resolve(h, "SetActiveDevice", sdk->set_active_device_, error) &&
    resolve(h, "RefresDevicesList", sdk->refresh_devices_list_, error) &&
    resolve(h, "StartControlAPI", sdk->start_control_api_, error) &&
    resolve(h, "StopControlAPI", sdk->stop_control_api_, error) &&
    resolve(h, "SetAngularControl", sdk->set_angular_control_, error) &&
    resolve(h, "EraseAllTrajectories", sdk->erase_all_trajectories_, error) &&
    resolve(h, "SendBasicTrajectory", sdk->send_basic_trajectory_, error) &&
    resolve(h, "GetAngularPosition", sdk->get_angular_position_, error) &&
    resolve(h, "GetAngularVelocity", sdk->get_angular_velocity_, error) &&
    resolve(h, "GetAngularForceGravityFree", sdk->get_angular_force_gravity_free_, error);
  if (!all_resolved) {
    return nullptr;
  }
  return sdk;
}

KinovaSdk::~KinovaSdk()
{
  if (command_layer_ != nullptr) {
    dlclose(command_layer_);
  }
  if (comm_layer_ != nullptr) {
    dlclose(comm_layer_);
  }
}

int KinovaSdk::getDevices(std::vector<KinovaDevice> & devices) const
{
  devices.clear();
  refresh_devices_list_();
  KinovaDevice list[MAX_KINOVA_DEVICE];
  int result = 0;
  const int count = get_devices_(list, result);
  devices.assign(list, list + std::max(0, std::min(count, MAX_KINOVA_DEVICE)));
  return result;
}

int KinovaSdk::getJointPositionsDeg(JointArrayF & out) const
{
  AngularPosition p{};
  const int rc = get_angular_position_(p);
  if (ok(rc)) {
    out = toArray(p.Actuators);
  }
  return rc;
}

int KinovaSdk::getJointVelocitiesRaw(JointArrayF & out) const
{
  AngularPosition v{};
  const int rc = get_angular_velocity_(v);
  if (ok(rc)) {
    out = toArray(v.Actuators);
  }
  return rc;
}

int KinovaSdk::getJointTorquesGravityFree(JointArrayF & out) const
{
  AngularPosition t{};
  const int rc = get_angular_force_gravity_free_(t);
  if (ok(rc)) {
    out = toArray(t.Actuators);
  }
  return rc;
}

int KinovaSdk::sendJointVelocitiesDeg(const JointArrayF & vel_deg) const
{
  TrajectoryPoint cmd;
  cmd.InitStruct();
  cmd.Position.Type = ANGULAR_VELOCITY;
  cmd.Position.Actuators.Actuator1 = vel_deg[0];
  cmd.Position.Actuators.Actuator2 = vel_deg[1];
  cmd.Position.Actuators.Actuator3 = vel_deg[2];
  cmd.Position.Actuators.Actuator4 = vel_deg[3];
  cmd.Position.Actuators.Actuator5 = vel_deg[4];
  cmd.Position.Actuators.Actuator6 = vel_deg[5];
  cmd.Position.Fingers.Finger1 = 0.0f;
  cmd.Position.Fingers.Finger2 = 0.0f;
  cmd.Position.Fingers.Finger3 = 0.0f;
  return send_basic_trajectory_(cmd);
}

}  // namespace kinova_driver
