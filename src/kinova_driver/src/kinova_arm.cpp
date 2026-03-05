// Copyright 2021 ros2_control Development Team
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
Author: 
    Cameron Hinkle, Colorado School of Mines
Purpose:
    Ros2 control hardware driver to move the kinovagen2 arm with ros2 jazzy.
    Uses the typical joint_trajectory control scheme for arm telemetry and control.
*/


#include "kinova_ros/kinova_arm.hpp"

// ros libraries for hardware interfaces
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"



// All kinova includes
#include "kinova_api/KinovaTypes.h"

// MAY NEED TO FIX FOR ARM SYSTEMS: check for rpi3 library build on github
#include "kinova_api/Kinova.API.USBCommLayerUbuntu.h"
#include "kinova_api/Kinova.API.USBCommandLayerUbuntu.h"


// helper libraries
#include <dlfcn.h>
#include <libgen.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <iostream>

// Add a small C symbol we can take the address of for dladdr
extern "C" void kinova_driver__dladdr_helper() {}





namespace kinova_driver
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("KinovaSystemHardware");
/*
Routine that runs when hardware interface is created
Performs:

*/

hardware_interface::CallbackReturn KinovaSystemHardware::on_init(const hardware_interface::HardwareInfo & info) {

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Find the directory of this shared object (the plugin) so we can open vendor .so next to it
  Dl_info dl_info;
  // use address of a local C symbol (function) to get the shared object path
  uintptr_t helper_addr = reinterpret_cast<uintptr_t>(reinterpret_cast<void (*)()>(kinova_driver__dladdr_helper));
  if (dladdr(reinterpret_cast<void*>(helper_addr), &dl_info) == 0) {
    RCLCPP_ERROR(LOGGER, "dladdr failed");
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::string plugin_path = dl_info.dli_fname;
  std::string plugin_dir = dirname(const_cast<char*>(plugin_path.c_str()));

  // try vendor filenames that exist in your package lib
  const char *candidates[] = {
    "USBCommandLayerUbuntu.so",
    "EthCommandLayerUbuntu.so",
    "Kinova.API.USBCommandLayerUbuntu.so",
    nullptr
  };

  commandLayer_handle = nullptr;
  for (const char **p = candidates; *p; ++p) {
    std::string full = plugin_dir + "/" + *p;
    commandLayer_handle = dlopen(full.c_str(), RTLD_NOW | RTLD_GLOBAL);
    if (commandLayer_handle) {
      RCLCPP_INFO(LOGGER, "dlopened vendor library: %s", full.c_str());
      break;
    }
  }
  if (!commandLayer_handle) {
    RCLCPP_ERROR(LOGGER, "Failed dlopen vendor libs: %s", dlerror());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // resolve symbols from the vendor library (or RTLD_DEFAULT)
  dlerror();
  MyInitAPI = reinterpret_cast<int (*)()>(dlsym(RTLD_DEFAULT, "InitAPI"));
  if (!MyInitAPI) { RCLCPP_ERROR(LOGGER, "InitAPI not found: %s", dlerror()); return hardware_interface::CallbackReturn::ERROR; }
  MyCloseAPI = reinterpret_cast<int (*)()>(dlsym(RTLD_DEFAULT, "CloseAPI"));
  if (!MyCloseAPI) { RCLCPP_ERROR(LOGGER, "CloseAPI not found: %s", dlerror()); return hardware_interface::CallbackReturn::ERROR; }
  MyGetDevices = reinterpret_cast<int (*)(KinovaDevice[], int &)>(dlsym(RTLD_DEFAULT, "GetDevices"));
  if (!MyGetDevices) { RCLCPP_ERROR(LOGGER, "GetDevices not found: %s", dlerror()); return hardware_interface::CallbackReturn::ERROR; }
  MySetActiveDevice = reinterpret_cast<int (*)(KinovaDevice)>(dlsym(RTLD_DEFAULT, "SetActiveDevice"));
  if (!MySetActiveDevice) { RCLCPP_ERROR(LOGGER, "SetActiveDevice not found: %s", dlerror()); return hardware_interface::CallbackReturn::ERROR; }
  MySendBasicTrajectory = reinterpret_cast<int (*)(TrajectoryPoint)>(dlsym(RTLD_DEFAULT, "SendBasicTrajectory"));
  if (!MySendBasicTrajectory) { RCLCPP_ERROR(LOGGER, "SendBasicTrajectory not found: %s", dlerror()); return hardware_interface::CallbackReturn::ERROR; }
  MyGetAngularPosition = reinterpret_cast<int (*)(AngularPosition &)>(dlsym(RTLD_DEFAULT, "GetAngularPosition"));
  if (!MyGetAngularPosition) { RCLCPP_ERROR(LOGGER, "GetAngularPosition not found: %s", dlerror()); return hardware_interface::CallbackReturn::ERROR; }
  MyGetAngularVelocity = reinterpret_cast<int (*)(AngularPosition &)>(dlsym(RTLD_DEFAULT, "GetAngularVelocity"));
  if (!MyGetAngularVelocity) { RCLCPP_ERROR(LOGGER, "GetAngularVelocity not found: %s", dlerror()); return hardware_interface::CallbackReturn::ERROR; }
  MyGetAngularForce = reinterpret_cast<int (*)(AngularPosition &)>(dlsym(RTLD_DEFAULT, "GetAngularForce"));
  if (!MyGetAngularForce) { RCLCPP_ERROR(LOGGER, "GetAngularForce not found: %s", dlerror()); return hardware_interface::CallbackReturn::ERROR; }
  MyStartControlAPI = reinterpret_cast<int (*)()>(dlsym(RTLD_DEFAULT, "StartControlAPI"));
  if (!MyStartControlAPI) { RCLCPP_ERROR(LOGGER, "StartControlAPI not found: %s", dlerror()); return hardware_interface::CallbackReturn::ERROR; }

  MyStopControlAPI = reinterpret_cast<int (*)()>(dlsym(RTLD_DEFAULT, "StopControlAPI"));
  if (!MyStopControlAPI) {
    RCLCPP_ERROR(LOGGER, "StopControlAPI not found: %s", dlerror());
    return hardware_interface::CallbackReturn::ERROR;
  }

  MySetAngularControl = reinterpret_cast<int (*)()>(dlsym(RTLD_DEFAULT, "SetAngularControl"));
  if (!MySetAngularControl) {
    RCLCPP_ERROR(LOGGER, "SetAngularControl not found: %s", dlerror());
    return hardware_interface::CallbackReturn::ERROR;
  }

  MySendAdvanceTrajectory = reinterpret_cast<int (*)(TrajectoryPoint)>(dlsym(RTLD_DEFAULT, "SendAdvanceTrajectory"));
  if (!MySendAdvanceTrajectory) {
    RCLCPP_ERROR(LOGGER, "SendAdvanceTrajectory not found: %s", dlerror());
    return hardware_interface::CallbackReturn::ERROR;
  }


  //Verify that all functions has been loaded correctly
	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetDevices == NULL) ||
        (MySetActiveDevice == NULL) || (MySendBasicTrajectory == NULL) || (MyGetAngularPosition == NULL) ||
        (MyGetAngularVelocity == NULL) || (MyGetAngularForce == NULL) ||
        (MyStartControlAPI == NULL) || (MyStopControlAPI == NULL) ||
        (MySetAngularControl == NULL) || (MySendAdvanceTrajectory == NULL))
  {
    RCLCPP_INFO(LOGGER, "* * *  E R R O R   D U R I N G   A P I   I N I T I A L I Z A T I O N  * * *");
		return hardware_interface::CallbackReturn::ERROR;
	}


  // resize and populate vectors
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), 0.0);



  RCLCPP_INFO(LOGGER, "I N I T I A L I Z A T I O N   C O M P L E T E D");

  return hardware_interface::CallbackReturn::SUCCESS;
}



/*
Creates the state interfaces for the hardware interface. Links the state variable.
*/


std::vector<hardware_interface::StateInterface> KinovaSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
    

  for (uint8_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}



/*
Creates the command interfaces for the hardware interface. Links the state variable.
*/

std::vector<hardware_interface::CommandInterface> KinovaSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  

    for (uint8_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}




/*
Routine to run when the hardware interface is called to be configured.

Performs:
  - finds kinova arm on USB bus
  - sets active device to Kinova arm (if found)
*/

hardware_interface::CallbackReturn KinovaSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  
  MyInitAPI();

  KinovaDevice list[MAX_KINOVA_DEVICE];

  int result;
	int devicesCount = MyGetDevices(list, result);

  if (devicesCount > 0) {
    RCLCPP_INFO(LOGGER, "Found a robot on the USB bus (%s)",list[0].SerialNumber);
    // std::cout << "Found a robot on the USB bus (" << list[0].SerialNumber << ")" << std::endl;

    MySetActiveDevice(list[0]);

  } else {
    RCLCPP_ERROR(LOGGER, "No robots found on serial bus.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }


  return hardware_interface::CallbackReturn::SUCCESS;
}





/*
Routine to run when hardware interface is activated.

Performs:
  - set all command interfaces to 0
*/

hardware_interface::CallbackReturn KinovaSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Activating driver...");

  int result = MyStartControlAPI();
  RCLCPP_INFO(LOGGER, "API result: %i",result);
  MySetAngularControl();
  RCLCPP_INFO(LOGGER, "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}





/*
Routine that runs when the hardware interface is finished.
Performs:
  - closes api handle
  - closes library connection to commandLayer_handle
*/

hardware_interface::CallbackReturn KinovaSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  
  // close kinova API connection to arm
  
  // std::cout << "Initialization's result :" << result << std::endl;
  MyStopControlAPI();
  RCLCPP_INFO(LOGGER, "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}




/*
Routine that runs when the hardware interface is finished.
Performs:
  - closes api handle
  - closes library connection to commandLayer_handle
*/

hardware_interface::CallbackReturn KinovaSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  int result = (*MyCloseAPI)();
  (void)result;
  RCLCPP_INFO(LOGGER, "WARNING: Your robot is now set to angular control. If you use the joystick, it will be a joint by joint movement.");
  //std::cout << std::endl << "WARNING: Your robot is now set to angular control. If you use the joystick, it will be a joint by joint movement." << std::endl;
	RCLCPP_INFO(LOGGER, "C L O S I N G   A P I");
  // std::cout << std::endl << "C L O S I N G   A P I" << std::endl;


  // close handle to kinova API
  dlclose(commandLayer_handle);

  return hardware_interface::CallbackReturn::SUCCESS;
}





/*
Routine to handle all of the joint read variables
Performs:

*/

hardware_interface::return_type KinovaSystemHardware::read(
const rclcpp::Time & /*time*/, const rclcpp::Duration & period) {
  
  // 
  AngularPosition current_pos;
  AngularPosition current_vel;
  AngularPosition current_torq;

  // populate current position, velocity, and torque
  (*MyGetAngularPosition)(current_pos);
  (*MyGetAngularVelocity)(current_vel);
  (*MyGetAngularForce)(current_torq);


  // update joint 1;
  hw_positions_[0] = degreesToRadians(current_pos.Actuators.Actuator1);
  hw_velocities_[0] = kinovaVelocityToRos(current_vel.Actuators.Actuator1);
  hw_efforts_[0] = current_torq.Actuators.Actuator1;

  // update joint 2:
  hw_positions_[1] = degreesToRadians(current_pos.Actuators.Actuator2);
  hw_velocities_[1] = kinovaVelocityToRos(current_vel.Actuators.Actuator2);
  hw_efforts_[1] = current_torq.Actuators.Actuator2;

  // update joint 3:
  hw_positions_[2] = degreesToRadians(current_pos.Actuators.Actuator3);
  hw_velocities_[2] = kinovaVelocityToRos(current_vel.Actuators.Actuator3);
  hw_efforts_[2] = current_torq.Actuators.Actuator3;

  // update joint 4:
  hw_positions_[3] = degreesToRadians(current_pos.Actuators.Actuator4);
  hw_velocities_[3] = kinovaVelocityToRos(current_vel.Actuators.Actuator4);
  hw_efforts_[3] = current_torq.Actuators.Actuator4;

  // update joint 5:
  hw_positions_[4] = degreesToRadians(current_pos.Actuators.Actuator5);
  hw_velocities_[4] = kinovaVelocityToRos(current_vel.Actuators.Actuator5);
  hw_efforts_[4] = current_torq.Actuators.Actuator5;

  // update joint 6:
  hw_positions_[5] = degreesToRadians(current_pos.Actuators.Actuator6);
  hw_velocities_[5] = kinovaVelocityToRos(current_vel.Actuators.Actuator6);
  hw_efforts_[5] = current_torq.Actuators.Actuator6;

  return hardware_interface::return_type::OK;
}





/*
Routine to handle all write actions
Performs:

*/

hardware_interface::return_type KinovaSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // create trajectory command to send
  TrajectoryPoint ang_command;

  ang_command.InitStruct();

  ang_command.Position.Type = ANGULAR_VELOCITY;

  // for (size_t i = 0; i < hw_commands_.size(); ++i) {
  //   RCLCPP_INFO(LOGGER, "Joint %zu velocity command: %f", i+1, hw_commands_[i]);
  // }

  ang_command.Position.Actuators.Actuator1 = rosVelocityToKinova(hw_commands_[0]);
  ang_command.Position.Actuators.Actuator2 = rosVelocityToKinova(hw_commands_[1]);
  ang_command.Position.Actuators.Actuator3 = rosVelocityToKinova(hw_commands_[2]);
  ang_command.Position.Actuators.Actuator4 = rosVelocityToKinova(hw_commands_[3]);
  ang_command.Position.Actuators.Actuator5 = rosVelocityToKinova(hw_commands_[4]);
  ang_command.Position.Actuators.Actuator6 = rosVelocityToKinova(hw_commands_[5]);

  ang_command.Position.Fingers.Finger1 = 0;
  ang_command.Position.Fingers.Finger2 = 0;
  ang_command.Position.Fingers.Finger3 = 0;

  MySendAdvanceTrajectory(ang_command);

  // int result = MySendBasicTrajectory(ang_command);
  // RCLCPP_INFO(LOGGER, "MySendBasicTrajectory result: %d", result);

  
  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  kinova_driver::KinovaSystemHardware, hardware_interface::SystemInterface)