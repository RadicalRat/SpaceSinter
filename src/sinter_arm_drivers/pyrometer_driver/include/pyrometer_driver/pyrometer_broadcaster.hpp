// Copyright 2026 Space Sinter Team, Colorado School of Mines
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

// pyrometer_broadcaster.hpp — ros2_control broadcaster (non-chainable
// controller_interface::ControllerInterface) that republishes the
// pyrometer_driver's state interfaces as ROS topics.
//
// Publishes, every update() cycle (i.e. at the controller_manager's update
// rate), one sensor_msgs::msg::Temperature per temperature channel:
//   ~/process_temperature, ~/detector_temperature, ~/box_temperature,
//   ~/ratio_temperature, ~/t1_temperature, ~/t2_temperature
// plus one std_msgs::msg::Float64 for the unitless attenuation reading:
//   ~/attenuation
//
// Parameters:
//   sensor_name — must match the <sensor name="..."> declared in the URDF
//                 for the pyrometer_driver hardware component (required).
//   frame_id    — frame_id stamped on every Temperature message header
//                 (default: same as sensor_name).
#ifndef PYROMETER_DRIVER__PYROMETER_BROADCASTER_HPP_
#define PYROMETER_DRIVER__PYROMETER_BROADCASTER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/float64.hpp"

namespace pyrometer_driver
{

class PyrometerBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── Parameters ─────────────────────────────────────────────────────────────
  std::string sensor_name_;  ///< must match the <sensor name="..."> in the URDF
  std::string frame_id_;     ///< frame_id stamped on published Temperature messages

  // ── State interface layout ────────────────────────────────────────────────
  // Fixed order — must match the order returned by state_interface_configuration(),
  // since the controller_manager assigns state_interfaces_ in that same order.
  enum StateIdx
  {
    PROCESS = 0,
    DETECTOR,
    BOX,
    RATIO,
    T1,
    T2,
    ATTENUATION,
    NUM_STATES
  };
  static constexpr std::array<const char *, NUM_STATES> kInterfaceSuffixes{
    "process_temperature", "detector_temperature", "box_temperature",
    "ratio_temperature", "t1_temperature", "t2_temperature", "attenuation"};

  // ── Publishers — one Temperature topic per temperature channel ──────────────
  static constexpr size_t kNumTemperatureChannels = 6;  // excludes ATTENUATION
  std::array<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Temperature>>,
    kNumTemperatureChannels>
    temperature_publishers_;
  std::array<
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Temperature>>,
    kNumTemperatureChannels>
    rt_temperature_publishers_;

  // ── Publisher for the unitless attenuation reading ───────────────────────
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> attenuation_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>
    rt_attenuation_publisher_;
};

}  // namespace pyrometer_driver

#endif  // PYROMETER_DRIVER__PYROMETER_BROADCASTER_HPP_
