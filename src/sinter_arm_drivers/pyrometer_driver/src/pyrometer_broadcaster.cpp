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

#include "pyrometer_driver/pyrometer_broadcaster.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pyrometer_driver
{

namespace
{
constexpr double kUnknownVariance = 0.0;  ///< sensor_msgs/Temperature: 0 == "variance unknown"
}  // namespace

controller_interface::InterfaceConfiguration
PyrometerBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
PyrometerBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(kInterfaceSuffixes.size());
  for (const char * suffix : kInterfaceSuffixes) {
    config.names.push_back(sensor_name_ + "/" + suffix);
  }
  return config;
}

controller_interface::CallbackReturn PyrometerBroadcaster::on_init()
{
  try {
    sensor_name_ = auto_declare<std::string>("sensor_name", "");
    frame_id_ = auto_declare<std::string>("frame_id", "");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PyrometerBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (sensor_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Required parameter 'sensor_name' is empty — must match the <sensor name=\"...\"> "
      "declared in the URDF for the pyrometer hardware component.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (frame_id_.empty()) {
    frame_id_ = sensor_name_;
  }

  static constexpr std::array<const char *, kNumTemperatureChannels> kTemperatureTopics{
    "~/process_temperature", "~/detector_temperature", "~/box_temperature",
    "~/ratio_temperature", "~/t1_temperature", "~/t2_temperature"};

  for (size_t i = 0; i < kNumTemperatureChannels; ++i) {
    temperature_publishers_[i] = get_node()->create_publisher<sensor_msgs::msg::Temperature>(
      kTemperatureTopics[i], rclcpp::SystemDefaultsQoS());
    rt_temperature_publishers_[i] =
      std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Temperature>>(
        temperature_publishers_[i]);
    rt_temperature_publishers_[i]->msg_.header.frame_id = frame_id_;
    rt_temperature_publishers_[i]->msg_.variance = kUnknownVariance;
  }

  attenuation_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>(
    "~/attenuation", rclcpp::SystemDefaultsQoS());
  rt_attenuation_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(
      attenuation_publisher_);

  RCLCPP_INFO(get_node()->get_logger(),
    "Configured for sensor '%s' (frame_id '%s').", sensor_name_.c_str(), frame_id_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PyrometerBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PyrometerBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PyrometerBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < kNumTemperatureChannels; ++i) {
    const auto value = state_interfaces_[i].get_optional<double>();
    if (!value.has_value()) continue;

    if (rt_temperature_publishers_[i]->trylock()) {
      auto & msg = rt_temperature_publishers_[i]->msg_;
      msg.header.stamp = time;
      msg.temperature = value.value();
      rt_temperature_publishers_[i]->unlockAndPublish();
    }
  }

  const auto attenuation = state_interfaces_[ATTENUATION].get_optional<double>();
  if (attenuation.has_value() && rt_attenuation_publisher_->trylock()) {
    rt_attenuation_publisher_->msg_.data = attenuation.value();
    rt_attenuation_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace pyrometer_driver

PLUGINLIB_EXPORT_CLASS(
  pyrometer_driver::PyrometerBroadcaster, controller_interface::ControllerInterface)
