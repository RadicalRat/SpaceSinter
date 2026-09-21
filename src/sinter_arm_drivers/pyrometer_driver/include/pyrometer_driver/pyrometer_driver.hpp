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

// pyrometer_driver.hpp — ros2_control read-only hardware plugin for an
// Optris CT-ratio series pyrometer connected over a serial (RS232 / USB-serial)
// link.
//
// This is a hardware_interface::SensorInterface: it exports ONLY state
// interfaces (no command interfaces — hardware_interface::SensorInterface::
// write() is finalised upstream to a no-op), matching a passive, read-only
// temperature sensor.
//
// State interfaces (all double, on the single <sensor> declared in the URDF):
//   process_temperature    — °C, primary process temperature reading
//   detector_temperature   — °C, detector temperature
//   box_temperature        — °C, housing/box temperature
//   ratio_temperature       — °C, ratio-mode temperature
//   t1_temperature          — °C, auxiliary temperature 1
//   t2_temperature          — °C, auxiliary temperature 2
//   attenuation             — %, signal attenuation (clamped to [0, 100])
//
// All serial I/O runs on a dedicated background thread so that read() never
// blocks the ros2_control update loop on serial round-trip latency. The
// background thread polls the device continuously (target rate set by the
// "poll_rate_hz" hardware parameter, >= 100 Hz by default) and read() simply
// copies the latest values out of a mutex-protected buffer.
#ifndef PYROMETER_DRIVER__PYROMETER_DRIVER_HPP_
#define PYROMETER_DRIVER__PYROMETER_DRIVER_HPP_

#include <atomic>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace pyrometer_driver
{

class PyrometerSensorHardware : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PyrometerSensorHardware)

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

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── Sensor state (exported as StateInterfaces) ────────────────────────────
  // Written only by read(), from the bg_* mirrors below.
  double hw_process_temp_c_{std::numeric_limits<double>::quiet_NaN()};
  double hw_detector_temp_c_{std::numeric_limits<double>::quiet_NaN()};
  double hw_box_temp_c_{std::numeric_limits<double>::quiet_NaN()};
  double hw_ratio_temp_c_{std::numeric_limits<double>::quiet_NaN()};
  double hw_t1_temp_c_{std::numeric_limits<double>::quiet_NaN()};
  double hw_t2_temp_c_{std::numeric_limits<double>::quiet_NaN()};
  double hw_attenuation_pct_{std::numeric_limits<double>::quiet_NaN()};

  // ── Hardware parameters (from the URDF <hardware><param> tags) ───────────
  std::string device_{"/dev/pyrometer"};   ///< serial device path
  int baud_rate_{115200};                  ///< matches Optris CT-ratio default
  double poll_rate_hz_{100.0};             ///< target background poll rate
  int response_timeout_ms_{50};            ///< per-command read timeout
  std::string sensor_name_;                ///< info_.sensors[0].name, used as interface prefix

  // ── Serial port ────────────────────────────────────────────────────────────
  int serial_fd_{-1};

  // ── Background polling thread ─────────────────────────────────────────────
  // All serial I/O happens exclusively on this thread. read() only exchanges
  // data with the thread through a mutex-protected buffer, so the
  // ros2_control update loop never stalls on serial latency.
  std::thread poll_thread_;
  std::atomic<bool> poll_thread_running_{false};
  mutable std::mutex io_mutex_;   ///< guards all bg_* members below

  double bg_process_temp_c_{std::numeric_limits<double>::quiet_NaN()};
  double bg_detector_temp_c_{std::numeric_limits<double>::quiet_NaN()};
  double bg_box_temp_c_{std::numeric_limits<double>::quiet_NaN()};
  double bg_ratio_temp_c_{std::numeric_limits<double>::quiet_NaN()};
  double bg_t1_temp_c_{std::numeric_limits<double>::quiet_NaN()};
  double bg_t2_temp_c_{std::numeric_limits<double>::quiet_NaN()};
  double bg_attenuation_pct_{std::numeric_limits<double>::quiet_NaN()};

  /// Polling diagnostics (bg thread → main thread, mutex-protected).
  struct PollDiagnostics
  {
    double cycle_ms{0.0};          ///< last full 7-command poll cycle duration
    uint64_t cycle_count{0};       ///< successful poll cycles since activation
    uint64_t comm_error_count{0};  ///< poll cycles with a dropped/malformed response
  };
  PollDiagnostics bg_diag_;

  uint64_t read_log_counter_{0};  ///< cycle counter for periodic diagnostic log in read()

  void poll_thread_func();

  // ── Serial helpers (only ever called from on_configure/on_cleanup/poll thread) ──
  bool open_serial_port();
  void close_serial_port();
  void send_laser(bool on);
  bool write_bytes(const uint8_t * data, size_t len);
  bool read_exact(uint8_t * buf, size_t len, int timeout_ms);

  /// Sends a single-byte read command and interprets the 2-byte response as a
  /// signed-tenths-of-a-degree temperature, per the Optris CT-ratio
  /// communication interface manual.
  ///   raw = data[0] << 8 | data[1]
  ///   if has_status_bit and bit15 set: raw &= 0x7FFF  (strip status flag)
  ///   value_c = (raw - 1000) / 10.0
  bool read_temperature_channel(uint8_t command, bool has_status_bit, double & out_value_c);

  /// Command 0x0D — attenuation is unsigned tenths-of-a-percent, clamped to 100%.
  bool read_attenuation_channel(double & out_value_pct);
};

}  // namespace pyrometer_driver

#endif  // PYROMETER_DRIVER__PYROMETER_DRIVER_HPP_
