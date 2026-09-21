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

// pyrometer_driver.cpp — ros2_control read-only hardware plugin for an
// Optris CT-ratio series pyrometer.
// Byte-level protocol ported from the vendor Python reference test script
// (Optris CT-ratio communication interface manual):
//   Laser on:            0x25 0x01 0x24
//   Laser off:            0x25 0x00 0x25
//   Read process temp:    0x01 -> 2 bytes, status-bit at b15
//   Read detector temp:   0x02 -> 2 bytes
//   Read box temp:        0x03 -> 2 bytes
//   Read ratio temp:      0x0A -> 2 bytes, status-bit at b15
//   Read T2:               0x0B -> 2 bytes
//   Read T1:               0x0C -> 2 bytes
//   Read attenuation:      0x0D -> 2 bytes, unsigned tenths-of-a-percent
//
// ── Hardware parameters (URDF <hardware><param>) ──────────────────────────────
//   port              — serial device path (default "/dev/pyrometer", see udev/)
//   baud_rate          — serial baud rate (default 115200)
//   poll_rate_hz       — target background poll rate, Hz (default 100.0)
//   response_timeout_ms — per-command read timeout, ms (default 50)
//
// ── Startup sequence (on_configure) ───────────────────────────────────────────
//   1. Open + configure the serial port (raw mode, 8N1).
//   2. Send the laser-on command and wait for the device to stabilize.
//   3. Flush the input buffer to discard any startup/laser-on backlog bytes.
//   4. Start the background poll thread.
#include "pyrometer_driver/pyrometer_driver.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <cstring>
#include <type_traits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pyrometer_driver
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("PyrometerSensorHardware");

namespace
{
constexpr uint8_t CMD_LASER_ON[3] = {0x25, 0x01, 0x24};
constexpr uint8_t CMD_LASER_OFF[3] = {0x25, 0x00, 0x25};
constexpr uint8_t CMD_READ_PROCESS = 0x01;
constexpr uint8_t CMD_READ_DETECTOR = 0x02;
constexpr uint8_t CMD_READ_BOX = 0x03;
constexpr uint8_t CMD_READ_RATIO = 0x0A;
constexpr uint8_t CMD_READ_T2 = 0x0B;
constexpr uint8_t CMD_READ_T1 = 0x0C;
constexpr uint8_t CMD_READ_ATTENUATION = 0x0D;

speed_t baud_to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}
}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// on_init — parse hardware parameters, validate the declared sensor
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn PyrometerSensorHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SensorInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  if (info_.sensors.size() != 1) {
    RCLCPP_ERROR(LOGGER, "Expected exactly 1 <sensor> in the URDF, got %zu.",
      info_.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  sensor_name_ = info_.sensors[0].name;

  auto get_param = [this](const char * key, auto default_value) {
    auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end()) return default_value;
    try {
      if constexpr (std::is_same_v<decltype(default_value), int>) {
        return std::stoi(it->second);
      } else if constexpr (std::is_same_v<decltype(default_value), double>) {
        return std::stod(it->second);
      } else {
        return it->second;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(LOGGER, "Invalid value for parameter '%s': '%s' (%s). Using default.",
        key, it->second.c_str(), e.what());
      return default_value;
    }
  };

  device_ = get_param("port", std::string("/dev/pyrometer"));
  baud_rate_ = get_param("baud_rate", 115200);
  poll_rate_hz_ = get_param("poll_rate_hz", 100.0);
  response_timeout_ms_ = get_param("response_timeout_ms", 50);

  if (poll_rate_hz_ < 100.0) {
    RCLCPP_WARN(LOGGER,
      "poll_rate_hz=%.1f is below the recommended minimum of 100 Hz.", poll_rate_hz_);
  }

  RCLCPP_INFO(LOGGER,
    "Initialized sensor '%s': port=%s baud=%d poll_rate_hz=%.1f timeout=%dms",
    sensor_name_.c_str(), device_.c_str(), baud_rate_, poll_rate_hz_, response_timeout_ms_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_state_interfaces — read-only: no export_command_interfaces override,
// hardware_interface::SensorInterface finalises write() to a no-op.
// ─────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface> PyrometerSensorHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.emplace_back(sensor_name_, "process_temperature", &hw_process_temp_c_);
  si.emplace_back(sensor_name_, "detector_temperature", &hw_detector_temp_c_);
  si.emplace_back(sensor_name_, "box_temperature", &hw_box_temp_c_);
  si.emplace_back(sensor_name_, "ratio_temperature", &hw_ratio_temp_c_);
  si.emplace_back(sensor_name_, "t1_temperature", &hw_t1_temp_c_);
  si.emplace_back(sensor_name_, "t2_temperature", &hw_t2_temp_c_);
  si.emplace_back(sensor_name_, "attenuation", &hw_attenuation_pct_);
  return si;
}

// ─────────────────────────────────────────────────────────────────────────────
// Serial helpers
// ─────────────────────────────────────────────────────────────────────────────
bool PyrometerSensorHardware::open_serial_port()
{
  serial_fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(LOGGER, "Failed to open '%s': %s", device_.c_str(), std::strerror(errno));
    return false;
  }

  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(LOGGER, "tcgetattr('%s') failed: %s", device_.c_str(), std::strerror(errno));
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  cfmakeraw(&tty);
  const speed_t speed = baud_to_speed(baud_rate_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;   // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;  // no hardware flow control
  tty.c_cflag &= ~PARENB;   // no parity
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;       // 8 data bits

  // Non-blocking reads — read_exact() handles timing via poll().
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(LOGGER, "tcsetattr('%s') failed: %s", device_.c_str(), std::strerror(errno));
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  return true;
}

void PyrometerSensorHardware::close_serial_port()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

void PyrometerSensorHardware::send_laser(bool on)
{
  write_bytes(on ? CMD_LASER_ON : CMD_LASER_OFF, 3);
}

bool PyrometerSensorHardware::write_bytes(const uint8_t * data, size_t len)
{
  size_t sent = 0;
  while (sent < len) {
    const ssize_t n = ::write(serial_fd_, data + sent, len - sent);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // Output buffer momentarily full — yield instead of busy-spinning.
        pollfd pfd{serial_fd_, POLLOUT, 0};
        ::poll(&pfd, 1, response_timeout_ms_);
        continue;
      }
      if (errno == EINTR) continue;
      RCLCPP_ERROR(LOGGER, "write() failed: %s", std::strerror(errno));
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

bool PyrometerSensorHardware::read_exact(uint8_t * buf, size_t len, int timeout_ms)
{
  size_t got = 0;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);

  while (got < len) {
    const auto remaining = deadline - std::chrono::steady_clock::now();
    const int remaining_ms =
      static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count());
    if (remaining_ms <= 0) return false;

    pollfd pfd{serial_fd_, POLLIN, 0};
    const int rv = ::poll(&pfd, 1, remaining_ms);
    if (rv <= 0) return false;  // timeout or error

    const ssize_t n = ::read(serial_fd_, buf + got, len - got);
    if (n > 0) {
      got += static_cast<size_t>(n);
    } else if (n == 0) {
      // POLLIN with a 0-byte read means the far end (device) hung up —
      // fail fast instead of spinning until the deadline.
      return false;
    } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      return false;
    }
  }
  return true;
}

bool PyrometerSensorHardware::read_temperature_channel(
  uint8_t command, bool has_status_bit, double & out_value_c)
{
  if (!write_bytes(&command, 1)) return false;

  uint8_t resp[2];
  if (!read_exact(resp, 2, response_timeout_ms_)) return false;

  const uint16_t raw = (static_cast<uint16_t>(resp[0]) << 8) | resp[1];
  double signed_val;
  // Bit 15 is a status flag on the process/ratio channels — mask it out
  // before converting to a signed tenths-of-a-degree value.
  if (has_status_bit && (raw & 0x8000)) {
    signed_val = static_cast<double>(raw & 0x7FFF) - 1000.0;
  } else {
    signed_val = static_cast<double>(raw) - 1000.0;
  }
  out_value_c = signed_val / 10.0;
  return true;
}

bool PyrometerSensorHardware::read_attenuation_channel(double & out_value_pct)
{
  uint8_t command = CMD_READ_ATTENUATION;
  if (!write_bytes(&command, 1)) return false;

  uint8_t resp[2];
  if (!read_exact(resp, 2, response_timeout_ms_)) return false;

  const uint16_t raw = (static_cast<uint16_t>(resp[0]) << 8) | resp[1];
  double pct = static_cast<double>(raw) / 10.0;
  if (pct > 100.0) pct = 100.0;  // clamp — firmware can report slightly over 100%
  out_value_pct = pct;
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_configure — open the serial port, power up the laser, clear backlog,
// start the background poll thread.
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn PyrometerSensorHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!open_serial_port()) {
    RCLCPP_ERROR(LOGGER,
      "Could not open pyrometer serial port '%s'. Check wiring and udev rule:\n"
      "  sudo cp <pkg>/udev/99-pyrometer.rules /etc/udev/rules.d/\n"
      "  sudo udevadm control --reload-rules && sudo udevadm trigger", device_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Power up the laser/measurement and give the device time to stabilize.
  send_laser(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Discard any bytes accumulated during power-up / the laser-on command
  // itself (matches the reference script's reset_input_buffer() call).
  tcflush(serial_fd_, TCIFLUSH);

  {
    std::lock_guard<std::mutex> lk(io_mutex_);
    bg_diag_ = PollDiagnostics{};
  }
  read_log_counter_ = 0;

  poll_thread_running_.store(true, std::memory_order_release);
  poll_thread_ = std::thread(&PyrometerSensorHardware::poll_thread_func, this);

  RCLCPP_INFO(LOGGER, "Configured — laser on, poll thread started (port=%s).", device_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_activate / on_deactivate — states are already valid once configured
// (per hardware_interface::SensorInterface semantics), so these are no-ops
// beyond logging.
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn PyrometerSensorHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(LOGGER, "Activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PyrometerSensorHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(LOGGER, "Deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_cleanup — stop the poll thread, laser off, close the port.
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn PyrometerSensorHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  poll_thread_running_.store(false, std::memory_order_release);
  if (poll_thread_.joinable()) {
    poll_thread_.join();
  }

  if (serial_fd_ >= 0) {
    send_laser(false);
    close_serial_port();
  }

  RCLCPP_INFO(LOGGER, "Cleaned up — laser off, port closed.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// read — non-blocking: copies latest sensor data from the background poll thread
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type PyrometerSensorHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  PollDiagnostics diag;
  {
    std::lock_guard<std::mutex> lk(io_mutex_);
    hw_process_temp_c_ = bg_process_temp_c_;
    hw_detector_temp_c_ = bg_detector_temp_c_;
    hw_box_temp_c_ = bg_box_temp_c_;
    hw_ratio_temp_c_ = bg_ratio_temp_c_;
    hw_t1_temp_c_ = bg_t1_temp_c_;
    hw_t2_temp_c_ = bg_t2_temp_c_;
    hw_attenuation_pct_ = bg_attenuation_pct_;
    diag = bg_diag_;
  }

  ++read_log_counter_;
  // Periodic diagnostic log (~every 5 s at a 100 Hz controller_manager rate).
  if (read_log_counter_ % 500 == 0) {
    RCLCPP_INFO(LOGGER,
      "[read] process=%.1fC detector=%.1fC box=%.1fC ratio=%.1fC t1=%.1fC t2=%.1fC att=%.1f%% | "
      "poll cycle=%.2fms n=%" PRIu64 " errs=%" PRIu64,
      hw_process_temp_c_, hw_detector_temp_c_, hw_box_temp_c_, hw_ratio_temp_c_,
      hw_t1_temp_c_, hw_t2_temp_c_, hw_attenuation_pct_,
      diag.cycle_ms, diag.cycle_count, diag.comm_error_count);
  }

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// poll_thread_func — dedicated thread for all pyrometer serial communication.
//
// Runs at best-effort poll_rate_hz_ (>= 100 Hz recommended). Each iteration
// issues all 7 read commands sequentially (the device is half-duplex,
// request/response only) and publishes the results to the bg_* buffer under
// mutex. A cycle with any dropped/malformed response is counted as a comm
// error and the previous values are left in place (no partial/torn update).
// ─────────────────────────────────────────────────────────────────────────────
void PyrometerSensorHardware::poll_thread_func()
{
  RCLCPP_INFO(LOGGER, "[poll_thread] Started (target %.1f Hz).", poll_rate_hz_);

  const auto target_period = std::chrono::duration<double>(1.0 / poll_rate_hz_);

  while (poll_thread_running_.load(std::memory_order_acquire)) {
    const auto t_cycle_start = std::chrono::steady_clock::now();

    double process_c, detector_c, box_c, ratio_c, t1_c, t2_c, atten_pct;
    bool ok = true;
    ok = ok && read_temperature_channel(CMD_READ_PROCESS, /*has_status_bit=*/true, process_c);
    ok = ok && read_temperature_channel(CMD_READ_DETECTOR, /*has_status_bit=*/false, detector_c);
    ok = ok && read_temperature_channel(CMD_READ_BOX, /*has_status_bit=*/false, box_c);
    ok = ok && read_temperature_channel(CMD_READ_RATIO, /*has_status_bit=*/true, ratio_c);
    ok = ok && read_temperature_channel(CMD_READ_T2, /*has_status_bit=*/false, t2_c);
    ok = ok && read_temperature_channel(CMD_READ_T1, /*has_status_bit=*/false, t1_c);
    ok = ok && read_attenuation_channel(atten_pct);

    const auto t_cycle_end = std::chrono::steady_clock::now();
    const double cycle_ms =
      std::chrono::duration<double, std::milli>(t_cycle_end - t_cycle_start).count();

    {
      std::lock_guard<std::mutex> lk(io_mutex_);
      if (ok) {
        bg_process_temp_c_ = process_c;
        bg_detector_temp_c_ = detector_c;
        bg_box_temp_c_ = box_c;
        bg_ratio_temp_c_ = ratio_c;
        bg_t1_temp_c_ = t1_c;
        bg_t2_temp_c_ = t2_c;
        bg_attenuation_pct_ = atten_pct;
        ++bg_diag_.cycle_count;
      } else {
        ++bg_diag_.comm_error_count;
      }
      bg_diag_.cycle_ms = cycle_ms;
    }

    // Throttle only if we finished faster than the configured target period —
    // never sleep longer than needed, and never fall behind silently.
    const auto elapsed = std::chrono::steady_clock::now() - t_cycle_start;
    if (elapsed < target_period) {
      std::this_thread::sleep_for(target_period - elapsed);
    }
  }

  RCLCPP_INFO(LOGGER, "[poll_thread] Stopped.");
}

}  // namespace pyrometer_driver

PLUGINLIB_EXPORT_CLASS(
  pyrometer_driver::PyrometerSensorHardware, hardware_interface::SensorInterface)
