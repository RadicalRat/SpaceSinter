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

// joint_math.hpp — pure, hardware-free conversions and control math for the
// Kinova Gen2 driver.  Kept free of ROS and SDK dependencies so it can be
// unit-tested in isolation (see test/test_joint_math.cpp).

#ifndef KINOVA_DRIVER__JOINT_MATH_HPP_
#define KINOVA_DRIVER__JOINT_MATH_HPP_

#include <algorithm>
#include <cmath>

namespace kinova_driver
{

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

/// Wrap an angle to (−π, π].
inline double wrapToPi(double rad)
{
  double r = std::remainder(rad, 2.0 * M_PI);  // [−π, π]
  if (r <= -M_PI) {
    r += 2.0 * M_PI;
  }
  return r;
}

/// Convert a raw firmware joint angle (degrees, [0, 360)) to the radian value
/// reported to ros2_control.  Continuous joints are wrapped to (−π, π] to match
/// MoveIt's representation; revolute joints keep the firmware's range.
inline double kinovaPositionToRad(float raw_deg, bool continuous)
{
  const double rad = static_cast<double>(raw_deg) * kDegToRad;
  return continuous ? wrapToPi(rad) : rad;
}

/// Convert a raw GetAngularVelocity reading to rad/s.
///
/// Two firmware quirks are corrected here:
///   * Values in (180, 360] encode negative speeds (−180, 0] deg/s.
///   * The reported magnitude is half the true joint speed.
inline double kinovaVelocityToRad(float raw)
{
  double deg_per_s = static_cast<double>(raw);
  if (deg_per_s > 180.0) {
    deg_per_s -= 360.0;
  }
  return 2.0 * deg_per_s * kDegToRad;
}

/// Position error used by the streaming P-controller.  For continuous joints
/// the error takes the shortest path around the circle, so a command of +179°
/// with the joint at −179° yields +2°, not −358°.
inline double positionError(double cmd_rad, double pos_rad, bool continuous)
{
  const double err = cmd_rad - pos_rad;
  return continuous ? wrapToPi(err) : err;
}

/// Velocity-based P-controller step: error (rad) → velocity command (deg/s),
/// clamped to ±max_vel_deg.
inline float positionErrorToVelocityDeg(double err_rad, double kp, double max_vel_deg)
{
  const double vel_deg = std::clamp(kp * err_rad * kRadToDeg, -max_vel_deg, max_vel_deg);
  return static_cast<float>(vel_deg);
}

/// Velocity passthrough: rad/s → deg/s, clamped to ±max_vel_deg.
inline float velocityCommandToDeg(double vel_rad, double max_vel_deg)
{
  return static_cast<float>(std::clamp(vel_rad * kRadToDeg, -max_vel_deg, max_vel_deg));
}

}  // namespace kinova_driver

#endif  // KINOVA_DRIVER__JOINT_MATH_HPP_
