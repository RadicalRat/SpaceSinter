#pragma once

#include <cmath>

namespace sun_pose_finder
{
struct SolarPosition
{
  double azimuth_deg;    // clockwise from true north
  double elevation_deg;  // above the horizon, no refraction correction
};

/**
 * @brief Sun position for an observer, using the NOAA solar calculator equations (Meeus).
 * @details Accurate to about 0.02 deg for years 1800-2100, which is well inside the sun camera's field of view, so it
 * is good enough to seed the closed-loop tracker.
 * @param unix_time_s UTC time in seconds since the Unix epoch.
 * @param latitude_deg Observer latitude, north positive.
 * @param longitude_deg Observer longitude, east positive.
 */
inline SolarPosition computeSolarPosition(double unix_time_s, double latitude_deg, double longitude_deg)
{
  constexpr double kDeg = M_PI / 180.0;
  const double julian_day = unix_time_s / 86400.0 + 2440587.5;
  const double t = (julian_day - 2451545.0) / 36525.0;  // Julian centuries since J2000

  const double mean_long = std::fmod(280.46646 + t * (36000.76983 + t * 0.0003032), 360.0);
  const double mean_anom = 357.52911 + t * (35999.05029 - 0.0001537 * t);
  const double ecc = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
  const double center = std::sin(mean_anom * kDeg) * (1.914602 - t * (0.004817 + 0.000014 * t)) +
                        std::sin(2 * mean_anom * kDeg) * (0.019993 - 0.000101 * t) +
                        std::sin(3 * mean_anom * kDeg) * 0.000289;
  const double omega = 125.04 - 1934.136 * t;
  const double app_long = mean_long + center - 0.00569 - 0.00478 * std::sin(omega * kDeg);
  const double obliq_mean = 23.0 + (26.0 + (21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60.0) / 60.0;
  const double obliq = obliq_mean + 0.00256 * std::cos(omega * kDeg);
  const double decl = std::asin(std::sin(obliq * kDeg) * std::sin(app_long * kDeg));

  const double y = std::pow(std::tan(obliq * kDeg / 2.0), 2);
  const double eq_of_time_min =
      4.0 / kDeg *
      (y * std::sin(2 * mean_long * kDeg) - 2 * ecc * std::sin(mean_anom * kDeg) +
       4 * ecc * y * std::sin(mean_anom * kDeg) * std::cos(2 * mean_long * kDeg) -
       0.5 * y * y * std::sin(4 * mean_long * kDeg) - 1.25 * ecc * ecc * std::sin(2 * mean_anom * kDeg));

  const double utc_minutes = std::fmod(unix_time_s, 86400.0) / 60.0;
  const double true_solar_min = std::fmod(utc_minutes + eq_of_time_min + 4.0 * longitude_deg + 1440.0, 1440.0);
  const double hour_angle = (true_solar_min / 4.0 - 180.0) * kDeg;

  const double lat = latitude_deg * kDeg;
  const double cos_zenith = std::sin(lat) * std::sin(decl) + std::cos(lat) * std::cos(decl) * std::cos(hour_angle);
  const double zenith = std::acos(std::fmax(-1.0, std::fmin(1.0, cos_zenith)));
  const double azimuth = std::atan2(std::sin(hour_angle),
                                    std::cos(hour_angle) * std::sin(lat) - std::tan(decl) * std::cos(lat)) /
                             kDeg +
                         180.0;

  return { std::fmod(azimuth + 360.0, 360.0), 90.0 - zenith / kDeg };
}
}  // namespace sun_pose_finder
