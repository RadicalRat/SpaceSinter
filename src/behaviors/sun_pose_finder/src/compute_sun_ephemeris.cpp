#include <sun_pose_finder/compute_sun_ephemeris.hpp>
#include <sun_pose_finder/solar_position.hpp>

#include <chrono>
#include <cmath>
#include <ctime>
#include <optional>

#include <Eigen/Geometry>
#include <fmt/format.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace sun_pose_finder
{
namespace
{
constexpr auto kLogSource = "ComputeSunEphemeris";

// Parses "YYYY-MM-DDTHH:MM:SS" (UTC) into seconds since the Unix epoch.
std::optional<double> parseUtc(const std::string& text)
{
  std::tm tm{};
  if (sscanf(text.c_str(), "%d-%d-%dT%d:%d:%d", &tm.tm_year, &tm.tm_mon, &tm.tm_mday, &tm.tm_hour, &tm.tm_min,
             &tm.tm_sec) != 6)
  {
    return std::nullopt;
  }
  tm.tm_year -= 1900;
  tm.tm_mon -= 1;
  return static_cast<double>(timegm(&tm));
}
}  // namespace

ComputeSunEphemeris::ComputeSunEphemeris(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList ComputeSunEphemeris::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("reference_pose", "{reference_pose}",
                                                   "Pose to rotate toward the sun (e.g. focal point in world). Its "
                                                   "frame must have +Z up."),
    BT::InputPort<double>("latitude_deg", 39.7555, "Observer latitude in degrees, north positive."),
    BT::InputPort<double>("longitude_deg", -105.2211, "Observer longitude in degrees, east positive."),
    BT::InputPort<double>("north_heading_deg", 90.0,
                          "Direction of true north in the reference frame, degrees counter-clockwise about +Z from +X "
                          "(90 means +Y points north)."),
    BT::InputPort<std::string>("utc_time", "",
                               "UTC time as YYYY-MM-DDTHH:MM:SS. Leave empty to use the computer's clock."),
    BT::InputPort<double>("min_elevation_deg", 5.0, "Fail if the sun is lower than this above the horizon."),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("aligned_pose", "{sun_seed_pose}",
                                                    "reference_pose with its +Z axis pointed at the sun."),
    BT::OutputPort<double>("sun_azimuth_deg", "{sun_azimuth_deg}", "Sun azimuth, clockwise from true north."),
    BT::OutputPort<double>("sun_elevation_deg", "{sun_elevation_deg}", "Sun elevation above the horizon."),
  };
}

BT::KeyValueVector ComputeSunEphemeris::metadata()
{
  return { { "description", "Computes the sun direction from date, time and latitude/longitude (NOAA ephemeris) and "
                            "rotates a pose so its +Z axis points at the sun. Used to seed the sun tracker." },
           { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus ComputeSunEphemeris::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::PoseStamped>("reference_pose"), getInput<double>("latitude_deg"),
      getInput<double>("longitude_deg"), getInput<double>("north_heading_deg"), getInput<std::string>("utc_time"),
      getInput<double>("min_elevation_deg"));
  if (!ports)
  {
    shared_resources_->logger->publishFailureMessage(kLogSource, "Missing input: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [reference_pose, latitude, longitude, north_heading, utc_time, min_elevation] = ports.value();

  double unix_time;
  if (utc_time.empty())
  {
    unix_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  }
  else if (const auto parsed = parseUtc(utc_time))
  {
    unix_time = *parsed;
  }
  else
  {
    shared_resources_->logger->publishFailureMessage(kLogSource,
                                                     "utc_time must look like 2026-10-23T18:00:00, got: " + utc_time);
    return BT::NodeStatus::FAILURE;
  }

  const auto sun = computeSolarPosition(unix_time, latitude, longitude);
  setOutput("sun_azimuth_deg", sun.azimuth_deg);
  setOutput("sun_elevation_deg", sun.elevation_deg);
  shared_resources_->logger->publishInfoMessage(
      kLogSource, fmt::format("Sun at azimuth {:.2f} deg, elevation {:.2f} deg", sun.azimuth_deg, sun.elevation_deg));

  if (sun.elevation_deg < min_elevation)
  {
    shared_resources_->logger->publishFailureMessage(
        kLogSource, fmt::format("Sun elevation {:.2f} deg is below min_elevation_deg {:.2f}", sun.elevation_deg,
                                min_elevation));
    return BT::NodeStatus::FAILURE;
  }

  // Azimuth is clockwise from north; frame angles are counter-clockwise from +X.
  constexpr double kDeg = M_PI / 180.0;
  const double heading = (north_heading - sun.azimuth_deg) * kDeg;
  const double elevation = sun.elevation_deg * kDeg;
  const Eigen::Vector3d sun_dir(std::cos(elevation) * std::cos(heading), std::cos(elevation) * std::sin(heading),
                                std::sin(elevation));

  // Smallest rotation taking the pose's +Z onto the sun direction keeps the roll about the lens axis unchanged.
  Eigen::Isometry3d pose;
  tf2::fromMsg(reference_pose.pose, pose);
  const Eigen::Quaterniond align =
      Eigen::Quaterniond::FromTwoVectors(pose.linear() * Eigen::Vector3d::UnitZ(), sun_dir);
  const Eigen::Quaterniond orientation = (align * Eigen::Quaterniond(pose.linear())).normalized();

  geometry_msgs::msg::PoseStamped aligned = reference_pose;
  aligned.pose.orientation = tf2::toMsg(orientation);
  setOutput("aligned_pose", aligned);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace sun_pose_finder
