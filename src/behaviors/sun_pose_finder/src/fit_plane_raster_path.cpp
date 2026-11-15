#include <sun_pose_finder/fit_plane_raster_path.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

#include <Eigen/Eigenvalues>
#include <fmt/format.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace sun_pose_finder
{
namespace
{
constexpr auto kLogSource = "FitPlaneRasterPath";
constexpr auto kTiltTopic = "/sinter/bed_tilt_deg";
constexpr auto kPlaneTopic = "/sinter/bed_plane";
constexpr auto kPathTopic = "/sinter/raster_path";
constexpr int kRansacIterations = 500;
constexpr int kRefineIterations = 3;
constexpr size_t kMinInliers = 3;

// Points within `distance` of the plane through `origin` with unit `normal`.
std::vector<Eigen::Vector3d> pointsNearPlane(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& origin,
                                             const Eigen::Vector3d& normal, double distance)
{
  std::vector<Eigen::Vector3d> near;
  for (const auto& p : points)
  {
    if (std::abs((p - origin).dot(normal)) <= distance)
    {
      near.push_back(p);
    }
  }
  return near;
}

// RANSAC: the plane through 3 random points that the most points lie within `distance` of. Fixed seed so the same
// cloud always gives the same raster.
std::vector<Eigen::Vector3d> ransacInliers(const std::vector<Eigen::Vector3d>& points, double distance)
{
  std::mt19937 rng(0);
  std::uniform_int_distribution<size_t> pick(0, points.size() - 1);
  size_t best_count = 0;
  Eigen::Vector3d best_origin = Eigen::Vector3d::Zero(), best_normal = Eigen::Vector3d::UnitZ();
  for (int i = 0; i < kRansacIterations; ++i)
  {
    const Eigen::Vector3d& a = points[pick(rng)];
    const Eigen::Vector3d cross = (points[pick(rng)] - a).cross(points[pick(rng)] - a);
    if (cross.norm() < 1e-12)
    {
      continue;  // repeated or collinear sample
    }
    const Eigen::Vector3d normal = cross.normalized();
    size_t count = 0;
    for (const auto& p : points)
    {
      count += std::abs((p - a).dot(normal)) <= distance;
    }
    if (count > best_count)
    {
      best_count = count;
      best_origin = a;
      best_normal = normal;
    }
  }
  return pointsNearPlane(points, best_origin, best_normal, distance);
}
}  // namespace

tl::expected<PlaneRaster, std::string> fitPlaneRaster(const std::vector<Eigen::Vector3d>& all_points,
                                                      double line_spacing, double point_spacing, double edge_margin,
                                                      double outlier_distance, int line_subdivisions, double square_size)
{
  if (all_points.size() < 3)
  {
    return tl::make_unexpected(fmt::format("Need at least 3 points to fit a plane, got {}", all_points.size()));
  }
  if (line_spacing <= 0.0)
  {
    return tl::make_unexpected("line_spacing must be greater than 0");
  }
  if (line_subdivisions < 1)
  {
    return tl::make_unexpected("line_subdivisions must be at least 1");
  }

  std::vector<Eigen::Vector3d> points = outlier_distance > 0.0 ? ransacInliers(all_points, outlier_distance) : all_points;
  if (points.size() < kMinInliers)
  {
    return tl::make_unexpected(fmt::format("Only {} of {} points are within {:.1f} mm of a plane", points.size(),
                                           all_points.size(), outlier_distance * 1000.0));
  }

  Eigen::Vector3d centroid;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
  // Least-squares refit on the inliers, then re-select inliers against the refined plane.
  for (int refine = 0;; ++refine)
  {
    centroid = Eigen::Vector3d::Zero();
    for (const auto& p : points)
    {
      centroid += p;
    }
    centroid /= static_cast<double>(points.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto& p : points)
    {
      const Eigen::Vector3d d = p - centroid;
      covariance += d * d.transpose();
    }
    covariance /= static_cast<double>(points.size());
    solver.compute(covariance);
    if (outlier_distance <= 0.0 || refine == kRefineIterations)
    {
      break;
    }
    auto refined = pointsNearPlane(all_points, centroid, solver.eigenvectors().col(0), outlier_distance);
    if (refined.size() < kMinInliers)
    {
      break;
    }
    points = std::move(refined);
  }

  // Eigenvalues ascending: the smallest is the normal, the largest the long in-plane axis.
  Eigen::Vector3d normal = solver.eigenvectors().col(0);
  Eigen::Vector3d u = solver.eigenvectors().col(2);
  if (normal.z() < 0.0)
  {
    normal = -normal;
  }
  if (square_size > 0.0)
  {
    // Fixed square: lines along the plane's projection of the points' frame +X (repeatable; the PCA long axis of a
    // square crop is arbitrary), falling back to +Y if the plane is nearly perpendicular to X.
    Eigen::Vector3d axis = Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitX().dot(normal) * normal;
    if (axis.norm() < 1e-6)
    {
      axis = Eigen::Vector3d::UnitY() - Eigen::Vector3d::UnitY().dot(normal) * normal;
    }
    u = axis.normalized();
  }
  const Eigen::Vector3d v = normal.cross(u).normalized();
  u = v.cross(normal);

  Eigen::Isometry3d plane = Eigen::Isometry3d::Identity();
  plane.linear().col(0) = u;
  plane.linear().col(1) = v;
  plane.linear().col(2) = normal;
  plane.translation() = centroid;

  double u_min = std::numeric_limits<double>::max(), u_max = -u_min;
  double v_min = u_min, v_max = -u_min;
  for (const auto& p : points)
  {
    const Eigen::Vector3d d = p - centroid;
    u_min = std::min(u_min, d.dot(u));
    u_max = std::max(u_max, d.dot(u));
    v_min = std::min(v_min, d.dot(v));
    v_max = std::max(v_max, d.dot(v));
  }
  if (square_size > 0.0)
  {
    // The square must lie on the measured surface (within 5 mm of the inliers' extent), or part of the raster
    // would sit on a plane extrapolated past the bed edge.
    const double half = 0.5 * square_size, slack = 0.005;
    if (-half < u_min - slack || half > u_max + slack || -half < v_min - slack || half > v_max + slack)
    {
      return tl::make_unexpected(fmt::format(
          "raster_size {:.3f} m does not fit on the fitted surface ({:.3f} x {:.3f} m around its centroid, offsets "
          "u [{:.3f}, {:.3f}] v [{:.3f}, {:.3f}]): enlarge or re-centre the crop box, or reduce raster_size_m",
          square_size, u_max - u_min, v_max - v_min, u_min, u_max, v_min, v_max));
    }
    // A square of side square_size centred on the fitted plane's centroid, independent of where the points end.
    u_min = v_min = -0.5 * square_size;
    u_max = v_max = 0.5 * square_size;
  }
  u_min += edge_margin;
  u_max -= edge_margin;
  v_min += edge_margin;
  v_max -= edge_margin;
  if (u_min > u_max || v_min > v_max)
  {
    return tl::make_unexpected(fmt::format("edge_margin {:.3f} m leaves no area on a {:.3f} x {:.3f} m plane",
                                           edge_margin, u_max - u_min + 2 * edge_margin,
                                           v_max - v_min + 2 * edge_margin));
  }

  // Clip each scan line to the points within half a line spacing of it, so the raster covers the surface itself and
  // not its bounding rectangle in (u, v). That rectangle overhangs a square crop when u lands on a diagonal (the long
  // axis of a square is arbitrary), putting waypoints off the surface and often out of reach.
  const int num_lines = static_cast<int>(std::floor((v_max - v_min) / line_spacing + 1e-9)) + 1;
  std::vector<double> line_u_min(num_lines, std::numeric_limits<double>::max());
  std::vector<double> line_u_max(num_lines, -std::numeric_limits<double>::max());
  if (square_size > 0.0)
  {
    std::fill(line_u_min.begin(), line_u_min.end(), u_min - edge_margin);
    std::fill(line_u_max.begin(), line_u_max.end(), u_max + edge_margin);
  }
  for (const auto& p : points)
  {
    if (square_size > 0.0)
    {
      break;  // fixed square: every line spans the full square
    }
    const Eigen::Vector3d d = p - centroid;
    const int line = static_cast<int>(std::lround((d.dot(v) - v_min) / line_spacing));
    if (line >= 0 && line < num_lines)
    {
      line_u_min[line] = std::min(line_u_min[line], d.dot(u));
      line_u_max[line] = std::max(line_u_max[line], d.dot(u));
    }
  }

  PlaneRaster result{ plane, {}, std::sqrt(std::max(0.0, solver.eigenvalues()(0))), points.size(),
                      all_points.size() - points.size(), {} };
  int lines_used = 0;
  for (int line = 0; line < num_lines; ++line)
  {
    const double lo = std::max(u_min, line_u_min[line] + edge_margin);
    const double hi = std::min(u_max, line_u_max[line] - edge_margin);
    if (lo > hi)
    {
      continue;  // no surface along this line (or too little to keep edge_margin)
    }
    // The finer of point_spacing and line_subdivisions wins.
    const int segments_per_line =
        std::max(line_subdivisions,
                 point_spacing > 0.0 ? static_cast<int>(std::ceil((hi - lo) / point_spacing - 1e-9)) : 1);
    // Square mode: centre the lines in the square (the span is rarely a whole number of spacings).
    const double v_offset = square_size > 0.0 ? 0.5 * ((v_max - v_min) - (num_lines - 1) * line_spacing) : 0.0;
    const double v_line = v_min + v_offset + line * line_spacing;
    const size_t first = result.poses.size();
    const size_t last = first + segments_per_line;
    const bool forward = lines_used % 2 == 0;  // forward lines start at lo
    // Reachability check points: both ends of every line plus its middle waypoint (if it has one). Corners alone
    // missed interior joint-limit/collision failures, and checking every waypoint was too slow.
    result.extremes.push_back(first);
    if (segments_per_line >= 2)
    {
      result.extremes.push_back(first + segments_per_line / 2);
    }
    result.extremes.push_back(last);
    for (int k = 0; k <= segments_per_line; ++k)
    {
      const double frac = static_cast<double>(k) / segments_per_line;
      const double u_pt = forward ? lo + frac * (hi - lo) : hi - frac * (hi - lo);
      Eigen::Isometry3d pose = plane;
      pose.translation() = centroid + u_pt * u + v_line * v;
      result.poses.push_back(pose);
    }
    ++lines_used;
  }
  return result;
}

FitPlaneRasterPath::FitPlaneRasterPath(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
  // Latched so a rosbag (or ros2 topic echo) started after the fit still gets the last bed measurement.
  , tilt_pub_(shared_resources->node->create_publisher<std_msgs::msg::Float64>(kTiltTopic,
                                                                               rclcpp::QoS(1).transient_local()))
  , plane_pub_(shared_resources->node->create_publisher<geometry_msgs::msg::PoseStamped>(
        kPlaneTopic, rclcpp::QoS(1).transient_local()))
  , path_pub_(shared_resources->node->create_publisher<nav_msgs::msg::Path>(kPathTopic,
                                                                            rclcpp::QoS(1).transient_local()))
{
}

BT::PortsList FitPlaneRasterPath::providedPorts()
{
  return {
    BT::InputPort<sensor_msgs::msg::PointCloud2>("point_cloud", "{point_cloud_cropped}",
                                                 "Cropped point cloud of the surface to sinter."),
    BT::InputPort<double>("line_spacing", 0.01, "M: distance between scan lines, in meters."),
    BT::InputPort<double>("point_spacing", 0.0,
                          "Maximum distance between waypoints along a scan line, in meters. 0 = line ends only."),
    BT::InputPort<double>("raster_size_m", 0.0,
                          "If > 0: the raster is a square of this side centred on the fitted plane's centroid "
                          "(lines along the plane's projection of world +X, centred in the square). The measured "
                          "surface must cover the square (within 5 mm), so make the crop box a little larger than "
                          "the square; the fit fails otherwise. 0 = fill the cropped surface."),
    BT::InputPort<int>("line_subdivisions", 1,
                       "Split each scan line into this many equal moves (4 = waypoints at 1/4, 2/4 and 3/4 between the "
                       "ends), so the sun is re-checked more often. 1 = line ends only."),
    BT::InputPort<double>("edge_margin", 0.0, "Shrink the raster this far in from the edge of the cloud, in meters."),
    BT::InputPort<double>("outlier_distance", 0.005,
                          "Drop points farther than this from the plane (RANSAC), in meters. 0 = keep all points."),
    BT::InputPort<int>("min_points", 1000,
                       "Fail if fewer plane inliers than this: the crop box missed the surface or depth is bad."),
    BT::InputPort<double>("max_tilt_deg", 90.0,
                          "Fail if the plane normal is tilted more than this from +Z of the cloud's frame. 90 = no "
                          "limit (the bed may be propped toward the sun)."),
    BT::InputPort<double>("surface_offset", 0.0,
                          "Move the waypoints this far along the plane normal, in meters (+ = up, off the surface). "
                          "Corrects a depth camera that sees the surface too low or high, so the focus lands on it."),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("path", "{raster_path}",
                                                                 "Zig-zag waypoints on the fitted plane."),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("plane_pose", "{plane_pose}",
                                                    "Fitted plane: centroid, X along scan lines, Z along normal."),
    BT::OutputPort<double>("tilt_deg", "{bed_tilt_deg}", "Tilt of the plane normal from +Z of the cloud's frame."),
    BT::OutputPort<int>("num_waypoints", "{num_waypoints}", "N: number of waypoints in the path."),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "extremes", "{raster_extremes}",
        "Waypoints of `path` to check for reachability: both ends and the middle waypoint of every scan line (about "
        "3 per line, in path order). Much faster than checking every waypoint, and catches interior failures that "
        "the raster corners alone miss."),
  };
}

BT::KeyValueVector FitPlaneRasterPath::metadata()
{
  return { { "description", "Fits a plane to a point cloud and generates a zig-zag raster of N waypoints on it, with "
                            "scan lines M apart." },
           { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus FitPlaneRasterPath::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<sensor_msgs::msg::PointCloud2>("point_cloud"), getInput<double>("line_spacing"),
      getInput<double>("point_spacing"), getInput<double>("edge_margin"), getInput<double>("outlier_distance"),
      getInput<int>("min_points"), getInput<double>("max_tilt_deg"), getInput<double>("surface_offset"),
      getInput<int>("line_subdivisions"));
  if (!ports)
  {
    shared_resources_->logger->publishFailureMessage(kLogSource, "Missing input: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [cloud, line_spacing, point_spacing, edge_margin, outlier_distance, min_points, max_tilt_deg,
               surface_offset, line_subdivisions] =
      ports.value();

  std::vector<Eigen::Vector3d> points;
  points.reserve(static_cast<size_t>(cloud.width) * cloud.height);
  for (sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"); x != x.end();
       ++x, ++y, ++z)
  {
    if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z))
    {
      points.emplace_back(*x, *y, *z);
    }
  }

  const double raster_size = getInput<double>("raster_size_m").value_or(0.0);
  const auto raster = fitPlaneRaster(points, line_spacing, point_spacing, edge_margin, outlier_distance,
                                     line_subdivisions, raster_size);
  if (!raster)
  {
    shared_resources_->logger->publishFailureMessage(kLogSource, raster.error());
    return BT::NodeStatus::FAILURE;
  }

  // Tilt of the normal from +Z of the cloud's frame: a level surface seen through a correct TF chain gives ~0.
  const double tilt_deg = std::acos(std::clamp(raster->plane.linear()(2, 2), -1.0, 1.0)) * 180.0 / M_PI;
  const auto fit_summary =
      fmt::format("{} points, dropped {} outliers (RMS {:.1f} mm, tilted {:.1f} deg from {} +Z)", raster->num_inliers,
                  raster->num_outliers, raster->rms_m * 1000.0, tilt_deg, cloud.header.frame_id);
  if (static_cast<int>(raster->num_inliers) < min_points || tilt_deg > max_tilt_deg)
  {
    shared_resources_->logger->publishFailureMessage(
        kLogSource, fmt::format("Rejected plane fit: {}. Need at least {} points and at most {:.0f} deg tilt; check that "
                                "the crop box is on the surface.",
                                fit_summary, min_points, max_tilt_deg));
    return BT::NodeStatus::FAILURE;
  }

  std::vector<geometry_msgs::msg::PoseStamped> path;
  const Eigen::Vector3d lift = surface_offset * raster->plane.linear().col(2);
  for (const auto& pose : raster->poses)
  {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = cloud.header;
    Eigen::Isometry3d lifted = pose;
    lifted.translation() += lift;
    stamped.pose = tf2::toMsg(lifted);
    path.push_back(stamped);
  }
  geometry_msgs::msg::PoseStamped plane_pose;
  plane_pose.header = cloud.header;
  plane_pose.pose = tf2::toMsg(raster->plane);
  std_msgs::msg::Float64 tilt_msg;
  tilt_msg.data = tilt_deg;
  tilt_pub_->publish(tilt_msg);
  plane_pub_->publish(plane_pose);
  nav_msgs::msg::Path path_msg;
  path_msg.header = cloud.header;
  path_msg.poses = path;
  path_pub_->publish(path_msg);

  setOutput("path", path);
  setOutput("plane_pose", plane_pose);
  setOutput("num_waypoints", static_cast<int>(path.size()));
  std::vector<geometry_msgs::msg::PoseStamped> extremes;
  for (const size_t i : raster->extremes)
  {
    extremes.push_back(path[i]);
  }
  setOutput("extremes", extremes);
  setOutput("tilt_deg", tilt_deg);
  shared_resources_->logger->publishInfoMessage(
      kLogSource, fmt::format("Fitted plane to {}; raster has {} waypoints", fit_summary, path.size()));
  return BT::NodeStatus::SUCCESS;
}
}  // namespace sun_pose_finder
