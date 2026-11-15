#include <sun_pose_finder/raster_scan_sim.hpp>

#include <algorithm>
#include <cmath>

#include <Eigen/SVD>
#include <fmt/format.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit_studio_agent_msgs/msg/robot_joint_state.hpp>
#include <sun_pose_finder/track_raster_with_sun.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace sun_pose_finder
{
namespace
{
Eigen::Vector3d rotationError(const Eigen::Matrix3d& target, const Eigen::Matrix3d& actual)
{
  const Eigen::AngleAxisd aa(target * actual.transpose());
  return aa.angle() * aa.axis();
}

/// One sun-feedforward step: rotate R by omega_ff projected perpendicular to R's lens axis (+Z), as the tracker does.
Eigen::Matrix3d turnWithSun(const Eigen::Matrix3d& R, const Eigen::Vector3d& omega_ff, double dt)
{
  const Eigen::Vector3d a = R.col(2);
  const Eigen::Vector3d w = omega_ff - omega_ff.dot(a) * a;
  const double angle = w.norm() * dt;
  return angle > 0.0 ? Eigen::Matrix3d(Eigen::AngleAxisd(angle, w.normalized()) * R) : R;
}
}  // namespace

ScanSimResult simulateRasterScan(const Eigen::VectorXd& q0, const std::vector<Eigen::Vector3d>& path,
                                 const Eigen::Matrix3d& R0, const ScanSimParams& p, const FkFn& fk, const JacFn& jac,
                                 const std::vector<std::tuple<double, double, bool>>& bounds)
{
  ScanSimResult r;
  if (path.empty() || p.step_m <= 0.0 || p.scan_speed_mps <= 0.0 || p.lift_speed_mps <= 0.0 ||
      bounds.size() != static_cast<size_t>(q0.size()))
  {
    r.why = "bad input (empty path, non-positive step/speed, or bounds size mismatch)";
    return r;
  }
  // Targets: (position, segment speed) along descend -> raster -> lift; the lens axis turns with the sun.
  Eigen::Matrix3d R = turnWithSun(R0, p.omega_ff_world, p.acquire_margin_s);
  struct Target
  {
    Eigen::Vector3d pos;
    double speed;
    bool lifted;  // lifted targets are positioned along the CURRENT lens axis
    double lift;
  };
  std::vector<Target> targets;
  auto addSegment = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b, double speed) {
    const double len = (b - a).norm();
    if (len < 1e-9)
    {
      return;  // duplicate path point: no motion, and a zero time step would give non-increasing trajectory times
    }
    const int n = std::max(1, static_cast<int>(std::ceil(len / p.step_m)));
    for (int k = 1; k <= n; ++k)
    {
      targets.push_back({ a + (b - a) * (static_cast<double>(k) / n), speed, false, 0.0 });
    }
  };
  // Descent: lift from lift_m to 0 (positions computed per step from the current axis below).
  const int n_lift = std::max(1, static_cast<int>(std::ceil(p.lift_m / p.step_m)));
  for (int k = n_lift - 1; k >= 0; --k)
  {
    targets.push_back({ path.front(), p.lift_speed_mps, true, p.lift_m * k / n_lift });
  }
  for (size_t i = 1; i < path.size(); ++i)
  {
    addSegment(path[i - 1], path[i], p.scan_speed_mps);
  }
  for (int k = 1; k <= n_lift; ++k)
  {
    targets.push_back({ path.back(), p.lift_speed_mps, true, p.lift_m * k / n_lift });
  }

  Eigen::VectorXd q = q0;
  Eigen::VectorXd q_min = q0, q_max = q0;
  double t = p.acquire_margin_s;
  const int n = static_cast<int>(q0.size());
  r.samples.push_back(q);
  r.times.push_back(0.0);
  Eigen::Vector3d prev_pos = fk(q).translation();
  for (const auto& tg : targets)
  {
    const Eigen::Vector3d pos = tg.lifted ? Eigen::Vector3d(tg.pos + tg.lift * R.col(2)) : tg.pos;
    const double dt = (pos - prev_pos).norm() / tg.speed;
    prev_pos = pos;
    R = turnWithSun(R, p.omega_ff_world, dt);
    t += dt;
    const Eigen::VectorXd q_prev = q;
    double residual = 0.0;
    double rot_residual = 0.0;
    for (int it = 0; it < p.newton_iterations; ++it)
    {
      const Eigen::Isometry3d T = fk(q);
      Eigen::Matrix<double, 6, 1> e;
      e.head<3>() = pos - T.translation();
      e.tail<3>() = rotationError(R, T.linear());
      residual = e.head<3>().norm();
      if (residual < 1e-6 && e.tail<3>().norm() < 1e-6)
      {
        break;
      }
      const auto J = jac(q);
      if (!J)
      {
        r.why = "Jacobian unavailable";
        return r;
      }
      const Eigen::MatrixXd JJt = (*J) * J->transpose() + p.damping * p.damping * Eigen::MatrixXd::Identity(6, 6);
      q += J->transpose() * JJt.ldlt().solve(e);
    }
    {
      const Eigen::Isometry3d T = fk(q);
      residual = (pos - T.translation()).norm();
      rot_residual = rotationError(R, T.linear()).norm();
    }
    const auto J = jac(q);
    if (!J)
    {
      r.why = "Jacobian unavailable";
      return r;
    }
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(*J);
    r.min_sigma = std::min(r.min_sigma, svd.singularValues()(svd.singularValues().size() - 1));
    r.min_dexterity = std::min(r.min_dexterity, dexterityFromJacobian(*J, p.dexterity_length_m));
    r.max_residual = std::max(r.max_residual, residual);
    r.max_rot_residual = std::max(r.max_rot_residual, rot_residual);
    if (dt > 0.0)
    {
      r.max_joint_speed = std::max(r.max_joint_speed, (q - q_prev).cwiseAbs().maxCoeff() / dt);
    }
    for (int j = 0; j < n; ++j)
    {
      const auto& [lo, hi, bounded] = bounds[j];
      if (bounded)
      {
        r.min_bounded_margin = std::min(r.min_bounded_margin, std::min(q(j) - lo, hi - q(j)));
      }
    }
    q_min = q_min.cwiseMin(q);
    q_max = q_max.cwiseMax(q);
    r.samples.push_back(q);
    r.times.push_back(t - p.acquire_margin_s);
  }
  for (int j = 0; j < n; ++j)
  {
    if (!std::get<2>(bounds[j]))
    {
      r.max_continuous_travel = std::max(r.max_continuous_travel, q_max(j) - q_min(j));
    }
  }
  if (r.max_residual > p.max_residual_m)
  {
    r.why = fmt::format("focal point misses the path by {:.2f} mm (limit {:.2f} mm): out of reach or singular",
                        1e3 * r.max_residual, 1e3 * p.max_residual_m);
  }
  else if (r.max_rot_residual > p.max_rot_residual_rad)
  {
    r.why = fmt::format("lens orientation misses the sun line by {:.2f} mrad (limit {:.2f} mrad): out of reach or singular",
                        1e3 * r.max_rot_residual, 1e3 * p.max_rot_residual_rad);
  }
  else if (r.min_bounded_margin < 0.0)
  {
    r.why = fmt::format("a joint limit is crossed (margin {:.3f} rad)", r.min_bounded_margin);
  }
  else if (r.max_joint_speed > p.max_joint_speed_rps)
  {
    r.why = fmt::format("needs {:.3f} rad/s at a joint (controller limit {:.3f} rad/s): too close to a singularity",
                        r.max_joint_speed, p.max_joint_speed_rps);
  }
  else
  {
    r.ok = true;
  }
  return r;
}

// ---------------------------------------------------------------- OffsetPosesAlongAxis
OffsetPosesAlongAxis::OffsetPosesAlongAxis(const std::string& name, const BT::NodeConfiguration& config,
                                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList OffsetPosesAlongAxis::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("poses", "Poses to offset (positions used)."),
    BT::InputPort<geometry_msgs::msg::Quaternion>("axis_orientation", "{sun_orientation}",
                                                  "Orientation whose +Z is the offset direction (the lens axis); "
                                                  "also set as the output orientation."),
    BT::InputPort<double>("distance", "Offset along that +Z, m."),
    BT::InputPort<bool>("include_original", false, "Also output the original poses (originals first)."),
    BT::InputPort<bool>("first_only", false, "Only offset (and output) the first pose."),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("poses_out", "Offset poses."),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("first_out", "The first offset pose."),
  };
}

BT::KeyValueVector OffsetPosesAlongAxis::metadata()
{
  return { { "description", "Copies poses offset along the +Z of an orientation (e.g. lift raster points along the "
                            "lens axis)." },
           { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus OffsetPosesAlongAxis::tick()
{
  const auto poses = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("poses");
  const auto q = getInput<geometry_msgs::msg::Quaternion>("axis_orientation");
  const auto d = getInput<double>("distance");
  if (!poses || !q || !d || poses->empty())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Need non-empty poses, axis_orientation and distance.");
    return BT::NodeStatus::FAILURE;
  }
  const Eigen::Vector3d axis =
      Eigen::Quaterniond(q->w, q->x, q->y, q->z).normalized().toRotationMatrix().col(2);
  const bool first_only = getInput<bool>("first_only").value_or(false);
  std::vector<geometry_msgs::msg::PoseStamped> out;
  if (getInput<bool>("include_original").value_or(false))
  {
    out.insert(out.end(), poses->begin(), first_only ? poses->begin() + 1 : poses->end());
  }
  const size_t n = first_only ? 1 : poses->size();
  const size_t first_offset_index = out.size();
  for (size_t i = 0; i < n; ++i)
  {
    auto ps = (*poses)[i];
    ps.pose.position.x += *d * axis.x();
    ps.pose.position.y += *d * axis.y();
    ps.pose.position.z += *d * axis.z();
    ps.pose.orientation = *q;
    out.push_back(ps);
  }
  setOutput("poses_out", out);
  setOutput("first_out", out[first_offset_index]);
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------- SimulateRasterScan
SimulateRasterScan::SimulateRasterScan(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

SimulateRasterScan::~SimulateRasterScan() = default;

BT::PortsList SimulateRasterScan::providedPorts()
{
  return {
    BT::InputPort<moveit_studio_agent_msgs::msg::RobotJointState>("seed_joint_state", "{scan_seed}",
                                                                   "Joint state at the LIFTED start (IK solution)."),
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("path", "{raster_path}", "Raster (world)."),
    BT::InputPort<geometry_msgs::msg::Quaternion>("orientation", "Lens orientation for the scan (world)."),
    BT::InputPort<double>("lift_m", 0.08, "Lifted start/end height along the lens axis."),
    BT::InputPort<double>("scan_speed_mps", 0.001, "Scan speed V."),
    BT::InputPort<double>("lift_speed_mps", 0.003, "Lift/descent speed."),
    BT::InputPort<double>("step_m", 0.002, "Spacing of simulated targets."),
    BT::InputPort<double>("damping", 0.005, "DLS lambda (match velocity_force_controller jacobian_damping)."),
    BT::InputPort<double>("max_residual_m", 0.0005, "Fail if a target is missed by more than this."),
    BT::InputPort<double>("max_joint_speed_rps", 0.3, "Fail if a joint must move faster (controller limit)."),
    BT::InputPort<double>("acquire_margin_s", 30.0, "Assumed acquisition time before the descent (sun drift)."),
    BT::InputPort<bool>("sun_feedforward", true, "Turn the orientation with the sun as the tracker does."),
    BT::InputPort<double>("latitude_deg", 39.7555, "Site latitude."),
    BT::InputPort<double>("north_heading_deg", 125.0, "True north in world, deg CCW from +X."),
    BT::InputPort<double>("sun_feedforward_scale", 1.0, "Feedforward scale (match the tracker's)."),
    BT::InputPort<std::string>("planning_group", "manipulator", "Joint group."),
    BT::InputPort<std::string>("tip_link", "ee_focal_point_link", "Controlled frame."),
    BT::InputPort<double>("dexterity_length_m", 0.4, "Translation/rotation scale for dexterity."),
    BT::OutputPort<double>("min_dexterity", "{scan_min_dexterity}", "Worst dexterity along the simulated scan."),
    BT::OutputPort<double>("min_sigma", "{scan_min_sigma}", "Smallest Jacobian singular value along it."),
    BT::OutputPort<double>("min_joint_margin_rad", "{scan_min_joint_margin}", "Smallest margin to a joint limit."),
    BT::OutputPort<double>("max_joint_speed_rps", "{scan_max_joint_speed}", "Peak joint speed at the planned speeds."),
    BT::OutputPort<double>("wrist_travel_rad", "{scan_wrist_travel}", "Largest range swept by a continuous joint."),
    BT::OutputPort<trajectory_msgs::msg::JointTrajectory>("trajectory", "{scan_trajectory}",
                                                          "Simulated joint trajectory (for ValidateTrajectory)."),
  };
}

BT::KeyValueVector SimulateRasterScan::metadata()
{
  return { { "description", "Rehearses the continuous scan kinematically from a seed joint state (descend, raster, "
                            "lift) with the controller's damped least squares and the sun-turning orientation; "
                            "fails on missed targets, joint limits or excessive joint speed, and outputs the worst "
                            "dexterity and the simulated trajectory." },
           { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus SimulateRasterScan::tick()
{
  const auto seed = getInput<moveit_studio_agent_msgs::msg::RobotJointState>("seed_joint_state");
  const auto path = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("path");
  const auto q = getInput<geometry_msgs::msg::Quaternion>("orientation");
  if (!seed || !path || !q || path->empty())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Need seed_joint_state, a non-empty path and orientation.");
    return BT::NodeStatus::FAILURE;
  }
  if (!evaluator_)
  {
    evaluator_ = std::make_unique<DexterityEvaluator>(
        shared_resources_, getInput<std::string>("planning_group").value_or("manipulator"),
        getInput<std::string>("tip_link").value_or("ee_focal_point_link"),
        getInput<double>("dexterity_length_m").value_or(0.4));
  }
  if (!evaluator_->error().empty())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Robot model unavailable: " + evaluator_->error());
    return BT::NodeStatus::FAILURE;
  }
  const auto q0 = evaluator_->groupPositions(seed->joint_state.name, seed->joint_state.position);
  if (!q0)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Seed joint state does not match the robot model.");
    return BT::NodeStatus::FAILURE;
  }
  ScanSimParams p;
  p.lift_m = getInput<double>("lift_m").value_or(0.08);
  p.scan_speed_mps = getInput<double>("scan_speed_mps").value_or(0.001);
  p.lift_speed_mps = getInput<double>("lift_speed_mps").value_or(0.003);
  p.step_m = getInput<double>("step_m").value_or(0.002);
  p.damping = getInput<double>("damping").value_or(0.005);
  p.max_residual_m = getInput<double>("max_residual_m").value_or(0.0005);
  p.max_joint_speed_rps = getInput<double>("max_joint_speed_rps").value_or(0.3);
  p.acquire_margin_s = getInput<double>("acquire_margin_s").value_or(30.0);
  p.dexterity_length_m = evaluator_->characteristicLength();
  const auto use_ff = getInput<bool>("sun_feedforward");
  if (!use_ff)
  {
    shared_resources_->logger->publishFailureMessage(name(), "sun_feedforward: " + use_ff.error());
    return BT::NodeStatus::FAILURE;
  }
  if (*use_ff)
  {
    p.omega_ff_world = sunFeedforwardWorld(getInput<double>("latitude_deg").value_or(39.7555),
                                           getInput<double>("north_heading_deg").value_or(125.0),
                                           getInput<double>("sun_feedforward_scale").value_or(1.0));
  }
  // The simulation runs in the robot model frame: refuse a path in any other frame rather than silently misplace it.
  const std::string model_frame = evaluator_->modelFrame();
  for (const auto& ps : *path)
  {
    if (!ps.header.frame_id.empty() && ps.header.frame_id != model_frame)
    {
      shared_resources_->logger->publishFailureMessage(
          name(), fmt::format("Path pose in frame '{}', expected the robot model frame '{}'.", ps.header.frame_id,
                              model_frame));
      return BT::NodeStatus::FAILURE;
    }
  }
  std::vector<Eigen::Vector3d> pts;
  for (const auto& ps : *path)
  {
    pts.emplace_back(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
  }
  const Eigen::Matrix3d R0 = Eigen::Quaterniond(q->w, q->x, q->y, q->z).normalized().toRotationMatrix();
  auto* ev = evaluator_.get();
  const auto r = simulateRasterScan(
      *q0, pts, R0, p, [ev](const Eigen::VectorXd& x) { return ev->tipPose(x); },
      [ev](const Eigen::VectorXd& x) { return ev->jacobianWorld(x); }, evaluator_->bounds());

  setOutput("min_dexterity", r.min_dexterity);
  setOutput("min_sigma", r.min_sigma);
  setOutput("min_joint_margin_rad", r.min_bounded_margin);
  setOutput("max_joint_speed_rps", r.max_joint_speed);
  setOutput("wrist_travel_rad", r.max_continuous_travel);
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = evaluator_->variableNames();
  // Every 5th sample (~10 mm apart, enough for collision validation) plus the final (lifted end) configuration.
  std::vector<size_t> keep;
  for (size_t i = 0; i < r.samples.size(); i += 5)
  {
    keep.push_back(i);
  }
  if (!r.samples.empty() && keep.back() != r.samples.size() - 1)
  {
    keep.push_back(r.samples.size() - 1);
  }
  for (const size_t i : keep)
  {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(r.samples[i].data(), r.samples[i].data() + r.samples[i].size());
    // Strictly increasing times (a trajectory validator may reject equal stamps).
    const double t_prev = traj.points.empty() ? -1.0 : rclcpp::Duration(traj.points.back().time_from_start).seconds();
    pt.time_from_start = rclcpp::Duration::from_seconds(std::max(r.times[i], t_prev + 1e-3));
    traj.points.push_back(pt);
  }
  setOutput("trajectory", traj);
  const std::string summary =
      fmt::format("min dexterity {:.3f}, min sigma {:.4f}, joint-limit margin {:.2f} rad, peak joint speed {:.3f} "
                  "rad/s, wrist travel {:.1f} deg, max residual {:.2f} mm / {:.2f} mrad",
                  r.min_dexterity, r.min_sigma, r.min_bounded_margin, r.max_joint_speed,
                  r.max_continuous_travel * 180.0 / M_PI, 1e3 * r.max_residual, 1e3 * r.max_rot_residual);
  if (!r.ok)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Simulated scan fails: " + r.why + " (" + summary + ")");
    return BT::NodeStatus::FAILURE;
  }
  shared_resources_->logger->publishInfoMessage(name(), "Simulated scan OK: " + summary);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace sun_pose_finder
