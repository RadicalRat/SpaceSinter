#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <sun_pose_finder/dexterity.hpp>

namespace sun_pose_finder
{
/**
 * Conventions: world frame +Z up; the controlled frame is ee_focal_point_link, whose +Z is the lens axis (pointing
 * from the focal point toward the lens / sun). Jacobians are 6 x n with rows [v; omega] of the focal-point origin,
 * expressed in world axes. Orientation error is the rotation vector of R_target * R_actual^T (world axes).
 */
struct ScanSimParams
{
  double step_m = 0.002;          // spacing of simulated targets along the path
  double damping = 0.005;         // DLS lambda: the velocity_force_controller's jacobian_damping
  double max_residual_m = 0.0005; // fail if the solved focal point misses a target by more than this
  double max_rot_residual_rad = 0.001;  // ... or its orientation by more than this
  double lift_m = 0.08;           // lifted start/end height along the lens axis
  double scan_speed_mps = 0.001;  // V, sets the time base for the orientation drift and joint speeds
  double lift_speed_mps = 0.003;
  double acquire_margin_s = 30.0; // time assumed between the scored start and the descent (acquisition)
  double max_joint_speed_rps = 0.3;
  double dexterity_length_m = 0.4;
  Eigen::Vector3d omega_ff_world = Eigen::Vector3d::Zero();  // sun feedforward (projected per step, as executed)
  int newton_iterations = 10;
};

struct ScanSimResult
{
  bool ok = false;
  std::string why;
  double min_dexterity = 1.0;
  double min_sigma = 1e9;              // smallest singular value of the raw Jacobian (what the damping sees)
  double min_bounded_margin = 1e9;     // rad, over joints with position limits
  double max_joint_speed = 0.0;        // rad/s at the planned speeds
  double max_continuous_travel = 0.0;  // rad, largest range swept by a continuous joint (cable wrap)
  double max_residual = 0.0;           // m
  double max_rot_residual = 0.0;       // rad
  std::vector<Eigen::VectorXd> samples;  // joint vectors along the simulated scan
  std::vector<double> times;             // s, matching samples
};

using FkFn = std::function<Eigen::Isometry3d(const Eigen::VectorXd&)>;
using JacFn = std::function<std::optional<Eigen::MatrixXd>(const Eigen::VectorXd&)>;

/**
 * Kinematic rehearsal of what TrackRasterWithSun will command: from joint configuration q0 (at the lifted start),
 * descend lift_m along the lens axis to path[0], follow the polyline, lift at the end. Each target is solved with
 * damped-least-squares Newton steps from the previous solution, while the target orientation turns with the projected
 * sun feedforward over the planned time. The Newton iterations converge to the exact IK solution on the branch nearest
 * the previous one; the real controller tracks that with a small lag (it does not reproduce the DLS lag itself), so
 * the metrics describe the configurations the arm passes through, not the transient tracking error.
 * bounds: (lower, upper, bounded) per joint.
 */
ScanSimResult simulateRasterScan(const Eigen::VectorXd& q0, const std::vector<Eigen::Vector3d>& path,
                                 const Eigen::Matrix3d& R0, const ScanSimParams& p, const FkFn& fk, const JacFn& jac,
                                 const std::vector<std::tuple<double, double, bool>>& bounds);

/// Copies poses offset `distance` along the +Z axis of `axis_orientation` (world), orientation set to it.
class OffsetPosesAlongAxis : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  OffsetPosesAlongAxis(const std::string& name, const BT::NodeConfiguration& config,
                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);
  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;
};

/// Behavior wrapper of simulateRasterScan using the robot model; outputs metrics and the simulated trajectory.
class SimulateRasterScan : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  SimulateRasterScan(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);
  ~SimulateRasterScan() override;
  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;

private:
  std::unique_ptr<DexterityEvaluator> evaluator_;
};
}  // namespace sun_pose_finder
