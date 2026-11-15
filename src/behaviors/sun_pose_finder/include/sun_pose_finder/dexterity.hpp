#pragma once

#include <memory>
#include <optional>
#include <tuple>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit_pro_behavior_interface/behavior_context.hpp>

namespace moveit_pro::base
{
class RobotModel;
class RobotState;
class JointModelGroup;
class LinkModel;
}  // namespace moveit_pro::base

namespace sun_pose_finder
{
/**
 * @brief Distance from singularity of a 6-DOF task Jacobian, in [0, 1]: the inverse condition number
 * sigma_min / sigma_max after dividing the translational rows by `characteristic_length_m` so metres and radians
 * are comparable. 0 = singular (a direction the focal point or lens cannot move), 1 = perfectly isotropic.
 * Rows are [v; w] as MoveIt returns them.
 */
double dexterityFromJacobian(const Eigen::MatrixXd& jacobian, double characteristic_length_m);

/**
 * @brief Evaluates dexterity at the focal frame for joint positions given by name, using the robot model from the
 * BehaviorContext (or loaded from robot_description if the context has none).
 */
class DexterityEvaluator
{
public:
  DexterityEvaluator(const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& context, const std::string& group,
                     const std::string& tip_link, double characteristic_length_m);
  ~DexterityEvaluator();

  /// Empty if usable, else why not.
  const std::string& error() const { return error_; }

  /// Dexterity for these joints (others keep default values). nullopt on failure.
  std::optional<double> evaluate(const std::vector<std::string>& names, const std::vector<double>& positions);

  /**
   * @brief d(dexterity)/d(roll): how dexterity changes per rad of lens roll about `axis_world` (the optical axis),
   * estimated by stepping the joints along J^+ * [0; axis] by +-step_rad. nullopt on failure.
   */
  std::optional<double> rollGradient(const std::vector<std::string>& names, const std::vector<double>& positions,
                                     const Eigen::Vector3d& axis_world, double step_rad = 0.02);

  // ---- Accessors for kinematic simulation (all in the model's world frame) ----
  /// Group joint vector from named joint positions (other joints keep their current values). Empty on failure.
  std::optional<Eigen::VectorXd> groupPositions(const std::vector<std::string>& names,
                                                const std::vector<double>& positions);
  /// Pose of the tip link in world for group joints q.
  Eigen::Isometry3d tipPose(const Eigen::VectorXd& q);
  /// 6 x n Jacobian of the tip link origin in WORLD axes, rows [v; w].
  std::optional<Eigen::MatrixXd> jacobianWorld(const Eigen::VectorXd& q);
  /// Position bounds per group variable: (lower, upper, bounded).
  std::vector<std::tuple<double, double, bool>> bounds() const;
  double characteristicLength() const { return length_; }
  /// Frame tipPose/jacobianWorld are expressed in (the robot model frame, e.g. "world"). Empty if unusable.
  std::string modelFrame() const;
  /// Variable names of the group, in the order of the group joint vector.
  std::vector<std::string> variableNames() const;

private:
  std::optional<Eigen::MatrixXd> jacobianAt(const Eigen::VectorXd& group_positions);

  std::shared_ptr<const moveit_pro::base::RobotModel> model_;
  std::unique_ptr<moveit_pro::base::RobotState> state_;
  const moveit_pro::base::JointModelGroup* group_ = nullptr;
  const moveit_pro::base::LinkModel* tip_ = nullptr;
  double length_ = 0.4;
  std::string error_;
};
}  // namespace sun_pose_finder
