#include <sun_pose_finder/dexterity.hpp>

#include <mutex>

#include <Eigen/SVD>
#include <moveit_pro_base/robot_model/robot_model.hpp>
#include <moveit_pro_base/robot_model_loader/robot_model_loader.hpp>
#include <moveit_pro_base/robot_state/robot_state.hpp>

namespace sun_pose_finder
{
double dexterityFromJacobian(const Eigen::MatrixXd& jacobian, double characteristic_length_m)
{
  if (jacobian.rows() != 6 || jacobian.cols() < 1 || characteristic_length_m <= 0.0)
  {
    return 0.0;
  }
  Eigen::MatrixXd scaled = jacobian;
  scaled.topRows(3) /= characteristic_length_m;  // m/rad -> rad-equivalent
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(scaled);
  const auto& s = svd.singularValues();
  if (s.size() < 6 || s(0) <= 0.0)
  {
    return 0.0;  // fewer than 6 joints: some task direction is always unreachable
  }
  return s(s.size() - 1) / s(0);
}

namespace
{
// One robot model per process if the BehaviorContext did not load one.
std::shared_ptr<const moveit_pro::base::RobotModel> fallbackModel(const rclcpp::Node::SharedPtr& node)
{
  static std::mutex m;
  static std::unique_ptr<moveit_pro::base::robot_model_loader::RobotModelLoader> loader;
  std::lock_guard<std::mutex> lk(m);
  if (!loader)
  {
    moveit_pro::base::robot_model_loader::RobotModelLoader::Options opt("robot_description");
    opt.load_kinematics_solvers = false;
    loader = std::make_unique<moveit_pro::base::robot_model_loader::RobotModelLoader>(node, opt);
  }
  return loader->getModel();
}
}  // namespace

DexterityEvaluator::DexterityEvaluator(const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& context,
                                       const std::string& group, const std::string& tip_link,
                                       double characteristic_length_m)
  : length_(characteristic_length_m)
{
  try
  {
    model_ = context->robot_model ? std::shared_ptr<const moveit_pro::base::RobotModel>(context->robot_model) :
                                    fallbackModel(context->node);
  }
  catch (const std::exception& e)
  {
    error_ = std::string("could not load the robot model: ") + e.what();
    return;
  }
  if (!model_)
  {
    error_ = "no robot model (robot_description not available)";
    return;
  }
  group_ = model_->getJointModelGroup(group);
  tip_ = model_->getLinkModel(tip_link);
  if (!group_ || !tip_)
  {
    error_ = "unknown planning group '" + group + "' or link '" + tip_link + "'";
    return;
  }
  state_ = std::make_unique<moveit_pro::base::RobotState>(model_);
  state_->setToDefaultValues();
}

DexterityEvaluator::~DexterityEvaluator() = default;

std::optional<Eigen::MatrixXd> DexterityEvaluator::jacobianAt(const Eigen::VectorXd& q)
{
  state_->setJointGroupPositions(group_, q);
  state_->update();
  Eigen::MatrixXd j;
  if (!state_->getJacobian(group_, tip_, Eigen::Vector3d::Zero(), j))
  {
    return std::nullopt;
  }
  return j;
}

std::optional<double> DexterityEvaluator::evaluate(const std::vector<std::string>& names,
                                                   const std::vector<double>& positions)
{
  if (!error_.empty() || names.size() != positions.size())
  {
    return std::nullopt;
  }
  try
  {
    state_->setVariablePositions(names, positions);
    Eigen::VectorXd q;
    state_->copyJointGroupPositions(group_, q);
    const auto j = jacobianAt(q);
    if (!j)
    {
      return std::nullopt;
    }
    return dexterityFromJacobian(*j, length_);
  }
  catch (const std::exception&)
  {
    return std::nullopt;  // e.g. a joint name not in the model
  }
}

std::optional<double> DexterityEvaluator::rollGradient(const std::vector<std::string>& names,
                                                       const std::vector<double>& positions,
                                                       const Eigen::Vector3d& axis_world, double step_rad)
{
  if (!error_.empty() || names.size() != positions.size() || step_rad <= 0.0)
  {
    return std::nullopt;
  }
  try
  {
    state_->setVariablePositions(names, positions);
    Eigen::VectorXd q;
    state_->copyJointGroupPositions(group_, q);
    const auto j = jacobianAt(q);
    if (!j)
    {
      return std::nullopt;
    }
    // The Jacobian is expressed in the group's root link frame; bring the roll axis there.
    const auto* root = group_->getJointModels().front()->getParentLinkModel();
    Eigen::Matrix3d r_root = Eigen::Matrix3d::Identity();
    if (root)
    {
      r_root = state_->getGlobalLinkTransform(root).linear();
    }
    Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
    twist.tail<3>() = r_root.transpose() * axis_world.normalized();
    const Eigen::VectorXd qdot = j->completeOrthogonalDecomposition().pseudoInverse() * twist;  // joints per rad
    const auto plus = jacobianAt(q + step_rad * qdot);
    const auto minus = jacobianAt(q - step_rad * qdot);
    state_->setJointGroupPositions(group_, q);
    if (!plus || !minus)
    {
      return std::nullopt;
    }
    return (dexterityFromJacobian(*plus, length_) - dexterityFromJacobian(*minus, length_)) / (2.0 * step_rad);
  }
  catch (const std::exception&)
  {
    return std::nullopt;
  }
}
}  // namespace sun_pose_finder
