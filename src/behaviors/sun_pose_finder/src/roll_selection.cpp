#include <sun_pose_finder/roll_selection.hpp>

#include <algorithm>

#include <fmt/format.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit_studio_agent_msgs/msg/robot_joint_state.hpp>

namespace sun_pose_finder
{
ComputeDexterity::ComputeDexterity(const std::string& name, const BT::NodeConfiguration& config,
                                   const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

ComputeDexterity::~ComputeDexterity() = default;

BT::PortsList ComputeDexterity::providedPorts()
{
  return {
    BT::InputPort<moveit_studio_agent_msgs::msg::RobotJointState>("joint_state", "{ik_solution}",
                                                                   "Joint state, e.g. ComputeInverseKinematics "
                                                                   "solution."),
    BT::InputPort<std::string>("planning_group", "manipulator", "Joint group."),
    BT::InputPort<std::string>("tip_link", "ee_focal_point_link", "Frame whose 6-DOF motion is assessed."),
    BT::InputPort<double>("characteristic_length_m", 0.4, "Scales translation against rotation (lens focal "
                                                          "length is a natural choice)."),
    BT::InputPort<double>("running_min_in", 1.0, "Previous minimum (same key as running_min_out)."),
    BT::OutputPort<double>("dexterity", "{dexterity}", "Dexterity of this state, 0 (singular) .. 1."),
    BT::OutputPort<double>("running_min_out", "{min_dexterity}", "min(running_min_in, dexterity)."),
  };
}

BT::KeyValueVector ComputeDexterity::metadata()
{
  return { { "description", "Distance from singularity (inverse condition number of the focal-frame Jacobian, "
                            "translation scaled by characteristic_length_m) for a joint state, plus a running "
                            "minimum." },
           { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus ComputeDexterity::tick()
{
  const auto js = getInput<moveit_studio_agent_msgs::msg::RobotJointState>("joint_state");
  if (!js)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Missing joint_state: " + js.error());
    return BT::NodeStatus::FAILURE;
  }
  if (!evaluator_)
  {
    evaluator_ = std::make_unique<DexterityEvaluator>(
        shared_resources_, getInput<std::string>("planning_group").value_or("manipulator"),
        getInput<std::string>("tip_link").value_or("ee_focal_point_link"),
        getInput<double>("characteristic_length_m").value_or(0.4));
  }
  if (!evaluator_->error().empty())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Dexterity unavailable: " + evaluator_->error());
    return BT::NodeStatus::FAILURE;
  }
  const auto d = evaluator_->evaluate(js->joint_state.name, js->joint_state.position);
  if (!d)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Could not compute the Jacobian for this state.");
    return BT::NodeStatus::FAILURE;
  }
  setOutput("dexterity", *d);
  setOutput("running_min_out", std::min(getInput<double>("running_min_in").value_or(1.0), *d));
  return BT::NodeStatus::SUCCESS;
}

KeepBestRoll::KeepBestRoll(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList KeepBestRoll::providedPorts()
{
  return {
    BT::InputPort<double>("candidate_score", "Score of this candidate (e.g. its worst-point dexterity)."),
    BT::InputPort<geometry_msgs::msg::Quaternion>("candidate_orientation", "Orientation of this candidate."),
    BT::InputPort<std::string>("candidate_name", "", "Label for the log, e.g. 'roll 45 deg'."),
    BT::InputPort<double>("best_score_in", -1.0, "Best score so far (same key as best_score_out)."),
    BT::InputPort<double>("min_improvement", 0.25, "A later candidate must beat the best so far by this fraction "
                                                   "(0.25 = 25 %). Evaluate candidates nearest first, so a far roll "
                                                   "(a big wrist rotation in the lead-in) is only taken when it is "
                                                   "clearly better."),
    BT::OutputPort<double>("best_score_out", "{best_dexterity}", "Best score after this candidate."),
    BT::OutputPort<geometry_msgs::msg::Quaternion>("best_orientation", "{sun_orientation}",
                                                   "Orientation of the best candidate so far."),
    BT::OutputPort<std::string>("best_name", "{best_roll_name}", "Label of the best candidate so far."),
    BT::InputPort<moveit_studio_agent_msgs::msg::RobotJointState>("candidate_joint_state",
                                                                   "Optional: joint state that goes with this "
                                                                   "candidate (e.g. its scan seed)."),
    BT::InputPort<bool>("require_joint_state", false, "Fail (keeping the previous best) if candidate_joint_state is "
                                                      "missing, so the best orientation and its seed cannot "
                                                      "come from different candidates."),
    BT::OutputPort<moveit_studio_agent_msgs::msg::RobotJointState>("best_joint_state", "{best_scan_seed}",
                                                                    "Joint state of the best candidate so far."),
  };
}

BT::KeyValueVector KeepBestRoll::metadata()
{
  return { { "description", "Keeps the candidate with the highest score (and its orientation) across calls." },
           { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus KeepBestRoll::tick()
{
  const auto score = getInput<double>("candidate_score");
  const auto q = getInput<geometry_msgs::msg::Quaternion>("candidate_orientation");
  if (!score || !q)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Missing candidate_score or candidate_orientation.");
    return BT::NodeStatus::FAILURE;
  }
  const auto js = getInput<moveit_studio_agent_msgs::msg::RobotJointState>("candidate_joint_state");
  if (!js && getInput<bool>("require_joint_state").value_or(false))
  {
    shared_resources_->logger->publishFailureMessage(name(), "require_joint_state is set but candidate_joint_state "
                                                             "is missing.");
    return BT::NodeStatus::FAILURE;
  }
  const double best = getInput<double>("best_score_in").value_or(-1.0);
  const std::string label = getInput<std::string>("candidate_name").value_or("");
  shared_resources_->logger->publishInfoMessage(name(), fmt::format("{}: worst-point dexterity {:.3f}", label, *score));
  const double margin = std::max(0.0, getInput<double>("min_improvement").value_or(0.25));
  if (best < 0.0 ? *score >= 0.0 : *score > best * (1.0 + margin))
  {
    shared_resources_->logger->publishInfoMessage(name(), fmt::format("{} is the best so far.", label));
    setOutput("best_score_out", *score);
    setOutput("best_orientation", *q);
    setOutput("best_name", label);
    if (js)
    {
      setOutput("best_joint_state", *js);
    }
  }
  return BT::NodeStatus::SUCCESS;
}
}  // namespace sun_pose_finder
