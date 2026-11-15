#pragma once

#include <memory>
#include <optional>

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <sun_pose_finder/dexterity.hpp>

namespace sun_pose_finder
{
/**
 * @brief Dexterity (distance from singularity, 0..1, see dexterityFromJacobian) of an IK solution at the focal
 * frame, and a running minimum: wire `running_min_in` and `running_min_out` to the same blackboard key (reset it to
 * 1.0 first) to get the worst point of a set, e.g. inside the ForEach of Raster Reachable With Roll.
 */
class ComputeDexterity : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  ComputeDexterity(const std::string& name, const BT::NodeConfiguration& config,
                   const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);
  ~ComputeDexterity() override;
  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;

private:
  std::unique_ptr<DexterityEvaluator> evaluator_;
};

/**
 * @brief Keeps the best of several candidates: if `candidate_score` > `best_score_in`, writes it and
 * `candidate_orientation` to `best_score_out` / `best_orientation`. Wire best_score_in/out to one key, initialised to
 * -1. Used to pick the lens roll whose worst raster point is farthest from a singularity.
 */
class KeepBestRoll : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  KeepBestRoll(const std::string& name, const BT::NodeConfiguration& config,
               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);
  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;
};
}  // namespace sun_pose_finder
