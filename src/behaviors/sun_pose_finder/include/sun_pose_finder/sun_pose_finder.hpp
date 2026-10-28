#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <rclcpp/publisher.hpp>

// This header includes the SharedResourcesNode type
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>




namespace sun_pose_finder
{
/**
 * @brief TODO(...)
 */
class SunPoseFinder : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  /**
   * @brief Constructor for the sun_pose_finder behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @param shared_resources A shared_ptr to a BehaviorContext that is shared among all SharedResourcesNode Behaviors in the behavior tree. This BehaviorContext is owned by the Studio Agent's ObjectiveServerNode.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after the initialize() function is called, so these classes should not be used within the constructor.
   */
  SunPoseFinder(const std::string& name, const BT::NodeConfiguration& config, const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  
  /**
   * @brief Implementation of the required providedPorts() function for the sun_pose_finder Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function must return an empty BT::PortsList.
   * This function returns a list of ports with their names and port info, which is used internally by the behavior tree.
   * @return sun_pose_finder does not expose any ports, so this function returns an empty list.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
   * subcategory, in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

  /**
   * @brief Implementation of BT::SyncActionNode::tick() for SunPoseFinder.
   * @details This function is where the Behavior performs its work when the behavior tree is being run. Since SunPoseFinder is derived from BT::SyncActionNode, it is very important that its tick() function always finishes very quickly. If tick() blocks before returning, it will block execution of the entire behavior tree, which may have undesirable consequences for other Behaviors that require a fast update rate to work correctly.
   */
  BT::NodeStatus tick() override;

private:
  // Every sun check is published on /sinter/sun_error so rosbags record the pointing error as the controller sees it:
  // x, y = tan of the sun's angle from the aim point along image +x / +y (aim offset already removed), z = alignment
  // error in rad. Stamped with the image header. All NaN when no blob is found.
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr sun_error_pub_;
};
}  // namespace sun_pose_finder
