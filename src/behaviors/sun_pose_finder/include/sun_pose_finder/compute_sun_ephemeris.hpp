#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace sun_pose_finder
{
/**
 * @brief Seeds the sun tracker from an ephemeris model (date, time, latitude/longitude).
 * @details Computes the sun's azimuth/elevation, converts it to a direction in the frame of `reference_pose` (which
 * must have +Z up, e.g. world), and outputs `reference_pose` rotated by the smallest rotation that points its +Z axis
 * at the sun. Pass the current ee_focal_point_link pose in world to get a "rotate the lens about the focal point"
 * target. Returns FAILURE if the sun is below `min_elevation_deg`.
 *
 * | Data Port Name     | Port Type | Object Type                     |
 * | ------------------ | --------- | ------------------------------- |
 * | reference_pose     | input     | geometry_msgs::msg::PoseStamped |
 * | latitude_deg       | input     | double                          |
 * | longitude_deg      | input     | double                          |
 * | north_heading_deg  | input     | double                          |
 * | utc_time           | input     | std::string                     |
 * | min_elevation_deg  | input     | double                          |
 * | aligned_pose       | output    | geometry_msgs::msg::PoseStamped |
 * | sun_azimuth_deg    | output    | double                          |
 * | sun_elevation_deg  | output    | double                          |
 */
class ComputeSunEphemeris : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  ComputeSunEphemeris(const std::string& name, const BT::NodeConfiguration& config,
                      const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace sun_pose_finder
