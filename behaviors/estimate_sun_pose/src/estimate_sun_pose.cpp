#include <estimate_sun_pose/estimate_sun_pose.hpp>

#include "spdlog/spdlog.h"


// ROS pose and image message types
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/image.hpp>


// ROS cv_bridge
#include <cv_bridge/cv_bridge.hpp>



// OpenCV includes
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <map>
#include <iostream>


namespace estimate_sun_pose
{
EstimateSunPose::EstimateSunPose(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{ }



BT::PortsList EstimateSunPose::providedPorts()
{
  // TODO: Define any input/output ports required for your behavior.

  /*Port Types
    InputPorts:
      - sensor_msgs::msg::Image -> incoming image from camera stream to detect blobs
      - geometry_msgs::msg::Pose -> current pose of end effector (ideally camera frame)
    OutputPorts:
      - geometry_msgs::msg::Pose -> pose to move to, should just be a rotated pose from current position

  */ 
  return BT::PortsList({BT::InputPort<sensor_msgs::msg::Image>("image","Imcoming image from camera stream to detect blobs on"), 
                        BT::BidirectionalPort<geometry_msgs::msg::Pose>("cameraPose", "Pose changed by the system to the rotated new pose")});
}

BT::KeyValueVector EstimateSunPose::metadata()
{
  // TODO: Define your behavior here.
  return { {"description", "CPP Behavior node that takes a ROS2 image and calculates the estimated sun pose for closed loop control."},{"subcategory", "User Created Behaviors"} };
}

BT::NodeStatus EstimateSunPose::tick()
{
  // TODO: Implement synchronous work logic here.
  // Publish messages to the UI like this:
  // shared_resources_->logger->publishInfoMessage(name(), "Hello, MoveIt Pro.");
  // shared_resources_->logger->publishWarnMessage(name(), "Hello, MoveIt Pro!");
  // shared_resources_->logger->publishFailureMessage(name(), "Goodbye, MoveIt Pro.");
  
  // Log messages to the console like this:
  // spdlog::info("Hello, MoveIt Pro.");
  spdlog::warn("Hello, MoveIt Pro!");
  // spdlog::error("Goodbye, MoveIt Pro.");

  shared_resources_->logger->publishInfoMessage("behavior node run");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace estimate_sun_pose
