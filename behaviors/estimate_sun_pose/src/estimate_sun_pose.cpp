#include <estimate_sun_pose/estimate_sun_pose.hpp>

#include "spdlog/spdlog.h"


// ROS pose and image message types
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/image.hpp>


// ROS cv_bridge
#include <cv_bridge/cv_bridge.h>


// OpenCV includes
// #include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <map>
#include <iostream>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>


namespace {
  cv::Ptr<cv::SimpleBlobDetector> detector;
}

namespace estimate_sun_pose
{
EstimateSunPose::EstimateSunPose(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{ 
  cv::SimpleBlobDetector::Params params;
  params.minThreshold = 200;
  params.maxThreshold = 200;
  params.filterByArea = true;
  params.minArea = 50;
  params.filterByCircularity = true;
  params.minCircularity = 0.5;
  params.filterByConvexity = true;
  params.minConvexity = 0.5;
  params.filterByInertia = false;
  params.minInertiaRatio = 0.01;


  if (!detector) {
    detector = cv::SimpleBlobDetector::create(params);
  }
}


BT::PortsList EstimateSunPose::providedPorts()
{
  // TODO: Define any input/output ports required for your behavior.

  /*Port Types
    InputPorts:
      - sensor_msgs::msg::Image -> incoming image from camera stream to detect blobs
      - geometry_msgs::msg::Pose -> current pose of end effector (ideally camera frame)
    OutputPorts:
      - vector<double> -> normal vector of the center of the camera to the sun in the camera frame

  */ 
  return BT::PortsList({BT::InputPort<sensor_msgs::msg::Image::ConstSharedPtr>("image","Imcoming image from camera stream to detect blobs on"), 
                        BT::InputPort<sensor_msgs::msg::CameraInfo>("camera_info","Intrinsics info message of the camera"),
                        BT::OutputPort<std::vector<double, std::allocator<double>>>("cameraOrientation", "Orientation normalized vector to the sun in the camera frame")});
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
  spdlog::warn("Hello, MoveIt Pro!");
  shared_resources_->logger->publishInfoMessage("behavior node run!");

  // get the image
  auto image = getInput<sensor_msgs::msg::Image::ConstSharedPtr>("image");

  if (!image) {
    throw BT::RuntimeError("missing required input [message]: ",
      image.error());
  }

  // get the camera info
  auto info = getInput<sensor_msgs::msg::CameraInfo>("camera_info");
  if (!info) {
    throw BT::RuntimeError("missing required input [camera_info]: ",
      info.error());
  }

  // extract the camera intrinsics matrix
  // cv::Mat K = cv::Mat(3, 3, CV_64F, const_cast<void*>(static_cast<const void*>(info.value().k.data())));
  cv::Mat K = cv::Mat(3, 3, CV_64F, info.value().k.data());


  // == Do blob detection ==
  // convert image into opencv type
  const auto cv_ptr = cv_bridge::toCvShare(image.value(), "rgb8");

  std::vector<cv::KeyPoint> keypoints;
  detector->detect(cv_ptr->image, keypoints);
  

  if (!keypoints.empty()) {
    // take biggest blob found
    cv::Point2f sun_center = keypoints[0].pt;

    // Extract intrinsics
    double fx = info.value().k[0];
    double fy = info.value().k[4];
    double cx = info.value().k[2];
    double cy = info.value().k[5];

    // calculate vector segments
    double vx = (sun_center.x - cx) / fx;
    double vy = (sun_center.y - cy) / fy;
    double vz = 1.0;
    
    // create a 3d opencv vector and normalize
    cv::Vec3d sun_vector(vx, vy, vz);
    sun_vector = cv::normalize(sun_vector);

    // convert vector into output form and output vector
    std::vector<double, std::allocator<double>> sun_vec = {0, sun_vector[0], sun_vector[1], sun_vector[2]};
    setOutput("cameraOrientation", sun_vec);

    return BT::NodeStatus::SUCCESS;

  } else { // no keypoints found, return failure
    return BT::NodeStatus::FAILURE;
  }

}

}  // namespace estimate_sun_pose
