#include <sun_pose_finder/sun_pose_finder.hpp>

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
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace {
  cv::Ptr<cv::SimpleBlobDetector> detector;
}

namespace sun_pose_finder
{
SunPoseFinder::SunPoseFinder(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
  cv::SimpleBlobDetector::Params params;
  params.minThreshold = 240;
  params.maxThreshold = 255;
  params.thresholdStep = 5;
  params.minRepeatability = 1;
  params.filterByColor = true;
  params.blobColor = 255;  // detect bright blobs, not dark ones
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


BT::PortsList SunPoseFinder::providedPorts()
{
  // TODO: Define any input/output ports required for your behavior.
  return BT::PortsList({BT::InputPort<sensor_msgs::msg::Image>("image","Imcoming image from camera stream to detect blobs on"), 
                        BT::InputPort<sensor_msgs::msg::CameraInfo>("camera_info","Intrinsics info message of the camera"),
                        BT::OutputPort<geometry_msgs::msg::Quaternion>("cameraOrientation", "Orientation normalized vector to the sun in the camera frame"),
                        BT::OutputPort<sensor_msgs::msg::Image>("output_image","Outgoing image stream with blob shown on the image, used for debugging"),
                      });
}

BT::KeyValueVector SunPoseFinder::metadata()
{
  // TODO: Define your behavior here.
  return { {"description", "takes a camera topic, and then finds a blob using opencv, and reports the desired orientation vector to reach the blob. returns true if within error bounds, and false if not."},{"subcategory", "User Created Behaviors"} };
}

BT::NodeStatus SunPoseFinder::tick()
{
  /// TODO: Implement synchronous work logic here.
  // Publish messages to the UI like this:
  spdlog::warn("Hello, MoveIt Pro!");
  // shared_resources_->logger->publishInfoMessage("behavior node run!");

  // get the image
  auto image = getInput<sensor_msgs::msg::Image>("image");

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


  // == Do blob detection ==
  // convert image into opencv type
  const auto cv_ptr = cv_bridge::toCvCopy(image.value(), "rgb8");

  std::vector<cv::KeyPoint> keypoints;

  // cv::Mat dark;
  // cv_ptr->image.convertTo(dark, -1, 0.1, 0);  // darken for visualization only

  detector->detect(cv_ptr->image, keypoints);  // detect on full-exposure image

  // sort largest blob first
  std::sort(keypoints.begin(), keypoints.end(),
            [](const cv::KeyPoint& a, const cv::KeyPoint& b){ return a.size > b.size; });

  shared_resources_->logger->publishInfoMessage(
    "SunPoseFinder: detected " + std::to_string(keypoints.size()) + " blob(s). Image size: " +
    std::to_string(cv_ptr->image.cols) + "x" + std::to_string(cv_ptr->image.rows));

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
    // cv::Vec3d sun_vector(vx, vy, vz);
    // sun_vector = cv::normalize(sun_vector);

    // convert vector into output form and output vector
    // std::vector<double, std::allocator<double>> sun_vec = {0, sun_vector[0], sun_vector[1], sun_vector[2]};

    tf2::Quaternion camera_transform;

    camera_transform.setRPY(-vy, vx, 0);  // roll corrects vertical (vy), pitch corrects horizontal (vx)
    camera_transform.normalize();
    geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(camera_transform);

    shared_resources_->logger->publishInfoMessage(
      "SunPoseFinder: sun at pixel (" + std::to_string(static_cast<int>(sun_center.x)) + ", " +
      std::to_string(static_cast<int>(sun_center.y)) + "), blob size=" +
      std::to_string(static_cast<int>(keypoints[0].size)) +
      ", vx=" + std::to_string(vx).substr(0, 6) + " vy=" + std::to_string(vy).substr(0, 6));

    setOutput("cameraOrientation", quat_msg);

    // == Draw visualization onto image ==
    cv::Mat annotated;
    // Draw detected blob circle
    cv::drawKeypoints(cv_ptr->image, keypoints, annotated,
                      cv::Scalar(0, 255, 0),  // green
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // Draw arrow from image principal point toward the blob (angular offset direction)
    cv::arrowedLine(annotated,
                    cv::Point2f(static_cast<float>(cx), static_cast<float>(cy)),
                    sun_center,
                    cv::Scalar(255, 165, 0), 3);  // orange

    // Overlay vx, vy, vz values as text
    const std::string text = cv::format("vx=%.3f  vy=%.3f  vz=%.1f", vx, vy, vz);
    cv::putText(annotated, text, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 4);   // black outline
    cv::putText(annotated, text, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2); // white fill

    sensor_msgs::msg::Image output_msg;
    cv_bridge::CvImage(image.value().header, "rgb8", annotated).toImageMsg(output_msg);
    setOutput("output_image", output_msg);

    return BT::NodeStatus::SUCCESS;

  } else {
    shared_resources_->logger->publishInfoMessage(
      "SunPoseFinder: no bright blobs found (threshold=" + std::to_string(240) + "). Returning FAILURE.");
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace sun_pose_finder
