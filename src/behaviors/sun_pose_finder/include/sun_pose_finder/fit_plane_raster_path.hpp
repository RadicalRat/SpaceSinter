#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/float64.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <tl_expected/expected.hpp>

namespace sun_pose_finder
{
struct PlaneRaster
{
  Eigen::Isometry3d plane;               // origin at the centroid, X along the scan lines, Z along the normal
  std::vector<Eigen::Isometry3d> poses;  // zig-zag waypoints, oriented like `plane`
  double rms_m;                          // RMS distance of the inliers from the fitted plane
  size_t num_inliers;                    // points used for the fit and the raster extent
  size_t num_outliers;                   // points dropped as farther than `outlier_distance` from the plane
};

/**
 * @brief Fits a plane to the points (least squares, via PCA) and lays a zig-zag raster on it.
 * @details If `outlier_distance` > 0, RANSAC first finds the dominant plane and points farther than `outlier_distance`
 * from it (e.g. sun glare noise in the depth image) are dropped from both the fit and the raster extent. Scan lines run along the plane's longest extent and are `line_spacing` apart; each line
 * spans only the points within half a line spacing of it, so the raster stays on the surface. Along each line, waypoints
 * are at most `point_spacing` apart (0 means line end points only), and each line is split into at least
 * `line_subdivisions` equal moves. `edge_margin` shrinks the area on every side. The
 * normal is flipped to point toward +Z of the points' frame.
 */
tl::expected<PlaneRaster, std::string> fitPlaneRaster(const std::vector<Eigen::Vector3d>& points,
                                                      double line_spacing, double point_spacing, double edge_margin,
                                                      double outlier_distance, int line_subdivisions = 1);

/**
 * @brief Behavior wrapper around fitPlaneRaster for a PointCloud2 (use a cropped cloud).
 *
 * | Data Port Name   | Port Type | Object Type                                  |
 * | ---------------- | --------- | -------------------------------------------- |
 * | point_cloud      | input     | sensor_msgs::msg::PointCloud2                |
 * | line_spacing     | input     | double                                       |
 * | point_spacing    | input     | double                                       |
 * | line_subdivisions| input     | int                                          |
 * | edge_margin      | input     | double                                       |
 * | outlier_distance | input     | double                                       |
 * | surface_offset   | input     | double                                       |
 * | path             | output    | std::vector<geometry_msgs::msg::PoseStamped> |
 * | plane_pose       | output    | geometry_msgs::msg::PoseStamped              |
 * | num_waypoints    | output    | int                                          |
 * | tilt_deg         | output    | double                                       |
 *
 * Each successful fit also publishes the bed tilt (deg, std_msgs/Float64) on /sinter/bed_tilt_deg and the fitted
 * plane pose on /sinter/bed_plane and the waypoints (nav_msgs/Path) on /sinter/raster_path, all latched, so rosbags
 * record the bed angle and the commanded raster.
 */
class FitPlaneRasterPath : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  FitPlaneRasterPath(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tilt_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr plane_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};
}  // namespace sun_pose_finder
