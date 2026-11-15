#pragma once

#include <array>
#include <chrono>
#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <moveit_pro_controllers_msgs/msg/velocity_force_command.hpp>
#include <opencv2/features2d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sun_pose_finder/dexterity.hpp>
#include <std_msgs/msg/float64.hpp>

namespace sun_pose_finder
{
/// Path-following reference along a polyline at constant speed.
class PolylineReference
{
public:
  explicit PolylineReference(std::vector<Eigen::Vector3d> points);
  double length() const { return cumulative_.empty() ? 0.0 : cumulative_.back(); }
  /// Point at arc length s (clamped to [0, length]).
  Eigen::Vector3d point(double s) const;
  /// Unit tangent of the segment that contains s (zero for a degenerate path).
  Eigen::Vector3d tangent(double s) const;

private:
  size_t segment(double s) const;
  std::vector<Eigen::Vector3d> points_;
  std::vector<double> cumulative_;  // arc length at each point
};

/// Sun-sensor error -> commanded angular velocity in the focal frame (same convention as SunPoseFinder:
/// the correction rotation is RPY(-vy, vx, 0) in ee_focal_point_link). vx, vy are tan offsets from the aim point.
Eigen::Vector3d sunErrorToFocalAngularVelocity(double vx, double vy, double gain);

/// Clamp a vector's norm.
Eigen::Vector3d clampNorm(const Eigen::Vector3d& v, double max_norm);

/**
 * @brief Continuous dual-objective tracking: the focal point follows a raster path at constant speed (3 DOF) while
 * the lens axis tracks the sun from the sun sensor (3 DOF), by streaming twists to velocity_force_controller.
 * @details First acquires the sun: the focal point holds the path start while the lens rotates onto the sun, until
 * the error is below `acquire_tolerance_rad`. Then runs the scan. If the sun is lost it pauses (holds the focal
 * point and the lens, stops the path) and re-acquires before resuming; by default it never fails for a missing sun.
 * `start_at_m` + `path_world` resume a stopped run; `progress_m` reports how far it got. Runs a timer at
 * `rate_hz`. Rotation: omega = gain_rot * (sun-sensor error) rotated to world, roll held.
 * Translation: v = speed * tangent + gain_pos * (reference - focal point from TF). If the sun is lost the lens holds
 * still and the path reference stops advancing. Fails (after a zero twist) on large tracking or pointing error, a
 * stale sun past `sun_lost_abort_s`, or missing TF. Succeeds when the reference has reached the end of the path and
 * the focal point is within `finish_tolerance_m` of it. velocity_force_controller must be active.
 */
class TrackRasterWithSun : public moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  TrackRasterWithSun(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  struct SunMeasurement
  {
    rclcpp::Time stamp;
    double vx = 0.0, vy = 0.0;
    bool valid = false;
  };

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
  void controlStep();
  void publishTwist(const Eigen::Vector3d& v, const Eigen::Vector3d& w);
  void stop(const std::string& why, BT::NodeStatus result);
  /// Lift the focus off the bed (if lift_m > 0), then stop with this result.
  void finish(const std::string& why, BT::NodeStatus result, const rclcpp::Time& now);
  void teardown();

  // Parameters (read in onStart)
  std::string world_frame_, focal_frame_, command_topic_;
  double speed_ = 0.0, gain_rot_ = 1.0, gain_pos_ = 1.0, max_lin_ = 0.01, max_ang_ = 0.02;
  double max_track_err_ = 0.003, max_point_err_ = 0.0175, sun_timeout_ = 0.5, sun_abort_ = 10.0;
  double finish_tol_ = 0.0005, aim_x_ = 0.0, aim_y_ = 0.0, start_tol_ = 0.005;
  double rate_hz_ = 50.0;
  double acquire_tol_ = 0.002, acquire_timeout_ = 30.0;
  // Singularity handling: monitor dexterity (0..1) and optionally roll the lens about its axis (free DOF) up the
  // dexterity gradient.
  double min_dexterity_ = 0.0, gain_roll_ = 0.0, max_roll_rate_ = 0.01, dex_length_ = 0.4;
  std::unique_ptr<DexterityEvaluator> dex_;
  std::vector<std::string> js_names_;
  std::vector<double> js_positions_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dex_pub_;
  // Timing (ms): whole control step, its dexterity part, sun detection per camera frame, actual tick period.
  std::vector<double> t_step_, t_dex_, t_detect_, t_period_;
  std::chrono::steady_clock::time_point t_prev_{};
  double camera_timeout_ = 3.0;
  // Defocus lift
  double lift_m_ = 0.03, lift_speed_ = 0.01, lift_ = 0.0, max_acquire_err_ = 0.15, finish_timeout_ = 5.0;
  Eigen::Vector3d lift_dir_ = Eigen::Vector3d::UnitZ();
  bool finishing_ = false;
  std::string finish_msg_;
  BT::NodeStatus finish_result_ = BT::NodeStatus::FAILURE;
  double finish_deadline_ = 0.0, at_end_since_ = -1.0, acquire_over_since_ = -1.0;
  std::mutex cmd_mutex_;  // orders twist publishing between the control step and halt/stop
  rclcpp::CallbackGroup::SharedPtr image_cb_group_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lift_pub_;
  std::atomic<int64_t> last_frame_ns_{ 0 };  // node time of the last usable camera frame
  double dex_bad_since_ = -1.0;              // time the dexterity evaluation started failing, -1 if fine
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr loop_ms_pub_;
  void logTiming();
  bool acquired_ = false;
  rclcpp::Time start_time_;

  std::optional<PolylineReference> path_;
  std::atomic<double> s_{ 0.0 };  // reference arc length; written by the timer, read by onRunning/onHalted
  bool paused_ = false;
  bool ever_acquired_ = false;           // sun lost: holding position until it is re-acquired
  double over_since_ = -1.0;  // time the pointing error first exceeded the limit, -1 if not
  rclcpp::Time last_tick_;
  rclcpp::Time last_sun_seen_;
  bool started_clock_ = false;

  std::mutex mutex_;
  SunMeasurement sun_;
  std::optional<std::array<double, 4>> intrinsics_;  // fx, fy, cx, cy
  cv::Ptr<cv::SimpleBlobDetector> detector_;

  std::atomic<bool> done_{ false };
  BT::NodeStatus result_ = BT::NodeStatus::RUNNING;
  std::string message_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<moveit_pro_controllers_msgs::msg::VelocityForceCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr sun_pub_, track_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr progress_pub_;
};
}  // namespace sun_pose_finder
