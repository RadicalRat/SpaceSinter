#include <sun_pose_finder/track_raster_with_sun.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

#include <cv_bridge/cv_bridge.h>
#include <fmt/format.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace sun_pose_finder
{
namespace
{
constexpr auto kPathPort = "path";
constexpr auto kSpeedPort = "scan_speed_mps";
constexpr auto kImageTopicPort = "image_topic";
constexpr auto kInfoTopicPort = "camera_info_topic";
}  // namespace

// ---------------------------------------------------------------- PolylineReference
PolylineReference::PolylineReference(std::vector<Eigen::Vector3d> points) : points_(std::move(points))
{
  cumulative_.reserve(points_.size());
  double s = 0.0;
  for (size_t i = 0; i < points_.size(); ++i)
  {
    if (i > 0)
    {
      s += (points_[i] - points_[i - 1]).norm();
    }
    cumulative_.push_back(s);
  }
}

size_t PolylineReference::segment(double s) const
{
  // Index i of the segment [points_[i], points_[i+1]] that contains s.
  if (points_.size() < 2)
  {
    return 0;
  }
  const auto it = std::upper_bound(cumulative_.begin(), cumulative_.end(), s);
  const size_t i = it == cumulative_.begin() ? 0 : static_cast<size_t>(it - cumulative_.begin()) - 1;
  return std::min(i, points_.size() - 2);
}

Eigen::Vector3d PolylineReference::point(double s) const
{
  if (points_.empty())
  {
    return Eigen::Vector3d::Zero();
  }
  if (points_.size() == 1)
  {
    return points_.front();
  }
  s = std::clamp(s, 0.0, length());
  const size_t i = segment(s);
  const double len = cumulative_[i + 1] - cumulative_[i];
  const double f = len > 1e-12 ? (s - cumulative_[i]) / len : 0.0;
  return points_[i] + f * (points_[i + 1] - points_[i]);
}

Eigen::Vector3d PolylineReference::tangent(double s) const
{
  if (points_.size() < 2)
  {
    return Eigen::Vector3d::Zero();
  }
  size_t i = segment(std::clamp(s, 0.0, length()));
  // Skip zero-length segments (duplicate points).
  while (i + 1 < points_.size() - 1 && (points_[i + 1] - points_[i]).norm() < 1e-12)
  {
    ++i;
  }
  const Eigen::Vector3d d = points_[i + 1] - points_[i];
  return d.norm() > 1e-12 ? Eigen::Vector3d(d.normalized()) : Eigen::Vector3d(Eigen::Vector3d::Zero());
}

Eigen::Vector3d sunErrorToFocalAngularVelocity(double vx, double vy, double gain)
{
  // SunPoseFinder applies RPY(-vy, vx, 0) in the focal frame to bring the lens onto the sun; as a rate: gain * that.
  return gain * Eigen::Vector3d(-vy, vx, 0.0);
}

Eigen::Vector3d clampNorm(const Eigen::Vector3d& v, double max_norm)
{
  const double n = v.norm();
  return (max_norm > 0.0 && n > max_norm) ? Eigen::Vector3d(v * (max_norm / n)) : v;
}

// ---------------------------------------------------------------- Behavior
TrackRasterWithSun::TrackRasterWithSun(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
  cv::SimpleBlobDetector::Params p;  // same detector as SunPoseFinder
  p.minThreshold = 240;
  p.maxThreshold = 255;
  p.thresholdStep = 5;
  p.minRepeatability = 1;
  p.filterByColor = true;
  p.blobColor = 255;
  p.filterByArea = true;
  p.minArea = 15;
  p.filterByCircularity = true;
  p.minCircularity = 0.5;
  p.filterByConvexity = true;
  p.minConvexity = 0.5;
  p.filterByInertia = false;
  detector_ = cv::SimpleBlobDetector::create(p);
}

BT::PortsList TrackRasterWithSun::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(kPathPort, "{raster_path}",
                                                                "Focal-point path (positions used; orientations "
                                                                "ignored). The focal point must start at its first "
                                                                "point."),
    BT::InputPort<double>(kSpeedPort, 0.001, "Scan speed along the path, m/s."),
    BT::InputPort<std::string>(kImageTopicPort, "/camera/image_raw", "Sun camera image topic (rgb8)."),
    BT::InputPort<std::string>(kInfoTopicPort, "/camera/camera_info", "Sun camera intrinsics topic."),
    BT::InputPort<std::string>("command_topic", "/velocity_force_controller/command",
                               "velocity_force_controller command topic."),
    BT::InputPort<std::string>("world_frame", "world", "Fixed frame of the path and of the commanded twist."),
    BT::InputPort<std::string>("focal_frame", "ee_focal_point_link",
                               "Controlled frame (must be the controller's ee_frame)."),
    BT::InputPort<double>("rate_hz", 50.0, "Control loop rate."),
    BT::InputPort<double>("gain_rot", 1.0, "Pointing gain, 1/s (angular rate per rad of sun error)."),
    BT::InputPort<double>("gain_pos", 1.0, "Path-tracking gain, 1/s (m/s per m of position error)."),
    BT::InputPort<double>("max_linear_mps", 0.01, "Commanded linear speed limit, m/s."),
    BT::InputPort<double>("max_angular_rps", 0.02, "Commanded angular speed limit, rad/s."),
    BT::InputPort<double>("max_tracking_error_m", 0.003, "Fail if the focal point is farther than this from the "
                                                         "reference."),
    BT::InputPort<double>("max_pointing_error_rad", 0.0175, "Fail if the sun error stays above this for 0.5 s."),
    BT::InputPort<double>("sun_timeout_s", 0.5, "No sun for this long: hold the lens and pause the path."),
    BT::InputPort<double>("sun_lost_abort_s", 0.0, "No sun for this long: fail. 0 = never (pause until the sun "
                                                   "returns)."),
    BT::InputPort<double>("lift_m", 0.03, "Defocus lift: the focal point is held this far above the path point "
                                          "(along the lens axis, toward the lens) whenever the sun is not locked "
                                          "(start, clouds) and before reporting any stop, so the spot never dwells "
                                          "focused on one point. 0 = off."),
    BT::InputPort<double>("lift_speed_mps", 0.01, "Speed of the defocus lift, m/s."),
    BT::InputPort<double>("max_acquire_error_rad", 0.15, "While acquiring (lifted), fail if the sun error stays "
                                                         "above this for 1 s (a glint or wrong blob would otherwise "
                                                         "steer the lens without limit)."),
    BT::InputPort<double>("finish_timeout_s", 5.0, "At the path end, accept the finish after this long if the focal "
                                                   "point is within max_tracking_error_m but not finish_tolerance_m."),
    BT::InputPort<double>("camera_timeout_s", 3.0, "No sun-camera frames at all for this long (camera or driver "
                                                   "down, not a cloud): fail, so the objective can retreat. "
                                                   "0 = never."),
    BT::InputPort<double>("start_tolerance_m", 0.005, "Fail at start if the focal point is farther than this from "
                                                      "the path point at start_at_m."),
    BT::InputPort<double>("finish_tolerance_m", 0.0005, "Succeed once the reference is at the end and the focal "
                                                        "point is within this of it."),
    BT::InputPort<double>("aim_offset_x_rad", 0.0, "As SunPoseFinder: where the sun sits (tan, image +x) when the "
                                                   "lens is on it."),
    BT::InputPort<double>("aim_offset_y_rad", 0.0, "As above, image +y."),
    BT::InputPort<double>("acquire_tolerance_rad", 0.002, "Hold the path start until the sun error is below this "
                                                          "(then start the scan and the pointing-error abort)."),
    BT::InputPort<double>("acquire_timeout_s", 0.0, "Fail if the sun is not acquired within this time at the "
                                                    "start. 0 = wait as long as it takes."),
    BT::InputPort<double>("min_dexterity", 0.0, "Fail if the dexterity (distance from singularity, 0..1, see "
                                                "ComputeDexterity) drops below this during the scan. 0 = off."),
    BT::InputPort<double>("gain_roll", 0.0, "Roll the lens about its optical axis (does not move the focus or the "
                                            "sun alignment) up the dexterity gradient: rad/s per unit d(dex)/d(roll). "
                                            "0 = hold roll."),
    BT::InputPort<double>("max_roll_rate_rps", 0.01, "Limit on that roll rate, rad/s."),
    BT::InputPort<std::string>("planning_group", "manipulator", "Joint group for the dexterity Jacobian."),
    BT::InputPort<double>("dexterity_length_m", 0.4, "Characteristic length scaling translation vs rotation."),
    BT::InputPort<double>("start_at_m", 0.0, "Start this far along the path (arc length, m), e.g. the progress_m of "
                                             "a stopped run, to resume it."),
    BT::OutputPort<double>("progress_m", "{track_progress_m}", "How far along the path the reference got (arc "
                                                              "length, m). Updated while running and on stop."),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        "path_world", "{track_path_world}",
        "The path as used, in world_frame (fixed when tracking started). Feed it back with start_at_m to resume."),
  };
}

BT::KeyValueVector TrackRasterWithSun::metadata()
{
  return { { "description",
             "Continuous dual tracking: streams twists to velocity_force_controller so the focal point follows the "
             "path at constant speed (3 DOF) while the lens axis tracks the sun from the sun camera (3 DOF, roll "
             "held). Publishes /sinter/sun_error_stream, /sinter/track_error and /sinter/track_progress. Requires "
             "velocity_force_controller to be active." },
           { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus TrackRasterWithSun::onStart()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<std::vector<geometry_msgs::msg::PoseStamped>>(kPathPort), getInput<double>(kSpeedPort),
      getInput<std::string>(kImageTopicPort), getInput<std::string>(kInfoTopicPort),
      getInput<std::string>("command_topic"), getInput<std::string>("world_frame"),
      getInput<std::string>("focal_frame"), getInput<double>("rate_hz"), getInput<double>("gain_rot"),
      getInput<double>("gain_pos"), getInput<double>("max_linear_mps"), getInput<double>("max_angular_rps"),
      getInput<double>("max_tracking_error_m"), getInput<double>("max_pointing_error_rad"),
      getInput<double>("sun_timeout_s"), getInput<double>("sun_lost_abort_s"), getInput<double>("start_tolerance_m"),
      getInput<double>("finish_tolerance_m"), getInput<double>("aim_offset_x_rad"),
      getInput<double>("aim_offset_y_rad"));
  if (!ports)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Missing input: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [path_msgs, speed, image_topic, info_topic, command_topic, world, focal, rate, gr, gp, ml, ma, mte,
               mpe, st, sa, start_tol, finish_tol, ax, ay] = ports.value();
  world_frame_ = world;
  focal_frame_ = focal;
  command_topic_ = command_topic;
  speed_ = speed;
  rate_hz_ = rate;
  gain_rot_ = gr;
  gain_pos_ = gp;
  max_lin_ = ml;
  max_ang_ = ma;
  max_track_err_ = mte;
  max_point_err_ = mpe;
  sun_timeout_ = st;
  sun_abort_ = sa;
  start_tol_ = start_tol;
  finish_tol_ = finish_tol;
  aim_x_ = ax;
  aim_y_ = ay;
  acquire_tol_ = getInput<double>("acquire_tolerance_rad").value_or(0.002);
  acquire_timeout_ = getInput<double>("acquire_timeout_s").value_or(0.0);
  const double start_at = getInput<double>("start_at_m").value_or(0.0);
  acquired_ = false;
  min_dexterity_ = getInput<double>("min_dexterity").value_or(0.0);
  gain_roll_ = getInput<double>("gain_roll").value_or(0.0);
  max_roll_rate_ = getInput<double>("max_roll_rate_rps").value_or(0.01);
  dex_length_ = getInput<double>("dexterity_length_m").value_or(0.4);
  camera_timeout_ = getInput<double>("camera_timeout_s").value_or(3.0);
  lift_m_ = std::max(0.0, getInput<double>("lift_m").value_or(0.03));
  lift_speed_ = std::max(1e-4, getInput<double>("lift_speed_mps").value_or(0.01));
  max_acquire_err_ = getInput<double>("max_acquire_error_rad").value_or(0.15);
  finish_timeout_ = getInput<double>("finish_timeout_s").value_or(5.0);
  lift_ = 0.0;
  finishing_ = false;
  at_end_since_ = -1.0;
  acquire_over_since_ = -1.0;
  dex_bad_since_ = -1.0;
  dex_.reset();
  if (min_dexterity_ > 0.0 || gain_roll_ > 0.0)
  {
    dex_ = std::make_unique<DexterityEvaluator>(shared_resources_,
                                                getInput<std::string>("planning_group").value_or("manipulator"),
                                                focal_frame_, dex_length_);
    if (!dex_->error().empty())
    {
      shared_resources_->logger->publishFailureMessage(name(), "Dexterity check unavailable: " + dex_->error());
      return BT::NodeStatus::FAILURE;
    }
  }
  if (path_msgs.empty() || speed_ <= 0.0 || rate_hz_ <= 0.0)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Need a non-empty path, speed > 0 and rate > 0.");
    return BT::NodeStatus::FAILURE;
  }

  // Path into the world frame.
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(path_msgs.size());
  try
  {
    for (const auto& ps : path_msgs)
    {
      Eigen::Vector3d p(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
      if (!ps.header.frame_id.empty() && ps.header.frame_id != world_frame_)
      {
        const auto tf = shared_resources_->transform_buffer_ptr->lookupTransform(world_frame_, ps.header.frame_id,
                                                                                 tf2::TimePointZero);
        p = tf2::transformToEigen(tf) * p;
      }
      pts.push_back(p);
    }
    const PolylineReference ref(pts);
    const auto tf_focal =
        shared_resources_->transform_buffer_ptr->lookupTransform(world_frame_, focal_frame_, tf2::TimePointZero);
    const Eigen::Vector3d p0 = tf2::transformToEigen(tf_focal).translation();
    lift_dir_ = tf2::transformToEigen(tf_focal).linear().col(2).normalized();  // focal +Z: toward the lens/sun
    const Eigen::Vector3d start = ref.point(start_at);
    if ((p0 - start).norm() > start_tol_)
    {
      shared_resources_->logger->publishFailureMessage(
          name(), fmt::format("Focal point is {:.1f} mm from the path point at {:.1f} mm (limit {:.1f} mm); move "
                              "there first.",
                              1e3 * (p0 - start).norm(), 1e3 * start_at, 1e3 * start_tol_));
      return BT::NodeStatus::FAILURE;
    }
  }
  catch (const tf2::TransformException& e)
  {
    shared_resources_->logger->publishFailureMessage(name(), std::string("TF lookup failed: ") + e.what());
    return BT::NodeStatus::FAILURE;
  }
  {
    std::vector<geometry_msgs::msg::PoseStamped> world_path;
    for (const auto& p : pts)
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = world_frame_;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.position.z = p.z();
      ps.pose.orientation.w = 1.0;
      world_path.push_back(ps);
    }
    setOutput("path_world", world_path);
  }
  path_.emplace(std::move(pts));
  s_ = std::clamp(start_at, 0.0, path_->length());
  paused_ = false;
  ever_acquired_ = false;
  setOutput("progress_m", s_.load());
  over_since_ = -1.0;
  started_clock_ = false;
  done_ = false;
  result_ = BT::NodeStatus::RUNNING;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    sun_.valid = false;
  }

  auto& node = shared_resources_->node;
  cmd_pub_ = node->create_publisher<moveit_pro_controllers_msgs::msg::VelocityForceCommand>(command_topic_, 10);
  sun_pub_ = node->create_publisher<geometry_msgs::msg::Vector3Stamped>("/sinter/sun_error_stream", 10);
  track_pub_ = node->create_publisher<geometry_msgs::msg::Vector3Stamped>("/sinter/track_error", 10);
  progress_pub_ = node->create_publisher<std_msgs::msg::Float64>("/sinter/track_progress", 10);
  dex_pub_ = node->create_publisher<std_msgs::msg::Float64>("/sinter/dexterity", 10);
  loop_ms_pub_ = node->create_publisher<std_msgs::msg::Float64>("/sinter/track_loop_ms", 10);
  lift_pub_ = node->create_publisher<std_msgs::msg::Float64>("/sinter/track_lift", 10);
  t_prev_ = {};
  for (auto* v : { &t_step_, &t_dex_, &t_detect_, &t_period_ })
  {
    v->clear();
    v->reserve(4096);
  }
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = shared_resources_->reentrant_callback_group;
  info_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr& m) { onCameraInfo(m); }, opts);
  if (dex_)
  {
    js_names_.clear();
    js_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::ConstSharedPtr& m) {
          std::lock_guard<std::mutex> lk(mutex_);
          js_names_ = m->name;
          js_positions_ = m->position;
        },
        opts);
  }
  rclcpp::SubscriptionOptions image_opts;
  image_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  image_opts.callback_group = image_cb_group_;
  image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& m) { onImage(m); }, image_opts);
  last_sun_seen_ = node->now();
  last_frame_ns_ = node->now().nanoseconds();
  start_time_ = node->now();
  timer_ = node->create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_), [this] { controlStep(); },
                                   shared_resources_->callback_group_mutually_exclusive);
  shared_resources_->logger->publishInfoMessage(
      name(), fmt::format("Tracking {:.1f} mm of path at {:.2f} mm/s ({} points).", 1e3 * path_->length(),
                          1e3 * speed_, path_msgs.size()));
  return BT::NodeStatus::RUNNING;
}

void TrackRasterWithSun::onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
{
  if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0)
  {
    return;
  }
  std::lock_guard<std::mutex> lk(mutex_);
  intrinsics_ = std::array<double, 4>{ msg->k[0], msg->k[4], msg->k[2], msg->k[5] };
}

void TrackRasterWithSun::onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  std::optional<std::array<double, 4>> k;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    k = intrinsics_;
  }
  if (!k)
  {
    return;
  }
  cv_bridge::CvImageConstPtr cv;
  try
  {
    cv = cv_bridge::toCvShare(msg, "rgb8");
  }
  catch (const cv_bridge::Exception&)
  {
    return;
  }
  last_frame_ns_ = shared_resources_->node->now().nanoseconds();  // a usable frame arrived (sun in it or not)
  std::vector<cv::KeyPoint> kps;
  const auto t_det0 = std::chrono::steady_clock::now();
  detector_->detect(cv->image, kps);
  const double det_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_det0).count();
  SunMeasurement m;
  m.stamp = shared_resources_->node->now();
  if (!kps.empty())
  {
    const auto& best = *std::max_element(kps.begin(), kps.end(),
                                         [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.size < b.size; });
    const auto& [fx, fy, cx, cy] = *k;
    m.vx = (best.pt.x - cx) / fx - aim_x_;
    m.vy = (best.pt.y - cy) / fy - aim_y_;
    m.valid = true;
  }
  geometry_msgs::msg::Vector3Stamped out;
  out.header = msg->header;
  out.vector.x = m.valid ? m.vx : std::nan("");
  out.vector.y = m.valid ? m.vy : std::nan("");
  out.vector.z = m.valid ? std::atan(std::hypot(m.vx, m.vy)) : std::nan("");
  if (sun_pub_)
  {
    sun_pub_->publish(out);
  }
  std::lock_guard<std::mutex> lk(mutex_);
  sun_ = m;
  if (t_detect_.size() < 200000)
  {
    t_detect_.push_back(det_ms);
  }
}

void TrackRasterWithSun::publishTwist(const Eigen::Vector3d& v, const Eigen::Vector3d& w)
{
  moveit_pro_controllers_msgs::msg::VelocityForceCommand cmd;
  cmd.header.stamp = shared_resources_->node->now();
  cmd.header.frame_id = world_frame_;
  cmd.control_frame_id = world_frame_;
  cmd.control_frame_offset.rotation.w = 1.0;
  cmd.velocity_controlled_axes.x = cmd.velocity_controlled_axes.y = cmd.velocity_controlled_axes.z = true;
  cmd.velocity_controlled_axes.rx = cmd.velocity_controlled_axes.ry = cmd.velocity_controlled_axes.rz = true;
  cmd.twist.linear.x = v.x();
  cmd.twist.linear.y = v.y();
  cmd.twist.linear.z = v.z();
  cmd.twist.angular.x = w.x();
  cmd.twist.angular.y = w.y();
  cmd.twist.angular.z = w.z();
  cmd_pub_->publish(cmd);
}

void TrackRasterWithSun::stop(const std::string& why, BT::NodeStatus result)
{
  {
    std::lock_guard<std::mutex> lk(mutex_);
    message_ = why;
    result_ = result;
  }
  {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    publishTwist(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    done_ = true;
  }
  if (timer_)
  {
    timer_->cancel();
  }
}

void TrackRasterWithSun::finish(const std::string& why, BT::NodeStatus result, const rclcpp::Time& now)
{
  if (lift_m_ <= 0.0)
  {
    stop(why, result);
    return;
  }
  if (finishing_)
  {
    return;
  }
  finishing_ = true;
  finish_msg_ = why;
  finish_result_ = result;
  finish_deadline_ = now.seconds() + (lift_m_ - lift_) / lift_speed_ + 3.0;
  shared_resources_->logger->publishWarnMessage(
      name(), fmt::format("{} Lifting the focus {:.0f} mm off the bed before stopping.", why, 1e3 * lift_m_));
}

void TrackRasterWithSun::controlStep()
{
  if (done_)
  {
    return;
  }
  // Time the whole step (also on early returns), and the actual period between ticks.
  const auto t_step0 = std::chrono::steady_clock::now();
  if (t_prev_.time_since_epoch().count() != 0)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (t_period_.size() < 200000)
    {
      t_period_.push_back(std::chrono::duration<double, std::milli>(t_step0 - t_prev_).count());
    }
  }
  t_prev_ = t_step0;
  struct StepTimer
  {
    TrackRasterWithSun* self;
    std::chrono::steady_clock::time_point t0;
    ~StepTimer()
    {
      const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      {
        std::lock_guard<std::mutex> lk(self->mutex_);
        if (self->t_step_.size() < 200000)
        {
          self->t_step_.push_back(ms);
        }
      }
      std_msgs::msg::Float64 msg;
      msg.data = ms;
      if (self->loop_ms_pub_)
      {
        self->loop_ms_pub_->publish(msg);
      }
    }
  } step_timer{ this, t_step0 };
  const rclcpp::Time now = shared_resources_->node->now();
  const double dt = started_clock_ ? std::clamp((now - last_tick_).seconds(), 0.0, 0.2) : 0.0;
  last_tick_ = now;
  started_clock_ = true;

  Eigen::Isometry3d T_world_focal;
  try
  {
    T_world_focal = tf2::transformToEigen(
        shared_resources_->transform_buffer_ptr->lookupTransform(world_frame_, focal_frame_, tf2::TimePointZero));
  }
  catch (const tf2::TransformException& e)
  {
    stop(std::string("TF lookup failed: ") + e.what(), BT::NodeStatus::FAILURE);
    return;
  }
  SunMeasurement sun;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    sun = sun_;
  }
  // Camera watchdog: frames with no sun are a cloud (pause below); no frames at all is a dead camera or driver,
  // and the real sun may still be focused on the bed, so fail and let the objective retreat.
  const double frame_age = 1e-9 * static_cast<double>(now.nanoseconds() - last_frame_ns_.load());
  if (camera_timeout_ > 0.0 && frame_age > camera_timeout_)
  {
    finish(fmt::format("No sun-camera frames for {:.1f} s (camera, its USB link or camera_info is down).", frame_age),
           BT::NodeStatus::FAILURE, now);
  }
  const bool sun_fresh = sun.valid && (now - sun.stamp).seconds() < sun_timeout_;

  // Finishing: lift the focus off the bed (lens held still), then report the stored result.
  if (finishing_)
  {
    lift_ = std::min(lift_m_, lift_ + lift_speed_ * dt);
    const Eigen::Vector3d p_ref = path_->point(s_.load()) + lift_ * lift_dir_;
    const Eigen::Vector3d err = p_ref - T_world_focal.translation();
    const bool lifted = lift_ >= lift_m_ - 1e-9 && err.norm() < 0.002;
    if (lifted || now.seconds() > finish_deadline_)
    {
      stop(lifted ? finish_msg_ :
                    finish_msg_ + fmt::format(" (lift incomplete: {:.1f} mm from the lifted point)", 1e3 * err.norm()),
           finish_result_);
      return;
    }
    const Eigen::Vector3d v = clampNorm((lift_ < lift_m_ ? lift_speed_ : 0.0) * lift_dir_ + gain_pos_ * err, max_lin_);
    std_msgs::msg::Float64 lift_msg;
    lift_msg.data = lift_;
    lift_pub_->publish(lift_msg);
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    if (!done_)
    {
      publishTwist(v, Eigen::Vector3d::Zero());
    }
    return;
  }
  if (sun_fresh)
  {
    last_sun_seen_ = now;
  }
  else if (sun_abort_ > 0.0 && (now - last_sun_seen_).seconds() > sun_abort_)
  {
    finish(fmt::format("No sun for {:.1f} s.", sun_abort_), BT::NodeStatus::FAILURE, now);
    return;
  }

  // Sun lost: pause. Hold the focal point, hold the lens, stop the path; re-acquire before resuming (the sun
  // moved while it was hidden).
  if (!sun_fresh && !paused_ && ever_acquired_)  // before the first acquisition it is just waiting for frames
  {
    paused_ = true;
    acquired_ = false;
    shared_resources_->logger->publishWarnMessage(
        name(), fmt::format("Sun lost at {:.1f} mm along the path: pausing, focus lifted off the bed.", 1e3 * s_.load()));
  }

  // Acquire: hold the path start until the lens is on the sun.
  if (!acquired_)
  {
    if (sun_fresh && std::atan(std::hypot(sun.vx, sun.vy)) < acquire_tol_)
    {
      acquired_ = true;
      ever_acquired_ = true;
      if (paused_)
      {
        paused_ = false;
        shared_resources_->logger->publishInfoMessage(
            name(), fmt::format("Sun re-acquired; resuming at {:.1f} mm.", 1e3 * s_.load()));
      }
      else
      {
        shared_resources_->logger->publishInfoMessage(
            name(), fmt::format("Sun acquired after {:.1f} s; starting the scan.", (now - start_time_).seconds()));
      }
    }
    else if (acquire_timeout_ > 0.0 && !paused_ && (now - start_time_).seconds() > acquire_timeout_)
    {
      finish(fmt::format("Sun not acquired within {:.0f} s.", acquire_timeout_), BT::NodeStatus::FAILURE, now);
      return;
    }
    // A wrong blob (glint) far off the axis would steer the lens without limit while acquiring.
    const double acq_err = sun_fresh ? std::atan(std::hypot(sun.vx, sun.vy)) : 0.0;
    if (!acquired_ && sun_fresh && acq_err > max_acquire_err_)
    {
      if (acquire_over_since_ < 0.0)
      {
        acquire_over_since_ = now.seconds();
      }
      else if (now.seconds() - acquire_over_since_ > 1.0)
      {
        finish(fmt::format("Sun error {:.1f} deg while acquiring (limit {:.1f} deg): wrong blob or lost alignment.",
                           acq_err * 180.0 / M_PI, max_acquire_err_ * 180.0 / M_PI),
               BT::NodeStatus::FAILURE, now);
        return;
      }
    }
    else
    {
      acquire_over_since_ = -1.0;
    }
  }

  // Pointing: sustained large error is a fault (wrong blob, lost alignment); one bad frame is not.
  const double point_err = sun_fresh ? std::atan(std::hypot(sun.vx, sun.vy)) : 0.0;
  if (acquired_ && sun_fresh && point_err > max_point_err_)
  {
    if (over_since_ < 0.0)
    {
      over_since_ = now.seconds();
    }
    else if (now.seconds() - over_since_ > 0.5)
    {
      over_since_ = -1.0;
      finish(fmt::format("Pointing error {:.2f} deg above the {:.2f} deg limit for 0.5 s.", point_err * 180.0 / M_PI,
                         max_point_err_ * 180.0 / M_PI),
             BT::NodeStatus::FAILURE, now);
      return;
    }
  }
  else
  {
    over_since_ = -1.0;
  }

  // Defocus lift: up whenever the sun is not locked, down once it is. The path only advances fully down and locked,
  // so the spot is never focused on one point while the lens waits.
  const double lift_target = (acquired_ && sun_fresh) ? 0.0 : lift_m_;
  const double lift_prev = lift_;
  lift_ = lift_target > lift_ ? std::min(lift_target, lift_ + lift_speed_ * dt) :
                                std::max(lift_target, lift_ - lift_speed_ * dt);
  const double lift_rate = dt > 0.0 ? (lift_ - lift_prev) / dt : 0.0;
  const bool scanning = sun_fresh && acquired_ && lift_ <= 1e-9;
  if (scanning)
  {
    s_ = std::min(s_.load() + speed_ * dt, path_->length());
  }
  const Eigen::Vector3d p = T_world_focal.translation();
  const double s_now = s_.load();
  const Eigen::Vector3d p_ref = path_->point(s_now) + lift_ * lift_dir_;
  const Eigen::Vector3d err = p_ref - p;
  if (err.norm() > max_track_err_)
  {
    finish(fmt::format("Focal point {:.2f} mm off the path (limit {:.2f} mm).", 1e3 * err.norm(),
                       1e3 * max_track_err_),
           BT::NodeStatus::FAILURE, now);
    return;
  }
  const bool at_end = s_now >= path_->length() - 1e-9;
  if (at_end && lift_ <= 1e-9)
  {
    if (at_end_since_ < 0.0)
    {
      at_end_since_ = now.seconds();
    }
    const bool settled = err.norm() < finish_tol_;
    if (settled || now.seconds() - at_end_since_ > finish_timeout_)
    {
      finish(settled ? std::string("Path complete.") :
                       fmt::format("Path complete (end point settled to {:.2f} mm).", 1e3 * err.norm()),
             BT::NodeStatus::SUCCESS, now);
      return;
    }
  }

  const Eigen::Vector3d v_ff = ((scanning && !at_end) ? Eigen::Vector3d(speed_ * path_->tangent(s_now))
                                                       : Eigen::Vector3d(Eigen::Vector3d::Zero())) +
                               lift_rate * lift_dir_;
  const Eigen::Vector3d v = clampNorm(v_ff + gain_pos_ * err, max_lin_);
  const Eigen::Vector3d w_focal =
      sun_fresh ? sunErrorToFocalAngularVelocity(sun.vx, sun.vy, gain_rot_) : Eigen::Vector3d(Eigen::Vector3d::Zero());
  Eigen::Vector3d w = clampNorm(T_world_focal.linear() * w_focal, max_ang_);

  // Singularity: monitor dexterity and (optionally) use the free roll to climb away from singularities.
  if (dex_)
  {
    const auto t_dex0 = std::chrono::steady_clock::now();
    std::vector<std::string> names;
    std::vector<double> positions;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      names = js_names_;
      positions = js_positions_;
    }
    const auto d_now = names.empty() ? std::nullopt : dex_->evaluate(names, positions);
    if (!d_now)
    {
      // No joint states yet, or the Jacobian failed: tolerate briefly, then fail rather than run unprotected.
      if (dex_bad_since_ < 0.0)
      {
        dex_bad_since_ = now.seconds();
      }
      else if (now.seconds() - dex_bad_since_ > 2.0)
      {
        finish("Dexterity check not working for 2 s (no /joint_states, or joints not in the robot model).",
               BT::NodeStatus::FAILURE, now);
        return;
      }
    }
    else
    {
      dex_bad_since_ = -1.0;
    }
    if (!names.empty())
    {
      if (const auto& d = d_now)
      {
        std_msgs::msg::Float64 dm;
        dm.data = *d;
        dex_pub_->publish(dm);
        if (min_dexterity_ > 0.0 && *d < min_dexterity_)
        {
          finish(fmt::format("Near a singularity: dexterity {:.3f} below the {:.3f} limit.", *d, min_dexterity_),
                 BT::NodeStatus::FAILURE, now);
          return;
        }
      }
      if (gain_roll_ > 0.0 && acquired_)
      {
        const Eigen::Vector3d axis = T_world_focal.linear().col(2);
        if (const auto g = dex_->rollGradient(names, positions, axis))
        {
          w += std::clamp(gain_roll_ * *g, -max_roll_rate_, max_roll_rate_) * axis;
        }
      }
    }
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (t_dex_.size() < 200000)
      {
        t_dex_.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_dex0).count());
      }
    }
  }
  {
    // Check and publish under the same lock as onHalted/stop, so no non-zero twist can follow their zero twist.
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    if (done_)
    {
      return;
    }
    publishTwist(v, w);
  }

  geometry_msgs::msg::Vector3Stamped te;
  te.header.stamp = now;
  te.header.frame_id = world_frame_;
  te.vector.x = err.x();
  te.vector.y = err.y();
  te.vector.z = err.z();
  track_pub_->publish(te);
  std_msgs::msg::Float64 prog;
  prog.data = path_->length() > 0.0 ? s_now / path_->length() : 1.0;
  progress_pub_->publish(prog);
  std_msgs::msg::Float64 lift_msg;
  lift_msg.data = lift_;
  lift_pub_->publish(lift_msg);
}

BT::NodeStatus TrackRasterWithSun::onRunning()
{
  setOutput("progress_m", s_.load());
  if (!done_)
  {
    return BT::NodeStatus::RUNNING;
  }
  std::string msg;
  BT::NodeStatus res;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    msg = message_;
    res = result_;
  }
  teardown();
  logTiming();
  if (res == BT::NodeStatus::SUCCESS)
  {
    shared_resources_->logger->publishInfoMessage(name(), msg);
  }
  else
  {
    shared_resources_->logger->publishFailureMessage(name(), msg);
  }
  return res;
}

void TrackRasterWithSun::onHalted()
{
  {
    std::lock_guard<std::mutex> lk(cmd_mutex_);  // a control step in flight cannot publish after this zero twist
    done_ = true;
    if (cmd_pub_)
    {
      publishTwist(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    }
  }
  teardown();
  logTiming();
  setOutput("progress_m", s_.load());
  shared_resources_->logger->publishWarnMessage(
      name(), fmt::format("Stopped at {:.1f} mm along the path (zero twist sent).", 1e3 * s_.load()));
}

void TrackRasterWithSun::logTiming()
{
  auto stats = [](std::vector<double> v) -> std::string {
    if (v.empty())
    {
      return "n/a";
    }
    std::sort(v.begin(), v.end());
    double mean = 0.0;
    for (double x : v)
    {
      mean += x;
    }
    mean /= static_cast<double>(v.size());
    const double p99 = v[std::min(v.size() - 1, static_cast<size_t>(0.99 * static_cast<double>(v.size())))];
    return fmt::format("mean {:.3f} / p99 {:.3f} / max {:.3f} ms (n={})", mean, p99, v.back(), v.size());
  };
  std::vector<double> step, dex, det, per;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    step = t_step_;
    dex = t_dex_;
    det = t_detect_;
    per = t_period_;
  }
  shared_resources_->logger->publishInfoMessage(
      name(), fmt::format("Timing: control step {} | dexterity part {} | sun detection per frame {} | tick period {} "
                          "(target {:.1f} ms)",
                          stats(step), stats(dex), stats(det), stats(per), 1000.0 / rate_hz_));
}

void TrackRasterWithSun::teardown()
{
  if (timer_)
  {
    timer_->cancel();
    timer_.reset();
  }
  image_sub_.reset();
  info_sub_.reset();
  js_sub_.reset();
}
}  // namespace sun_pose_finder
