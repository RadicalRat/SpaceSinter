// Publishes the sun position from the NOAA ephemeris at a fixed rate, independent of any running objective, so every
// rosbag has the ground-truth sun direction (for the FK-vs-ephemeris orientation check and the open-loop baseline) and
// the solar incidence angle on the last fitted bed plane.

#include <sun_pose_finder/solar_position.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace sun_pose_finder
{
class SunEphemerisNode : public rclcpp::Node
{
public:
  SunEphemerisNode()
    : Node("sun_ephemeris")
    , latitude_deg_(declare_parameter("latitude_deg", 39.7555))
    , longitude_deg_(declare_parameter("longitude_deg", -105.2211))
    , north_heading_deg_(declare_parameter("north_heading_deg", 100.0))
    , frame_id_(declare_parameter("frame_id", std::string("world")))
    , tf_buffer_(get_clock())
    , tf_listener_(tf_buffer_)
    , sun_pub_(create_publisher<geometry_msgs::msg::Vector3Stamped>("/sinter/sun_ephemeris", 10))
    , azimuth_pub_(create_publisher<std_msgs::msg::Float64>("/sinter/sun_azimuth_deg", 10))
    , elevation_pub_(create_publisher<std_msgs::msg::Float64>("/sinter/sun_elevation_deg", 10))
    , incidence_pub_(create_publisher<std_msgs::msg::Float64>("/sinter/bed_incidence_deg", 10))
    // FitPlaneRasterPath latches the plane, so this also gets a fit made before the node started.
    , plane_sub_(create_subscription<geometry_msgs::msg::PoseStamped>(
          "/sinter/bed_plane", rclcpp::QoS(1).transient_local(),
          [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { bed_plane_ = *msg; }))
  {
    const double rate_hz = declare_parameter("rate_hz", 1.0);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz), [this] { publish(); });
    RCLCPP_INFO(get_logger(), "Sun ephemeris at lat %.4f, lon %.4f, north heading %.1f deg in %s", latitude_deg_,
                longitude_deg_, north_heading_deg_, frame_id_.c_str());
  }

private:
  void publish()
  {
    const auto stamp = now();
    const auto sun = computeSolarPosition(std::chrono::duration<double>(
                                              std::chrono::system_clock::now().time_since_epoch())
                                              .count(),
                                          latitude_deg_, longitude_deg_);
    const Eigen::Vector3d sun_dir = sunDirection(sun, north_heading_deg_);

    geometry_msgs::msg::Vector3Stamped sun_msg;
    sun_msg.header.stamp = stamp;
    sun_msg.header.frame_id = frame_id_;
    sun_msg.vector.x = sun_dir.x();
    sun_msg.vector.y = sun_dir.y();
    sun_msg.vector.z = sun_dir.z();
    sun_pub_->publish(sun_msg);
    azimuth_pub_->publish(std_msgs::msg::Float64().set__data(sun.azimuth_deg));
    elevation_pub_->publish(std_msgs::msg::Float64().set__data(sun.elevation_deg));

    if (const auto normal = bedNormal())
    {
      // Angle between the sun ray and the bed normal: 0 = sun straight onto the bed, 90 = grazing.
      const double incidence = std::acos(std::clamp(normal->dot(sun_dir), -1.0, 1.0)) * 180.0 / M_PI;
      incidence_pub_->publish(std_msgs::msg::Float64().set__data(incidence));
    }
  }

  // Unit normal (+Z of the plane pose) of the last fitted bed, in frame_id_.
  std::optional<Eigen::Vector3d> bedNormal()
  {
    if (!bed_plane_)
    {
      return std::nullopt;
    }
    Eigen::Isometry3d plane;
    tf2::fromMsg(bed_plane_->pose, plane);
    Eigen::Vector3d normal = plane.linear().col(2);
    if (bed_plane_->header.frame_id != frame_id_)
    {
      try
      {
        normal = tf2::transformToEigen(
                     tf_buffer_.lookupTransform(frame_id_, bed_plane_->header.frame_id, tf2::TimePointZero))
                     .linear() *
                 normal;
      }
      catch (const tf2::TransformException& e)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "No bed incidence: %s", e.what());
        return std::nullopt;
      }
    }
    return normal.normalized();
  }

  const double latitude_deg_;
  const double longitude_deg_;
  const double north_heading_deg_;
  const std::string frame_id_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr sun_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr azimuth_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr elevation_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr incidence_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr plane_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<geometry_msgs::msg::PoseStamped> bed_plane_;
};
}  // namespace sun_pose_finder

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sun_pose_finder::SunEphemerisNode>());
  rclcpp::shutdown();
  return 0;
}
