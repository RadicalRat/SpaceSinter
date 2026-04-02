#include <behaviortree_cpp_v3/action_node.h>
#include "moveit_studio_behavior_interface/shared_resources_node.hpp"
#include "SpiceUsr.h"
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

namespace my_robot_behaviors {

class GetSolarPointing : public moveit_studio_behavior_interface::SharedResourcesNode {
public:
  GetSolarPointing(const std::string& name, const BT::NodeConfiguration& config,
                   const std::shared_ptr<moveit_studio_behavior_interface::BehaviorContext>& shared_resources)
      : SharedResourcesNode(name, config, shared_resources) {}

  // Define the inputs/outputs for the BT
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("kernel_path", "solar_kernels.tm", "Path to SPICE meta-kernel"),
        BT::OutputPort<geometry_msgs::msg::Quaternion>("target_quaternion", "Solar pointing orientation")
    };
  }

  // This is where your main() logic goes
  BT::NodeStatus tick() override;
};

} // namespace my_robot_behaviors

BT::NodeStatus GetSolarPointing::tick() {
  // 1. Get Input from BT
  auto kernel_path = getInput<std::string>("kernel_path");
  if (!kernel_path) return BT::NodeStatus::FAILURE;

  // 2. Load Kernels (Equivalent to your Step 1)
  furnsh_c(kernel_path->c_str());

  // ... [Insert your Lat/Lon and Earth Radii logic here] ...
  // Define Observer (Golden, CO)
  SpiceDouble lat_deg = 39.75;
  SpiceDouble lon_deg = -105.22;
  SpiceDouble alt = 1.7;

  // Get Earth radii from the PCK
  SpiceDouble radii[3], re, rp, f, obspos[3]
  SpiceInt n;
  bodvrd_c("EARTH", "RADII", 3, &n, radii);
  re = radii[0];
  rp = radii[2];
  f = (re - rp) / re;

  // Convert to rectangular
  SpiceDouble lon_rad = lon_deg * rpd_c();
  SpiceDouble lat_rad = lat_deg * rpd_c();
  georec_c(lon_rad, lat_rad, alt, re, f, obspos);

  // 3. Get Current Time (Using C++ style or keeping your C snippets)
  time_t rawtime = time(NULL);
  struct tm *utc = gmtime(&rawtime);
  char timstr[32];
  SpiceDouble et;

  sprintf(timstr, "%04d-%02d-%02dT%02d:%02d:%02d", utc->tm_year + 1900, utc->tm_mon +1, utc->tm_mday, utc->tm_hour, utc->tm_min, utc->tm_sec);
  str2et_c(timstr, &et);

  SpiceDouble azlstat[6], lt;

  // (Your existing SPICE logic for azlcpo_c and twovec_c goes here)
  azlcpo_c("ELLIPSOID", "SUN", et, "LT+S", SPICEFALSE, SPICETRUE, obspos, "EARTH", "IAU_EARTH", azlstat, &lt);

  SpiceDouble az_rad = azlstat[1];
  SpiceDouble el_rad = azlstat[2];
  SpiceDouble unit_enu[3];
  unit_enu[0] = sin(az_rad) * cos(el_rad);
  unit_enu[1] = cos(az_rad) * sin(el_rad);
  unit_enu[2] = sin(el_rad);

  // 1. Look up the transform from World (where Sun ENU lives) to your Robot Base
  geometry_msgs::msg::TransformStamped tf_base_to_world;

  try {
    // "base_link" is your robot's longitudinal axis frame
    // "world" is the frame where your ENU Solar vector is defined
    tf_base_to_world = shared_resources_->tf_buffer->lookupTransform(
        "base_link", "world", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(node_->get_logger(), "Could not get transform: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // 2. Convert to Eigen to extract the "Up" vector
  Eigen::Isometry3d base_to_world_eigen = tf2::transformToEigen(tf_base_to_world);

  // The "Up" vector of the world, expressed in the robot's base frame
  // If world Z is Up, we transform the (0,0,1) vector into the base frame
  Eigen::Vector3d world_up_in_base = base_to_world_eigen.rotation() * Eigen::Vector3d::UnitZ();

  // 3. Convert back to SpiceDouble for your twovec_c function
  SpiceDouble base_up[3] = {world_up_in_base.x(), world_up_in_base.y(), world_up_in_base.z()};

  // 1. Get the Sun Vector (your SPICE unit_enu) as an Eigen vector
  Eigen::Vector3d sun_direction(unit_enu[0], unit_enu[1], unit_enu[2]);

  // 2. Define which axis of your robot "head" points at the sun
  // Change UnitZ() to UnitX() if your alignment axis is the X-axis
  Eigen::Vector3d robot_pointer_axis = Eigen::Vector3d::UnitZ();

  // 3. Define your "Up" constraint (as we discussed, using the Base Longitudinal axis)
  // Let's assume your longitudinal axis is the Base X-axis
  Eigen::Vector3d base_longitudinal_axis = Eigen::Vector3d::UnitX();

  // 4. Create the rotation (Quaternion)
  // This finds the rotation that maps robot_pointer_axis to sun_direction
  Eigen::Quaterniond q_pointing = Eigen::Quaterniond::FromTwoVectors(robot_pointer_axis, sun_direction);

  // 5. Convert to ROS message
  geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q_pointing);
  setOutput("target_quaternion", q_msg);

  kclear_c(); // Important to prevent memory leaks/pool overflows
  return BT::NodeStatus::SUCCESS;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(my_robot_behaviors::GetSolarPointing, BT::TreeNodeBase)