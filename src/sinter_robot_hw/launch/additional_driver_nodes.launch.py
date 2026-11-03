from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # --- Paths ---
    # camera_pkg = get_package_share_directory("sun_camera_driver")
    # config_path = PathJoinSubstitution([camera_pkg, "config", "trackingcam.yaml"])

    # --- Nodes ---
    # node_params = ["0", "0", "0", "1.5708", "0", "0", "world", "rotated_world"]
    # static_transform_publisher_node = Node(package="tf2_ros", executable="static_transform_publisher", arguments=node_params)


    # Solar flux from NREL SRRL (Golden), once a minute, so every rosbag has the DNI for the trial.
    srrl_irradiance_node = Node(package="srrl_irradiance", executable="srrl_irradiance_node", output="screen")

    # Ephemeris sun direction at 1 Hz, plus the solar incidence angle on the last fitted bed plane.
    # Keep north_heading_deg equal to the north_heading_deg tunable in sinter_raster_closed_loop.xml.
    sun_ephemeris_node = Node(
        package="sun_pose_finder",
        executable="sun_ephemeris_node",
        output="screen",
        parameters=[{"latitude_deg": 39.7555, "longitude_deg": -105.2211, "north_heading_deg": 125.0}],
    )

    return LaunchDescription([srrl_irradiance_node, sun_ephemeris_node])