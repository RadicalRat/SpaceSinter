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


    return LaunchDescription([])