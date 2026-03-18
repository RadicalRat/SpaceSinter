from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # --- Paths ---
    # camera_pkg = get_package_share_directory("sun_camera_driver")
    # config_path = PathJoinSubstitution([camera_pkg, "config", "trackingcam.yaml"])

    # --- Nodes ---

    return LaunchDescription([])