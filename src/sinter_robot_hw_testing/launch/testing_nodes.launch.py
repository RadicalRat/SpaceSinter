"""Driver nodes for indoor testing: the real hardware's extra nodes plus the simulated sun camera."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    hw_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("sinter_robot_hw"),
            "launch", "additional_driver_nodes.launch.py"))
    )

    # Renders the sun on /camera/image_raw from the real arm's TF.
    sim_sun_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("sinter_robot_mock"),
            "launch", "sim_nodes.launch.py"))
    )

    return LaunchDescription([hw_nodes, sim_sun_camera])
