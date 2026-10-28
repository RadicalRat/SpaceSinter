"""Extra nodes for the mock robot: a simulated sun-tracking camera."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mock_share = get_package_share_directory("sinter_robot_mock")
    calibration = os.path.join(
        get_package_share_directory("sun_camera_driver"), "config", "trackingcam.yaml"
    )

    sim_sun_camera = Node(
        package="sinter_robot_mock",
        executable="sim_sun_camera.py",
        name="sim_sun_camera",
        output="screen",
        parameters=[
            os.path.join(mock_share, "config", "sim_sun_camera.yaml"),
            {"calibration_file": calibration},
        ],
    )

    return LaunchDescription([sim_sun_camera])
