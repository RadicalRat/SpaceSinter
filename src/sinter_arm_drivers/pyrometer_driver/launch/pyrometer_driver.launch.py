from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # --- Paths ---
    pkg_share = get_package_share_directory("pyrometer_driver")
    urdf_path = PathJoinSubstitution([pkg_share, "urdf", "pyrometer.urdf.xacro"])
    controllers_yaml = PathJoinSubstitution(
        [pkg_share, "config", "control", "ros2_controllers.yaml"]
    )

    # --- Launch arguments ---
    sensor_name_arg = DeclareLaunchArgument(
        "sensor_name", default_value="pyrometer",
        description="Name of the <sensor> declared in the URDF; must match config/control/ros2_controllers.yaml"
    )
    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/pyrometer",
        description="Serial device path for the pyrometer (see udev/99-pyrometer.rules)"
    )
    baud_rate_arg = DeclareLaunchArgument(
        "baud_rate", default_value="115200", description="Serial baud rate"
    )
    poll_rate_arg = DeclareLaunchArgument(
        "poll_rate_hz", default_value="100.0",
        description="Background serial poll rate, Hz (>= 100 recommended)"
    )
    response_timeout_arg = DeclareLaunchArgument(
        "response_timeout_ms", default_value="50", description="Per-command serial read timeout, ms"
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id", default_value="pyrometer_link",
        description="Frame stamped on published sensor_msgs/Temperature messages"
    )

    # --- Robot description (single-sensor URDF + ros2_control block) ---
    robot_description = ParameterValue(
        Command([
            "xacro ", urdf_path,
            " sensor_name:=", LaunchConfiguration("sensor_name"),
            " port:=", LaunchConfiguration("port"),
            " baud_rate:=", LaunchConfiguration("baud_rate"),
            " poll_rate_hz:=", LaunchConfiguration("poll_rate_hz"),
            " response_timeout_ms:=", LaunchConfiguration("response_timeout_ms"),
            " frame_id:=", LaunchConfiguration("frame_id"),
        ]),
        value_type=str,
    )

    # --- Nodes ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[{"robot_description": robot_description}, controllers_yaml],
    )

    pyrometer_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pyrometer_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        sensor_name_arg,
        port_arg,
        baud_rate_arg,
        poll_rate_arg,
        response_timeout_arg,
        frame_id_arg,
        robot_state_publisher,
        controller_manager,
        pyrometer_broadcaster_spawner,
    ])
