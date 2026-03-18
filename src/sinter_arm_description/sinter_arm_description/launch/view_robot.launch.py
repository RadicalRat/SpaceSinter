from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # --- Paths ---
    description_pkg = get_package_share_directory("sinter_arm_description")
    urdf_path = PathJoinSubstitution([description_pkg, "urdf", "j2n6s300.urdf.xacro"])
    rviz_config = PathJoinSubstitution([description_pkg, "launch", "view_robot.rviz"])

    # --- Nodes ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(Command(["xacro ", urdf_path]), value_type=str)}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui", executable="joint_state_publisher_gui"
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([robot_state_publisher, joint_state_publisher_gui, rviz2])
