from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # -----------------------
    # Launch Arguments
    # -----------------------
    hardware_interface = LaunchConfiguration("hardware_interface")

    declare_hardware_interface = DeclareLaunchArgument(
        "hardware_interface",
        default_value="mock",
        description="Which hardware interface implementation to use, if any: ''/mock",
    )

    # -----------------------
    # Resolve package share paths
    # -----------------------

    example_hw_share = get_package_share_directory("example_arm_hardware_interface")
    example_desc_share = get_package_share_directory("example_arm_description")

    # URDF via xacro
    urdf_path = os.path.join(example_hw_share, "urdf", "r6bot.urdf.xacro")

    robot_description_content = Command(
        ["xacro ", urdf_path, " hardware_interface:=", hardware_interface]
    )
    robot_description = {"robot_description": robot_description_content}

    # Controller config
    robot_controllers = os.path.join(
        example_hw_share, "config", "ros2_controllers.yaml"
    )

    # RViz config
    rviz_config_file = os.path.join(example_desc_share, "launch", "view_robot.rviz")

    # -----------------------
    # Nodes
    # -----------------------

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        emulate_tty=True,
        parameters=[robot_description, robot_controllers],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        emulate_tty=True,
        parameters=[robot_description],
    )

    controller_spawner = Node(
        name="robot_controller_spawner",
        package="controller_manager",
        executable="spawner",
        output="both",
        emulate_tty=True,
        arguments=[
            "joint_trajectory_controller",
            "--param-file",
            robot_controllers,
        ],
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=1.5,
        actions=[
            Node(
                name="joint_state_broadcaster_spawner",
                package="controller_manager",
                executable="spawner",
                output="both",
                emulate_tty=True,
                arguments=["joint_state_broadcaster"],
            )
        ],
    )

    rviz_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                output="both",
                emulate_tty=True,
                arguments=["-d", rviz_config_file],
            )
        ],
    )

    return LaunchDescription(
        [
            declare_hardware_interface,
            controller_manager_node,
            robot_state_publisher_node,
            controller_spawner,
            joint_state_broadcaster_spawner,
            rviz_node,
        ]
    )
