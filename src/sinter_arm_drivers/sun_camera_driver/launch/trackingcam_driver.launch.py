from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # --- Paths ---
    camera_pkg = get_package_share_directory("sun_camera_driver")
    config_path = PathJoinSubstitution([camera_pkg, "config", "trackingcam.yaml"])

    # --- Nodes ---

    camera_driver = Node(
        package="camera_ros",
        executable="camera_node",
        parameters=[{
            'camera': 2,
            'frame_id': 'j2n6s300_suncam_optical',
            'camera_info_url': ['file://', config_path],
            'orientation': 180,
            'width': 1920,
            'height': 1080,
            'format': 'MJPEG',
            }],
    )



    return LaunchDescription([camera_driver])