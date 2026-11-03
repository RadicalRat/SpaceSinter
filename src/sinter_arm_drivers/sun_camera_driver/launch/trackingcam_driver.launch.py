import glob
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Arducam_16MP sun camera (USB vendor:product).
DEFAULT_USB_ID = "0c45:636d"


def _read(path):
    with open(path) as f:
        return f.read().strip()


def find_libcamera_ids(usb_id):
    """Return libcamera UVC camera IDs for video devices matching usb_id ("vvvv:pppp").

    Mirrors PipelineHandlerUVC::generateId() in libcamera: the firmware (ACPI) path of
    the nearest ancestor that has one, then the USB path without the bus number
    (e.g. "4:1.0"), then "vid:pid". This makes the selection independent of the
    camera index, which shifts whenever another camera is plugged in or drops out.
    """
    ids = set()
    for video in glob.glob("/sys/class/video4linux/video*"):
        intf = os.path.realpath(os.path.join(video, "device"))
        try:
            vid = _read(os.path.join(intf, "..", "idVendor"))
            pid = _read(os.path.join(intf, "..", "idProduct"))
        except OSError:
            continue
        if f"{vid}:{pid}" != usb_id.lower():
            continue
        controller, path = None, intf
        while len(path) > 1:
            path = os.path.dirname(path)
            fw = os.path.join(path, "firmware_node", "path")
            if os.path.exists(fw):
                controller = _read(fw)
                break
        if controller is None:
            continue
        port = os.path.basename(intf)
        port = port[port.find("-") + 1:]
        ids.add(f"{controller}-{port}-{vid}:{pid}")
    return sorted(ids)


def launch_camera(context):
    usb_id = LaunchConfiguration("usb_id").perform(context)
    ids = find_libcamera_ids(usb_id)
    actions = []
    if ids:
        # Lowest interface number (":1.0") is the video interface libcamera registers.
        camera = ids[0]
        actions.append(LogInfo(msg=f"Sun camera {usb_id} -> libcamera id '{camera}'"))
    else:
        # Pass the USB id anyway so camera_node fails with a clear "does not exist" error.
        camera = usb_id
        actions.append(LogInfo(msg=f"ERROR: sun camera {usb_id} not found on USB (unplugged or dropped out)"))

    config_path = PathJoinSubstitution(
        [get_package_share_directory("sun_camera_driver"), "config", "trackingcam.yaml"])
    actions.append(Node(
        package="camera_ros",
        executable="camera_node",
        parameters=[{
            'camera': camera,
            'frame_id': 'ee_camera_optical_link',
            'camera_info_url': ['file://', config_path],
            'orientation': 0,
            'width': 1280,  # 30 fps; trackingcam.yaml is derived for this mode
            'height': 720,
            'format': 'MJPEG',
        }],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("usb_id", default_value=DEFAULT_USB_ID,
                              description="USB vendor:product of the sun camera"),
        OpaqueFunction(function=launch_camera),
    ])
