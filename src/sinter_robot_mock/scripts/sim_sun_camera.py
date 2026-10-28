#!/usr/bin/env python3
# Copyright 2026 Space Sinter Team, Colorado School of Mines
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Simulated sun-tracking camera.

Stands in for the real Arducam (camera_ros) on the mock robot.  The sun is
modelled as a direction at infinity in the world frame.  Every frame the node
looks up the camera's pose from TF (driven by the mock arm's joint states),
projects the sun direction through the real camera's intrinsics, and renders a
saturated disk with a soft halo on a dark, noisy background.  Moving the arm
therefore moves the blob in the image exactly like the real camera would, so
the SunPoseFinder → move loop can be exercised end to end.

Publishes (same topics as the real driver):
  /camera/image_raw    sensor_msgs/Image (rgb8)
  /camera/camera_info  sensor_msgs/CameraInfo
  TF  world → sim_sun  (a frame along the sun direction, for visualisation)

Services:
  ~/place_sun_in_view  std_srvs/Trigger — put the sun just off the current
                       camera axis (by `initial_offset_deg`)

Parameters are documented in config/sim_sun_camera.yaml.  Sun parameters can be
changed live, e.g.:
  ros2 param set /sim_sun_camera sun_azimuth_deg 45.0
"""

import math

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def quat_to_matrix(x, y, z, w):
    """Rotation matrix for a unit quaternion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def direction_from_az_el(az_deg, el_deg):
    az, el = math.radians(az_deg), math.radians(el_deg)
    return np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])


def az_el_from_direction(d):
    return math.degrees(math.atan2(d[1], d[0])), math.degrees(math.asin(np.clip(d[2], -1, 1)))


class SimSunCamera(Node):

    def __init__(self):
        super().__init__('sim_sun_camera')

        p = self.declare_parameter
        self.world_frame = p('world_frame', 'world').value
        self.camera_frame = p('camera_frame', 'ee_camera_optical_link').value
        calibration_file = p('calibration_file', '').value
        self.scale = p('image_scale', 1.0).value
        rate_hz = p('publish_rate_hz', 10.0).value

        # Sun model.
        p('sun_placement', 'in_view')  # 'in_view' or 'fixed'
        p('sun_azimuth_deg', 0.0)
        p('sun_elevation_deg', 45.0)
        p('sun_drift_deg_per_s', 0.0)
        p('initial_offset_deg', [4.0, -3.0])
        self.angular_diameter_deg = p('sun_angular_diameter_deg', 1.5).value
        self.halo_scale = p('halo_scale', 1.5).value
        self.background_level = p('background_level', 12).value
        self.noise_stddev = p('noise_stddev', 3.0).value
        self.marker_distance = p('sun_marker_distance_m', 1.5).value
        self.max_tf_age = Duration(seconds=p('max_tf_age_s', 0.1).value)

        self.width, self.height, self.k = self._load_intrinsics(calibration_file)
        self.camera_info = self._make_camera_info()

        self.sun_dir = None  # unit vector in world frame; None until placed
        if self.get_parameter('sun_placement').value == 'fixed':
            self.sun_dir = direction_from_az_el(
                self.get_parameter('sun_azimuth_deg').value,
                self.get_parameter('sun_elevation_deg').value)

        # A few pre-rendered background frames, cycled, so per-frame cost is
        # just a copy plus the sun patch.
        rng = np.random.default_rng(0)
        self.backgrounds = [
            np.clip(rng.normal(self.background_level, self.noise_stddev,
                               (self.height, self.width, 1)), 0, 255)
            .astype(np.uint8).repeat(3, axis=2)
            for _ in range(4)
        ]
        self.frame_idx = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 1)
        self.create_service(Trigger, '~/place_sun_in_view', self._on_place_sun)
        self.add_on_set_parameters_callback(self._on_params)

        # Render in its own callback group so the multi-threaded executor keeps
        # processing /tf while a frame is drawn.  Otherwise /tf queues up behind
        # rendering and the sun is drawn for where the arm was, not where it is.
        self.period = 1.0 / rate_hz
        self.create_timer(self.period, self._tick, callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(
            f'Simulated sun camera: {self.width}x{self.height} @ {rate_hz:.0f} Hz, '
            f'frame={self.camera_frame}, sun diameter={self.angular_diameter_deg}°')

    # ── Setup ────────────────────────────────────────────────────────────────

    def _load_intrinsics(self, path):
        if path:
            with open(path, encoding='utf-8') as f:
                calib = yaml.safe_load(f)
            k = np.array(calib['camera_matrix']['data'], dtype=float).reshape(3, 3)
            width, height = calib['image_width'], calib['image_height']
        else:
            self.get_logger().warn('No calibration_file given; using a generic 640x480 camera.')
            width, height = 640, 480
            k = np.array([[500.0, 0, 320.0], [0, 500.0, 240.0], [0, 0, 1]])
        if self.scale != 1.0:
            k[:2, :] *= self.scale
            width, height = int(round(width * self.scale)), int(round(height * self.scale))
        return width, height, k

    def _make_camera_info(self):
        # The image is rendered with an ideal pinhole, so report zero distortion.
        info = CameraInfo()
        info.header.frame_id = self.camera_frame
        info.width, info.height = self.width, self.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = self.k.flatten().tolist()
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.k[0, 0], 0.0, self.k[0, 2], 0.0,
                  0.0, self.k[1, 1], self.k[1, 2], 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    # ── Sun placement ────────────────────────────────────────────────────────

    def _camera_rotation(self):
        """Rotation taking world-frame vectors into the camera optical frame."""
        tf = self.tf_buffer.lookup_transform(self.camera_frame, self.world_frame, Time())
        age = self.get_clock().now() - Time.from_msg(tf.header.stamp)
        if age > self.max_tf_age:
            self.get_logger().warn(
                f'Camera TF is {age.nanoseconds * 1e-9:.2f} s old; the sun is drawn for a stale arm pose.',
                throttle_duration_sec=2.0)
        q = tf.transform.rotation
        return quat_to_matrix(q.x, q.y, q.z, q.w)

    def _place_sun_in_view(self, r_cam_world):
        # Camera +Z (optical axis) tilted by the configured offset, in world frame.
        ox, oy = (math.radians(v) for v in self.get_parameter('initial_offset_deg').value)
        d_cam = np.array([math.tan(ox), math.tan(oy), 1.0])
        self.sun_dir = r_cam_world.T @ (d_cam / np.linalg.norm(d_cam))
        az, el = az_el_from_direction(self.sun_dir)
        msg = f'Sun placed near camera axis: azimuth={az:.1f}°, elevation={el:.1f}°'
        self.get_logger().info(msg)
        return msg

    def _on_place_sun(self, _request, response):
        try:
            response.message = self._place_sun_in_view(self._camera_rotation())
            response.success = True
        except TransformException as e:
            response.success = False
            response.message = f'No TF {self.world_frame} → {self.camera_frame}: {e}'
        return response

    def _on_params(self, params):
        values = {prm.name: prm.value for prm in params}
        if 'sun_azimuth_deg' in values or 'sun_elevation_deg' in values:
            az, el = (az_el_from_direction(self.sun_dir) if self.sun_dir is not None
                      else (0.0, 45.0))
            az = values.get('sun_azimuth_deg', az)
            el = values.get('sun_elevation_deg', el)
            self.sun_dir = direction_from_az_el(az, el)
            self.get_logger().info(f'Sun moved to azimuth={az:.1f}°, elevation={el:.1f}°')
        return SetParametersResult(successful=True)

    # ── Rendering ────────────────────────────────────────────────────────────

    def _tick(self):
        try:
            r_cam_world = self._camera_rotation()
        except TransformException:
            self.get_logger().info(
                f'Waiting for TF {self.world_frame} → {self.camera_frame}...',
                throttle_duration_sec=5.0)
            return
        if self.sun_dir is None:
            self._place_sun_in_view(r_cam_world)

        drift = self.get_parameter('sun_drift_deg_per_s').value
        if drift:
            a = math.radians(drift * self.period)  # rotate about world Z
            c, s = math.cos(a), math.sin(a)
            self.sun_dir = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ self.sun_dir

        stamp = self.get_clock().now().to_msg()
        image = self.backgrounds[self.frame_idx % len(self.backgrounds)].copy()
        self.frame_idx += 1
        self._draw_sun(image, r_cam_world @ self.sun_dir)

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame
        msg.height, msg.width = self.height, self.width
        msg.encoding = 'rgb8'
        msg.step = self.width * 3
        msg.data = image.tobytes()
        self.camera_info.header.stamp = stamp
        self.image_pub.publish(msg)
        self.info_pub.publish(self.camera_info)
        self._publish_sun_tf(stamp)

    def _draw_sun(self, image, d_cam):
        if d_cam[2] <= 1e-6:
            return  # sun is behind the camera
        u = self.k[0, 0] * d_cam[0] / d_cam[2] + self.k[0, 2]
        v = self.k[1, 1] * d_cam[1] / d_cam[2] + self.k[1, 2]
        radius = self.k[0, 0] * math.tan(math.radians(self.angular_diameter_deg) / 2.0)
        halo = max(1.0, self.halo_scale * radius)
        reach = radius + 3.0 * halo

        x0, x1 = int(max(0, u - reach)), int(min(self.width, u + reach + 1))
        y0, y1 = int(max(0, v - reach)), int(min(self.height, v + reach + 1))
        if x0 >= x1 or y0 >= y1:
            return  # off-image

        ys, xs = np.mgrid[y0:y1, x0:x1]
        dist = np.hypot(xs - u, ys - v)
        # Saturated core; halo stays below the detector's 240 threshold so the
        # detected blob size tracks the configured angular diameter.
        glow = 200.0 * np.exp(-np.square(np.maximum(dist - radius, 0.0) / halo))
        patch = image[y0:y1, x0:x1].astype(np.float32)
        tint = np.array([1.0, 0.9, 0.65], dtype=np.float32)  # warm halo
        patch = np.maximum(patch, glow[..., None] * tint)
        patch[dist <= radius] = 255.0
        image[y0:y1, x0:x1] = patch.astype(np.uint8)

    def _publish_sun_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.world_frame
        t.child_frame_id = 'sim_sun'
        pos = self.sun_dir * self.marker_distance
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = pos
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = SimSunCamera()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
