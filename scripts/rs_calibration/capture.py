"""Save a RealSense cloud + TF every time the arm comes to rest at a new pose.

Runs inside the sinter-recorder image (see run_capture.sh). Each snapshot is
OUT_DIR/snap_NN.npz with the cloud in its own frame, T_world_link (FK of
ee_realsense_link), T_link_cloud (the mount + factory extrinsics in use) and the joints.
"""
import os
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

CLOUD = "/camera/camera/depth/color/points"
STILL_S = 1.5        # arm still this long before a capture
STILL_VEL = 0.004    # rad/s
NEW_POSE_RAD = 0.03  # a capture needs a joint change of at least this since the last one
FRAMES = 3           # clouds averaged per snapshot (stacked)


def mat(tf):
    q, t = tf.transform.rotation, tf.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [t.x, t.y, t.z]
    return T


class Capture(Node):
    def __init__(self, out):
        super().__init__("rs_mount_capture")
        self.out = out
        self.buf = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.clouds = []
        self.q = None
        self.q_time = 0.0
        self.moving_since = time.time()
        self.still_since = None
        self.last_q = None
        self.n = len([f for f in os.listdir(out) if f.startswith("snap_")])
        self.create_subscription(PointCloud2, CLOUD, self.on_cloud, 5)
        self.create_subscription(JointState, "/joint_states", self.on_js, 20)
        self.create_timer(0.2, self.tick)
        self.get_logger().info(f"capturing into {out}; Ctrl-C to stop")

    def on_cloud(self, m):
        self.clouds.append((time.time(), m))
        self.clouds = self.clouds[-FRAMES:]

    def on_js(self, m):
        now = time.time()
        q = dict(zip(m.name, m.position))
        # Speed from successive positions (the driver's velocity field may be empty or noisy).
        moving = False
        if self.q is not None and now > self.q_time:
            dq = max((abs(q[k] - self.q[k]) for k in q if k in self.q), default=0.0)
            moving = dq / max(now - self.q_time, 1e-3) > STILL_VEL and dq > 2e-4
        if self.q is None or moving or now - self.q_time > 0.05:
            self.q, self.q_time = q, now
        if moving:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now

    def tick(self):
        if self.q is None or self.still_since is None:
            return
        if time.time() - self.still_since < STILL_S:
            return
        names = sorted(self.q)
        qv = np.array([self.q[k] for k in names])
        if self.last_q is not None and np.max(np.abs(qv - self.last_q)) < NEW_POSE_RAD:
            return
        fresh = [m for t, m in self.clouds if t > self.still_since + 0.3]
        if len(fresh) < FRAMES:
            return
        frame = fresh[-1].header.frame_id
        try:
            Twl = mat(self.buf.lookup_transform("world", "ee_realsense_link", rclpy.time.Time()))
            Tlc = mat(self.buf.lookup_transform("ee_realsense_link", frame, rclpy.time.Time()))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"tf not ready: {e}")
            return
        P = np.concatenate([pc2.read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=True)
                            for m in fresh]).astype(np.float32)
        path = os.path.join(self.out, f"snap_{self.n:02d}.npz")
        np.savez_compressed(path, P=P, T_world_link=Twl, T_link_cloud=Tlc, frame=frame,
                            joint_names=np.array(names), q=qv, wall_time=time.time())
        self.get_logger().info(f"saved {path}: {len(P)} pts, link at {Twl[:3, 3].round(3)}, "
                               f"view dir {(Twl[:3, :3] @ [1, 0, 0]).round(2)}")
        self.n += 1
        self.last_q = qv


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "/out"
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    node = Capture(out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
