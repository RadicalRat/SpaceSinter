"""Forward kinematics from bagged /tf: focal-point pose over time, saved alongside the topic dump."""
import pickle
import sys

import numpy as np

d = pickle.load(open(sys.argv[1], "rb"))


def mat(p, q):
    x, y, z, w = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


static = {c: (p, mat(pos, q)) for _, _, p, c, pos, q in d["/tf_static"]}
chain = ["j2n6s300_link_1", "j2n6s300_link_2", "j2n6s300_link_3", "j2n6s300_link_4", "j2n6s300_link_5",
         "j2n6s300_link_6"]
# Group dynamic arm transforms by header stamp; the publisher sends all six together.
by_stamp = {}
for t, st, p, c, pos, q in d["/tf"]:
    if c in chain:
        by_stamp.setdefault(st, {})[c] = mat(pos, q)

T_world_base = static["j2n6s300_link_base"][1]
T_6_ee = static["ee_end_effector_link"][1]
T_ee_focal = static["ee_focal_point_link"][1]
T_ee_cam = static["ee_camera_link"][1] @ static["ee_camera_optical_link"][1]

times, focal, cam = [], [], []
for st in sorted(by_stamp):
    links = by_stamp[st]
    if len(links) != 6:
        continue
    T = T_world_base
    for c in chain:
        T = T @ links[c]
    T = T @ T_6_ee
    times.append(st)
    focal.append(T @ T_ee_focal)
    cam.append(T @ T_ee_cam)

np.savez(sys.argv[2], t=np.array(times), focal=np.array(focal), cam=np.array(cam))
print(len(times), "FK samples")
print("T_ee_focal\n", T_ee_focal.round(4), "\nT_ee_cam_optical\n", T_ee_cam.round(4))
