"""GoPro AprilTag lens pose vs FK for rasters with little rotation (e.g. Trial 11): travel scale and residuals.

    pose_travel.py TAGS.pkl INTRINSICS.json FK.npz OUT.npz --frame0 UNIX_S --fps 59.94 --tag 0.03175

pose.py's hand-eye fit needs rotations of several degrees, which a 4 mm-step raster doesn't have. Here the tag-body
origin measured by the camera is compared with FK through a similarity transform:

    p_cam(t) = s * R @ (p_focal(t + dt) + R_focal(t + dt) @ y) + c

where R, c place the world in the camera, y is the tag-body origin in the focal frame, dt the clock offset and s the
scale. With the true tag size, s = (distance the camera sees the lens move) / (distance FK says it moved), so s < 1
means the arm moves less than its model. Rotations are compared as small rotation vectors relative to the first frame.
"""
import argparse
import json
import pickle

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot, Slerp

ap = argparse.ArgumentParser()
ap.add_argument("tags"); ap.add_argument("intrinsics"); ap.add_argument("fk"); ap.add_argument("out")
ap.add_argument("--frame0", type=float, required=True, help="ROS/UNIX time of video frame 0 (coarse)")
ap.add_argument("--fps", type=float, default=60000 / 1001)
ap.add_argument("--tag", type=float, default=0.03175, help="AprilTag black-square edge, m")
ap.add_argument("--dt-range", type=float, default=5.0, help="search clock offset within +- this, s")
ap.add_argument("--focal-in-body", type=str, default="0,0,0",
                help="focal point in the tag-body frame, m (x,y,z), e.g. from a hand-eye fit; matters only via rotation")
a = ap.parse_args()

cal = json.load(open(a.intrinsics))["models"]["pinhole_k3"]
K, Dist = np.array(cal["K"]), np.array(cal["D"])
r = pickle.load(open(a.tags, "rb"))
tags = r["result"] if "result" in r else r
frames = sorted(tags)
counts = {}
for f in frames:
    for i in tags[f]:
        counts[i] = counts.get(i, 0) + 1
ids = sorted(i for i, c in counts.items() if c > 0.2 * len(frames))
print("tag counts", counts, "-> using", ids)
h = a.tag / 2
tag_obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)


def T_from(rv, tv):
    T = np.eye(4); T[:3, :3] = cv2.Rodrigues(np.asarray(rv, float))[0]; T[:3, 3] = np.ravel(tv); return T


def single(c):
    ok, rv, tv = cv2.solvePnP(tag_obj, c.astype(np.float64), K, Dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return T_from(rv, tv) if ok else None


# Tag layout relative to the most-seen tag (mode of pairwise poses), then bundle adjustment.
ref = max(ids, key=lambda i: counts[i])
layout = {ref: np.eye(4)}
for i in ids:
    if i == ref:
        continue
    rel = [np.linalg.inv(single(tags[f][ref])) @ single(tags[f][i]) for f in frames[::3] if ref in tags[f] and i in tags[f]]
    if len(rel) < 10:
        print(f"tag {i}: {len(rel)} shared frames, dropped"); continue
    rel = np.array(rel); rv = Rot.from_matrix(rel[:, :3, :3])
    cand = np.random.default_rng(0).choice(len(rel), min(200, len(rel)), replace=False)
    mode = rv[cand[int(np.argmax([((rv * rv[c].inv()).magnitude() < np.radians(5)).sum() for c in cand]))]]
    good = (rv * mode.inv()).magnitude() < np.radians(5)
    T = np.eye(4); T[:3, :3] = Rot.from_matrix(rel[good, :3, :3]).mean().as_matrix(); T[:3, 3] = np.median(rel[good, :3, 3], 0)
    layout[i] = T
ids = [i for i in ids if i in layout]
others = [i for i in ids if i != ref]
obj_pts = lambda lay: {i: (lay[i] @ np.c_[tag_obj, np.ones(4)].T).T[:, :3] for i in ids}


def pnp(f, pts):
    seen = [i for i in ids if i in tags[f]]
    obj = np.concatenate([pts[i] for i in seen]); img = np.concatenate([tags[f][i] for i in seen]).astype(np.float64)
    ok, rv, tv = cv2.solvePnP(obj, img, K, Dist, flags=cv2.SOLVEPNP_SQPNP)
    ok, rv, tv = cv2.solvePnP(obj, img, K, Dist, rv, tv, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    pr, _ = cv2.projectPoints(obj, rv, tv, K, Dist)
    return T_from(rv, tv), float(np.sqrt(np.mean(np.sum((pr.reshape(-1, 2) - img) ** 2, 1)))), len(seen)


rich = [f for f in frames if sum(i in tags[f] for i in ids) >= len(ids) - 1][::25][:150]
pts0 = obj_pts(layout)
P0 = [pnp(f, pts0)[0] for f in rich]


def unpack(x):
    lay = {ref: np.eye(4)}
    for k, i in enumerate(others):
        lay[i] = T_from(x[6 * k:6 * k + 3], x[6 * k + 3:6 * k + 6])
    o = 6 * len(others)
    return lay, [T_from(x[o + 6 * k:o + 6 * k + 3], x[o + 6 * k + 3:o + 6 * k + 6]) for k in range(len(rich))]


def ba_res(x):
    lay, poses = unpack(x); pts = obj_pts(lay); res = []
    for f, P in zip(rich, poses):
        for i in ids:
            if i in tags[f]:
                pr, _ = cv2.projectPoints(pts[i], cv2.Rodrigues(P[:3, :3])[0], P[:3, 3], K, Dist)
                res.append((pr.reshape(-1, 2) - tags[f][i]).ravel())
    return np.concatenate(res)


x0 = np.concatenate([np.r_[cv2.Rodrigues(layout[i][:3, :3])[0].ravel(), layout[i][:3, 3]] for i in others] +
                    [np.r_[cv2.Rodrigues(P[:3, :3])[0].ravel(), P[:3, 3]] for P in P0])
ba = least_squares(ba_res, x0, loss="huber", f_scale=1.0)
layout, _ = unpack(ba.x)
print(f"layout BA on {len(rich)} frames: rms {np.sqrt(np.mean(ba_res(x0) ** 2)):.2f} -> {np.sqrt(np.mean(ba.fun ** 2)):.2f} px")
pts = obj_pts(layout)

M, rms, tv = [], [], []
for f in frames:
    if sum(i in tags[f] for i in ids) < 3:
        continue
    T, e, n = pnp(f, pts)
    M.append(T); rms.append(e); tv.append(f / a.fps)
M, rms, tv = np.array(M), np.array(rms), np.array(tv)
keep = rms < 2.0
M, rms, tv = M[keep], rms[keep], tv[keep]
# Drop flipped / bad poses: rotation more than 2 deg from the median orientation.
Rall = Rot.from_matrix(M[:, :3, :3]); rmed = Rall.mean()
for _ in range(3):
    ok = (Rall * rmed.inv()).magnitude() < np.radians(2)
    rmed = Rall[ok].mean()
print(f"{keep.sum()} frames with >= 3 tags and rms < 2 px; {ok.sum()} within 2 deg of the median orientation "
      f"(reprojection rms median {np.median(rms):.2f} px)")
M, rms, tv = M[ok], rms[ok], tv[ok]
print(f"camera to tags ~{np.median(np.linalg.norm(M[:, :3, 3], axis=1)):.2f} m")

fk = np.load(a.fk); t_fk, F = fk["t"], fk["focal"]
sl = Slerp(t_fk, Rot.from_matrix(F[:, :3, :3]))


def fk_at(t):
    t = np.clip(t, t_fk[0], t_fk[-1]); T = np.tile(np.eye(4), (len(t), 1, 1))
    T[:, :3, :3] = sl(t).as_matrix()
    for k in range(3):
        T[:, k, 3] = np.interp(t, t_fk, F[:, k, 3])
    return T


yb = np.array([float(v) for v in a.focal_in_body.split(",")])
p_cam = M[:, :3, 3] + np.einsum("nij,j->ni", M[:, :3, :3], yb)  # the focal point as the camera sees it


def sim_res(q, dt, sel):
    s, R, c = q[0], Rot.from_rotvec(q[1:4]).as_matrix(), q[4:7]
    pf = fk_at(a.frame0 + dt + tv[sel])[:, :3, 3]
    return 1000 * (p_cam[sel] - (s * pf @ R.T + c)).ravel()


def init(dt, sel):
    A, B = fk_at(a.frame0 + dt + tv[sel])[:, :3, 3], p_cam[sel]
    am, bm = A.mean(0), B.mean(0); U, _, Vt = np.linalg.svd((B - bm).T @ (A - am))
    R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    s = np.sqrt(np.sum((B - bm) ** 2) / np.sum((A - am) ** 2))
    return np.r_[s, Rot.from_matrix(R).as_rotvec(), bm - s * R @ am]


def fit(dt, sel):
    q = least_squares(sim_res, init(dt, sel), args=(dt, sel), loss="soft_l1", f_scale=1.0).x
    r = sim_res(q, dt, sel).reshape(-1, 3)
    return q, float(np.sqrt(np.mean(np.minimum(np.sum(r ** 2, 1), 25.0))))  # truncated rms, mm


inside = (a.frame0 + tv > t_fk[0] + 2) & (a.frame0 + tv < t_fk[-1] - 2)
sub = np.where(inside)[0][::3]
curve = [(fit(d, sub)[1], d) for d in np.arange(-a.dt_range, a.dt_range + 1e-9, 0.25)]
d0 = min(curve)[1]
best, dt = min((fit(d, sub)[1], d) for d in np.arange(d0 - 0.3, d0 + 0.3, 1 / 30))
print("dt curve (truncated rms mm):", " ".join(f"{d:+.2f}:{c:.2f}" for c, d in curve))
idx = np.where(inside)[0]
q, _ = fit(dt, idx)
s, R, c = q[0], Rot.from_rotvec(q[1:4]).as_matrix(), q[4:7]
res = sim_res(q, dt, idx).reshape(-1, 3) / 1000
# Bootstrap over 10 s blocks for the scale uncertainty
blocks = np.floor((tv[idx] - tv[idx][0]) / 10).astype(int); rng = np.random.default_rng(1); ss = []
for _ in range(40):
    pick = rng.choice(np.unique(blocks), len(np.unique(blocks)))
    sel = idx[np.concatenate([np.where(blocks == b)[0] for b in pick])]
    ss.append(least_squares(sim_res, q, args=(dt, sel), loss="soft_l1", f_scale=1.0).x[0])
print(f"\nclock offset dt {dt:+.3f} s -> frame0 ROS time {a.frame0 + dt:.3f}")
print(f"TRAVEL SCALE s = {s:.4f} +- {np.std(ss):.4f} (camera-seen / FK travel, tag {1000 * a.tag:.2f} mm, focal-in-body {yb})")
pf = fk_at(a.frame0 + dt + tv[idx])[:, :3, 3]
print("FK travel in the clip (mm): x %.1f y %.1f z %.1f" % tuple(1000 * np.ptp(pf, 0)))
rn = 1000 * np.linalg.norm(res, axis=1)
print(f"position residual: median {np.median(rn):.2f} mm, p95 {np.percentile(rn, 95):.2f} mm")
res_w = res @ R / s
print("residual rms per world axis (mm): x %.2f y %.2f z %.2f" % tuple(1000 * np.sqrt(np.mean(res_w ** 2, 0))))
W = fk_at(a.frame0 + dt + tv[idx])
Rc = Rot.from_matrix(M[idx, :3, :3]); Rf = Rot.from_matrix(W[:, :3, :3])
rc = (Rc * Rc.mean().inv()).as_rotvec(); rf = (Rf * Rf.mean().inv()).as_rotvec() @ R.T
dr = np.degrees(np.linalg.norm(rc - rf, axis=1))
print(f"rotation about the clip mean: camera range {np.degrees(np.ptp(np.linalg.norm(rc, axis=1))):.2f} deg, FK "
      f"{np.degrees(np.ptp(np.linalg.norm(rf, axis=1))):.2f} deg; camera-vs-FK difference median {np.median(dr):.3f}, "
      f"p95 {np.percentile(dr, 95):.3f} deg")
np.savez(a.out, t=a.frame0 + dt + tv[idx], p_cam=p_cam[idx], p_fk=pf, res=res, res_world=res_w, s=s, s_boot=ss, R=R,
         c=c, dt=dt, rot_cam=rc, rot_fk=rf, rms=rms[idx], fk_focal=W)
