"""GoPro lens-pose estimation from AprilTags, time sync and hand-eye fit against FK.

Model: cam_T_body(t) = X @ world_T_focal(t + dt) @ Y, where X = cam_T_world (fixed GoPro) and
Y = focal_T_body (tags rigid on the lens frame). Fitting X, Y and dt absorbs constant offsets, so the
residuals show only configuration- or time-dependent disagreement between the camera and FK.
"""
import json
import pickle
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

S = sys.argv[1]
TAG = 0.030
FPS = 30000 / 1001
COARSE_FRAME0 = 1793390134 + 20 / FPS  # GoPro timecode 13:55:34;20 MDT

cal = json.load(open(f"{S}/gopro_intrinsics.json"))["models"]["pinhole_k3"]
K, Dist = np.array(cal["K"]), np.array(cal["D"])

ch1 = pickle.load(open(f"{S}/tags_ch1.pkl", "rb"))
tags = dict(ch1["result"]) if "result" in ch1 else dict(ch1)  # partial checkpoints are the bare dict
if len(sys.argv) < 3 or sys.argv[2] != "ch1":
    ch2 = pickle.load(open(f"{S}/tags_ch2.pkl", "rb"))
    tags.update({k + ch1["n_frames"]: v for k, v in ch2["result"].items()})
frames = sorted(tags)

# Keep tag ids that show up often; rare ids are 16h5 false positives.
counts = {}
for f in frames:
    for i in tags[f]:
        counts[i] = counts.get(i, 0) + 1
ids = sorted(i for i, c in counts.items() if c > 0.2 * len(frames))
print("tag counts", counts, "-> using", ids)

h = TAG / 2
tag_obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)


def T_from(rvec, tvec):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(rvec, float))[0]
    T[:3, 3] = np.ravel(tvec)
    return T


def single_tag(c):
    ok, rv, tv = cv2.solvePnP(tag_obj, c.astype(np.float64), K, Dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return T_from(rv, tv) if ok else None


# 1. Tag layout in the body frame (= reference tag), from robust averages of pairwise relative poses,
#    then bundle-adjusted over a subset of frames.
ref = max(ids, key=lambda i: counts[i])
layout = {ref: np.eye(4)}
for i in ids:
    if i == ref:
        continue
    rel = []
    for f in frames[::5]:
        if ref in tags[f] and i in tags[f]:
            a, b = single_tag(tags[f][ref]), single_tag(tags[f][i])
            if a is not None and b is not None:
                rel.append(np.linalg.inv(a) @ b)
    rel = np.array(rel)
    if len(rel) < 10:
        print(f"tag {i}: only {len(rel)} frames shared with ref tag {ref}, dropped")
        continue
    rv = Rot.from_matrix(rel[:, :3, :3])
    # Single-tag IPPE poses flip, so vote: the candidate with most neighbours within 5 deg is the mode.
    cand = np.random.default_rng(0).choice(len(rel), min(200, len(rel)), replace=False)
    votes = [((rv * rv[c].inv()).magnitude() < np.radians(5)).sum() for c in cand]
    mode = rv[cand[int(np.argmax(votes))]]
    good = (rv * mode.inv()).magnitude() < np.radians(5)
    T = np.eye(4)
    T[:3, :3] = Rot.from_matrix(rel[good, :3, :3]).mean().as_matrix()
    T[:3, 3] = np.median(rel[good, :3, 3], axis=0)
    layout[i] = T
    print(f"tag {i}: {len(rel)} pairs, {good.sum()} inliers, pos in ref frame (mm) {1000 * T[:3, 3].round(4)}")


def body_points(lay):
    global ids
    ids = [i for i in ids if i in lay]
    return {i: (lay[i] @ np.c_[tag_obj, np.ones(4)].T).T[:, :3] for i in ids}


def multi_pnp(f, pts, guess=None):
    obj = np.concatenate([pts[i] for i in ids if i in tags[f]])
    img = np.concatenate([tags[f][i] for i in ids if i in tags[f]]).astype(np.float64)
    if guess is None:
        ok, rv, tv = cv2.solvePnP(obj, img, K, Dist, flags=cv2.SOLVEPNP_SQPNP)
    else:
        rv, tv = cv2.Rodrigues(guess[:3, :3])[0], guess[:3, 3].copy()
        ok, rv, tv = cv2.solvePnP(obj, img, K, Dist, rv, tv, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    proj, _ = cv2.projectPoints(obj, rv, tv, K, Dist)
    err = np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)
    return T_from(rv, tv), float(np.sqrt((err ** 2).mean()))


# Bundle adjustment of the layout on frames that see most tags.
rich = [f for f in frames if sum(i in tags[f] for i in ids) >= len(ids) - 1][::20][:150]
pts0 = body_points(layout)
poses0 = [multi_pnp(f, pts0)[0] for f in rich]
others = [i for i in ids if i != ref]


def unpack(x):
    lay = {ref: np.eye(4)}
    for k, i in enumerate(others):
        lay[i] = T_from(x[6 * k:6 * k + 3], x[6 * k + 3:6 * k + 6])
    off = 6 * len(others)
    poses = [T_from(x[off + 6 * k:off + 6 * k + 3], x[off + 6 * k + 3:off + 6 * k + 6]) for k in range(len(rich))]
    return lay, poses


def ba_res(x):
    lay, poses = unpack(x)
    pts = body_points(lay)
    r = []
    for f, P in zip(rich, poses):
        for i in ids:
            if i in tags[f]:
                proj, _ = cv2.projectPoints(pts[i], cv2.Rodrigues(P[:3, :3])[0], P[:3, 3], K, Dist)
                r.append((proj.reshape(-1, 2) - tags[f][i]).ravel())
    return np.concatenate(r)


x0 = np.concatenate([np.r_[cv2.Rodrigues(layout[i][:3, :3])[0].ravel(), layout[i][:3, 3]] for i in others] +
                    [np.r_[cv2.Rodrigues(P[:3, :3])[0].ravel(), P[:3, 3]] for P in poses0])
ba = least_squares(ba_res, x0, loss="huber", f_scale=1.0)
layout, _ = unpack(ba.x)
print(f"layout BA on {len(rich)} frames: rms px {np.sqrt(np.mean(ba_res(x0) ** 2)):.2f} -> {np.sqrt(np.mean(ba.fun ** 2)):.2f}")
pts = body_points(layout)

# 2. Per-frame lens-body pose.
cam_T_body, rms, tf = [], [], []
for f in frames:
    if sum(i in tags[f] for i in ids) < 3:
        continue
    T, e = multi_pnp(f, pts)
    cam_T_body.append(T)
    rms.append(e)
    tf.append(f / FPS)
cam_T_body, rms, tf = np.array(cam_T_body), np.array(rms), np.array(tf)
keep = rms < 2.0
print(f"{len(tf)} frames with >=3 tags; reprojection rms median {np.median(rms):.2f} px; kept {keep.sum()}")
cam_T_body, tf, rms = cam_T_body[keep], tf[keep], rms[keep]

# 3. FK interpolation.
fk = np.load(f"{S}/t2_fk.npz")
t_fk, F = fk["t"], fk["focal"]
rot_fk = Rot.from_matrix(F[:, :3, :3])
from scipy.spatial.transform import Slerp  # noqa: E402

slerp = Slerp(t_fk, rot_fk)


def fk_at(t):
    t = np.clip(t, t_fk[0], t_fk[-1])
    T = np.tile(np.eye(4), (len(t), 1, 1))
    T[:, :3, :3] = slerp(t).as_matrix()
    for k in range(3):
        T[:, k, 3] = np.interp(t, t_fk, F[:, k, 3])
    return T


def params_to_XY(p):
    return T_from(p[0:3], p[3:6]), T_from(p[6:9], p[9:12])


def scaled(T, k):
    """Tag-size scale: PnP translations (and the layout) scale with the assumed tag size."""
    T = T.copy()
    T[..., :3, 3] *= k
    return T


def residuals(p, dt, sel, w_t=1.0):
    X, Y = params_to_XY(p)
    k = p[12] if len(p) > 12 else 1.0
    pred = X @ fk_at(COARSE_FRAME0 + dt + tf[sel]) @ scaled(Y, k)
    meas = scaled(cam_T_body[sel], k)
    dR = Rot.from_matrix(np.einsum("nji,njk->nik", pred[:, :3, :3], meas[:, :3, :3])).as_rotvec()
    dp = meas[:, :3, 3] - pred[:, :3, 3]
    return np.concatenate([np.degrees(dR).ravel(), w_t * 1000 * dp.ravel() / 10])  # deg and cm-ish weighting


def init_XY(dt, sel):
    """Park-Martin AX = XB on pose pairs (A in camera, B in world), then Y by averaging."""
    W = fk_at(COARSE_FRAME0 + dt + tf[sel])
    M = cam_T_body[sel]
    rng = np.random.default_rng(0)
    i, j = rng.integers(0, len(sel), (2, 4000))
    A = M[i] @ np.linalg.inv(M[j])
    B = W[i] @ np.linalg.inv(W[j])
    a, b = Rot.from_matrix(A[:, :3, :3]).as_rotvec(), Rot.from_matrix(B[:, :3, :3]).as_rotvec()
    big = (np.linalg.norm(b, axis=1) > np.radians(2)) & (np.linalg.norm(a, axis=1) > np.radians(2))
    # Kabsch: a = R b for the rotation axes.
    U, _, Vt = np.linalg.svd(a[big].T @ b[big])
    R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    lhs = (A[big, :3, :3] - np.eye(3)).reshape(-1, 3)
    rhs = (np.einsum("ij,nj->ni", R, B[big, :3, 3]) - A[big, :3, 3]).ravel()
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = R, np.linalg.lstsq(lhs, rhs, rcond=None)[0]
    Ys = np.linalg.inv(W) @ np.linalg.inv(X) @ M
    Y = np.eye(4)
    Y[:3, :3] = Rot.from_matrix(Ys[:, :3, :3]).mean().as_matrix()
    Y[:3, 3] = np.median(Ys[:, :3, 3], axis=0)
    return np.r_[cv2.Rodrigues(X[:3, :3])[0].ravel(), X[:3, 3], cv2.Rodrigues(Y[:3, :3])[0].ravel(), Y[:3, 3]]


# Restrict to the time FK covers, sub-sample for the dt search.
in_fk = (COARSE_FRAME0 + tf > t_fk[0] + 15) & (COARSE_FRAME0 + tf < t_fk[-1] - 15)
idx = np.where(in_fk)[0]
sub = idx[:: max(1, len(idx) // 1500)]


def cost(dt):
    p = least_squares(residuals, init_XY(dt, sub), args=(dt, sub), loss="soft_l1").x
    r = residuals(p, dt, sub).reshape(2, -1, 3)
    return np.median(np.linalg.norm(r[0], axis=1)), p


import os  # noqa: E402

if os.environ.get("FIX_DT"):
    # Pose residuals are dominated by real, non-rigid disagreement, so the dt search can be flat;
    # pass the offset found from correction-move timing instead.
    dt = float(os.environ["FIX_DT"])
    best_cost = cost(dt)[0]
    coarse = []
else:
    grid = np.arange(-12, 4, 0.5)
    coarse = [(cost(dt)[0], dt) for dt in grid]
    dt0 = min(coarse)[1]
    fine = [(cost(dt)[0], dt) for dt in np.arange(dt0 - 0.6, dt0 + 0.6, 1 / FPS)]
    best_cost, dt = min(fine)
print("dt coarse curve:", " ".join(f"{d:+.1f}:{c:.3f}" for c, d in coarse))
print(f"best dt {dt:+.3f} s (median rot residual {best_cost:.3f} deg); frame0 ROS time {COARSE_FRAME0 + dt:.3f}")

p = least_squares(residuals, np.r_[init_XY(dt, idx), 1.0], args=(dt, idx), loss="soft_l1").x
scale = p[12]
print(f"fitted tag scale {scale:.4f} -> effective tag size {1000 * TAG * scale:.2f} mm")
X, Y = params_to_XY(p)
Y = scaled(Y, scale)
cam_T_body = scaled(cam_T_body, scale)
pred = X @ fk_at(COARSE_FRAME0 + dt + tf[idx]) @ Y
meas = cam_T_body[idx]
# Residuals expressed in the world frame: camera-measured focal pose vs FK.
world_meas = np.linalg.inv(X) @ meas @ np.linalg.inv(Y)
world_fk = fk_at(COARSE_FRAME0 + dt + tf[idx])
dR = Rot.from_matrix(np.einsum("nij,nkj->nik", world_meas[:, :3, :3], world_fk[:, :3, :3])).as_rotvec()
dP = world_meas[:, :3, 3] - world_fk[:, :3, 3]
ang = np.degrees(np.linalg.norm(dR, axis=1))
axis_err = np.degrees(np.arccos(np.clip(np.einsum("ni,ni->n", world_meas[:, :3, 2], world_fk[:, :3, 2]), -1, 1)))
pos = 1000 * np.linalg.norm(dP, axis=1)
print(f"residual rotation (deg): median {np.median(ang):.3f} p95 {np.percentile(ang, 95):.3f}")
print(f"residual lens-axis direction (deg): median {np.median(axis_err):.3f} p95 {np.percentile(axis_err, 95):.3f}")
print(f"residual focal-point position (mm): median {np.median(pos):.2f} p95 {np.percentile(pos, 95):.2f}")
print("X cam_T_world:\n", X.round(4), "\nY focal_T_body:\n", Y.round(4))
np.savez(f"{S}/t2_pose.npz", t=COARSE_FRAME0 + dt + tf[idx], world_meas=world_meas, world_fk=world_fk,
         rms=rms[idx], X=X, Y=Y, dt=dt, scale=scale, frame0=COARSE_FRAME0 + dt)
