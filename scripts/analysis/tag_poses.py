#!/usr/bin/env python3
"""Lens-body poses from AprilTag corners, with ONE tag layout shared by several clips of the same day.

    tag_poses.py OUT_DIR INTRINSICS.json TAG_M NAME=TAGS.pkl [NAME=TAGS.pkl ...]

1. Layout: tag poses in the body frame (= frame of the most-seen tag), from robust pairwise single-tag poses, then
   bundle-adjusted over frames pooled from all clips (clips that mostly see 2 tags borrow the layout of richer clips).
2. Per frame: multi-tag PnP. If all seen tags are coplanar, both IPPE solutions are refined and the one closest to the
   previous frame is kept when their reprojection errors are within 30 % (the two-tag flip seen in T19/T20).
Writes OUT_DIR/layout.npz and OUT_DIR/NAME_campose.npz (frame index, cam_T_body, rms, number of tags, ambiguous flag).
Translations are in units of the assumed tag size TAG_M (the black-square edge = the detected outer corners).
"""
import json
import pickle
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

OUT, INTR, TAG = sys.argv[1], sys.argv[2], float(sys.argv[3])
CLIPS = dict(a.split("=", 1) for a in sys.argv[4:])
IDS_OK = {3, 4, 6, 7, 8, 9, 10}  # lens tags; anything else is a 16h5 false positive

cal = json.load(open(INTR))["models"]["pinhole_k3"]
K, D = np.array(cal["K"]), np.array(cal["D"])
h = TAG / 2
tag_obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)


def load(p):
    r = pickle.load(open(p, "rb"))
    r = r["result"] if "result" in r else r
    return {f: {i: c for i, c in v.items() if i in IDS_OK} for f, v in r.items()}


tags = {n: load(p) for n, p in CLIPS.items()}


def T_from(rv, tv):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(rv, float).reshape(3, 1))[0]
    T[:3, 3] = np.ravel(tv)
    return T


def vec(T):
    return np.r_[cv2.Rodrigues(T[:3, :3])[0].ravel(), T[:3, 3]]


def single(c):
    ok, rv, tv = cv2.solvePnP(tag_obj, c.astype(np.float64), K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return T_from(rv, tv) if ok else None


pool = [(n, f) for n in tags for f in tags[n]]
counts = {}
for n, f in pool:
    for i in tags[n][f]:
        counts[i] = counts.get(i, 0) + 1
ids = sorted(counts)
ref = max(ids, key=lambda i: counts[i])
print("tag counts", counts, "ref", ref)

# 1a. Pairwise init through any already-placed tag (a tag never seen with ref is chained through another).
layout = {ref: np.eye(4)}
for _ in range(3):
    for i in ids:
        if i in layout:
            continue
        rel = []
        for n, f in pool[::3]:
            v = tags[n][f]
            if i not in v:
                continue
            for j in [j for j in v if j in layout][:2]:
                a, b = single(v[j]), single(v[i])
                if a is not None and b is not None:
                    rel.append(layout[j] @ np.linalg.inv(a) @ b)
        if len(rel) < 10:
            continue
        rel = np.array(rel)
        rv = Rot.from_matrix(rel[:, :3, :3])
        cand = np.random.default_rng(0).choice(len(rel), min(300, len(rel)), replace=False)
        votes = [((rv * rv[c].inv()).magnitude() < np.radians(5)).sum() for c in cand]
        mode = rv[cand[int(np.argmax(votes))]]
        good = (rv * mode.inv()).magnitude() < np.radians(5)
        T = np.eye(4)
        T[:3, :3] = Rot.from_matrix(rel[good, :3, :3]).mean().as_matrix()
        T[:3, 3] = np.median(rel[good, :3, 3], axis=0)
        layout[i] = T
        print(f"tag {i}: {len(rel)} pairs, {good.sum()} inliers")
ids = sorted(layout)


def pts_of(lay):
    return {i: (lay[i] @ np.c_[tag_obj, np.ones(4)].T).T[:, :3] for i in lay}


def pnp_frame(v, pts, prev=None):
    """Multi-tag PnP; returns (T, rms, ambiguous)."""
    seen = [i for i in ids if i in v]
    obj = np.concatenate([pts[i] for i in seen])
    img = np.concatenate([v[i] for i in seen]).astype(np.float64)
    cands = []
    ok, rvs, tvs, _ = cv2.solvePnPGeneric(obj, img, K, D, flags=cv2.SOLVEPNP_SQPNP)
    cands += list(zip(rvs, tvs))
    # coplanar? (smallest singular value of the centred object points)
    sv = np.linalg.svd(obj - obj.mean(0), compute_uv=False)
    planar = sv[2] < 1e-3 * sv[0] + 1e-4
    if planar:
        ok, rvs, tvs, _ = cv2.solvePnPGeneric(obj, img, K, D, flags=cv2.SOLVEPNP_IPPE)
        cands += list(zip(rvs, tvs))
    if prev is not None:
        cands.append((cv2.Rodrigues(prev[:3, :3])[0], prev[:3, 3].reshape(3, 1).copy()))
    sols = []
    for rv, tv in cands:
        rv, tv = cv2.solvePnPRefineLM(obj, img, K, D, rv.copy(), tv.copy())
        pr, _ = cv2.projectPoints(obj, rv, tv, K, D)
        e = float(np.sqrt(((pr.reshape(-1, 2) - img) ** 2).sum(1).mean()))
        T = T_from(rv, tv)
        if T[2, 3] > 0:
            sols.append((e, T))
    sols.sort(key=lambda s: s[0])
    # distinct solutions (rotation differs by > 3 deg)
    dist = [sols[0]]
    for s in sols[1:]:
        if all(Rot.from_matrix(s[1][:3, :3] @ d[1][:3, :3].T).magnitude() > np.radians(3) for d in dist):
            dist.append(s)
    amb = len(dist) > 1 and dist[1][0] < 1.3 * dist[0][0]
    if amb and prev is not None:
        best = min(dist[:2], key=lambda s: Rot.from_matrix(s[1][:3, :3] @ prev[:3, :3].T).magnitude())
    else:
        best = dist[0]
    return best[1], best[0], bool(amb)


# 1b. Bundle adjustment of the layout on pooled frames that see >= 3 tags, balanced over clips and tag sets.
pts0 = pts_of(layout)
rich = []
rng = np.random.default_rng(1)
for n in tags:
    fr = [f for f in tags[n] if sum(i in tags[n][f] for i in ids) >= 3]
    # prefer frames with more tags
    fr.sort(key=lambda f: -len(tags[n][f]))
    take = fr[:60] + list(rng.choice(fr, min(60, len(fr)), replace=False)) if fr else []
    rich += [(n, f) for f in sorted(set(take))]
print("BA frames per clip", {n: sum(r[0] == n for r in rich) for n in tags})
poses0 = [pnp_frame(tags[n][f], pts0)[0] for n, f in rich]
others = [i for i in ids if i != ref]


# Alternating BA (frame poses | tag poses): each sub-problem is small and well scaled.
def frame_res(n, f, P, lay):
    v = tags[n][f]
    seen = [i for i in ids if i in v]
    obj = np.concatenate([(lay[i] @ np.c_[tag_obj, np.ones(4)].T).T[:, :3] for i in seen])
    pr, _ = cv2.projectPoints(obj, cv2.Rodrigues(P[:3, :3])[0], P[:3, 3], K, D)
    return (pr.reshape(-1, 2) - np.concatenate([v[i] for i in seen])).ravel()


def total_rms(lay, poses):
    return np.sqrt(np.mean(np.concatenate([frame_res(n, f, P, lay) for (n, f), P in zip(rich, poses)]) ** 2))


poses = poses0
rms0 = total_rms(layout, poses)
for it in range(15):
    pts_it = pts_of(layout)
    poses = [pnp_frame(tags[n][f], pts_it, P)[0] for (n, f), P in zip(rich, poses)]
    for i in others:
        fr = [(k, n, f) for k, (n, f) in enumerate(rich) if i in tags[n][f]]
        if len(fr) < 5:
            continue

        def tres(z, fr=fr, i=i):
            L = T_from(z[:3], z[3:])
            obj = (L @ np.c_[tag_obj, np.ones(4)].T).T[:, :3]
            r = []
            for k, n, f in fr:
                P = poses[k]
                pr, _ = cv2.projectPoints(obj, cv2.Rodrigues(P[:3, :3])[0], P[:3, 3], K, D)
                r.append((pr.reshape(-1, 2) - tags[n][f][i]).ravel())
            return np.concatenate(r)
        z = least_squares(tres, vec(layout[i]), loss="huber", f_scale=1.0).x
        layout[i] = T_from(z[:3], z[3:])
    r = total_rms(layout, poses)
    if it % 3 == 0 or it == 14:
        print(f"  BA iter {it}: rms {r:.3f} px")
print(f"layout BA on {len(rich)} frames: rms px {rms0:.3f} -> {total_rms(layout, poses):.3f}")
per_tag = {}
for (n, f), P in zip(rich, poses):
    v = tags[n][f]
    rr = frame_res(n, f, P, layout).reshape(-1, 8)
    for i, row in zip([i for i in ids if i in v], rr):
        per_tag.setdefault(i, []).append(np.sqrt(np.mean(row ** 2)))
print("per-tag BA rms px", {i: round(float(np.median(v)), 2) for i, v in sorted(per_tag.items())})

pts = pts_of(layout)
cen = {i: layout[i][:3, 3] for i in ids}
print(f"tag centres in body frame (mm, at {1000 * TAG:.2f} mm tags):")
for i in ids:
    print(f"  {i}: {np.round(1000 * cen[i], 1)}  normal {np.round(layout[i][:3, 2], 3)}")
print("centre-to-centre distances (mm):")
for a in ids:
    print("  ", a, " ".join(f"{b}:{1000 * np.linalg.norm(cen[a] - cen[b]):6.1f}" for b in ids if b > a))
np.savez(f"{OUT}/layout.npz", ids=ids, ref=ref, tag=TAG, T=np.array([layout[i] for i in ids]))

# 2. Per-frame poses with temporal continuity.
for n in tags:
    rows = []
    prev, prev_f = None, -10 ** 9
    for f in sorted(tags[n]):
        v = tags[n][f]
        if not any(i in v for i in ids):
            continue
        p = prev if f - prev_f < 60 else None  # continuity only within ~1 s
        T, e, amb = pnp_frame(v, pts, p)
        rows.append((f, T, e, len([i for i in ids if i in v]), amb))
        if e < 2.0:
            prev, prev_f = T, f
    fr = np.array([r[0] for r in rows])
    np.savez(f"{OUT}/{n}_campose.npz", frame=fr, T=np.array([r[1] for r in rows]), rms=np.array([r[2] for r in rows]),
             ntags=np.array([r[3] for r in rows]), amb=np.array([r[4] for r in rows]), tag=TAG)
    rms = np.array([r[2] for r in rows])
    nt = np.array([r[3] for r in rows])
    print(f"{n}: {len(rows)} poses, rms median {np.median(rms):.2f} px, ambiguous {np.mean([r[4] for r in rows]):.1%}, "
          f"tags/frame {np.bincount(nt)}")
