#!/usr/bin/env python3
"""Diagnostics of GoPro lens pose (tag_poses.py) against FK: time sync, jitter, hand-eye conditioning, travel scale.

    gopro_diag.py CAMPOSE.npz FK.npz TRIAL.pkl OUT_PREFIX --frame0 ROS_T0 [--fps 59.94] [--dt-range 15]

All checks that can be done without a hand-eye calibration are done first, because they are invariant to it:
  * time sync: cross-correlation of rotation speed |w| (invariant to the hand-eye AND to the tag size) and of
    translation speed (invariant to the hand-eye, scaled by the tag size) between camera and FK;
  * rotation-angle ratio: angle(A_ij) = angle(B_ij) for any hand-eye (A = X B X^-1), so camera vs FK relative
    rotation angles test the camera's rotation accuracy and the joint encoders without fitting anything;
  * jitter on static windows (FK still), split into camera x, y (lateral) and z (depth).
Then the hand-eye + scale model, with the lever arm handled properly:
    R_cam_body(t) = Rx R_f(t) Ry,   p_cam_body(t) = Rx k (p_f(t) + R_f(t) y) + c
  k = camera units per FK metre (the travel ratio at the assumed tag size), y = body origin in the focal frame (FK m).
  Its conditioning (parameter standard errors from the Jacobian) is printed, and k is also fitted on motion subsets.
"""
import argparse
import json
import pickle

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402
from scipy.spatial.transform import Rotation as Rot, Slerp  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("campose"); ap.add_argument("fk"); ap.add_argument("trial"); ap.add_argument("out")
ap.add_argument("--frame0", type=float, required=True)
ap.add_argument("--fps", type=float, default=60000 / 1001)
ap.add_argument("--dt-range", type=float, default=300.0)
ap.add_argument("--max-rms", type=float, default=1.5)
ap.add_argument("--min-tags", type=int, default=2)
ap.add_argument("--t-max", type=float, default=1e9, help="ignore video after this time (s), e.g. a camera bump")
ap.add_argument("--t-min", type=float, default=-1.0, help="ignore video before this time (s)")
ap.add_argument("--fix-dt", type=float, default=None, help="use this clock offset (s) instead of the speed correlation")
a = ap.parse_args()
D = np.degrees
res = {}

cp = np.load(a.campose)
good = (cp["rms"] < a.max_rms) & (cp["ntags"] >= a.min_tags)
tv = cp["frame"][good] / a.fps
good_t = (tv < a.t_max) & (tv > a.t_min)
tv, Tc, nt, amb = tv[good_t], cp["T"][good][good_t], cp["ntags"][good][good_t], cp["amb"][good][good_t]
Rc, pc = Rot.from_matrix(Tc[:, :3, :3]), Tc[:, :3, 3]
fk = np.load(a.fk)
tf, F = fk["t"], fk["focal"]
slerp = Slerp(tf, Rot.from_matrix(F[:, :3, :3]))
d = pickle.load(open(a.trial, "rb"))
P = np.array(d["/sinter/track_progress"])
scan = (P[np.argmax(P[:, 1] > 0), 0], P[np.argmax(P[:, 1] >= P[:, 1].max() - 1e-9), 0])


def fk_at(t):
    t = np.clip(t, tf[0], tf[-1])
    return slerp(t), np.c_[[np.interp(t, tf, F[:, k, 3]) for k in range(3)]].T


inside = lambda t: (t > tf[0] + 1) & (t < tf[-1] - 1)  # noqa: E731

# ---------------------------------------------------------------- 1. time sync from speeds
G = 0.1  # s grid
tg = np.arange(tv[0] + 1, tv[-1] - 1, G)
# camera signals on the grid, only where a measurement is within 0.25 s
near = np.abs(np.interp(tg, tv, tv) - tg) < 0.25
idx = np.clip(np.searchsorted(tv, tg), 1, len(tv) - 1)
gap_ok = (tv[idx] - tv[idx - 1]) < 0.35
H = 0.5  # half-window for speeds


def cam_speed(t):
    i0 = np.clip(np.searchsorted(tv, t - H), 0, len(tv) - 1)
    i1 = np.clip(np.searchsorted(tv, t + H), 0, len(tv) - 1)
    dtt = tv[i1] - tv[i0]
    v = np.linalg.norm(pc[i1] - pc[i0], axis=1) / np.maximum(dtt, 1e-3)
    w = D((Rc[i1] * Rc[i0].inv()).magnitude()) / np.maximum(dtt, 1e-3)
    ok = (dtt > 1.5 * H) & (dtt < 2.6 * H)
    return v, w, ok


def fk_speed(t):
    r0, p0 = fk_at(t - H)
    r1, p1 = fk_at(t + H)
    return np.linalg.norm(p1 - p0, axis=1) / (2 * H), D((r1 * r0.inv()).magnitude()) / (2 * H)


vc, wc, okc = cam_speed(tg)
okc &= near & gap_ok
# FK speeds precomputed on a fine grid, then interpolated per lag (lags up to minutes: the GoPro clock is not synced)
tfg = np.arange(tf[0] + H, tf[-1] - H, 0.02)
vfg, wfg = fk_speed(tfg)
lags = np.arange(-a.dt_range, a.dt_range, 0.02)
cv, cw = [], []
for L in lags:
    t = a.frame0 + tg + L
    m = okc & inside(t)
    if m.sum() < 100:
        cv.append(np.nan); cw.append(np.nan)
        continue
    vf, wf = np.interp(t[m], tfg, vfg), np.interp(t[m], tfg, wfg)
    cv.append(np.corrcoef(vc[m], vf)[0, 1])
    cw.append(np.corrcoef(np.minimum(wc[m], 20), np.minimum(wf, 20))[0, 1])
cv, cw = np.array(cv), np.array(cw)
dt_v, dt_w = lags[np.nanargmax(cv)], lags[np.nanargmax(cw)]
# half-width of the correlation peak (s) as a sharpness measure
hw = lambda c, i: float(0.01 * np.sum(np.nan_to_num(c, nan=-1) > c[i] - 0.05))  # half-width (s) of the region within 0.05 of the peak  # noqa: E731
res["sync"] = dict(dt_translation_speed=float(dt_v), r_v=float(np.nanmax(cv)), dt_rotation_speed=float(dt_w),
                   r_w=float(np.nanmax(cw)), peak_halfwidth_v=hw(cv, np.nanargmax(cv)), peak_halfwidth_w=hw(cw, np.nanargmax(cw)),
                   second_peak_v=float(np.nanmax(np.where(np.abs(lags - lags[np.nanargmax(cv)]) > 5, cv, np.nan))))
dt = float(dt_v) if a.fix_dt is None else a.fix_dt
print("sync", res["sync"])

# ---------------------------------------------------------------- 2. invariants on the synced data
t_ros = a.frame0 + dt + tv
m_in = inside(t_ros)
tv, Rc, pc, nt, amb, t_ros = tv[m_in], Rc[m_in], pc[m_in], nt[m_in], amb[m_in], t_ros[m_in]
Rf, pf = fk_at(t_ros)

# 2a. jitter where FK is still for >= 2 s (speed over +-1 s window)
vf1, wf1 = fk_speed(t_ros)
still = (vf1 < 0.05e-3) & (wf1 < 0.003)
blocks, cur = [], [0]
for i in range(1, len(tv)):
    if still[i] and still[i - 1] and tv[i] - tv[i - 1] < 0.3:
        cur.append(i)
    else:
        if len(cur) >= 15:
            blocks.append(cur)
        cur = [i]
if len(cur) >= 15:
    blocks.append(cur)
jp, jr = [], []
for b in blocks:
    b = np.array(b)
    if not still[b].all():
        b = b[still[b]]
    x = tv[b] - tv[b].mean()
    for k in range(3):
        c = np.polyfit(x, pc[b, k], 1)
        jp.append((k, pc[b, k] - np.polyval(c, x)))
    rv = (Rc[b] * Rc[b[len(b) // 2]].inv()).as_rotvec()
    jr.append(rv - rv.mean(0))
if blocks:
    sd = [1e3 * np.std(np.concatenate([r for k2, r in jp if k2 == k])) for k in range(3)]
    sr = D(np.std(np.concatenate(jr), axis=0))
    res["jitter"] = dict(n_windows=len(blocks), n_frames=int(sum(len(b) for b in blocks)),
                         pos_sd_cu_mm_xyz=[float(s) for s in sd], rot_sd_deg_xyz=[float(s) for s in sr])
    print("jitter", res["jitter"])

# 2b. rotation-angle ratio for pairs with large FK rotation
rng = np.random.default_rng(0)
i, j = rng.integers(0, len(tv), (2, 200000))
angf = D((Rf[i] * Rf[j].inv()).magnitude())
sel = angf > 3
if sel.sum() > 50:
    angc = D((Rc[i[sel]] * Rc[j[sel]].inv()).magnitude())
    rr = angc / angf[sel]
    res["rotation_angle_ratio"] = dict(pairs=int(sel.sum()), median=float(np.median(rr)),
                                       iqr=[float(np.percentile(rr, 25)), float(np.percentile(rr, 75))],
                                       max_fk_angle=float(angf.max()),
                                       slope_ls=float((angc * angf[sel]).sum() / (angf[sel] ** 2).sum()))
    print("rotation angle ratio", res["rotation_angle_ratio"])
else:
    res["rotation_angle_ratio"] = dict(pairs=int(sel.sum()), max_fk_angle=float(angf.max()))
    print("too little rotation for the rotation-angle test; max FK pair angle", angf.max())

# 2c. hand-eye conditioning: spread of FK relative-rotation axes for pairs > 1 deg
i, j = rng.integers(0, len(tv), (2, 20000))
B = (Rf[i] * Rf[j].inv()).as_rotvec()
big = np.linalg.norm(B, axis=1) > np.radians(1)
if big.sum() > 10:
    ax = B[big] / np.linalg.norm(B[big], axis=1)[:, None]
    ax *= np.sign(ax @ ax[0])[:, None]
    ev = np.linalg.eigvalsh(ax.T @ ax / len(ax))
    res["rotation_axis_spread_eigs"] = [float(e) for e in ev]
    print("FK rotation-axis scatter eigenvalues (1 axis only -> [0,0,1])", np.round(ev, 4))


# ---------------------------------------------------------------- 3. hand-eye + scale fit
def model(x, Rf_, pf_):
    Rx, Ry = Rot.from_rotvec(x[0:3]), Rot.from_rotvec(x[3:6])
    c, y, k = x[6:9], x[9:12], x[12]
    Rp = Rx * Rf_ * Ry
    pp = Rx.apply(k * (pf_ + Rf_.apply(y))) + c
    return Rp, pp


def resid(x, m, sig_r, sig_p):
    Rp, pp = model(x, Rf[m], pf[m])
    er = (Rc[m] * Rp.inv()).as_rotvec() / sig_r
    ep = (pc[m] - pp) / sig_p
    return np.concatenate([er.ravel(), ep.ravel()])


def init(m):
    """Kabsch/Umeyama on positions (ignores the lever arm), then Ry from the mean rotation."""
    A, Bp = pc[m], pf[m]
    ma, mb = A.mean(0), Bp.mean(0)
    U, S, Vt = np.linalg.svd((A - ma).T @ (Bp - mb))
    Dg = np.diag([1, 1, np.linalg.det(U @ Vt)])
    R = U @ Dg @ Vt
    k = (S * np.diag(Dg)).sum() / ((Bp - mb) ** 2).sum()
    Rx = Rot.from_matrix(R)
    Ry = Rot.from_matrix(np.mean((Rx * Rf[m]).inv().as_matrix() @ Rc[m].as_matrix(), axis=0))
    Ry = Rot.from_matrix(np.linalg.svd(Ry.as_matrix())[0] @ np.linalg.svd(Ry.as_matrix())[2])
    c = ma - k * R @ mb
    return np.r_[Rx.as_rotvec(), Ry.as_rotvec(), c, 0, 0, 0, k]


sig_r = np.radians(float(np.mean(res["jitter"]["rot_sd_deg_xyz"])) if "jitter" in res else 0.3)
sig_p = 1e-3 * (float(np.mean(res["jitter"]["pos_sd_cu_mm_xyz"])) if "jitter" in res else 1.0)
m_all = np.ones(len(tv), bool)


def fit(m, x0=None, fix_k=None):
    x0 = init(m) if x0 is None else x0.copy()
    if fix_k is not None:
        x0[12] = fix_k
        f = lambda z: resid(np.r_[z, fix_k], m, sig_r, sig_p)  # noqa: E731
        s = least_squares(f, x0[:12], loss="soft_l1", f_scale=3.0, x_scale="jac")
        return np.r_[s.x, fix_k], s
    s = least_squares(lambda z: resid(z, m, sig_r, sig_p), x0, loss="soft_l1", f_scale=3.0, x_scale="jac")
    return s.x, s


x, sol = fit(m_all)
# standard errors from the Jacobian (residuals already normalised by the jitter, inflated by the actual rms)
J = sol.jac
dof = max(1, len(sol.fun) - len(x))
s2 = (sol.fun ** 2).sum() / dof
try:
    cov = np.linalg.inv(J.T @ J) * s2
    se = np.sqrt(np.diag(cov))
except np.linalg.LinAlgError:
    se = np.full(len(x), np.nan)
Rp, pp = model(x, Rf, pf)
er = D((Rc * Rp.inv()).magnitude())
ep = 1e3 * np.linalg.norm(pc - pp, axis=1) / x[12]  # in FK mm
res["handeye"] = dict(k=float(x[12]), k_se=float(se[12]), y_m=[float(v) for v in x[9:12]],
                      y_se_m=[float(v) for v in se[9:12]], Rx_se_deg=float(D(np.linalg.norm(se[0:3]))),
                      Ry_se_deg=float(D(np.linalg.norm(se[3:6]))), rot_resid_deg_med_p95=[float(np.median(er)), float(np.percentile(er, 95))],
                      pos_resid_mm_med_p95=[float(np.median(ep)), float(np.percentile(ep, 95))], frames=int(len(tv)))
print("hand-eye", res["handeye"])

# k on motion subsets (Rx, y from the full fit; refit only c and k): robust to lever-arm and rotation errors
phase = np.full(len(tv), "other", object)
ts, te = scan
phase[(t_ros >= ts) & (t_ros <= te)] = "scan"
phase[(t_ros < ts) & (t_ros > ts - 60)] = "descent+leadin"
phase[(t_ros > te)] = "lift+after"
for ph in ("scan", "descent+leadin", "lift+after"):
    m = phase == ph
    if m.sum() < 30:
        continue
    q = pf[m] + Rf[m].apply(x[9:12])                      # FK position of the body origin (m)
    q = Rot.from_rotvec(x[0:3]).apply(q)                  # in camera axes
    A = np.c_[q - q.mean(0)]
    bvec = pc[m] - pc[m].mean(0)
    k = float((A * bvec).sum() / (A * A).sum())
    # per camera axis
    kax = [float((A[:, c] * bvec[:, c]).sum() / (A[:, c] ** 2).sum()) for c in range(3)]
    rngs = [float(1e3 * np.ptp(A[:, c])) for c in range(3)]
    # bootstrap over 10 s blocks
    blk = ((tv[m] - tv[m][0]) // 10).astype(int)
    ks = []
    for _ in range(300):
        pick = rng.choice(np.unique(blk), len(np.unique(blk)))
        ii = np.concatenate([np.flatnonzero(blk == b) for b in pick])
        Ab, bb = A[ii] - A[ii].mean(0), bvec[ii] - bvec[ii].mean(0)
        ks.append((Ab * bb).sum() / (Ab * Ab).sum())
    res[f"k_{ph}"] = dict(k=k, ci95=[float(np.percentile(ks, 2.5)), float(np.percentile(ks, 97.5))], k_cam_xyz=kax,
                          fk_range_mm_cam_xyz=rngs, frames=int(m.sum()))
    print(ph, res[f"k_{ph}"])

# lens axis: GoPro (camera rotation mapped back with the fitted Rx, Ry) vs FK
Wm = (Rot.from_rotvec(x[0:3]).inv() * Rc * Rot.from_rotvec(x[3:6]).inv())
zg, zf = Wm.apply([0, 0, 1]), Rf.apply([0, 0, 1])
axis_err = D(np.arccos(np.clip((zg * zf).sum(1), -1, 1)))
res["axis_gopro_vs_fk_deg"] = {ph: [float(np.median(axis_err[phase == ph])), float(np.percentile(axis_err[phase == ph], 95))]
                               for ph in ("scan", "descent+leadin", "lift+after") if (phase == ph).sum() > 30}
print("axis", res["axis_gopro_vs_fk_deg"])

# Configuration-dependent tilt during the scan. Rx, k, y from the full fit (the stow move's large rotations condition
# them); Ry is re-estimated on the scan so a constant offset is removed; then the residual rotation (world frame, small
# angle) is regressed on the FK focal position. 10 s block bootstrap for the CI. The lens-axis tilt is the part of the
# residual perpendicular to the axis.
m = phase == "scan"
if m.sum() > 200:
    Rx_ = Rot.from_rotvec(x[0:3])
    Ry_s = Rot.from_matrix((Rx_ * Rf[m]).inv().as_matrix() @ Rc[m].as_matrix()).mean()
    Wm_s = Rx_.inv() * Rc[m] * Ry_s.inv()
    e = D((Wm_s * Rf[m].inv()).as_rotvec())                       # deg, world frame
    zax = Rf[m].apply([0, 0, 1]).mean(0)
    e_perp = e - np.outer(e @ zax, zax)
    P = 1e3 * (pf[m] - pf[m].mean(0))                            # mm
    # raster directions: fast = along the lines (principal direction of FK velocity), slow = across the lines
    V = np.gradient(P, tv[m], axis=0)
    u_fast = np.linalg.svd(V[np.linalg.norm(V, axis=1) < 5], full_matrices=False)[2][0]
    Q = P - np.outer(P @ u_fast, u_fast)
    u_slow = np.linalg.svd(Q, full_matrices=False)[2][0]
    sf, ss, tt = P @ u_fast, P @ u_slow, tv[m] - tv[m].mean()
    # slow direction and time are nearly collinear (one line every ~40 s), so the slow-direction tilt is confounded
    # with any drift (camera mount, thermal, sun-tracking rate). The fast direction reverses every line: fit it with
    # time as a covariate.
    Xf = np.c_[sf, ss, tt, np.ones(len(sf))]
    blk = ((tv[m] - tv[m][0]) // 10).astype(int)
    ub = np.unique(blk)

    def tilt_fit(ii):
        c = np.linalg.lstsq(Xf[ii], e_perp[ii], rcond=None)[0]
        return 100 * np.linalg.norm(c[0]), 100 * np.linalg.norm(c[1]), c
    tf_, ts_, coef = tilt_fit(np.arange(len(sf)))
    boots = np.array([tilt_fit(np.concatenate([np.flatnonzero(blk == b) for b in rng.choice(ub, len(ub))]))[:2]
                      for _ in range(300)])
    axis_scan = np.linalg.norm(e_perp, axis=1)                   # deg
    bm = np.array([e_perp[blk == b].mean(0) for b in ub])
    corr_slow_time = float(np.corrcoef(ss, tt)[0, 1])
    res["scan_tilt"] = dict(
        fast_deg_per_100mm=float(tf_), fast_ci95=[float(np.percentile(boots[:, 0], 2.5)), float(np.percentile(boots[:, 0], 97.5))],
        slow_deg_per_100mm_confounded=float(ts_), slow_ci95=[float(np.percentile(boots[:, 1], 2.5)), float(np.percentile(boots[:, 1], 97.5))],
        corr_slow_vs_time=corr_slow_time, fast_range_mm=float(np.ptp(sf)), slow_range_mm=float(np.ptp(ss)),
        drift_deg_per_min=float(60 * np.linalg.norm(coef[2])),
        explained_frac=float(1 - ((e_perp - Xf @ coef) ** 2).sum() / ((e_perp - e_perp.mean(0)) ** 2).sum()),
        axis_resid_after_offset_deg_med_p95=[float(np.median(axis_scan)), float(np.percentile(axis_scan, 95))],
        axis_resid_10s_means_deg_med_max=[float(np.median(np.linalg.norm(bm, axis=1))), float(np.max(np.linalg.norm(bm, axis=1)))])
    print("scan tilt", res["scan_tilt"])
    # Per raster line (preferred; the regression above is biased up by noise because it reports a vector norm):
    # change of the GoPro-minus-FK axis residual from line start to line end, regressed on the signed travel along
    # the line. Lines alternate direction, so slow drift cancels; a sign-flip permutation gives the noise floor.
    q = P - np.c_[tt, np.ones(len(tt))] @ np.linalg.lstsq(np.c_[tt, np.ones(len(tt))], P, rcond=None)[0]
    s_line = P @ np.linalg.svd(q, full_matrices=False)[2][0]
    tgr = np.arange(tv[m][0], tv[m][-1], 0.5)
    vs = np.gradient(np.convolve(np.interp(tgr, tv[m], s_line), np.ones(5) / 5, "same"), tgr)
    edges = np.r_[tv[m][0], tgr[np.flatnonzero(np.diff(np.sign(vs)) != 0)], tv[m][-1]]
    rows = []
    for t0, t1 in zip(edges[:-1], edges[1:]):
        g = np.flatnonzero((tv[m] > t0 + 2) & (tv[m] < t1 - 2))
        if t1 - t0 >= 20 and len(g) >= 100:
            rows.append((s_line[g[-30:]].mean() - s_line[g[:30]].mean(), e_perp[g[-30:]].mean(0) - e_perp[g[:30]].mean(0)))
    if len(rows) >= 4:
        dsl, del_ = np.array([r[0] for r in rows]), np.array([r[1] for r in rows])
        def slope(w, d):  # deg per 100 mm of travel along the line
            return 100 * np.linalg.norm((d * w[:, None]).sum(0) / (w ** 2).sum())
        bs = [slope(dsl[i], del_[i]) for i in (rng.integers(0, len(dsl), len(dsl)) for _ in range(1000))]
        null = [slope(dsl * rng.choice([-1, 1], len(dsl)), del_) for _ in range(1000)]
        res["line_tilt"] = dict(lines=len(rows), mean_line_mm=float(np.abs(dsl).mean()), deg_per_100mm=slope(dsl, del_),
                                boot_ci95=[float(np.percentile(bs, 2.5)), float(np.percentile(bs, 97.5))],
                                signflip_null_p95=float(np.percentile(null, 95)))
        print("per-line tilt", res["line_tilt"])
    # pre-scan static pose vs scan: one configuration change, offset between the means (deg)
    pre = (phase == "other") & (t_ros < scan[0])
    if pre.sum() > 50:
        e_pre = D(((Rx_.inv() * Rc[pre] * Ry_s.inv()) * Rf[pre].inv()).as_rotvec())
        e_pre -= np.outer(e_pre @ zax, zax)
        dpos = 1e3 * np.linalg.norm(pf[pre].mean(0) - pf[m].mean(0))
        res["prescan_vs_scan"] = dict(axis_offset_deg=float(np.linalg.norm(np.median(e_pre, 0) - np.median(e_perp, 0))),
                                      fk_distance_mm=float(dpos), frames=int(pre.sum()))
        print("pre-scan vs scan", res["prescan_vs_scan"])

# pose.py-compatible output (gopro_vs_fk.py): camera-measured focal pose in the FK world, with the fitted scale
Rx_, Ry_ = Rot.from_rotvec(x[0:3]), Rot.from_rotvec(x[3:6])
Rm = Rx_.inv() * Rc * Ry_.inv()
pm = Rx_.inv().apply(pc - x[6:9]) / x[12] - Rm.apply(x[9:12])
Wmeas = np.tile(np.eye(4), (len(tv), 1, 1)); Wmeas[:, :3, :3] = Rm.as_matrix(); Wmeas[:, :3, 3] = pm
Wfk = np.tile(np.eye(4), (len(tv), 1, 1)); Wfk[:, :3, :3] = Rf.as_matrix(); Wfk[:, :3, 3] = pf
np.savez(a.out + "_pose.npz", t=t_ros, world_meas=Wmeas, world_fk=Wfk, dt=dt, scale=1 / x[12], frame0=a.frame0 + dt,
         x=x, tag_size_if_fk_exact_mm=1e3 * float(cp["tag"]) / x[12])

np.savez(a.out + "_diag.npz", t=t_ros, Rc=Rc.as_matrix(), pc=pc, Rf=Rf.as_matrix(), pf=pf, x=x, phase=phase.astype(str),
         lags=lags, cv=cv, cw=cw, tg=tg, vc=vc, wc=wc, okc=okc, dt=dt, ntags=nt)
json.dump(res, open(a.out + "_diag.json", "w"), indent=1)

# ---------------------------------------------------------------- plots
fig, axs = plt.subplots(4, 1, figsize=(11, 12))
axs[0].plot(lags, cv, label="translation speed")
axs[0].plot(lags, cw, label="rotation speed")
axs[0].set_xlabel("clock offset dt (s)"); axs[0].set_ylabel("corr"); axs[0].legend()
vf, wf = fk_speed(a.frame0 + dt + tg)
axs[1].plot(tg, 1e3 * vc / x[12], ".", ms=2, label="camera / k")
axs[1].plot(tg, 1e3 * vf, lw=0.8, label="FK")
axs[1].set_ylabel("speed (mm/s)"); axs[1].set_yscale("symlog", linthresh=1); axs[1].legend()
axs[2].plot(tv, axis_err, ".", ms=1)
axs[2].set_ylabel("axis GoPro vs FK (deg)"); axs[2].set_ylim(0, max(1, np.percentile(axis_err, 99)))
for c, lab in enumerate("xyz"):
    axs[3].plot(tv, 1e3 * (pc - pp)[:, c] / x[12], ".", ms=1, label=f"cam {lab}")
axs[3].set_ylabel("pos resid (FK mm)"); axs[3].set_xlabel("video time (s)"); axs[3].legend(markerscale=8)
fig.tight_layout(); fig.savefig(a.out + "_diag.png", dpi=100)
