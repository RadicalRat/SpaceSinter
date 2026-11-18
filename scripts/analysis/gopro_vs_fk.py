#!/usr/bin/env python3
"""GoPro lens pose (pose.py, tags at their true size) vs FK for the continuous trials of 2026-11-15.

    python3 gopro_vs_fk.py <pose dir with tNN_pose.npz> <dir with TrialNN.npz + metaNN.npz> <out dir> NN [NN ...]

1. Travel ratio k: least-squares slope of GoPro-measured focal-point displacement on FK displacement (both about
   their mean), over the whole clip (lead-in, 80 mm lift/descent, raster) and over the raster only. k = 1: the arm
   moves as its model says; the Oct 30 data suggested ~0.86 (0.81 with 30 mm tags).
2. Lens-axis disagreement GoPro vs FK (direction only), vs time and vs position along the raster: configuration-
   dependent tilt that FK cannot see (Oct 30: ~0.6 deg / 100 mm).
3. GoPro lens axis vs the ephemeris sun direction: an independent check of pointing VARIATION. The hand-eye fit
   anchors the GoPro world frame to FK on average, so the mean offset is not independent; the spread is.
"""
import json
import sys
from pathlib import Path

import numpy as np

POSE, DATA, OUT = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
TRIALS = [int(n) for n in sys.argv[4:]]
HEADING = 121.0  # deg, objective value used on 2026-11-15
OUT.mkdir(parents=True, exist_ok=True)
D = np.degrees


def ang(a, b):
    return D(np.arccos(np.clip(np.einsum("ni,ni->n", a, b), -1, 1)))


res = {}
for n in TRIALS:
    p = np.load(POSE / f"t{n}_pose.npz")
    t, Wm, Wf = p["t"], p["world_meas"], p["world_fk"]
    tr = np.load(DATA / f"Trial{n}.npz", allow_pickle=True)
    me = np.load(DATA / f"meta{n}.npz", allow_pickle=True)
    P = tr["prog"]
    ts, te = P[np.argmax(P[:, 1] > 0), 0], P[np.argmax(P[:, 1] >= P[:, 1].max() - 1e-9), 0]
    out = {"frames": int(len(t)), "dt": float(p["dt"])}
    for lab, m in (("clip", np.ones(len(t), bool)), ("scan", (t >= ts) & (t <= te))):
        if m.sum() < 50:
            continue
        pm, pf = Wm[m, :3, 3], Wf[m, :3, 3]
        dm, df = pm - pm.mean(0), pf - pf.mean(0)
        k = float((dm * df).sum() / (df * df).sum())
        resid = 1e3 * np.linalg.norm(dm - k * df, axis=1)
        out[f"travel_ratio_{lab}"] = k
        out[f"fk_range_mm_{lab}"] = float(1e3 * np.ptp(np.linalg.norm(df, axis=1)))
        out[f"pos_resid_mm_{lab}"] = [float(np.median(resid)), float(np.percentile(resid, 95))]
        a = ang(Wm[m, :3, 2], Wf[m, :3, 2])
        out[f"axis_gopro_vs_fk_deg_{lab}"] = [float(np.median(a)), float(np.percentile(a, 95))]
    # Tilt vs position during the scan: regress the GoPro-FK axis difference (small-angle vector) on FK position.
    m = (t >= ts) & (t <= te)
    if m.sum() > 50:
        dv = np.cross(Wf[m, :3, 2], Wm[m, :3, 2])            # ~ rotation vector between the axes (rad)
        X = np.c_[1e3 * (Wf[m, :3, 3] - Wf[m, :3, 3].mean(0)), np.ones(m.sum())]
        coef = np.linalg.lstsq(X, D(dv), rcond=None)[0]       # deg per mm, per world axis of position
        out["tilt_deg_per_100mm_by_axis"] = [float(100 * v) for v in np.linalg.norm(coef[:3], axis=1)]
        pred = X @ coef
        out["tilt_explained_frac"] = float(1 - ((D(dv) - pred) ** 2).sum() / ((D(dv) - D(dv).mean(0)) ** 2).sum())
        # 3. vs ephemeris (spread only)
        el = np.interp(t[m], me["sun_elevation_deg"][:, 0], me["sun_elevation_deg"][:, 1])
        az = np.interp(t[m], me["sun_azimuth_deg"][:, 0], me["sun_azimuth_deg"][:, 1])
        th = np.radians(HEADING - az)
        s = np.c_[np.cos(np.radians(el)) * np.cos(th), np.cos(np.radians(el)) * np.sin(th), np.sin(np.radians(el))]
        for lab, ax in (("gopro", Wm[m, :3, 2]), ("fk", Wf[m, :3, 2])):
            e = ang(ax, s)
            out[f"{lab}_axis_vs_ephemeris_deg"] = dict(median=float(np.median(e)),
                                                       spread_p95=float(np.percentile(np.abs(e - np.median(e)), 95)))
    res[f"T{n}"] = out
(OUT / "gopro_vs_fk.json").write_text(json.dumps(res, indent=1))
print(json.dumps(res, indent=1))
