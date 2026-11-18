#!/usr/bin/env python3
"""Continuous-control (TrackRasterWithSun) real-sun trials of 2026-11-15: pointing error, path error and dexterity
vs time, compared with the discrete Trials 2 and 11 (advisor_figs.py stats).

    python3 continuous_figs.py <dir with TrialNN.npz + metaNN.npz> <output dir>

Inputs are the per-trial dumps written by the session extractors (topics /sinter/sun_error_stream,
/sinter/track_error, /sinter/track_progress, /sinter/track_lift, /sinter/dexterity, /velocity_force_controller/command,
plus the bed-plane / sun-geometry / SRRL metadata). Pointing error = sun-camera error (atan|v|), the quantity the
loop controls; path error = FK focal point minus the commanded reference (not true spot placement).
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

DATA, OUT = Path(sys.argv[1]), Path(sys.argv[2])
OUT.mkdir(parents=True, exist_ok=True)
TRIALS = [14, 16, 19, 20]
# Reference palette, fixed categorical order (slots 1-6); T2/T11 keep their advisor-figure colours.
C = {"T2": "#2a78d6", "T11": "#eb6834", 14: "#1baf7a", 16: "#eda100", 19: "#e87ba4", 20: "#008300"}
INK, INK2, GRID, SURF, REF = "#1a1a19", "#5b5a56", "#e4e3df", "#ffffff", "#8a8984"
plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "axes.edgecolor": "#c9c8c3", "axes.labelcolor": INK2,
    "xtick.color": INK2, "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.7,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 9, "axes.titlesize": 10,
    "axes.titleweight": "bold", "axes.titlecolor": INK, "axes.titlelocation": "left", "legend.frameon": False,
    "lines.linewidth": 1.2})
R1_MM, R2, TOL, SUN_R = 2.0, 0.3, np.degrees(0.002), 0.2665   # deg; TOL = discrete alignment tolerance (2 mrad)
DISCRETE = {  # advisor_figs.py stats.json fig3/fig7 (1 s average, sun sensor; FK placement)
    "T2": dict(median=0.288, p95=0.875, max=1.083, over_R2=0.479, place_med=0.172, place_p95=0.350, minutes=39.6),
    "T11": dict(median=0.092, p95=0.180, max=0.550, over_R2=0.012, place_med=0.284, place_p95=0.647, minutes=15.1)}


def smooth(t, x, win=1.0):
    """Centred moving average over `win` seconds (handles uneven sampling)."""
    cs = np.concatenate([[0.0], np.cumsum(x)])
    lo = np.searchsorted(t, t - win / 2)
    hi = np.searchsorted(t, t + win / 2)
    return (cs[hi] - cs[lo]) / np.maximum(hi - lo, 1)


def load(n):
    d = np.load(DATA / f"Trial{n}.npz", allow_pickle=True)
    m = np.load(DATA / f"meta{n}.npz", allow_pickle=True)
    P = d["prog"]
    ts = P[np.argmax(P[:, 1] > 0), 0]
    te = P[np.argmax(P[:, 1] >= P[:, 1].max() - 1e-9), 0]
    S = d["sun"]
    S = S[(S[:, 0] >= ts) & (S[:, 0] <= te) & ~np.isnan(S[:, 1])]
    # Drop isolated glare detections (a wrong blob for one frame reads > 1 deg; the loop rejects it via debounce).
    S = S[np.degrees(np.arctan(np.hypot(S[:, 1], S[:, 2]))) < 1.0]
    vx, vy = smooth(S[:, 0], S[:, 1]), smooth(S[:, 0], S[:, 2])
    err1 = np.degrees(np.arctan(np.hypot(vx, vy)))                 # 1 s average of the error VECTOR
    raw = np.degrees(np.arctan(np.hypot(S[:, 1], S[:, 2])))
    T = d["track"]
    T = T[(T[:, 0] >= ts) & (T[:, 0] <= te)]
    Dx = d["dex"]
    Dx = Dx[(Dx[:, 0] >= ts) & (Dx[:, 0] <= te)]
    Cm = d["cmd"]
    Cm = Cm[(Cm[:, 0] >= ts) & (Cm[:, 0] <= te)]
    W = lambda A: A[(A[:, 0] >= ts) & (A[:, 0] <= te), 1]
    return dict(ts=ts, te=te, t=S[:, 0] - ts, err1=err1, raw=raw, vx=np.degrees(vx), vy=np.degrees(vy),
                tt=T[:, 0] - ts, path=1e3 * np.linalg.norm(T[:, 1:], axis=1), td=Dx[:, 0] - ts, dex=Dx[:, 1],
                tc=Cm[:, 0] - ts, v=Cm[:, 1:4], w=Cm[:, 4:7], inc=W(m["bed_incidence_deg"]),
                el=W(m["sun_elevation_deg"]), plane=m["planes"][-1], dni=m["dni_w_m2"][:, 1])


D = {n: load(n) for n in TRIALS}
stats = {}
for n, r in D.items():
    e = r["err1"]
    # Line direction: the raster lines alternate; sign of the commanded velocity along the dominant horizontal axis.
    vh = r["v"][:, :2]
    ax_ = np.linalg.svd(vh - vh.mean(0), full_matrices=False)[2][0]
    sgn = np.sign(vh @ ax_)
    sgn_t = np.interp(r["t"], r["tc"], sgn)
    fwd, back = e[sgn_t > 0.5], e[sgn_t < -0.5]
    dex_t = np.interp(r["t"], r["td"], r["dex"])
    slope = np.polyfit(r["t"] / 60, e, 1)[0]
    stats[f"T{n}"] = dict(
        minutes=float((r["te"] - r["ts"]) / 60), median=float(np.median(e)), p95=float(np.percentile(e, 95)),
        max=float(e.max()), over_R2=float((e > R2).mean()), over_sunR=float((e > SUN_R).mean()),
        over_tol=float((e > TOL).mean()), raw_median=float(np.median(r["raw"])),
        raw_p95=float(np.percentile(r["raw"], 95)), path_med=float(np.median(r["path"])),
        path_p95=float(np.percentile(r["path"], 95)), path_max=float(r["path"].max()),
        path_over_R1=float((r["path"] > R1_MM).mean()), dex_min=float(r["dex"].min()),
        dex_range=[float(r["dex"].min()), float(r["dex"].max())],
        corr_err_dex=float(np.corrcoef(e, dex_t)[0, 1]),
        line_dir_median=[float(np.median(fwd)), float(np.median(back))], trend_deg_per_min=float(slope),
        mean_vx=float(r["vx"].mean()), mean_vy=float(r["vy"].mean()),
        incidence=[float(r["inc"][0]), float(r["inc"][-1])], elevation=[float(r["el"][0]), float(r["el"][-1])],
        plane_tilt=float(np.degrees(np.arccos(r["plane"][6]))), dni_mean=float(r["dni"].mean()),
        w_cmd_med_deg_s=float(np.degrees(np.median(np.linalg.norm(r["w"], axis=1)))))
(OUT / "stats.json").write_text(json.dumps(dict(continuous=stats, discrete=DISCRETE), indent=1))

# ---- Fig 1: per-trial time series: pointing (1 s), path error, dexterity (small multiples, shared x) ----
fig, ax = plt.subplots(len(TRIALS), 3, figsize=(13, 2.1 * len(TRIALS)), sharex=True)
for i, n in enumerate(TRIALS):
    r, s = D[n], stats[f"T{n}"]
    a = ax[i, 0]
    a.plot(r["t"] / 60, r["err1"], color=C[n], lw=0.8)
    for v, lab, ls in ((R2, "R2 0.3°", ":"), (TOL, "discrete tol 0.115°", "--")):
        a.axhline(v, color=REF, ls=ls, lw=1)
        a.text(1.0, v, " " + lab, transform=a.get_yaxis_transform(), color=INK2, fontsize=7, va="center")
    a.set_ylim(0, 0.34)
    a.set_ylabel(f"Trial {n}\npointing (°)")
    a.text(0.01, 0.95, f"median {s['median']:.3f}°, p95 {s['p95']:.3f}°  |  incidence {s['incidence'][0]:.0f}°",
           transform=a.transAxes, fontsize=7.5, color=INK2, va="top")
    a = ax[i, 1]
    a.plot(r["tt"] / 60, r["path"], color=C[n], lw=0.6)
    a.axhline(R1_MM, color=REF, ls=":", lw=1)
    a.text(1.0, R1_MM, " R1 2 mm", transform=a.get_yaxis_transform(), color=INK2, fontsize=7, va="center")
    a.set_ylim(0, 2.6)
    a.set_ylabel("path error (mm)")
    a.text(0.01, 0.95, f"median {s['path_med']:.2f}, p95 {s['path_p95']:.2f}, max {s['path_max']:.2f} mm",
           transform=a.transAxes, fontsize=7.5, color=INK2, va="top")
    a = ax[i, 2]
    a.plot(r["td"] / 60, r["dex"], color=C[n], lw=1.0)
    a.set_ylim(0, 0.12)
    a.set_ylabel("dexterity")
    a.text(0.01, 0.95, f"min {s['dex_min']:.3f} (abort < 0.01)", transform=a.transAxes, fontsize=7.5,
           color=INK2, va="top")
ax[0, 0].set_title("Lens pointing error (sun camera, 1 s average)")
ax[0, 1].set_title("Focal-point path error (FK vs reference)")
ax[0, 2].set_title("Dexterity σmin/σmax")
for a in ax[-1]:
    a.set_xlabel("time since scan start (min)")
fig.tight_layout()
fig.savefig(OUT / "fig1_timeseries.png", dpi=160)
fig.savefig(OUT / "fig1_timeseries.pdf")

# ---- Fig 2: ECDF of 1 s pointing error, continuous trials, with discrete medians/p95 marked ----
fig, a = plt.subplots(figsize=(7.5, 4.2))
for n in TRIALS:
    e = np.sort(D[n]["err1"])
    y = np.arange(1, len(e) + 1) / len(e)
    a.plot(e, y, color=C[n], lw=2, label=f"T{n} continuous (incidence {stats[f'T{n}']['incidence'][0]:.0f}°)")
for k_, ds in DISCRETE.items():
    a.plot([ds["median"], ds["p95"]], [0.5, 0.95], "o", color=C[k_], ms=8, mec=SURF, mew=1.5)
    a.plot([ds["median"], ds["p95"]], [0.5, 0.95], color=C[k_], lw=1, ls="--", label=f"{k_} discrete: median, p95")
for v, lab in ((TOL, "discrete tol"), (SUN_R, "solar radius"), (R2, "R2")):
    a.axvline(v, color=REF, ls=":", lw=1)
    a.text(v, 0.3, " " + lab, color=INK2, fontsize=7.5, rotation=90, va="bottom")
a.set_xscale("log")
a.set_xlim(0.005, 1.2)
a.legend(loc="upper left", fontsize=8)
a.set_ylim(0, 1)
a.set_xlabel("lens pointing error, sun camera, 1 s average (°)")
a.set_ylabel("fraction of scan time ≤ x")
a.set_title("Continuous control (Nov 15) vs discrete stop-check-move (Trials 2, 11)")
fig.tight_layout()
fig.savefig(OUT / "fig2_ecdf.png", dpi=160)
fig.savefig(OUT / "fig2_ecdf.pdf")

# ---- Fig 3: pointing error vs dexterity (binned), all trials ----
fig, a = plt.subplots(figsize=(6.5, 3.8))
for n in TRIALS:
    r = D[n]
    dx = np.interp(r["t"], r["td"], r["dex"])
    bins = np.linspace(dx.min(), dx.max(), 7)
    idx = np.digitize(dx, bins[1:-1])
    xm = [dx[idx == k].mean() for k in range(6) if (idx == k).sum() > 100]
    ym = [np.median(r["err1"][idx == k]) for k in range(6) if (idx == k).sum() > 100]
    a.plot(xm, ym, "o-", color=C[n], ms=8, mec=SURF, mew=1.5)
    a.text(xm[-1] + 0.001, ym[-1], f"T{n}", color=INK, fontsize=8, va="center")
a.set_xlabel("dexterity (binned)")
a.set_ylabel("median pointing error (°)")
a.set_ylim(0, 0.08)
a.set_title("Pointing error vs dexterity")
fig.tight_layout()
fig.savefig(OUT / "fig3_err_vs_dex.png", dpi=160)
fig.savefig(OUT / "fig3_err_vs_dex.pdf")
print(json.dumps(stats, indent=1))
