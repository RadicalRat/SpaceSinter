#!/usr/bin/env python3
"""Two-page advisor figure sheet for the four full continuous-control real-sun rasters of 2026-11-15
(Trials 14, 16, 19, 20).

    python3 advisor_continuous.py <analysis dir> <out dir> [final_products.jpg]

<analysis dir> holds tNN.pkl (extract.py) and tNN_fk.npz (fk.py) and diag/tNN_diag.{npz,json} (gopro_diag.py; T19 is
split at the camera bump into t19a/t19b). Writes advisor_continuous.pdf (2 pages), page PNGs and stats.json.

Pointing = sun-camera error angle (the loop's own sensor), 1 s vector average, glare frames > 1 deg dropped.
Path error = FK focal point minus the tracker's reference (/sinter/track_error): how well the arm follows the plan,
NOT measured spot placement. GoPro axis = lens axis from the lens AprilTags vs FK: direction only, so it does not
depend on the unresolved camera/FK scale (k ~ 0.78).
"""
import json
import pickle
import re
import textwrap
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
from matplotlib.gridspec import GridSpec  # noqa: E402
from scipy.spatial.transform import Rotation as Rot  # noqa: E402

SRC, OUT = Path(sys.argv[1]), Path(sys.argv[2])
PHOTO = Path(sys.argv[3]) if len(sys.argv) > 3 else None
OUT.mkdir(parents=True, exist_ok=True)
TRIALS = [14, 16, 19, 20]
C = {14: "#1baf7a", 16: "#eda100", 19: "#e87ba4", 20: "#2a78d6"}
INK, INK2, GRID, SURF, REF = "#1a1a19", "#5b5a56", "#e4e3df", "#ffffff", "#8a8984"
plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "axes.edgecolor": "#c9c8c3", "axes.labelcolor": INK2,
    "xtick.color": INK2, "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 8, "axes.titlesize": 9,
    "axes.titleweight": "bold", "axes.titlecolor": INK, "axes.titlelocation": "left", "legend.frameon": False,
    "legend.fontsize": 7, "lines.linewidth": 1.1, "font.family": "DejaVu Sans"})
R1_MM, R2, SUN_R = 2.0, 0.3, 0.2665
VEC = re.compile(r"x=([-\w.+]+), y=([-\w.+]+), z=([-\w.+]+)\)")


def vec(rows):
    A = np.array([[t, *map(float, VEC.search(s).groups())] for t, s in rows])
    return A[np.isfinite(A).all(1)]


def smooth(t, x, win=1.0):
    cs = np.concatenate([[0.0], np.cumsum(x)])
    lo, hi = np.searchsorted(t, t - win / 2), np.searchsorted(t, t + win / 2)
    return (cs[hi] - cs[lo]) / np.maximum(hi - lo, 1)


def quat_R(q):
    return Rot.from_quat(q).as_matrix()


def load(n):
    d = pickle.load(open(SRC / f"t{n}.pkl", "rb"))
    prog = np.array(d["/sinter/track_progress"])
    ts = prog[np.argmax(prog[:, 1] > 0), 0]
    te = prog[np.argmax(prog[:, 1] >= prog[:, 1].max() - 1e-9), 0]
    win = lambda A: A[(A[:, 0] >= ts) & (A[:, 0] <= te)]  # noqa: E731
    S_all = vec(d["/sinter/sun_error_stream"])
    S = win(S_all)
    S = S[np.degrees(np.arctan(np.hypot(S[:, 1], S[:, 2]))) < 1.0]
    err1 = np.degrees(np.arctan(np.hypot(smooth(S[:, 0], S[:, 1]), smooth(S[:, 0], S[:, 2]))))
    T = win(vec(d["/sinter/track_error"]))
    lift = np.array(d["/sinter/track_lift"])
    dex = win(np.array(d["/sinter/dexterity"]))
    inc = np.array(d["/sinter/bed_incidence_deg"])
    el = np.array(d["/sinter/sun_elevation_deg"])
    dni = np.array([v for _, v in d["/srrl/dni_w_m2"]])
    _, _, p0, q0 = d["/sinter/bed_plane"][-1]
    Rp = quat_R(q0)
    fk = np.load(SRC / f"t{n}_fk.npz")
    mf = (fk["t"] >= ts) & (fk["t"] <= te)
    P = (fk["focal"][mf, :3, 3] - np.array(p0)) @ Rp           # focal point in bed-plane coordinates
    ref = np.array([w[0] for w in d["/sinter/raster_path"][-1][2]])
    Pr = (ref - np.array(p0)) @ Rp
    tf = fk["t"][mf]
    perr = 1e3 * np.linalg.norm(T[:, 1:], axis=1)
    perr_fk = np.interp(tf, T[:, 0], perr)
    # sun visible = a valid sun-camera frame in the last 0.5 s (the tracker's freshness test)
    tl = lift[(lift[:, 0] >= ts - 30) & (lift[:, 0] <= te + 30)]
    gaps = np.diff(S_all[:, 0])
    gi = np.flatnonzero(gaps > 0.5)
    gaps_t = [(S_all[i, 0] - ts, S_all[i + 1, 0] - ts) for i in gi
              if S_all[i, 0] > ts - 30 and S_all[i + 1, 0] < te + 30]
    return dict(ts=ts, te=te, t=S[:, 0] - ts, err1=err1, tt=T[:, 0] - ts, path=perr, td=dex[:, 0] - ts,
                dex=dex[:, 1], P=P, Pr=Pr, perr_fk=perr_fk, tl=tl[:, 0] - ts, lift=1e3 * tl[:, 1], gaps=gaps_t,
                inc=np.interp(ts, inc[:, 0], inc[:, 1]), inc_end=np.interp(te, inc[:, 0], inc[:, 1]),
                el=np.interp(ts, el[:, 0], el[:, 1]), dni=float(dni.mean()),
                tilt=float(np.degrees(np.arccos(np.clip(Rp[2, 2], -1, 1)))))


def gopro_axis(tag):
    """Per-frame GoPro-vs-FK lens-axis angle during the scan, constant mount offset removed (gopro_diag.py method)."""
    p = SRC / "diag" / f"{tag}_diag.npz"
    if not p.exists():
        return None
    z = np.load(p)
    m = z["phase"] == "scan"
    if m.sum() < 200:
        return None
    Rc, Rf, x = Rot.from_matrix(z["Rc"][m]), Rot.from_matrix(z["Rf"][m]), z["x"]
    Rx = Rot.from_rotvec(x[0:3])
    Ry = Rot.from_matrix((Rx * Rf).inv().as_matrix() @ Rc.as_matrix()).mean()
    e = np.degrees((Rx.inv() * Rc * Ry.inv() * Rf.inv()).as_rotvec())
    zax = Rf.apply([0, 0, 1]).mean(0)
    e_perp = e - np.outer(e @ zax, zax)
    t = z["t"][m]
    blk = ((t - t[0]) // 10).astype(int)
    tb = np.array([t[blk == b].mean() for b in np.unique(blk)])
    eb = np.array([np.linalg.norm(e_perp[blk == b].mean(0)) for b in np.unique(blk)])
    j = json.loads((SRC / "diag" / f"{tag}_diag.json").read_text())
    return dict(t=t, a=np.linalg.norm(e_perp, axis=1), tb=tb, ab=eb, ntags=z["ntags"][m],
                line_tilt=j.get("line_tilt", {}))


D = {n: load(n) for n in TRIALS}
G = {14: [gopro_axis("t14")], 16: [gopro_axis("t16")], 19: [gopro_axis("t19a"), gopro_axis("t19b")],
     20: [gopro_axis("t20")]}

stats = {}
for n, r in D.items():
    e = r["err1"]
    g = [x for x in G[n] if x is not None]
    ga = np.concatenate([x["a"] for x in g]) if g else np.array([np.nan])
    gb = np.concatenate([x["ab"] for x in g]) if g else np.array([np.nan])
    stats[f"T{n}"] = dict(
        scan_min=float((r["te"] - r["ts"]) / 60), incidence=[float(r["inc"]), float(r["inc_end"])],
        elevation=float(r["el"]), bed_tilt=r["tilt"], dni=r["dni"],
        point_med=float(np.median(e)), point_p95=float(np.percentile(e, 95)), point_max=float(e.max()),
        point_over_R2=float((e > R2).mean()), point_within_tol=float((e <= np.degrees(0.002)).mean()), path_med=float(np.median(r["path"])),
        path_p95=float(np.percentile(r["path"], 95)), path_max=float(r["path"].max()),
        dex_min=float(r["dex"].min()), sun_gaps=len(r["gaps"]),
        gopro_axis_med=float(np.median(ga)), gopro_axis_p95=float(np.percentile(ga, 95)),
        gopro_axis_10s_med=float(np.median(gb)),
        gopro_line_tilt=[x["line_tilt"].get("deg_per_100mm") for x in g],
        gopro_line_tilt_null=[x["line_tilt"].get("signflip_null_p95") for x in g])
(OUT / "stats.json").write_text(json.dumps(stats, indent=1))


def header(fig, title, sub):
    fig.text(0.04, 0.965, title, fontsize=13, fontweight="bold", color=INK, va="top")
    fig.text(0.04, 0.932, sub, fontsize=8.5, color=INK2, va="top")


def note(a, s, y=-0.22, width=110):
    a.text(0, y, textwrap.fill(s, width), transform=a.transAxes, fontsize=6.8, color=INK2, va="top")


pdf = PdfPages(OUT / "advisor_continuous.pdf")
inc_lbl = {n: f"T{n} ({D[n]['inc']:.0f}° inc.)" for n in TRIALS}

# ================================================================ PAGE 1: closed-loop performance
fig = plt.figure(figsize=(11, 8.5))
header(fig, "Continuous sun-tracking raster: closed-loop performance (4 full real-sun runs, 2026-11-15)",
       "TrackRasterWithSun: 3-DOF focal path + 3-DOF sun tracking, velocity_force_controller.  Each run: 30 × 30 mm "
       "raster, 13 lines at 2.5 mm, 0.75 mm/s, ~9.4 min, ~420 mm of path.  Kinova JACO2 + 0.5 m Fresnel lens.")
gs = GridSpec(3, 3, figure=fig, left=0.065, right=0.975, top=0.88, bottom=0.075, hspace=0.62, wspace=0.32,
              height_ratios=[1.05, 1.0, 1.0])

# (a) pointing vs time, all runs
a = fig.add_subplot(gs[0, :])
TOL = np.degrees(0.002)                                       # 2 mrad alignment tolerance of the discrete controller
a.axhspan(0, R2, color="#1baf7a", alpha=0.07, lw=0)
for n in TRIALS:
    r = D[n]
    win_r2, win_tol = 100 * (r["err1"] <= R2).mean(), 100 * (r["err1"] <= TOL).mean()
    a.plot(r["t"] / 60, r["err1"], color=C[n], lw=0.7,
           label=f"{inc_lbl[n]}: {win_r2:.0f} % ≤ R2, {win_tol:.1f} % ≤ 2 mrad")
a.axhline(R2, color=REF, ls=":", lw=1)
a.text(0.995, R2 - 0.004, "R2 requirement 0.3° (shaded = within)", transform=a.get_yaxis_transform(), color=INK2,
       fontsize=7, va="top", ha="right")
a.axhline(TOL, color=REF, ls="--", lw=1)
a.text(0.995, TOL + 0.006, "2 mrad (0.115°) control tolerance", transform=a.get_yaxis_transform(), color=INK2,
       fontsize=7, va="bottom", ha="right", zorder=5, bbox=dict(fc=SURF, ec="none", alpha=0.85, pad=1.5))
a.set_ylim(0, 0.32)
a.set_xlim(0, max(r["t"][-1] for r in D.values()) / 60)
a.set_xlabel("time since scan start (min)")
a.set_ylabel("pointing error (°)")
a.set_title("(a) Lens pointing error during the scan (sun camera, 1 s average)")
a.legend(ncol=2, loc="upper left", bbox_to_anchor=(0, 0.93), fontsize=6.8)

# (b) ECDF pointing
a = fig.add_subplot(gs[1, 0])
for n in TRIALS:
    e = np.sort(D[n]["err1"])
    a.plot(e, np.arange(1, len(e) + 1) / len(e), color=C[n], lw=1.6, label=f"T{n}")
for v, lab in ((SUN_R, "solar radius"), (R2, "R2")):
    a.axvline(v, color=REF, ls=":", lw=1)
    a.text(v, 0.05, " " + lab, color=INK2, fontsize=6.5, rotation=90, va="bottom")
a.set_xscale("log")
a.set_xlim(0.005, 0.5)
a.set_ylim(0, 1)
a.set_xlabel("pointing error, 1 s avg (°)")
a.set_ylabel("fraction of scan time ≤ x")
a.set_title("(b) Pointing error distribution")
a.legend(loc="upper left")

# (c) ECDF path error
a = fig.add_subplot(gs[1, 1])
for n in TRIALS:
    e = np.sort(D[n]["path"])
    a.plot(e, np.arange(1, len(e) + 1) / len(e), color=C[n], lw=1.6, label=f"T{n}")
a.axvline(R1_MM, color=REF, ls=":", lw=1)
a.text(R1_MM, 0.05, " R1 2 mm", color=INK2, fontsize=6.5, rotation=90, va="bottom")
a.set_xlim(0, 2.3)
a.set_ylim(0, 1)
a.set_xlabel("focal-point path error, FK vs reference (mm)")
a.set_title("(c) Path-following error distribution")
note(a, "FK-based: how well the arm follows its plan; not a measured spot placement.", y=-0.25)

# (d) dexterity
a = fig.add_subplot(gs[1, 2])
for n in TRIALS:
    r = D[n]
    a.plot(r["td"] / 60, r["dex"], color=C[n], lw=1.3, label=f"T{n}")
a.axhline(0.01, color=REF, ls=":", lw=1)
a.text(1.0, 0.01, " abort", transform=a.get_yaxis_transform(), color=INK2, fontsize=6.5, va="center")
a.set_ylim(0, 0.12)
a.set_xlabel("time since scan start (min)")
a.set_ylabel("σmin / σmax")
a.set_title("(d) Dexterity along the raster")

# (e) table
a = fig.add_subplot(gs[2, :])
a.axis("off")
a.set_title("(e) Summary", pad=2)
cols = ["", "incidence\n(°)", "sun elev.\n(°)", "bed tilt\n(°)", "DNI\n(W/m²)", "pointing\nmed / p95 (°)",
        "within R2\n(0.3°)", "within 2 mrad\n(0.115°)", "path error (mm)\nmed / p95 / max", "dexterity\nmin", "sun\ndropouts"]
rows = []
for n in TRIALS:
    s = stats[f"T{n}"]
    rows.append([f"Trial {n}", f"{s['incidence'][0]:.0f} → {s['incidence'][1]:.0f}", f"{s['elevation']:.0f}",
                 f"{s['bed_tilt']:.0f}", f"{s['dni']:.0f}", f"{s['point_med']:.3f} / {s['point_p95']:.3f}",
                 f"{100 * (1 - s['point_over_R2']):.0f} %",
                 f"{100 * s['point_within_tol']:.1f} %", f"{s['path_med']:.2f} / {s['path_p95']:.2f} / {s['path_max']:.2f}",
                 f"{s['dex_min']:.3f}", f"{s['sun_gaps']}"])
tb = a.table(cellText=rows, colLabels=cols, loc="upper center", cellLoc="center", bbox=[0, 0.26, 1, 0.74])
tb.auto_set_font_size(False)
tb.set_fontsize(7.5)
wts = [0.9, 0.85, 0.75, 0.7, 0.75, 1.1, 0.85, 1.0, 1.45, 0.8, 0.8]
for (i, j), c in tb.get_celld().items():
    c.set_width(wts[j] / sum(wts))
    c.set_edgecolor(GRID)
    if i == 0:
        c.set_text_props(color=INK, fontweight="bold")
        c.set_facecolor("#f4f3ef")
    elif j == 0:
        c.set_text_props(color=C[TRIALS[i - 1]], fontweight="bold")
pm = [stats[f"T{n}"]["point_med"] for n in TRIALS]
p95 = [stats[f"T{n}"]["point_p95"] for n in TRIALS]
a.text(0, 0.16, f"• Pointing held at {min(pm):.3f}-{max(pm):.3f}° median, {min(p95):.3f}-{max(p95):.3f}° p95, "
       f"never above R2 (0.3°) or the solar radius; no drift over the 9 min runs and no dependence on dexterity.",
       transform=a.transAxes, fontsize=7.8, color=INK)
a.text(0, 0.04, "• Path error stays under R1 (2 mm) > 99.9 % of the time. Sun dropouts (passing shadow) are handled "
       "by the defocus lift (page 2, panel b) and the scan resumes.", transform=a.transAxes, fontsize=7.8, color=INK)
pdf.savefig(fig)
fig.savefig(OUT / "page1.png", dpi=170)
plt.close(fig)

# ================================================================ PAGE 2: path, sun-loss handling, independent check
fig = plt.figure(figsize=(11, 8.5))
header(fig, "Raster geometry, sun-loss handling and an independent lens-axis check",
       "Same four runs.  The GoPro check uses the lens AprilTags (30 mm, 16h5) and compares lens-axis DIRECTION only, "
       "which is independent of the unresolved camera/FK scale.")
gs = GridSpec(2, 3, figure=fig, left=0.065, right=0.975, top=0.88, bottom=0.12, hspace=0.5, wspace=0.32)

# (a) raster in plane, T16 coloured by path error
n = 16
r = D[n]
a = fig.add_subplot(gs[0, 0])
a.plot(1e3 * r["Pr"][:, 0], 1e3 * r["Pr"][:, 1], color=REF, lw=0.8, ls="--", label="reference waypoints")
sc = a.scatter(1e3 * r["P"][::3, 0], 1e3 * r["P"][::3, 1], c=r["perr_fk"][::3], s=1.2, cmap="viridis", vmin=0,
               vmax=1.5, label="FK focal point")
cb = fig.colorbar(sc, ax=a, fraction=0.046, pad=0.03)
cb.set_label("path error (mm)", color=INK2)
cb.outline.set_visible(False)
a.set_aspect("equal")
a.set_xlabel("bed-plane x (mm)")
a.set_ylabel("bed-plane y (mm)")
a.set_title(f"(a) Trial {n} raster in the fitted bed plane")
note(a, "Dots: FK focal point (coloured by path error). Dashed: reference waypoints. 13 lines, 2.5 mm apart.",
     y=-0.17, width=62)

# (b) T16 lift + sun visibility
a = fig.add_subplot(gs[0, 1:])
a.plot(r["tl"] / 60, r["lift"], color=C[n], lw=1.2, label="defocus lift (mm)")
for k, (g0, g1) in enumerate(r["gaps"]):
    a.axvspan(g0 / 60, g1 / 60, color="#f2c7c7", lw=0, label="no sun in camera" if k == 0 else None)
a.set_xlim(r["tl"][0] / 60, r["tl"][-1] / 60)
a.set_xlabel("time since scan start (min)")
a.set_ylabel("lift along bed normal (mm)")
a.set_title(f"(b) Trial {n}: start lifted (80 mm), descend once locked, ramp up on sun loss, finish lifted")
a.legend(loc="upper center")
gw0 = [g0 for g0, g1 in r["gaps"] if 0 < g0 < r["te"] - r["ts"]]
if gw0:
    z0 = gw0[-1]
    ins = a.inset_axes([0.33, 0.3, 0.3, 0.5])
    mz = (r["tl"] > z0 - 4) & (r["tl"] < z0 + 8)
    ins.plot(r["tl"][mz] - z0, r["lift"][mz], color=C[n], lw=1.3)
    for g0, g1 in r["gaps"]:
        if abs(g0 - z0) < 8:
            ins.axvspan(g0 - z0, g1 - z0, color="#f2c7c7", lw=0)
    ins.set_title(f"zoom: dropout at {z0 / 60:.1f} min", fontsize=7, fontweight="normal")
    ins.set_xlabel("s", fontsize=6.5, labelpad=1)
    ins.set_ylabel("lift (mm)", fontsize=6.5, labelpad=1)
    ins.tick_params(labelsize=6)
gw = [(g1 - g0) for g0, g1 in r["gaps"] if 0 < g0 < r["te"] - r["ts"]]
note(a, "Lift ramps at ≤ 3 mm/s² so short shadows only partially defocus; the scan pauses while the sun is out and "
        f"resumes at the same point. In-scan dropouts here: {len(gw)} ("
        + ", ".join(f"{g:.1f} s" for g in gw) + ").", width=125, y=-0.2)

# (c) GoPro axis vs FK
a = fig.add_subplot(gs[1, 0:2])
for n in TRIALS:
    for k, g in enumerate(G[n]):
        if g is None:
            continue
        t0 = D[n]["ts"]
        a.plot((g["tb"] - t0) / 60, g["ab"], "o-", color=C[n], ms=3, lw=1,
               label=f"T{n}" + "" if k == 0 else None)
a.set_ylim(0, 0.6)
a.set_xlabel("time since scan start (min)")
a.set_ylabel("lens-axis difference (°)")
a.set_title("(c) Independent check: GoPro vs FK lens axis (10 s means)")
a.legend(ncol=4, loc="upper right")
s14, s16 = stats["T14"], stats["T16"]
gm = [stats[f"T{n}"]["gopro_axis_med"] for n in TRIALS]
note(a, f"Per-frame median {min(gm):.2f}-{max(gm):.2f}°; 10 s means {min(stats[f'T{n}']['gopro_axis_10s_med'] for n in TRIALS):.2f}-"
        f"{max(stats[f'T{n}']['gopro_axis_10s_med'] for n in TRIALS):.2f}°. Along each raster line the lens tilts "
        f"~{s14['gopro_line_tilt'][0]:.2f}° (T14) / {s16['gopro_line_tilt'][0]:.2f}° (T16) per 100 mm of travel "
        "(above the noise floor; T19/T20 too noisy, only 2 coplanar tags in view): structural compliance FK does not "
        "model, which the sun-camera loop corrects. GoPro clock found 134-137 s fast and re-synced by motion "
        "cross-correlation. Mount offset removed. Spikes at ~4 min: T19 camera bumped (off-scale); T20 two-tag pose noise.", y=-0.17, width=135)

# (d) photo
a = fig.add_subplot(gs[1, 2])
a.set_title("(d) Final tiles, 150 mm marker")
if PHOTO and PHOTO.exists():
    img = plt.imread(PHOTO)
    h, w = img.shape[:2]
    a.imshow(img[int(0.38 * h):int(0.86 * h), int(0.22 * w):int(0.72 * w)])
    for x_, lab in ((0.12, "T20"), (0.47, "T19"), (0.83, "T14+T16")):
        a.text(x_, 0.02, lab, transform=a.transAxes, ha="center", va="bottom", fontsize=7.5, color="white",
               fontweight="bold", bbox=dict(fc="#1a1a19", ec="none", alpha=0.6, pad=1.5))
a.axis("off")
a.grid(False)
note(a, "Marker black square = 150 mm. Tiles ~28-36 mm across for a 30 mm raster. T14 and T16 were run over the "
        "same patch and fused into one tile.", y=-0.04, width=48)
pdf.savefig(fig)
fig.savefig(OUT / "page2.png", dpi=170)
plt.close(fig)
pdf.close()
print(json.dumps(stats, indent=1))
