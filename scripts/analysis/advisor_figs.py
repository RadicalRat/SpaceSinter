"""Advisor figure set: data-collection validation and closed-loop performance for Trial 2 (10-30) and Trial 11 (11-03).

    advisor_figs.py OUT_DIR T2_DIR T11_DIR GOPRO_CSV
Each trial dir holds <name>.pkl (extract.py), <name>_fk.npz (fk.py) and <name>_sunerr.npz (sun_error_offline.py),
with name = t2 / t11. GOPRO_CSV is Trial 2's lens_pose_gopro_vs_fk.csv. Writes figN.png/.pdf and stats.json.
"""
import json
import math
import pickle
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation as Rot  # noqa: E402

OUT, T2DIR, T11DIR, CSV = sys.argv[1:5]
C = {"T2": "#2a78d6", "T11": "#eb6834"}
LBL = {"T2": "Trial 2 (Oct 30): sun check at line ends", "T11": "Trial 11 (Nov 3): sun check every waypoint"}
SHORT = {"T2": "Trial 2", "T11": "Trial 11"}
INK, INK2, GRID, SURF = "#1a1a19", "#5b5a56", "#e4e3df", "#ffffff"
REF = "#8a8984"
plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "axes.edgecolor": "#c9c8c3", "axes.labelcolor": INK2,
    "xtick.color": INK2, "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.7,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 9, "axes.titlesize": 9.5,
    "axes.titleweight": "bold", "axes.titlecolor": INK, "axes.titlelocation": "left", "legend.frameon": False,
    "legend.fontsize": 8, "lines.linewidth": 1.6, "savefig.dpi": 220, "font.family": "DejaVu Sans"})

TOL = math.degrees(0.002)          # controller alignment tolerance, 2 mrad
R2 = 0.3                           # pointing requirement (outline v3, atan(2 mm / 0.37 m))
R1 = 2.0                           # placement requirement, mm
SUN_R = 0.2666                     # mean solar angular radius, deg
K_DEFL = 0.006                     # deg/mm, deflection coefficient fitted on Trial 2 (outline v3)
F_LENS = 0.37
STATS = {}


def load(d, name):
    p = pickle.load(open(f"{d}/{name}.pkl", "rb"))
    fk = np.load(f"{d}/{name}_fk.npz")
    se = np.load(f"{d}/{name}_sunerr.npz")
    return p, fk, se


def save(fig, n):
    fig.savefig(f"{OUT}/fig{n}.png", bbox_inches="tight")
    fig.savefig(f"{OUT}/fig{n}.pdf", bbox_inches="tight")
    plt.close(fig)


def panel(ax, letter):
    ax.text(-0.13, 1.06, letter, transform=ax.transAxes, fontsize=11, fontweight="bold", color=INK, va="bottom")


trials = {}
for key, d, name in (("T2", T2DIR, "t2"), ("T11", T11DIR, "t11")):
    p, fk, se = load(d, name)
    tf, F = fk["t"], fk["focal"]
    chk = p["/sinter/sun_error"]
    tc = np.array([r[1] for r in chk]); vc = np.array([[r[3], r[4]] for r in chk]); ec = np.degrees([r[5] for r in chk])
    pos = lambda x: F[np.clip(np.searchsorted(tf, x), 0, len(tf) - 1), :3, 3]  # noqa: E731
    Pc = np.array([pos(x) for x in tc])
    moved = np.r_[np.inf, np.linalg.norm(np.diff(Pc, axis=0), axis=1) * 1e3]
    # a re-check follows an over-tolerance reading (which triggered a correction) without a waypoint move;
    # corrections shift the FK focal point ~1-1.5 mm (Trial 2), waypoint moves are >= 2.4 mm
    post = (moved < 2.0) & np.r_[False, ec[:-1] > TOL]
    T_moved = moved
    o = np.argsort(se["t"]); ts = se["t"][o]; Vs = np.c_[se["vx"], se["vy"]][o]
    good = np.isfinite(Vs).all(1) & (np.degrees(np.arctan(np.linalg.norm(Vs, axis=1))) < 5)
    trials[key] = dict(p=p, tf=tf, F=F, tc=tc, vc=vc, ec=ec, Pc=Pc, post=post, corr_shift=moved[post], ts=ts[good], Vs=Vs[good],
                       scan=(tc[1], tc[-1]))

# ---------------------------------------------------------------- Fig 1: external references (NREL SRRL)
P_SITE_HPA = 816.0  # SRRL, 1829 m


def bennett(h_deg, p_hpa, t_c):
    """Atmospheric refraction (deg) for apparent altitude h, Bennett (1982) with pressure/temperature scaling."""
    r = 1.0 / np.tan(np.radians(h_deg + 7.31 / (h_deg + 4.4))) / 60.0
    return r * (p_hpa / 1010.0) * (283.0 / (273.0 + t_c))


def meinel_dni(zen_deg, h_km=1.829):
    """Clear-sky DNI (W/m2): Meinel & Meinel (1976) with altitude term (Laue 1970), air mass Kasten & Young (1989)."""
    am = 1.0 / (np.cos(np.radians(zen_deg)) + 0.50572 * (96.07995 - zen_deg) ** -1.6364)
    return 1353.0 * ((1 - 0.14 * h_km) * 0.7 ** (am ** 0.678) + 0.14 * h_km)


fig, ax = plt.subplots(1, 2, figsize=(7.4, 2.9), gridspec_kw={"wspace": 0.32})
f1 = {}
for key, T in trials.items():
    eph = T["p"]["/sinter/sun_ephemeris"]
    te = np.array([e[1] for e in eph]); V = np.array([e[3:6] for e in eph])
    el = np.degrees(np.arcsin(V[:, 2] / np.linalg.norm(V, axis=1)))
    rows = []
    for _, s in T["p"]["/srrl/irradiance"]:
        j = json.loads(s)
        tm = j["measurement_unix"]
        if not te[0] <= tm <= te[-1]:
            continue
        el_srrl = 90.0 - j["CR3000 Zen Angle [degrees]"]
        ours = float(np.interp(tm, te, el))
        refr = float(bennett(el_srrl, P_SITE_HPA, j["Tower Dry Bulb Temp [deg C]"]))
        rows.append((tm, ours - el_srrl, ours + refr - el_srrl, j["Direct NIP [W/m^2]"], j["dni_w_m2"],
                     j["CR3000 Zen Angle [degrees]"], j["Total Cloud Cover [%]"]))
    r = np.array(rows)
    t0 = T["scan"][0]
    m = (r[:, 0] - t0) / 60
    ax[0].plot(m, r[:, 1], "o", mfc="none", mec=C[key], ms=5, mew=1.2)
    ax[0].plot(m, r[:, 2], "o", color=C[key], ms=5, mec=SURF, mew=0.8)
    ax[1].plot(m, r[:, 3], "o", color=C[key], ms=5, mec=SURF, mew=0.8, label=f"{SHORT[key]}: measured (NIP)")
    zz = np.linspace(r[:, 5].min(), r[:, 5].max(), 50)
    ax[1].plot(np.interp(zz, r[::-1, 5] if r[0, 5] > r[-1, 5] else r[:, 5], m[::-1] if r[0, 5] > r[-1, 5] else m),
               meinel_dni(zz), "--", color=C[key], lw=1.2, label=f"{SHORT[key]}: clear-sky model")
    f1[key] = dict(n=len(r), raw_med=float(np.median(r[:, 1])), corr_med=float(np.median(r[:, 2])),
                   corr_max=float(np.abs(r[:, 2]).max()), dni=float(np.median(r[:, 3])),
                   dni_model=float(np.median(meinel_dni(r[:, 5]))), nip_vs_rsr=float(np.median(r[:, 3] - r[:, 4])),
                   cloud=(float(r[:, 6].min()), float(r[:, 6].max())), zen=(float(r[:, 5].min()), float(r[:, 5].max())))
ax[0].axhline(0, color=REF, lw=1)
ax[0].set_xlabel("minutes since scan start"); ax[0].set_ylabel("our ephemeris − SRRL (deg)")
ax[0].set_title("Solar elevation vs NREL SRRL logger")
from matplotlib.lines import Line2D  # noqa: E402

ax[0].legend(handles=[Line2D([], [], marker="o", ls="", color=C["T2"], label="Trial 2"),
                      Line2D([], [], marker="o", ls="", color=C["T11"], label="Trial 11"),
                      Line2D([], [], marker="o", ls="", mfc="none", mec=INK2, label="geometric (as computed)"),
                      Line2D([], [], marker="o", ls="", color=INK2, label="+ refraction (Bennett 1982)")],
             loc="center", bbox_to_anchor=(0.64, 0.38), fontsize=7.5)
ax[0].set_ylim(-0.035, 0.015); ax[0].set_xlim(-2, 45)
ax[1].set_xlabel("minutes since scan start"); ax[1].set_ylabel("DNI (W/m²)")
ax[1].set_title("DNI vs clear-sky model")
ax[1].set_ylim(700, 1050); ax[1].legend(loc="lower left", ncol=1)
panel(ax[0], "a"); panel(ax[1], "b")
save(fig, 1)
STATS["fig1"] = f1

# ---------------------------------------------------------------- Fig 2: sun-sensor measurement validation
fig, ax = plt.subplots(1, 2, figsize=(7.4, 2.9), gridspec_kw={"wspace": 0.32})
f2 = {}
ifov = None
for key, T in trials.items():
    ts, Vs = T["ts"], T["Vs"]
    off = []
    for x, e in zip(T["tc"], T["ec"]):
        m = (ts <= x) & (ts > x - 1.0)
        if m.any():
            off.append((e, np.degrees(np.arctan(np.linalg.norm(np.median(Vs[m][-5:], axis=0))))))
    off = np.array(off)
    ax[0].plot(off[:, 0], off[:, 1], "o", color=C[key], ms=4.5, mec=SURF, mew=0.6, label=SHORT[key], alpha=0.9)
    sc = (ts > T["scan"][0]) & (ts < T["scan"][1])
    dv = np.diff(Vs[sc], axis=0)
    noise = np.degrees(1.4826 * np.median(np.abs(dv - np.median(dv, axis=0)), axis=0) / np.sqrt(2))
    jitter = np.degrees(np.linalg.norm(dv, axis=1) / np.sqrt(2))
    ax[1].hist(jitter, bins=np.logspace(-3.5, -0.5, 60), histtype="step", color=C[key], lw=1.5, density=True,
               label=SHORT[key])
    f2[key] = dict(n=len(off), r=float(np.corrcoef(off.T)[0, 1]), med_abs_diff=float(np.median(np.abs(off[:, 0] - off[:, 1]))),
                   noise_x=float(noise[0]), noise_y=float(noise[1]), frames=int(len(ts)))
K = np.load(f"{T11DIR}/t11_sunerr.npz")["K"]
ifov = math.degrees(math.atan(1.0 / K[0, 0]))
lim = [0, 1.2]
ax[0].plot(lim, lim, color=REF, lw=1, ls="--", label="1 : 1")
ax[0].set_xlim(lim); ax[0].set_ylim(lim)
ax[0].set_xlabel("error logged by the controller (deg)"); ax[0].set_ylabel("independent offline re-detection (deg)")
ax[0].set_title("Sun-sensor readings reproduce offline"); ax[0].legend(loc="upper left")
ax[1].set_xscale("log")
for v, lab, ha in ((ifov, "1 pixel ", "right"), (TOL, " control\n tolerance", "left")):
    ax[1].axvline(v, color=REF, ls="--", lw=1)
    ax[1].text(v, ax[1].get_ylim()[1] * 0.95, lab, color=INK2, fontsize=7.5, va="top", ha=ha)
ax[1].set_xlabel("frame-to-frame change (deg, per axis)")
ax[1].set_ylabel("density"); ax[1].set_title("Sun-sensor noise floor (33 ms frames)")
ax[1].legend(loc="upper left")
panel(ax[0], "a"); panel(ax[1], "b")
save(fig, 2)
STATS["fig2"] = dict(f2, ifov_deg=ifov)

# ---------------------------------------------------------------- Fig 3: closed-loop pointing accuracy (ECDF)
fig, ax = plt.subplots(figsize=(7.4, 3.0))
f3 = {}
for key, T in trials.items():
    ts, Vs = T["ts"], T["Vs"]
    k = 28
    sm = np.array([np.convolve(Vs[:, j], np.ones(k) / k, "same") for j in range(2)]).T
    e = np.degrees(np.arctan(np.linalg.norm(sm, axis=1)))
    m = (ts > T["scan"][0] + 5) & (ts < T["scan"][1])
    e = np.sort(e[m])
    y = np.arange(1, len(e) + 1) / len(e)
    ax.plot(e, y, color=C[key], label=f"{LBL[key]}  (median {np.median(e):.2f}°, p95 {np.percentile(e, 95):.2f}°)")
    f3[key] = dict(median=float(np.median(e)), p95=float(np.percentile(e, 95)), max=float(e.max()),
                   frac_over_R2=float((e > R2).mean()), frac_over_tol=float((e > TOL).mean()),
                   frac_over_sunr=float((e > SUN_R).mean()), minutes=float((T["scan"][1] - T["scan"][0]) / 60))
for v, lab, y in ((TOL, "control tolerance (2 mrad)", 0.26), (SUN_R, "solar disk radius", 0.62), (R2, "R2 requirement", 0.62)):
    ax.axvline(v, color=REF, ls="--", lw=1)
    ax.text(v + 0.006, y, lab, color=INK2, fontsize=7.5, rotation=90, va="bottom", ha="left")
ax.set_xlim(0, 1.3); ax.set_ylim(0, 1.0)
ax.set_xlabel("lens pointing error, sun sensor, 1 s average (deg)"); ax.set_ylabel("fraction of scan time ≤ x")
ax.set_title("Closed-loop pointing accuracy over the whole scan")
ax.legend(loc="center right", bbox_to_anchor=(1.0, 0.42), fontsize=8)
save(fig, 3)
STATS["fig3"] = f3

# ---------------------------------------------------------------- Fig 4: realignment (R6) drift model
f4 = {}
for key, T in trials.items():
    eph = T["p"]["/sinter/sun_ephemeris"]
    te = np.array([e[1] for e in eph]); V = np.array([e[3:6] for e in eph])
    V = V / np.linalg.norm(V, axis=1)[:, None]
    w_sun = np.degrees(np.median(np.arccos(np.clip((V[1:] * V[:-1]).sum(1), -1, 1)) / np.diff(te)))
    tf, F = T["tf"], T["F"]
    rows = []
    tc, vc, post = T["tc"], T["vc"], T["post"]
    for kk in range(1, len(tc)):
        if post[kk]:
            continue
        # reference reading: the post-correction re-check if one precedes, else the previous reading (uncorrected)
        prev = kk - 1
        corrected_before = np.degrees(math.atan(np.linalg.norm(vc[prev]))) > TOL and not post[prev]
        if corrected_before:
            continue  # correction result not observed by a re-check (Trial 11): skip
        dt_ = tc[kk] - tc[prev]
        m = (tf >= tc[prev]) & (tf <= tc[kk])
        ds = 1e3 * np.linalg.norm(F[m, :3, 3][-1] - F[m, :3, 3][0]) if m.sum() > 1 else 0.0  # net travel (path sums add FK noise)
        grow = np.degrees(np.linalg.norm(np.arctan(vc[kk]) - np.arctan(vc[prev])))
        rows.append((dt_, ds, grow))
    r = np.array(rows)
    a, b = w_sun * r[:, 0], K_DEFL * r[:, 1]
    f4[key] = dict(n=len(r), w_sun=float(w_sun), dt_med=float(np.median(r[:, 0])), ds_med=float(np.median(r[:, 1])),
                   grow_med=float(np.median(r[:, 2])), lo=float(np.median(np.abs(a - b))), hi=float(np.median(a + b)))
    T["r6"] = (r, a, b)
fig, ax = plt.subplots(figsize=(7.4, 3.2))
for key in ("T2", "T11"):
    r, a, b = trials[key]["r6"]
    if key == "T2":
        keep = r[:, 1] > 50          # full raster lines only (drops the lead-in check)
        r, a, b = r[keep], a[keep], b[keep]
    lo, hi = np.median(np.abs(a - b)), np.median(a + b)
    ax.axvspan(lo, hi, color=C[key], alpha=0.10, lw=0)
    for v in (lo, hi):
        ax.axvline(v, color=C[key], ls="--", lw=1)
    g_ = np.sort(r[:, 2])
    ax.plot(g_, np.arange(1, len(g_) + 1) / len(g_), color=C[key], drawstyle="steps-post", lw=1.8,
            label=f"{SHORT[key]}: ~{np.median(r[:, 0]):.0f} s and ~{np.median(r[:, 1]):.0f} mm between checks "
                  f"(median drift {np.median(g_):.2f}°, n = {len(g_)})")
    f4[key].update(lo_plot=float(lo), hi_plot=float(hi), n_plot=int(len(g_)))
ax.axvline(R2, color=REF, ls=":", lw=1.2)
ax.text(R2 * 0.96, 0.9, "R2", color=INK2, fontsize=8, ha="right")
ax.text(0.0069, 1.02, "Trial 11 model range", color=C["T11"], fontsize=7.5, transform=ax.get_xaxis_transform(), va="bottom")
ax.text(0.37, 1.02, "Trial 2 model range", color=C["T2"], fontsize=7.5, transform=ax.get_xaxis_transform(), va="bottom")
ax.set_xscale("log"); ax.set_xlim(0.004, 2.0); ax.set_ylim(0, 1)
ax.set_xticks([0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2]); ax.set_xticklabels(["0.005", "0.01", "0.02", "0.05", "0.1", "0.2", "0.5", "1", "2"])
ax.set_xlabel("pointing drift between consecutive sun checks (deg, log scale)")
ax.set_ylabel("fraction of check intervals ≤ x")
ax.set_title("Drift between checks vs the sun ± deflection model (shaded: model range per trial)", pad=16)
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), fontsize=7.8, ncol=1)
save(fig, 4)
STATS["fig4"] = f4

# ---------------------------------------------------------------- Fig 5: independent verification (Trial 2 GoPro)
g = np.genfromtxt(CSV, delimiter=",", names=True)
tg = g["ros_time"]
zg = Rot.from_quat(np.c_[g["gopro_qx"], g["gopro_qy"], g["gopro_qz"], g["gopro_qw"]]).as_matrix()[:, :, 2]
zf = Rot.from_quat(np.c_[g["fk_qx"], g["fk_qy"], g["fk_qz"], g["fk_qw"]]).as_matrix()[:, :, 2]
T = trials["T2"]
eph = T["p"]["/sinter/sun_ephemeris"]; te = np.array([e[1] for e in eph]); V = np.array([e[3:6] for e in eph])
sun = np.array([np.interp(tg, te, V[:, k]) for k in range(3)]).T; sun /= np.linalg.norm(sun, axis=1)[:, None]
tc, ec, post = T["tc"], T["ec"], T["post"]
rows = []
for kk in range(1, len(tc) - 1):
    if not post[kk] or post[kk + 1]:
        continue
    a_, b_ = tc[kk] + 1, tc[kk + 1] - 1
    m = (tg > a_) & (tg < b_)
    if m.sum() < 30:
        continue
    i0, i1 = np.where(m)[0][[0, -1]]
    s0, s1 = slice(i0, i0 + 10), slice(i1 - 9, i1 + 1)
    tilt_g = np.degrees(np.linalg.norm(zg[s1].mean(0) - zg[s0].mean(0)))
    tilt_f = np.degrees(np.linalg.norm(zf[s1].mean(0) - zf[s0].mean(0)))
    pred_g = np.degrees(np.linalg.norm((zg[s1].mean(0) - sun[i1]) - (zg[s0].mean(0) - sun[i0])))
    pred_f = np.degrees(np.linalg.norm((zf[s1].mean(0) - sun[i1]) - (zf[s0].mean(0) - sun[i0])))
    travel = 1e3 * np.linalg.norm([g["fk_x"][i1] - g["fk_x"][i0], g["fk_y"][i1] - g["fk_y"][i0], g["fk_z"][i1] - g["fk_z"][i0]])
    rows.append((travel, tilt_g, tilt_f, pred_g, pred_f, ec[kk + 1]))
r = np.array(rows)
fig, ax = plt.subplots(1, 2, figsize=(7.8, 3.0), gridspec_kw={"wspace": 0.45})
ax[0].plot(r[:, 0], r[:, 1], "o", color=C["T2"], ms=5, mec=SURF, mew=0.6, label="external camera (AprilTags)")
ax[0].plot(r[:, 0], r[:, 2], "s", color=INK2, ms=4.5, mec=SURF, mew=0.6, label="forward kinematics (joint encoders)")
xx = np.array([0, 115])
for col, k_, ls in ((C["T2"], 1, "--"), (INK2, 2, ":")):
    full = r[:, 0] > 50
    sl = np.sum(r[full, 0] * r[full, k_]) / np.sum(r[full, 0] ** 2)
    ax[0].plot(xx, sl * xx, ls, color=col, lw=1)
    ax[0].text(xx[1] + 2, sl * xx[1] - 0.05, f"{100 * sl:.2f}°/100 mm", color=col, fontsize=7.5, va="top")
ax[0].set_xlim(-5, 140)
ax[0].set_xlabel("travel along the raster line (mm)"); ax[0].set_ylabel("lens-axis rotation over the line (deg)")
ax[0].set_title("Lens tilt during a line: camera vs FK"); ax[0].legend(loc="upper left")
ax[0].set_ylim(0, max(1.0, r[:, 1].max() * 1.15))
lim = [0, max(1.2, r[:, 3:].max() * 1.1)]
rg = np.corrcoef(r[:, 3], r[:, 5])[0, 1]; rf = np.corrcoef(r[:, 4], r[:, 5])[0, 1]
rmse_g = float(np.sqrt(np.mean((r[:, 3] - r[:, 5]) ** 2))); rmse_f = float(np.sqrt(np.mean((r[:, 4] - r[:, 5]) ** 2)))
ax[1].plot(lim, lim, color=REF, ls="--", lw=1)
ax[1].plot(r[:, 5], r[:, 3], "o", color=C["T2"], ms=5, mec=SURF, mew=0.6, label=f"camera + ephemeris  (r = {rg:.2f})")
ax[1].plot(r[:, 5], r[:, 4], "s", color=INK2, ms=4.5, mec=SURF, mew=0.6, label=f"FK + ephemeris  (r = {rf:.2f})")
ax[1].set_xlim(lim); ax[1].set_ylim(lim)
ax[1].set_xlabel("sun-sensor error at line end (deg)"); ax[1].set_ylabel("predicted error (deg)")
ax[1].set_title("Which model explains the sun sensor?"); ax[1].legend(loc="upper left")
panel(ax[0], "a"); panel(ax[1], "b")
save(fig, 5)
fit = np.polyfit(r[:, 0], r[:, 1], 1)
STATS["fig5"] = dict(n_lines=len(r), r_camera=float(rg), r_fk=float(rf), rmse_camera=rmse_g, rmse_fk=rmse_f,
                     tilt_camera_med=float(np.median(r[:, 1])), tilt_fk_med=float(np.median(r[:, 2])),
                     travel_med=float(np.median(r[:, 0])), slope_deg_per_100mm=float(100 * np.median(r[:, 1] / r[:, 0])))

# ---------------------------------------------------------------- Fig 6: onboard sensor vs external ground truth over time (Trial 2)
T = trials["T2"]
ts2, Vs2 = T["ts"], T["Vs"]
posts = T["tc"][T["post"]]
segs = list(zip(posts, list(posts[1:]) + [T["scan"][1] + 60]))
def smooth_cols(X, k):
    return np.array([np.convolve(X[:, j], np.ones(k) / k, "same") for j in range(X.shape[1])]).T
Vsm = smooth_cols(np.arctan(Vs2), 28)                       # sun sensor, 1 s
Eg = smooth_cols(zg - sun, 15); Ef = smooth_cols(zf - sun, 15)  # GoPro / FK minus sun, 1.5 s
t_s, y_s, t_g, y_g, y_f = [], [], [], [], []
for p0, p1 in segs:
    ms = (ts2 >= p0) & (ts2 < p1); mg = (tg >= p0) & (tg < p1)
    a_s = (ts2 >= p0) & (ts2 < p0 + 0.7); a_g = (tg >= p0) & (tg < p0 + 1.0)
    if ms.sum() < 10 or mg.sum() < 10 or not a_s.any() or not a_g.any():
        continue
    t_s.append(ts2[ms]); y_s.append(np.degrees(np.linalg.norm(Vsm[ms] - np.median(Vsm[a_s], 0), axis=1)))
    t_g.append(tg[mg])
    y_g.append(np.degrees(np.linalg.norm(Eg[mg] - Eg[a_g].mean(0), axis=1)))
    y_f.append(np.degrees(np.linalg.norm(Ef[mg] - Ef[a_g].mean(0), axis=1)))
    t_s.append([np.nan]); y_s.append([np.nan]); t_g.append([np.nan]); y_g.append([np.nan]); y_f.append([np.nan])
t_s, y_s, t_g, y_g, y_f = map(np.concatenate, (t_s, y_s, t_g, y_g, y_f))
ok = np.isfinite(t_g) & np.isfinite(y_g)
ys_on_g = np.interp(t_g[ok], t_s[np.isfinite(t_s)], np.nan_to_num(y_s[np.isfinite(t_s)]))
d_g = y_g[ok] - ys_on_g; d_f = y_f[ok] - ys_on_g
t0 = T["scan"][0]
fig, ax = plt.subplots(2, 1, figsize=(7.6, 5.0), gridspec_kw={"hspace": 0.5})
mid = posts[len(posts) // 2]
for axx, (lo_, hi_) in zip(ax, [(T["scan"][0], T["scan"][1]), (mid - 10, mid + 290)]):
    for tt, yy, col, lw, lab in ((t_f := t_g, y_f, INK2, 1.1, "robot model (FK + ephemeris)"),
                                 (t_g, y_g, C["T2"], 1.4, "external camera (GoPro tags + ephemeris), ground truth"),
                                 (t_s, y_s, "#1baf7a", 1.4, "onboard sun sensor")):
        m = (tt >= lo_) & (tt <= hi_) | ~np.isfinite(tt)
        axx.plot((tt[m] - t0) / 60, yy[m], color=col, lw=lw if axx is ax[0] else lw + 0.4, label=lab)
    axx.axhline(R2, color=REF, ls=":", lw=1)
    axx.text(1.005, R2, "R2", transform=axx.get_yaxis_transform(), color=INK2, fontsize=7.5, va="center")
    axx.set_ylabel("pointing error since\nlast correction (deg)"); axx.set_ylim(0, 1.3)
    axx.set_xlim((lo_ - t0) / 60, (hi_ - t0) / 60)
ax[0].set_title("Trial 2: pointing error over the whole scan, three independent estimates")
ax[1].set_title("5-minute zoom: the onboard sensor tracks the external camera; the robot model does not")
ax[1].set_xlabel("minutes since scan start")
ax[0].legend(loc="upper left", ncol=3, fontsize=7.5, bbox_to_anchor=(0, 1.0))
save(fig, 6)
STATS["fig6"] = dict(n=int(ok.sum()), rms_camera_minus_sensor=float(np.sqrt(np.nanmean(d_g ** 2))),
                     rms_fk_minus_sensor=float(np.sqrt(np.nanmean(d_f ** 2))),
                     r_camera_sensor=float(np.corrcoef(y_g[ok], ys_on_g)[0, 1]),
                     r_fk_sensor=float(np.corrcoef(y_f[ok], ys_on_g)[0, 1]),
                     med_abs_camera=float(np.nanmedian(np.abs(d_g))), med_abs_fk=float(np.nanmedian(np.abs(d_f))),
                     segments=len(segs))

# ---------------------------------------------------------------- Fig 7: placement (controller side)
fig, ax = plt.subplots(figsize=(7.4, 2.8))
f6 = {}
for key, T in trials.items():
    P = np.array([pp for pp, qq in T["p"]["/sinter/raster_path"][-1][2]])
    d = np.array([np.min(np.linalg.norm(P - x, axis=1)) for x in T["Pc"]]) * 1e3
    d = np.sort(d[d < 10])
    ax.plot(d, np.arange(1, len(d) + 1) / len(d), color=C[key], drawstyle="steps-post",
            label=f"{SHORT[key]}  (median {np.median(d):.2f} mm, p95 {np.percentile(d, 95):.2f} mm, n = {len(d)})")
    f6[key] = dict(n=len(d), median=float(np.median(d)), p95=float(np.percentile(d, 95)), max=float(d.max()))
ax.axvline(R1, color=REF, ls="--", lw=1); ax.text(R1 + 0.03, 0.95, "R1 placement\nrequirement", color=INK2, fontsize=7.5, va="top")
ax.set_xlim(0, 2.6); ax.set_ylim(0, 1)
ax.set_xlabel("focal point (FK) to commanded raster waypoint at each sun check (mm)")
ax.set_ylabel("fraction of checks ≤ x"); ax.set_title("Waypoint placement as the controller sees it")
ax.legend(loc="center", bbox_to_anchor=(0.43, 0.4), fontsize=8)
save(fig, 7)
STATS["fig7"] = f6

# ---------------------------------------------------------------- Logging health table
health = {}
nominal = {"/joint_states": 100.0, "/tf": None, "/camera/image_raw/compressed": 30.0, "/sinter/sun_ephemeris": 1.0,
           "/srrl/irradiance": 1 / 60.0, "/joint_trajectory_admittance_controller/controller_state": None}
for key, T in trials.items():
    p = T["p"]; rows = {}
    span = T["tf"][-1] - T["tf"][0]
    for top, nom in nominal.items():
        v = p.get(top, [])
        if not len(v):
            continue
        t = np.array([x if isinstance(x, float) else x[0] for x in v])
        t = np.sort(np.unique(t))
        rate = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 else 0
        gaps = np.diff(t)
        rows[top] = dict(n=len(v), rate=float(rate), nominal=nom, max_gap=float(gaps.max()) if len(gaps) else None)
    rows["sun frames with a detected sun"] = dict(frac=float(len(T["ts"]) / len(p["/camera/image_raw/compressed"])))
    rows["sun_error checks"] = dict(n=len(T["tc"]))
    rows["duration_s"] = float(span)
    health[key] = rows
STATS["health"] = health
json.dump(STATS, open(f"{OUT}/stats.json", "w"), indent=1, default=float)
print(json.dumps(STATS, indent=1, default=float)[:6000])
