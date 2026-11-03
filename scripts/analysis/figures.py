"""Trial 2 figures: pointing error timeline, sun-error prediction, tilt vs position, error budget."""
import datetime as dt
import pickle
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

S, OUT = sys.argv[1], sys.argv[2]
GOPRO, FK, SUNCAM = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
R2 = 0.14
F_LENS = 0.37
plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "axes.edgecolor": GRID, "axes.labelcolor": INK2,
    "xtick.color": INK2, "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11,
    "axes.titleweight": "bold", "axes.titlecolor": INK, "legend.frameon": False, "lines.linewidth": 2,
})

d = pickle.load(open(f"{S}/t2.pkl", "rb"))
p = np.load(f"{S}/t2_pose.npz")
t, Wm, Wf = p["t"], p["world_meas"], p["world_fk"]
eph = d["/sinter/sun_ephemeris"]
te = np.array([e[1] for e in eph])
Sv = np.array([e[3:6] for e in eph])
se = d["/sinter/sun_error"]
tse = np.array([r[0] for r in se])
ese = np.degrees([r[5] for r in se])
mdt = lambda x: dt.datetime.fromtimestamp(x, dt.timezone(dt.timedelta(hours=-6)))  # noqa: E731

# Continuous pointing error: lens axis minus sun direction (small-angle vector, world). The constant part
# (heading/level/mount bias, invisible to the closed loop) is removed using the instants right after each
# correction, when the sun camera says the lens is on the sun.
sun = np.array([np.interp(t, te, Sv[:, k]) for k in range(3)]).T
sun /= np.linalg.norm(sun, axis=1)[:, None]


def pointing_error(W):
    """Error accumulated since the last correction: each segment starts from the sun camera's
    post-correction reading. (The GoPro also drifts ~0.5 deg slowly against FK over the run while the
    sun camera stays on the sun, so a single constant bias would mostly show that artifact.)"""
    e = W[:, :3, 2] - sun
    out = np.full(len(t), np.nan)
    post = [tc for tc in tse[1::2] if t[0] < tc < t[-1]]
    for a, b in zip(post, post[1:] + [t[-1]]):
        m = (t >= a) & (t < b)
        if m.any():
            i0 = np.argmin(abs(t - a))
            out[m] = np.degrees(np.linalg.norm(e[m] - e[i0], axis=1))
    return out


err_cam = pointing_error(Wm)
err_fk = pointing_error(Wf)
k = 15  # 1.5 s smoothing of the 10 Hz GoPro poses
smooth = lambda x: np.convolve(np.nan_to_num(x), np.ones(k) / k, "same")  # noqa: E731
run = (t > tse[1]) & (t < tse[-1] + 20)

# --- Fig 1: pointing error vs time --------------------------------------------------------------
fig, axes = plt.subplots(2, 1, figsize=(10, 6.2), gridspec_kw={"height_ratios": [1, 1]})
for ax, (a, b) in zip(axes, [(t[run][0], t[run][-1]), (mdt(tse[20]).timestamp() - 5, mdt(tse[20]).timestamp() + 300)]):
    m = run & (t >= a) & (t <= b)
    x = [mdt(v) for v in t[m]]
    ax.axhspan(0, R2, color=GOPRO, alpha=0.08, lw=0)
    ax.plot(x, smooth(err_cam)[m] if ax is axes[1] else smooth(err_cam)[m], color=GOPRO, lw=1.5 if ax is axes[0] else 2,
            label="Real lens axis (GoPro tags)")
    ax.plot(x, smooth(err_fk)[m], color=FK, lw=1.5 if ax is axes[0] else 2, label="What FK believes")
    ms = (tse >= a) & (tse <= b)
    ax.plot([mdt(v) for v in tse[ms]], ese[ms], "o", color=SUNCAM, ms=6, mec=SURF, mew=1.5,
            label="Sun camera (before/after each correction)")
    ax.set_ylabel("pointing error (deg)")
    ax.set_ylim(0, 1.3)
    ax.text(x[0], R2 + 0.03, "R2 = 0.14°", color=INK2, fontsize=9)
    ax.xaxis.set_major_formatter(matplotlib.dates.DateFormatter("%H:%M", tz=dt.timezone(dt.timedelta(hours=-6))))
axes[0].set_title("Sun-pointing error built up since the last correction, Trial 2 (2026-10-30)")
axes[0].legend(loc="upper right", ncol=3, fontsize=9)
axes[1].set_title("Zoom: four raster lines (one correction per line, every ~70 s)")
axes[1].set_xlabel("time (MDT)")
sec = axes[1].secondary_yaxis("right", functions=(lambda d_: 1000 * F_LENS * np.tan(np.radians(d_)),
                                                   lambda mm: np.degrees(np.arctan(mm / 1000 / F_LENS))))
sec.set_ylabel("spot offset f·tanθ (mm)")
sec.tick_params(colors=INK2)
fig.tight_layout()
fig.savefig(f"{OUT}/fig_pointing_error.png", dpi=160)

# --- Fig 2: predicted vs measured sun error at line ends ---------------------------------------
rows = []
for j in range(3, len(se) - 1, 2):
    m = np.where((t > tse[j] + 1) & (t < tse[j + 1] - 1))[0]
    if len(m) < 20:
        continue
    s0, s1 = sun[m[:10]].mean(0), sun[m[-10:]].mean(0)
    dc = Wm[m[-10:], :3, 2].mean(0) - Wm[m[:10], :3, 2].mean(0)
    df = Wf[m[-10:], :3, 2].mean(0) - Wf[m[:10], :3, 2].mean(0)
    rows.append((ese[j + 1], np.degrees(np.linalg.norm(dc - (s1 - s0))), np.degrees(np.linalg.norm(df - (s1 - s0)))))
rows = np.array(rows)
fig, ax = plt.subplots(figsize=(5.6, 5.2))
ax.plot([0, 1.2], [0, 1.2], color=INK2, lw=1)
ax.text(1.02, 1.1, "y = x", color=INK2, fontsize=9)
for col, c, lab in ((1, GOPRO, "GoPro tilt + sun motion"), (2, FK, "FK tilt + sun motion")):
    r = np.corrcoef(rows[:, 0], rows[:, col])[0, 1]
    rms = np.sqrt(np.mean((rows[:, col] - rows[:, 0]) ** 2))
    ax.plot(rows[:, 0], rows[:, col], "o", color=c, ms=7, mec=SURF, mew=1.5, label=f"{lab}  (r = {r:.2f}, rms {rms:.2f}°)")
ax.set_xlabel("sun camera error at end of line, measured (deg)")
ax.set_ylabel("predicted error at end of line (deg)")
ax.set_xlim(0, 1.2)
ax.set_ylim(0, 1.2)
ax.set_aspect("equal")
ax.set_title("Only the externally measured tilt\npredicts what the sun camera sees")
ax.legend(loc="upper left", fontsize=9)
fig.tight_layout()
fig.savefig(f"{OUT}/fig_predicted_vs_measured.png", dpi=160)

# --- Fig 3: lens tilt (GoPro minus FK) vs position along the scan line -----------------------
path = np.array([q for q, _ in d["/sinter/raster_path"][1][2]])
u = path[1] - path[0]
u /= np.linalg.norm(u)
pos = 1000 * ((Wf[:, :3, 3] - Wf[:, :3, 3].mean(0)) @ u)
dz = Wm[:, :3, 2] - Wf[:, :3, 2]
on_line = np.zeros(len(t), bool)
direction = np.zeros(len(t))
for j in range(3, len(se) - 1, 2):
    m = (t > tse[j] + 1) & (t < tse[j + 1] - 1)
    on_line |= m
    idx = np.where(m)[0]
    if len(idx) > 1:
        direction[idx] = np.sign(pos[idx[-1]] - pos[idx[0]])
# Project the tilt difference on its dominant direction so the sign shows which way the lens leans.
dzc = dz[on_line] - dz[on_line].mean(0)
axis_dir = np.linalg.svd(dzc, full_matrices=False)[2][0]
tilt = np.degrees(dz @ axis_dir)
# Each line referenced to its own mean, removing the slow GoPro-vs-FK drift.
for j in range(3, len(se) - 1, 2):
    m = (t > tse[j] + 1) & (t < tse[j + 1] - 1)
    if m.any():
        # Remove the line's intercept, keep its slope: subtract the tilt at the line's mean position.
        a_, b_ = np.polyfit(pos[m], tilt[m], 1)
        tilt[m] -= b_
fig, ax = plt.subplots(figsize=(7, 4.6))
for sgn, c, lab in ((1, GOPRO, "moving +u (up the bed)"), (-1, INK2, "moving −u (down the bed)")):
    m = on_line & (direction == sgn)
    ax.plot(pos[m][::3], smooth(tilt)[m][::3], ".", color=c, ms=2.5, alpha=0.5, label=lab)
fit = np.polyfit(pos[on_line], tilt[on_line], 1)
xx = np.array([pos[on_line].min(), pos[on_line].max()])
ax.plot(xx, np.polyval(fit, xx), color=INK, lw=1.5)
ax.text(xx[1], np.polyval(fit, xx[1]), f"  {100 * fit[0]:.2f}° per 100 mm", color=INK, va="center", fontsize=9)
ax.set_xlabel("focal-point position along the scan line (mm)")
ax.set_ylabel("lens tilt not seen by FK (deg)")
ax.set_title("The lens tilt is a repeatable function of arm pose")
ax.legend(loc="upper left", fontsize=9, markerscale=4)
fig.tight_layout()
fig.savefig(f"{OUT}/fig_tilt_vs_position.png", dpi=160)

# --- Fig 4: error budget per line (where the pre-correction error comes from) -------------------
fig, ax = plt.subplots(figsize=(7, 3.8))
sun_drift = []
tilt_only = []
for j in range(3, len(se) - 1, 2):
    m = np.where((t > tse[j] + 1) & (t < tse[j + 1] - 1))[0]
    if len(m) < 20:
        continue
    sun_drift.append(np.degrees(np.linalg.norm(sun[m[-10:]].mean(0) - sun[m[:10]].mean(0))))
    dc = Wm[m[-10:], :3, 2].mean(0) - Wm[m[:10], :3, 2].mean(0)
    df = Wf[m[-10:], :3, 2].mean(0) - Wf[m[:10], :3, 2].mean(0)
    tilt_only.append(np.degrees(np.linalg.norm(dc - df)))
labels = ["Controller tracking\n(commanded vs FK)", "Sun drift\nover one line", "Pose-dependent\nlens tilt", "Sun-camera error\nbefore correction"]
track_deg = np.degrees(np.arctan(0.37e-3 / F_LENS))  # p95 focal-point tracking 0.37 mm expressed as an angle
vals = [track_deg, np.median(sun_drift), np.median(tilt_only), np.median(ese[2::2])]
bars = ax.barh(labels[::-1], vals[::-1], height=0.5, color=GOPRO)
for b_, v in zip(bars, vals[::-1]):
    ax.text(v + 0.015, b_.get_y() + b_.get_height() / 2, f"{v:.2f}°  ({1000 * F_LENS * np.tan(np.radians(v)):.1f} mm)",
            va="center", color=INK, fontsize=9)
ax.axvline(R2, color=INK2, lw=1)
ax.text(R2 + 0.01, 3.45, "R2", color=INK2, fontsize=9)
ax.set_xlim(0, 1.05)
ax.set_xlabel("median per line (deg; spot offset in mm at f = 0.37 m)")
ax.set_title("Error budget per ~105 mm raster line")
ax.grid(axis="y", visible=False)
fig.tight_layout()
fig.savefig(f"{OUT}/fig_error_budget.png", dpi=160)
print("tilt slope deg/100mm:", 100 * fit[0], "bars:", np.round(vals, 3))
