"""Camera-measured lens pose vs FK for Trial 2: overall residuals, per-line axis drift, plots."""
import datetime as dt
import pickle
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation as Rot  # noqa: E402

S = sys.argv[1]
p = np.load(f"{S}/t2_pose.npz")
d = pickle.load(open(f"{S}/t2.pkl", "rb"))
t, Wm, Wf, X = p["t"], p["world_meas"], p["world_fk"], p["X"]
loc = lambda x: dt.datetime.fromtimestamp(x, dt.timezone(dt.timedelta(hours=-6))).strftime("%H:%M:%S")  # noqa: E731

# Residuals: rotation in world, position split into camera-depth vs camera-lateral.
dR = Rot.from_matrix(np.einsum("nij,nkj->nik", Wm[:, :3, :3], Wf[:, :3, :3])).as_rotvec()
dP = Wm[:, :3, 3] - Wf[:, :3, 3]
dP_cam = np.einsum("ij,nj->ni", X[:3, :3], dP)
axis = np.degrees(np.arccos(np.clip(np.einsum("ni,ni->n", Wm[:, :3, 2], Wf[:, :3, 2]), -1, 1)))
print(f"{len(t)} poses {loc(t[0])}-{loc(t[-1])}")
print(f"lens-axis residual (deg): median {np.median(axis):.3f} p95 {np.percentile(axis, 95):.3f}")
print("position residual in camera frame (mm), median |x| |y| |z(depth)|:",
      (1000 * np.median(np.abs(dP_cam), axis=0)).round(2), " p95:",
      (1000 * np.percentile(np.abs(dP_cam), 95, axis=0)).round(2))

# Per raster line: axis rotation start->end from camera vs FK, and the sun-camera error at the line end.
se = d["/sinter/sun_error"]
tse = np.array([r[0] for r in se])
print("\nline      dur  move(mm)  FK axis rot  camera axis rot  cam-vs-FK  sun_err@end")
rows = []
for k in range(3, len(se) - 1, 2):
    a, b = tse[k] + 1, tse[k + 1] - 1
    m = (t > a) & (t < b)
    if m.sum() < 20:
        continue
    i0, i1 = np.where(m)[0][[0, -1]]
    n0 = slice(i0, i0 + 10)
    n1 = slice(i1 - 9, i1 + 1)
    zc0, zc1 = Wm[n0, :3, 2].mean(0), Wm[n1, :3, 2].mean(0)
    zf0, zf1 = Wf[n0, :3, 2].mean(0), Wf[n1, :3, 2].mean(0)
    ang = lambda u, v: np.degrees(np.arccos(np.clip(u @ v / np.linalg.norm(u) / np.linalg.norm(v), -1, 1)))  # noqa: E731
    move = 1000 * np.linalg.norm(Wf[i1, :3, 3] - Wf[i0, :3, 3])
    rel_c, rel_f = zc1 - zc0, zf1 - zf0
    diff = np.degrees(np.linalg.norm(rel_c - rel_f))
    rows.append((a, move, ang(zf0, zf1), ang(zc0, zc1), diff, np.degrees(se[k + 1][5])))
    print(f"{loc(a)} {b - a:4.0f}s {move:7.1f}  {ang(zf0, zf1):9.3f}  {ang(zc0, zc1):13.3f}  {diff:9.3f}  {np.degrees(se[k + 1][5]):9.3f}")
rows = np.array(rows)
if len(rows):
    print(f"\nmedian per-line axis drift: FK {np.median(rows[:, 2]):.3f}, camera {np.median(rows[:, 3]):.3f}, "
          f"camera-minus-FK {np.median(rows[:, 4]):.3f} deg; corr(camera-minus-FK, sun_err) "
          f"{np.corrcoef(rows[:, 4], rows[:, 5])[0, 1]:.2f}")

fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
tm = (t - t[0]) / 60
ax[0].plot(tm, axis, ".", ms=1)
ax[0].set_ylabel("lens axis: camera vs FK (deg)")
ax[0].set_ylim(0, np.percentile(axis, 99.5))
for k, lab in enumerate("xyz"):
    ax[1].plot(tm, 1000 * dP_cam[:, k], ".", ms=1, label=f"cam {lab}" + (" (depth)" if lab == "z" else ""))
ax[1].set_ylabel("focal pos residual (mm)")
ax[1].legend(markerscale=8)
ax[2].plot((tse - t[0]) / 60, np.degrees([r[5] for r in se]), "o-", ms=3)
ax[2].set_ylabel("sun_error (deg)")
ax[2].set_xlabel(f"minutes since {loc(t[0])} MDT")
fig.tight_layout()
fig.savefig(f"{S}/t2_pose_vs_fk.png", dpi=110)
print("plot", f"{S}/t2_pose_vs_fk.png")
