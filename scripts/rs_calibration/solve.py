"""Solve the RealSense mount rotation (and translation along the plane normals) from multi-pose plane views.

    python3 solve.py SNAP_DIR [--heading 125] [--lat 39.7555 --lon -105.2211]

Every snapshot (from capture.py) has planes (ground, bucket sand, ...) that are fixed in the world.
With the right mount transform, the same physical plane must come out with the same world normal
and offset from every arm pose. A wrong mount rotation makes them disagree, by an amount that
changes with the pose. So the solver looks for the correction dT (applied in the ee_realsense_link
frame) that makes the planes agree best. It starts from the current mount and from 180 deg flips
about each link axis, so a flipped mount cannot hide in a local minimum.

Needs numpy + scipy. Prints the corrected ee_realsense_joint origin for the URDF.
"""
import argparse
import datetime as dt
import glob
import math
import os

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

# Current URDF: ee_end_effector_link -> ee_realsense_link (end_effector_urdf_macro.xacro).
URDF_XYZ = np.array([0.031, -0.035, 0.14495])
URDF_RPY = np.array([math.pi, -1.5708, 0.0])


def ransac_planes(P, max_planes=4, tol=0.008, min_frac=0.06, iters=300, seed=0):
    rng = np.random.default_rng(seed)
    planes, rest = [], P
    min_pts = max(800, int(min_frac * len(P)))
    for _ in range(max_planes):
        if len(rest) < min_pts:
            break
        best = None
        for _ in range(iters):
            a, b, c = rest[rng.choice(len(rest), 3, replace=False)]
            n = np.cross(b - a, c - a)
            nn = np.linalg.norm(n)
            if nn < 1e-9:
                continue
            n /= nn
            k = np.count_nonzero(np.abs((rest - a) @ n) < tol)
            if best is None or k > best[0]:
                best = (k, n, a)
        if best is None or best[0] < min_pts:
            break
        inl = np.abs((rest - best[2]) @ best[1]) < tol
        Q = rest[inl]
        c = Q.mean(0)
        n = np.linalg.svd(Q - c, full_matrices=False)[2][2]
        inl = np.abs((rest - c) @ n) < tol  # refit pass
        Q = rest[inl]
        c = Q.mean(0)
        n = np.linalg.svd(Q - c, full_matrices=False)[2][2]
        if n @ c > 0:  # orient the normal toward the camera
            n = -n
        rms = np.sqrt(np.mean(((Q - c) @ n) ** 2))
        planes.append(dict(n=n, c=c, count=len(Q), rms=rms, extent=np.ptp(Q, 0)))
        rest = rest[~inl]
    return planes


def load(snap_dir, max_pts=60000):
    snaps = []
    for f in sorted(glob.glob(os.path.join(snap_dir, "snap_*.npz"))):
        z = np.load(f, allow_pickle=True)
        P = z["P"].astype(np.float64)
        r = np.linalg.norm(P, axis=1)
        P = P[(r > 0.25) & (r < 2.5)]
        if len(P) > max_pts:
            P = P[np.random.default_rng(1).choice(len(P), max_pts, replace=False)]
        planes = ransac_planes(P)
        snaps.append(dict(name=os.path.basename(f), A=z["T_world_link"], X=z["T_link_cloud"], planes=planes))
        print(f"{os.path.basename(f)}: {len(P)} pts, planes " +
              ", ".join(f"{p['count']} (rms {p['rms'] * 1e3:.1f} mm)" for p in planes))
    return snaps


def world_planes(snaps, dT):
    out = []
    for i, s in enumerate(snaps):
        T = s["A"] @ dT @ s["X"]
        for j, p in enumerate(s["planes"]):
            n = T[:3, :3] @ p["n"]
            c = T[:3, :3] @ p["c"] + T[:3, 3]
            out.append(dict(i=i, j=j, n=n, d=n @ c, c=c, w=p["count"]))
    return out


def associate(wp, ang_deg=12.0, off_m=0.06):
    """Greedy clustering of world planes, largest first; one plane per snapshot per cluster."""
    order = sorted(range(len(wp)), key=lambda k: -wp[k]["w"])
    clusters = []
    for k in order:
        p = wp[k]
        for cl in clusters:
            m = cl["n"]
            if p["i"] in cl["snaps"]:
                continue
            if np.degrees(np.arccos(np.clip(p["n"] @ m, -1, 1))) < ang_deg and abs(p["n"] @ cl["c"] - p["d"]) < off_m:
                cl["members"].append(k)
                cl["snaps"].add(p["i"])
                break
        else:
            clusters.append(dict(n=p["n"], c=p["c"], members=[k], snaps={p["i"]}))
    return [cl["members"] for cl in clusters if len(cl["members"]) >= 3]


def cluster_report(snaps, dT):
    wp = world_planes(snaps, dT)
    out = []
    for cl in sorted(associate(wp), key=lambda cl: -sum(wp[m]["w"] for m in cl)):
        ns = np.array([wp[m]["n"] for m in cl])
        nbar = ns.mean(0) / np.linalg.norm(ns.mean(0))
        ang = np.degrees(np.arccos(np.clip(ns @ nbar, -1, 1)))
        st = dict(n=nbar, ang_rms=float(np.sqrt(np.mean(ang ** 2))), size=len(cl))
        out.append(st)
        print(f"  plane seen {len(cl)}x: normal {nbar.round(3)} (tilt {np.degrees(np.arccos(nbar[2])):.1f} deg), "
              f"spread {st['ang_rms']:.2f} deg rms (max {ang.max():.2f})")
    return out


def dT_of(x):
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(x[:3]).as_matrix()
    T[:3, 3] = x[3:6]
    return T


def residuals(x, snaps, clusters, keys, t_prior=0.03):
    wp = world_planes(snaps, dT_of(x))
    res = []
    for cl in clusters:
        ns = np.array([wp[keys[m]]["n"] for m in cl])
        ds = np.array([wp[keys[m]]["d"] for m in cl])
        w = np.sqrt(np.array([wp[keys[m]]["w"] for m in cl]) / 5000.0)
        nbar = ns.mean(0)
        nbar /= np.linalg.norm(nbar)
        dbar = ds.mean()
        res.extend((w[:, None] * (ns - nbar)).ravel())          # ~rad
        res.extend(w * (ds - dbar) / 0.02)                       # 2 cm ~ 1 rad
    res.extend(x[3:6] / t_prior)  # weak prior: translation stays near the URDF
    return np.array(res)


def solve_from(snaps, seed_rotvec):
    x = np.zeros(6)
    x[:3] = seed_rotvec
    for _ in range(3):  # re-associate after each refinement
        wp = world_planes(snaps, dT_of(x))
        clusters = associate(wp)
        if not clusters:
            return None
        keys = {k: k for k in range(len(wp))}
        sol = least_squares(residuals, x, args=(snaps, clusters, keys), loss="soft_l1", f_scale=0.05)
        x = sol.x
    wp = world_planes(snaps, dT_of(x))
    clusters = associate(wp)
    stats = []
    for cl in clusters:
        ns = np.array([wp[m]["n"] for m in cl])
        nbar = ns.mean(0)
        nbar /= np.linalg.norm(nbar)
        ang = np.degrees(np.arccos(np.clip(ns @ nbar, -1, 1)))
        ds = np.array([wp[m]["d"] for m in cl])
        cen = np.mean([wp[m]["c"] for m in cl], axis=0)
        stats.append(dict(n=nbar, ang_rms=np.sqrt(np.mean(ang ** 2)), d_rms=ds.std(), size=len(cl), c=cen,
                          w=sum(wp[m]["w"] for m in cl)))
    covered = len(set(wp[m]["i"] for cl in clusters for m in cl))
    score = sum(s["ang_rms"] * s["size"] for s in stats) / max(1, sum(s["size"] for s in stats))
    return dict(x=x, stats=stats, score=score, covered=covered, n_clustered=sum(s["size"] for s in stats))


def sun_vector(t_utc, lat, lon, heading_deg):
    """Approximate (NOAA) solar direction in world, +X CCW from north by heading_deg... returns unit vector."""
    jd = t_utc.timestamp() / 86400.0 + 2440587.5
    n = jd - 2451545.0
    L = math.radians((280.460 + 0.9856474 * n) % 360)
    g = math.radians((357.528 + 0.9856003 * n) % 360)
    lam = L + math.radians(1.915) * math.sin(g) + math.radians(0.020) * math.sin(2 * g)
    eps = math.radians(23.439 - 0.0000004 * n)
    ra = math.atan2(math.cos(eps) * math.sin(lam), math.cos(lam))
    dec = math.asin(math.sin(eps) * math.sin(lam))
    gmst = (18.697374558 + 24.06570982441908 * n) % 24
    ha = math.radians(gmst * 15 + lon) - ra
    la = math.radians(lat)
    el = math.asin(math.sin(la) * math.sin(dec) + math.cos(la) * math.cos(dec) * math.cos(ha))
    az = math.atan2(-math.sin(ha), math.tan(dec) * math.cos(la) - math.sin(la) * math.cos(ha))  # from north, east +
    # world: north is at heading_deg CCW from +X; azimuth is clockwise from north.
    a = math.radians(heading_deg) - az
    return np.array([math.cos(el) * math.cos(a), math.cos(el) * math.sin(a), math.sin(el)]), math.degrees(el), math.degrees(az) % 360


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("snap_dir")
    ap.add_argument("--heading", type=float, default=125.0, help="north_heading_deg (true north, CCW from world +X)")
    ap.add_argument("--lat", type=float, default=39.7555)
    ap.add_argument("--lon", type=float, default=-105.2211)
    a = ap.parse_args()

    snaps = load(a.snap_dir)
    if len(snaps) < 4:
        raise SystemExit("need at least 4 snapshots")
    seeds = {"current URDF": [0, 0, 0], "flip 180 about link X (view axis)": [math.pi, 0, 0],
             "flip 180 about link Y": [0, math.pi, 0], "flip 180 about link Z": [0, 0, math.pi]}
    results = {}
    for name, rv in seeds.items():
        r = solve_from(snaps, np.array(rv, float))
        results[name] = r
        if r is None:
            print(f"\n[{name}] no consistent planes")
            continue
        print(f"\n[{name}] plane-normal disagreement {r['score']:.2f} deg rms, {r['n_clustered']} plane views "
              f"from {r['covered']}/{len(snaps)} snapshots")
    print("\nCurrent URDF, same plane matching as the solver:")
    nominal = cluster_report(snaps, np.eye(4))
    best_name = min((k for k in results if results[k]), key=lambda k: (-results[k]["n_clustered"], results[k]["score"]))
    best = results[best_name]
    nom = results.get("current URDF")
    if nom and nom["covered"] == len(snaps) and nominal and max(st["ang_rms"] for st in nominal[:2]) < 1.5:
        print("\nThe current URDF already makes the two largest planes agree to < 1.5 deg from every pose:"
              " the mount is fine. The fit below is for reference only; do not apply it unless it is clearly better.")
    if np.linalg.norm(best["x"][3:6]) > 0.02:
        print("WARNING: fitted translation > 20 mm; translation is weakly observed from planes, treat it as unreliable.")
    x = best["x"]
    dR = R.from_rotvec(x[:3])
    print(f"\nBEST: started from '{best_name}'")
    print(f"  correction in ee_realsense_link: rotation {np.degrees(np.linalg.norm(x[:3])):.2f} deg about "
          f"{(x[:3] / max(np.linalg.norm(x[:3]), 1e-12)).round(3)}, rpy {np.degrees(dR.as_euler('xyz')).round(2)} deg, "
          f"translation {(x[3:6] * 1e3).round(1)} mm (only the parts along plane normals are measured)")
    stamp = os.path.getmtime(glob.glob(os.path.join(a.snap_dir, 'snap_*.npz'))[0])
    s, el, az = sun_vector(dt.datetime.fromtimestamp(stamp, dt.timezone.utc), a.lat, a.lon, a.heading)
    print(f"  sun at capture time: az {az:.1f} el {el:.1f} -> world {s.round(3)} (heading {a.heading})")
    for st in sorted(best["stats"], key=lambda st: -st["w"]):
        n = st["n"]
        print(f"  plane seen {st['size']}x: normal {n.round(3)} (tilt {np.degrees(np.arccos(n[2])):.1f} deg), "
              f"spread {st['ang_rms']:.2f} deg / {st['d_rms'] * 1e3:.1f} mm, centroid {st['c'].round(3)}, "
              f"sun incidence {np.degrees(np.arccos(np.clip(n @ s, -1, 1))):.1f} deg")
    Tpl = np.eye(4)
    Tpl[:3, :3] = R.from_euler("xyz", URDF_RPY).as_matrix()
    Tpl[:3, 3] = URDF_XYZ
    Tnew = Tpl @ dT_of(x)
    rpy = R.from_matrix(Tnew[:3, :3]).as_euler("xyz")
    print(f"\n  URDF ee_realsense_joint now:  xyz=\"{' '.join(f'{v:.5f}' for v in URDF_XYZ)}\" rpy=\"{' '.join(f'{v:.5f}' for v in URDF_RPY)}\"")
    print(f"  corrected:                    xyz=\"{' '.join(f'{v:.5f}' for v in Tnew[:3, 3])}\" rpy=\"{' '.join(f'{v:.5f}' for v in rpy)}\"")


if __name__ == "__main__":
    main()
