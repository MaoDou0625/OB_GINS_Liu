# -*- coding: utf-8 -*-
import argparse
import math
import csv
import os

def read_nav(path):
    N = {}
    with open(path, 'r') as f:
        for ln in f:
            s = ln.strip().split()
            if len(s) < 11:
                continue
            t = float(s[1])
            ti = round(t)
            if abs(t - ti) <= 5e-3:
                # lat[deg], lon[deg], h[m]
                N[int(ti)] = (float(s[2]), float(s[3]), float(s[4]))
    return N

def read_truth_pos(path):
    T = {}
    with open(path, 'r') as f:
        for ln in f:
            s = ln.strip().split()
            if len(s) < 4:
                continue
            # time, lat[deg], lon[deg], h[m]
            t = float(s[0])
            T[int(round(t))] = (float(s[1]), float(s[2]), float(s[3]))
    return T

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-n', '--nav', default=r'D:/Code/OB_GINS/dataset/robot/out_robot/OB_GINS_TXT.nav', help='OB_GINS_TXT.nav')
    ap.add_argument('-t', '--truth', default=r'D:/Code/OB_GINS/dataset/robot/Ground Truth/GINS_all.pos', help='GINS_all.pos (time, lat, lon, h)')
    ap.add_argument('-o', '--out', default=r'D:/Code/OB_GINS/dataset/robot/out_robot/nav_truth_xyz_latest.csv', help='output CSV for dx,dy,dz')
    ap.add_argument('-p', '--png', default=r'D:/Code/OB_GINS/dataset/robot/out_robot/nav_truth_xyz_latest.png', help='output PNG path for error curves')
    args = ap.parse_args()

    nav = read_nav(args.nav)
    tru = read_truth_pos(args.truth)
    keys = sorted(set(nav) & set(tru))
    if not keys:
        print('No overlapping integer-second timestamps between nav and truth.')
        return 2

    R = 6378137.0
    lat_ref = tru[keys[0]][0]
    lon_ref = tru[keys[0]][1]
    h_ref = tru[keys[0]][2]
    lat0 = math.radians(lat_ref)
    rows = []
    for k in keys:
        n = nav[k]; r = tru[k]
        dlat = math.radians(n[0] - r[0])
        dlon = math.radians(n[1] - r[1])
        dx = dlon * math.cos(lat0) * R  # East
        dy = dlat * R                   # North
        dz = n[2] - r[2]                # Up
        rows.append((k, dx, dy, dz))

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['time', 'dx(m)', 'dy(m)', 'dz(m)'])
        w.writerows(rows)

    n = len(rows)
    rmse = lambda i: (sum(r[i] ** 2 for r in rows) / n) ** 0.5
    rmse_h = (sum((r[1]**2 + r[2]**2) for r in rows) / n) ** 0.5
    rmse_3d = (sum((r[1]**2 + r[2]**2 + r[3]**2) for r in rows) / n) ** 0.5
    print('samples:', n)
    print('RMSE dx=%.3f m, dy=%.3f m, dz=%.3f m' % (rmse(1), rmse(2), rmse(3)))
    print('saved csv:', args.out)
    print('head:')
    for r in rows[:5]:
        print(r)

    # Optional: plot PNG (show nav & truth values on each axis)
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        ts = keys
        # helper for ENU relative to reference (east/north in meters, up relative to h_ref)
        def to_ENU(lat_deg, lon_deg, h_m):
            dlat = math.radians(lat_deg - lat_ref)
            dlon = math.radians(lon_deg - lon_ref)
            E = dlon * math.cos(lat0) * R
            N = dlat * R
            U = h_m - h_ref
            return E, N, U

        nav_E, nav_N, nav_U = [], [], []
        tru_E, tru_N, tru_U = [], [], []
        for k in ts:
            n = nav[k]; r = tru[k]
            e1, n1, u1 = to_ENU(n[0], n[1], n[2])
            e0, n0, u0 = to_ENU(r[0], r[1], r[2])
            nav_E.append(e1); nav_N.append(n1); nav_U.append(u1)
            tru_E.append(e0); tru_N.append(n0); tru_U.append(u0)

        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axs[0].plot(ts, tru_E, 'k-', lw=1.2, label='Truth')
        axs[0].plot(ts, nav_E, 'b--', lw=1.0, label='Nav')
        axs[0].set_ylabel('East [m]')
        axs[0].grid(True, ls='--', alpha=0.4)
        axs[0].legend()

        axs[1].plot(ts, tru_N, 'k-', lw=1.2, label='Truth')
        axs[1].plot(ts, nav_N, 'g--', lw=1.0, label='Nav')
        axs[1].set_ylabel('North [m]')
        axs[1].grid(True, ls='--', alpha=0.4)

        axs[2].plot(ts, tru_U, 'k-', lw=1.2, label='Truth')
        axs[2].plot(ts, nav_U, 'r--', lw=1.0, label='Nav')
        axs[2].set_ylabel('Up [m]')
        axs[2].set_xlabel('time [s]')
        axs[2].grid(True, ls='--', alpha=0.4)

        fig.suptitle('Nav vs Truth (E/N/U)\nRMSE_h=%.3f m, RMSE_3D=%.3f m' % (rmse_h, rmse_3d))
        fig.tight_layout(rect=[0, 0.06, 1, 0.94])
        fig.savefig(args.png, dpi=150)
        print('saved png:', args.png)
    except Exception as e:
        print('plot skipped (matplotlib not available?):', e)
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
