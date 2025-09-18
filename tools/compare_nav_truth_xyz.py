#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Compare OB_GINS navigation (OB_GINS_TXT.nav) against truth_200hz.nav and
save per-sample aligned XYZ (local ENU-ish meters) plus a plot.

Inputs:
  - nav:   OB_GINS_TXT.nav (time, lat[deg], lon[deg], h[m] ...)
  - truth: truth_200hz.nav  (week, time, lat[deg], lon[deg], h[m])

Outputs:
  - CSV with columns: time,nav_x,nav_y,nav_z,truth_x,truth_y,truth_z,ex,ey,ez
  - PNG with x/y/z time series overlayed (nav vs truth)
"""
import math
import argparse
from pathlib import Path
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_nav(path):
    t = []
    la = []
    lo = []
    h = []
    with open(path, 'r') as f:
        for ln in f:
            s = ln.strip().split()
            if len(s) < 5:
                continue
            # format: idx time lat lon h ...
            try:
                t.append(float(s[1]))
                la.append(float(s[2]))
                lo.append(float(s[3]))
                h.append(float(s[4]))
            except ValueError:
                continue
    return t, la, lo, h


def read_truth_nav(path):
    t = []
    la = []
    lo = []
    h = []
    with open(path, 'r') as f:
        for ln in f:
            s = ln.strip().split()
            if len(s) < 5:
                continue
            try:
                # format: week time lat lon h
                t.append(float(s[1]))
                la.append(float(s[2]))
                lo.append(float(s[3]))
                h.append(float(s[4]))
            except ValueError:
                continue
    return t, la, lo, h


def interp_seq(tn, xn, tq):
    # piecewise-linear interpolation xn(tn) -> xq(tq)
    if not tn:
        return [None] * len(tq)
    m = len(tn)
    out = [None] * len(tq)
    j = 0
    for i, tq_i in enumerate(tq):
        while j < m - 2 and tn[j + 1] < tq_i:
            j += 1
        if tq_i < tn[0] or tq_i > tn[-1]:
            out[i] = None
            continue
        if tq_i == tn[j] or j == m - 1:
            out[i] = xn[j]
            continue
        t0, t1 = tn[j], tn[j + 1]
        x0, x1 = xn[j], xn[j + 1]
        a = (tq_i - t0) / (t1 - t0) if t1 > t0 else 0.0
        out[i] = x0 + a * (x1 - x0)
    return out


def to_local_xy(la_deg, lo_deg, h_m, la0_deg, lo0_deg, h0_m):
    # Simple local tangent plane using spherical approximation (sufficient for short spans)
    R = 6378137.0
    lat0 = math.radians(la0_deg)
    dlat = math.radians(la_deg - la0_deg)
    dlon = math.radians(lo_deg - lo0_deg)
    x = dlon * math.cos(lat0) * R  # East
    y = dlat * R                   # North
    z = h_m - h0_m                 # Up
    return x, y, z


def main():
    ap = argparse.ArgumentParser(description='Compare nav vs truth and plot XYZ')
    ap.add_argument('nav', help='Path to OB_GINS_TXT.nav')
    ap.add_argument('truth', help='Path to truth_200hz.nav')
    ap.add_argument('--out', help='Output CSV path', default=None)
    ap.add_argument('--plot', help='Output PNG path', default=None)
    args = ap.parse_args()

    tn, lan, lon, hn = read_nav(args.nav)
    tt, lat, lot, ht = read_truth_nav(args.truth)
    if not tn or not tt:
        raise SystemExit('Empty nav or truth data')

    # Interpolate nav to truth timestamps for aligned comparison
    lan_i = interp_seq(tn, lan, tt)
    lon_i = interp_seq(tn, lon, tt)
    hn_i = interp_seq(tn, hn, tt)

    # Reference origin from first valid truth sample
    idx0 = next((i for i, (la, lo, hh) in enumerate(zip(lan_i, lon_i, hn_i)) if la is not None), None)
    if idx0 is None:
        raise SystemExit('No overlap between nav and truth timestamps')
    la0, lo0, h0 = lat[idx0], lot[idx0], ht[idx0]

    rows = []
    nav_xyz = []
    tru_xyz = []
    for t, la_n, lo_n, h_n, la_t, lo_t, h_t in zip(tt, lan_i, lon_i, hn_i, lat, lot, ht):
        if la_n is None:
            continue
        xn, yn, zn = to_local_xy(la_n, lo_n, h_n, la0, lo0, h0)
        xt, yt, zt = to_local_xy(la_t, lo_t, h_t, la0, lo0, h0)
        ex, ey, ez = (xn - xt), (yn - yt), (zn - zt)
        rows.append((t, xn, yn, zn, xt, yt, zt, ex, ey, ez))
        nav_xyz.append((t, xn, yn, zn))
        tru_xyz.append((t, xt, yt, zt))

    if not rows:
        raise SystemExit('No comparable samples after interpolation')

    # Save CSV if requested
    if args.out:
        outp = Path(args.out)
        outp.parent.mkdir(parents=True, exist_ok=True)
        with open(outp, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time', 'nav_x', 'nav_y', 'nav_z', 'truth_x', 'truth_y', 'truth_z', 'ex', 'ey', 'ez'])
            for r in rows:
                w.writerow([f"{v:.3f}" for v in r])
        print('Saved CSV:', outp)

    # Plot
    if args.plot:
        pp = Path(args.plot)
        pp.parent.mkdir(parents=True, exist_ok=True)
        tvals = [r[0] for r in nav_xyz]
        nx = [r[1] for r in nav_xyz]; ny = [r[2] for r in nav_xyz]; nz = [r[3] for r in nav_xyz]
        tx = [r[1] for r in tru_xyz]; ty = [r[2] for r in tru_xyz]; tz = [r[3] for r in tru_xyz]
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axes[0].plot(tvals, nx, label='nav x (E)', lw=1)
        axes[0].plot(tvals, tx, label='truth x (E)', lw=1)
        axes[0].set_ylabel('x [m]'); axes[0].grid(True); axes[0].legend(loc='best')
        axes[1].plot(tvals, ny, label='nav y (N)', lw=1)
        axes[1].plot(tvals, ty, label='truth y (N)', lw=1)
        axes[1].set_ylabel('y [m]'); axes[1].grid(True); axes[1].legend(loc='best')
        axes[2].plot(tvals, nz, label='nav z (U)', lw=1)
        axes[2].plot(tvals, tz, label='truth z (U)', lw=1)
        axes[2].set_ylabel('z [m]'); axes[2].set_xlabel('time [s]'); axes[2].grid(True); axes[2].legend(loc='best')
        fig.tight_layout()
        fig.savefig(pp, dpi=150)
        print('Saved plot:', pp)


if __name__ == '__main__':
    main()

