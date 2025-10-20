#!/usr/bin/env python3

import argparse
import os
import re
import glob
from pathlib import Path
from typing import Dict, Tuple, List, Optional

import numpy as np
import matplotlib.pyplot as plt


def _read_nav_like(path: str) -> Dict[str, np.ndarray]:
    """
    Robustly read nav-like file (text or csv-ish):
      - Detect time column between col0/col1 by monotonicity and range
      - Take the next three columns as lat/lon/h (order of lat/lon auto-detected by range)
    Returns dict with keys: time, lat_deg, lon_deg, h_m
    """
    rows: List[List[float]] = []
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # accept comma or whitespace
            line = line.replace(',', ' ')
            parts = re.split(r"\s+", line)
            if len(parts) < 4:
                continue
            try:
                vals = [float(x) for x in parts]
            except Exception:
                continue
            rows.append(vals)

    if not rows:
        raise RuntimeError(f"No valid rows parsed from {path}")

    # choose time column between col0 and col1
    col0 = np.array([r[0] for r in rows], dtype=float)
    col1 = np.array([r[1] for r in rows], dtype=float) if len(rows[0]) > 1 else None

    def score_time(c: np.ndarray) -> Tuple[bool, float]:
        if c is None or c.size == 0:
            return False, -np.inf
        dif = np.diff(c)
        monotonic = bool(np.all(dif >= 0))
        rng = float(np.nanmax(c) - np.nanmin(c))
        return monotonic, rng

    mono0, rng0 = score_time(col0)
    mono1, rng1 = score_time(col1) if col1 is not None else (False, -np.inf)

    if mono1 and not mono0:
        t_idx = 1
    elif mono0 and not mono1:
        t_idx = 0
    else:
        # tie-breaker by range
        t_idx = 1 if rng1 > rng0 else 0

    times: List[float] = []
    c2: List[float] = []
    c3: List[float] = []
    c4: List[float] = []

    for vals in rows:
        if len(vals) <= t_idx + 3:
            continue
        t = vals[t_idx]
        p0 = vals[t_idx + 1]
        p1 = vals[t_idx + 2]
        p2 = vals[t_idx + 3]
        times.append(t)
        c2.append(p0)
        c3.append(p1)
        c4.append(p2)

    if not times:
        raise RuntimeError(f"No valid rows parsed from {path}")

    times = np.asarray(times)
    c2 = np.asarray(c2)
    c3 = np.asarray(c3)
    c4 = np.asarray(c4)

    # detect lat/lon order by range across full series
    is_c2_lat = np.nanmedian(np.abs(c2)) <= 90.0 and np.nanmedian(np.abs(c3)) <= 180.0
    is_c3_lat = np.nanmedian(np.abs(c3)) <= 90.0 and np.nanmedian(np.abs(c2)) <= 180.0
    if is_c2_lat and not is_c3_lat:
        lat = c2
        lon = c3
    elif is_c3_lat and not is_c2_lat:
        lat = c3
        lon = c2
    else:
        # fallback: assume [lon, lat]
        lon = c2
        lat = c3

    return {
        'time': times,
        'lat_deg': lat,
        'lon_deg': lon,
        'h_m': c4,
    }


def _interp_truth(truth: Dict[str, np.ndarray], t: np.ndarray) -> Dict[str, np.ndarray]:
    idx = np.argsort(truth['time'])
    tt = truth['time'][idx]
    lat = truth['lat_deg'][idx]
    lon = truth['lon_deg'][idx]
    h = truth['h_m'][idx]

    def interp_linear_extrap(x: np.ndarray, xp: np.ndarray, fp: np.ndarray) -> np.ndarray:
        # require sorted xp
        if xp.size < 2:
            return np.full_like(x, fp[0] if fp.size else 0.0, dtype=float)
        y = np.interp(x, xp, fp)
        # linear extrap below
        below = x < xp[0]
        if np.any(below):
            slope = (fp[1] - fp[0]) / (xp[1] - xp[0]) if (xp[1] - xp[0]) != 0 else 0.0
            y[below] = fp[0] + slope * (x[below] - xp[0])
        # linear extrap above
        above = x > xp[-1]
        if np.any(above):
            slope = (fp[-1] - fp[-2]) / (xp[-1] - xp[-2]) if (xp[-1] - xp[-2]) != 0 else 0.0
            y[above] = fp[-1] + slope * (x[above] - xp[-1])
        return y

    return {
        'time': t,
        'lat_deg': interp_linear_extrap(t, tt, lat),
        'lon_deg': interp_linear_extrap(t, tt, lon),
        'h_m': interp_linear_extrap(t, tt, h),
    }


def _deg_to_meter(lat_deg: np.ndarray, dlat_deg: np.ndarray, dlon_deg: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    # approximate conversion using mean latitude
    lat_rad = np.deg2rad(lat_deg)
    dN = dlat_deg * 111_132.92  # meters per degree latitude
    dE = dlon_deg * (111_412.84 * np.cos(lat_rad))  # meters per degree longitude varies with latitude
    return dE, dN


def compute_errors(nav: Dict[str, np.ndarray], truth: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
    # interpolate truth at nav time
    tr_itp = _interp_truth(truth, nav['time'])
    dlat = nav['lat_deg'] - tr_itp['lat_deg']
    dlon = nav['lon_deg'] - tr_itp['lon_deg']
    dh = nav['h_m'] - tr_itp['h_m']
    dE, dN = _deg_to_meter(tr_itp['lat_deg'], dlat, dlon)
    return {'time': nav['time'], 'dE': dE, 'dN': dN, 'dU': dh}


def plot_timeseries(nav_main: Dict[str, np.ndarray], truth: Dict[str, np.ndarray], extras: Dict[str, Dict[str, np.ndarray]], out_png: str):
    errs = {'main': compute_errors(nav_main, truth)}
    for k, nav in extras.items():
        errs[k] = compute_errors(nav, truth)

    plt.figure(figsize=(14, 8))
    colors = {'main': 'C0', 'left': 'C1', 'right': 'C2'}
    labels = {'main': 'main', 'left': 'wheel_left', 'right': 'wheel_right'}
    # E, N, U
    for i, comp in enumerate(['dE', 'dN', 'dU']):
        ax = plt.subplot(3, 1, i + 1)
        for key, er in errs.items():
            ax.plot(er['time'], er[comp], label=labels.get(key, key), color=colors.get(key))
        ax.set_ylabel(comp + ' [m]')
        ax.grid(True, ls='--', alpha=0.4)
        if i == 0:
            ax.legend(loc='best')
    plt.xlabel('time [s]')
    plt.tight_layout()
    if out_png:
        os.makedirs(os.path.dirname(out_png), exist_ok=True)
        plt.savefig(out_png, dpi=150)
    plt.close()


def plot_traj(nav_main: Dict[str, np.ndarray], truth: Dict[str, np.ndarray], extras: Dict[str, Dict[str, np.ndarray]], out_png: str):
    plt.figure(figsize=(8, 8))
    # plot truth lon-lat
    plt.plot(truth['lon_deg'], truth['lat_deg'], 'k-', lw=2, label='truth')
    plt.plot(nav_main['lon_deg'], nav_main['lat_deg'], '-', lw=1.5, label='main', color='C0')
    if 'left' in extras:
        plt.plot(extras['left']['lon_deg'], extras['left']['lat_deg'], '-', lw=1.2, label='wheel_left', color='C1')
    if 'right' in extras:
        plt.plot(extras['right']['lon_deg'], extras['right']['lat_deg'], '-', lw=1.2, label='wheel_right', color='C2')
    plt.xlabel('lon [deg]')
    plt.ylabel('lat [deg]')
    plt.axis('equal')
    plt.grid(True, ls='--', alpha=0.4)
    plt.legend(loc='best')
    plt.tight_layout()
    if out_png:
        os.makedirs(os.path.dirname(out_png), exist_ok=True)
        plt.savefig(out_png, dpi=150)
    plt.close()


def plot_errors_only(nav_main: Dict[str, np.ndarray], truth: Dict[str, np.ndarray], extras: Dict[str, Dict[str, np.ndarray]], out_png: str):
    # convenience: same as timeseries but only error magnitudes
    errs = {'main': compute_errors(nav_main, truth)}
    for k, nav in extras.items():
        errs[k] = compute_errors(nav, truth)

    plt.figure(figsize=(12, 7))
    colors = {'main': 'C0', 'left': 'C1', 'right': 'C2'}
    labels = {'main': 'main', 'left': 'wheel_left', 'right': 'wheel_right'}
    for i, comp in enumerate(['dE', 'dN', 'dU']):
        ax = plt.subplot(3, 1, i + 1)
        for key, er in errs.items():
            ax.plot(er['time'], er[comp], label=labels.get(key, key), color=colors.get(key))
        ax.set_ylabel(comp + ' [m]')
        ax.grid(True, ls='--', alpha=0.4)
        if i == 0:
            ax.legend(loc='best')
    plt.xlabel('time [s]')
    plt.tight_layout()
    if out_png:
        os.makedirs(os.path.dirname(out_png), exist_ok=True)
        plt.savefig(out_png, dpi=150)
    plt.close()


def _interp_truth_to_nav(truth: Dict[str, np.ndarray], nav: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
    tr = _interp_truth(truth, nav['time'])
    return tr


def plot_nav_vs_truth_single(nav: Dict[str, np.ndarray], truth: Dict[str, np.ndarray], out_png: str, title: str):
    tr = _interp_truth_to_nav(truth, nav)
    t = nav['time']
    fig, axs = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
    axs[0].plot(t, nav['lat_deg'], label=f'{title} lat', color='C0')
    axs[0].plot(t, tr['lat_deg'], label='truth lat', color='k', alpha=0.7)
    axs[0].set_ylabel('lat [deg]')
    axs[0].grid(True, ls='--', alpha=0.4)
    axs[0].legend(loc='best')

    axs[1].plot(t, nav['lon_deg'], label=f'{title} lon', color='C1')
    axs[1].plot(t, tr['lon_deg'], label='truth lon', color='k', alpha=0.7)
    axs[1].set_ylabel('lon [deg]')
    axs[1].grid(True, ls='--', alpha=0.4)
    axs[1].legend(loc='best')

    axs[2].plot(t, nav['h_m'], label=f'{title} h', color='C2')
    axs[2].plot(t, tr['h_m'], label='truth h', color='k', alpha=0.7)
    axs[2].set_ylabel('h [m]')
    axs[2].set_xlabel('time [s]')
    axs[2].grid(True, ls='--', alpha=0.4)
    axs[2].legend(loc='best')

    fig.suptitle(f'Nav vs Truth: {title}')
    plt.tight_layout(rect=[0, 0.03, 1, 0.97])
    if out_png:
        os.makedirs(os.path.dirname(out_png), exist_ok=True)
        plt.savefig(out_png, dpi=150)
    plt.close()


def plot_errors_single(nav: Dict[str, np.ndarray], truth: Dict[str, np.ndarray], out_png: str, title: str):
    er = compute_errors(nav, truth)
    t = er['time']
    fig, axs = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
    axs[0].plot(t, er['dE'], label='dE', color='C0')
    axs[0].set_ylabel('E [m]')
    axs[0].grid(True, ls='--', alpha=0.4)

    axs[1].plot(t, er['dN'], label='dN', color='C1')
    axs[1].set_ylabel('N [m]')
    axs[1].grid(True, ls='--', alpha=0.4)

    axs[2].plot(t, er['dU'], label='dU', color='C2')
    axs[2].set_ylabel('U [m]')
    axs[2].set_xlabel('time [s]')
    axs[2].grid(True, ls='--', alpha=0.4)

    fig.suptitle(f'Nav Errors: {title}')
    plt.tight_layout(rect=[0, 0.03, 1, 0.97])
    if out_png:
        os.makedirs(os.path.dirname(out_png), exist_ok=True)
        plt.savefig(out_png, dpi=150)
    plt.close()


def plot_errors_compare(nav_main: Dict[str, np.ndarray], truth: Dict[str, np.ndarray], extras: Dict[str, Dict[str, np.ndarray]], out_png: str):
    # Reuse timeseries (errors across IMUs in one figure)
    plot_timeseries(nav_main, truth, extras, out_png)


def _discover_side_navs(nav_main_path: str) -> Tuple[Optional[str], Optional[str]]:
    """Given main nav path, try to discover left/right nav files in the same directory.
    Rules:
      - Prefer exact sibling names: '<stem>_left<ext>' and '<stem>_right<ext>'
      - Fallback to glob '*left*.nav' / '*right*.nav' if exact not found
    Returns (nav_left, nav_right) which may be None if not found.
    """
    p = Path(nav_main_path)
    stem = p.stem
    ext = p.suffix or '.nav'
    cand_left = p.with_name(f"{stem}_left{ext}")
    cand_right = p.with_name(f"{stem}_right{ext}")

    nav_left = str(cand_left) if cand_left.exists() else None
    nav_right = str(cand_right) if cand_right.exists() else None

    if nav_left is None:
        hits = sorted(glob.glob(str(p.parent / "*left*.nav")))
        if hits:
            nav_left = hits[0]
    if nav_right is None:
        hits = sorted(glob.glob(str(p.parent / "*right*.nav")))
        if hits:
            nav_right = hits[0]
    return nav_left, nav_right


def _default_outputs(nav_main_path: str, png: Optional[str], traj: Optional[str], errpng: Optional[str]) -> Tuple[str, str, str]:
    """If outputs are not provided, create sensible defaults next to nav_main."""
    p = Path(nav_main_path)
    outdir = p.parent
    if not png:
        png = str(outdir / "nav_truth_timeseries.png")
    if not traj:
        traj = str(outdir / "nav_truth_traj2.png")
    if not errpng:
        errpng = str(outdir / "nav_truth_errors.png")
    return png, traj, errpng


def main():
    ap = argparse.ArgumentParser(description='Compare multi-chain nav with truth and plot time series + traj + errors.')
    ap.add_argument('nav_main', help='main nav file (OB_GINS_TXT.nav)')
    ap.add_argument('truth', help='truth nav file (truth_200hz.nav)')
    ap.add_argument('--nav-left', dest='nav_left', default=None, help='left wheel nav file (auto-discover if omitted)')
    ap.add_argument('--nav-right', dest='nav_right', default=None, help='right wheel nav file (auto-discover if omitted)')
    ap.add_argument('--png', dest='png', default=None, help='output timeseries png (default next to nav_main)')
    ap.add_argument('--traj', dest='traj', default=None, help='output traj png (default next to nav_main)')
    ap.add_argument('--errpng', dest='errpng', default=None, help='output errors png (default next to nav_main)')

    args = ap.parse_args()

    # auto-discover left/right if omitted
    nav_left_path, nav_right_path = args.nav_left, args.nav_right
    if nav_left_path is None or nav_right_path is None:
        auto_left, auto_right = _discover_side_navs(args.nav_main)
        nav_left_path = nav_left_path or auto_left
        nav_right_path = nav_right_path or auto_right

    # default outputs if not provided
    args.png, args.traj, args.errpng = _default_outputs(args.nav_main, args.png, args.traj, args.errpng)

    nav_main = _read_nav_like(args.nav_main)
    truth = _read_nav_like(args.truth)
    extras: Dict[str, Dict[str, np.ndarray]] = {}
    if nav_left_path and os.path.isfile(nav_left_path):
        extras['left'] = _read_nav_like(nav_left_path)
    if nav_right_path and os.path.isfile(nav_right_path):
        extras['right'] = _read_nav_like(nav_right_path)

    # New required outputs per IMU (nav vs truth; errors per IMU)
    outdir = str(Path(args.png).parent)  # base outdir
    # Per-IMU nav vs truth
    plot_nav_vs_truth_single(nav_main, truth, str(Path(outdir) / 'nav_vs_truth_main.png'), 'main')
    if 'left' in extras:
        plot_nav_vs_truth_single(extras['left'], truth, str(Path(outdir) / 'nav_vs_truth_left.png'), 'wheel_left')
    if 'right' in extras:
        plot_nav_vs_truth_single(extras['right'], truth, str(Path(outdir) / 'nav_vs_truth_right.png'), 'wheel_right')

    # Per-IMU errors vs truth
    plot_errors_single(nav_main, truth, str(Path(outdir) / 'nav_err_main.png'), 'main')
    if 'left' in extras:
        plot_errors_single(extras['left'], truth, str(Path(outdir) / 'nav_err_left.png'), 'wheel_left')
    if 'right' in extras:
        plot_errors_single(extras['right'], truth, str(Path(outdir) / 'nav_err_right.png'), 'wheel_right')

    # All-IMUs errors compare in one figure
    plot_errors_compare(nav_main, truth, extras, str(Path(outdir) / 'nav_errors_compare.png'))

    # Keep legacy outputs if explicitly requested
    if args.png:
        plot_timeseries(nav_main, truth, extras, args.png)
    if args.traj:
        plot_traj(nav_main, truth, extras, args.traj)
    if args.errpng:
        plot_errors_only(nav_main, truth, extras, args.errpng)


if __name__ == '__main__':
    main()
