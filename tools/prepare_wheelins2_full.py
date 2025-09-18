#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Prepare OB_GINS inputs from wheel-ins2_dataset.

Inputs (default base): D:/Code/OB_GINS/dataset/wheel-ins2_dataset
  - body-imu/tri_C{cam}_imu.bin           (float64, N×7: t,gx,gy,gz,ax,ay,az)
  - left-wheel-imu/tri_C{cam}_imu.bin     (same as above)
  - right-wheel-imu/tri_C{cam}_imu.bin    (same as above)
  - ground_truth/GINS-body.bin            (float64, N×5: tag/time, lat, lon, h, ?)

Outputs (under base/converted/C{cam}/):
  - Body-IMU/C{cam}_imu_increments.txt
  - Wheel-IMU-Left/C{cam}_imu_increments.txt
  - Wheel-IMU-Right/C{cam}_imu_increments.txt
  - Ground Truth/GINS_all.pos             (1 Hz POS)
  - Ground Truth/truth_200hz.nav          (expanded 200 Hz nav)

Usage:
  python OB_GINS/tools/prepare_wheelins2_full.py --base "D:/Code/OB_GINS/dataset/wheel-ins2_dataset" --cam 1
"""
import argparse
import numpy as np
from pathlib import Path


def imu_rates_to_increments(src: Path, dst: Path):
    M = np.fromfile(src, dtype=np.float64)
    if M.size % 7 != 0:
        raise RuntimeError(f'IMU size not multiple of 7: {M.size} at {src}')
    M = M.reshape(-1, 7)
    # sort by time just in case
    M = M[np.argsort(M[:, 0])]
    t = M[:, 0]
    gyr = M[:, 1:4]
    acc = M[:, 4:7]
    dt = np.diff(t)
    if len(dt) == 0:
        raise RuntimeError(f'IMU has too few samples: {src}')
    dtheta = 0.5 * (gyr[:-1] + gyr[1:]) * dt[:, None]
    dvel = 0.5 * (acc[:-1] + acc[1:]) * dt[:, None]
    out = np.column_stack([t[1:], dtheta, dvel])
    dst.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(dst, out, fmt='%.9f %.9e %.9e %.9e %.9e %.9e %.9e')
    return float(t[0]), float(t[-1]), out.shape[0]


def gnss_bin_to_pos_and_truth(src_bin: Path, dst_pos: Path, dst_truth200: Path):
    M = np.fromfile(src_bin, dtype=np.float64)
    if M.size % 5 == 0:
        M = M.reshape(-1, 5)
        # Keep position rows (time > 1.0)
        P = M[M[:, 0] > 1.0]
        P = P[np.argsort(P[:, 0])]
        t = P[:, 0]; la = P[:, 1]; lo = P[:, 2]; h = P[:, 3]
    elif M.size % 11 == 0:
        M = M.reshape(-1, 11)
        # Columns: week, time, lat, lon, h, ...
        P = M[np.argsort(M[:, 1])]
        t = P[:, 1]; la = P[:, 2]; lo = P[:, 3]; h = P[:, 4]
    else:
        raise RuntimeError(f'Unsupported GNSS record width (not 5 or 11): doubles={M.size} at {src_bin}')

    # truth_200hz.nav (week time lat lon h)
    dst_truth200.parent.mkdir(parents=True, exist_ok=True)
    with open(dst_truth200, 'w') as f:
        for ti, lai, loi, hi in zip(t, la, lo, h):
            f.write(f"2140 {ti:.3f} {lai:.10f} {loi:.10f} {hi:.3f}\n")

    # GINS_all.pos at integer seconds via linear interpolation
    t0 = int(np.ceil(t[0]))
    t1 = int(np.floor(t[-1]))
    Ti = np.arange(t0, t1 + 1, 1.0, dtype=float)
    def lin(tt, xx, ti):
        out = np.empty_like(ti)
        j = 0
        n = len(tt)
        for i, tv in enumerate(ti):
            while j < n - 2 and tt[j + 1] < tv:
                j += 1
            if tv <= tt[0]:
                out[i] = xx[0]
            elif tv >= tt[-1]:
                out[i] = xx[-1]
            else:
                t0, t1 = tt[j], tt[j + 1]
                a = (tv - t0) / (t1 - t0) if t1 > t0 else 0.0
                out[i] = xx[j] + a * (xx[j + 1] - xx[j])
        return out
    La_i = lin(t, la, Ti)
    Lo_i = lin(t, lo, Ti)
    H_i = lin(t, h, Ti)

    dst_pos.parent.mkdir(parents=True, exist_ok=True)
    with open(dst_pos, 'w') as f:
        for ti, lai, loi, hi in zip(Ti, La_i, Lo_i, H_i):
            f.write(f"{ti:10.3f}  {lai:.10f}  {loi:.10f}     {hi:.3f}    0.020    0.020    0.050\n")
    return float(Ti[0]), float(Ti[-1]), Ti.size


def main():
    ap = argparse.ArgumentParser(description='Prepare wheel-ins2 dataset for OB_GINS')
    ap.add_argument('--base', default=r'D:/Code/OB_GINS/dataset/wheel-ins2_dataset', help='Base dataset directory')
    ap.add_argument('--cam', type=int, default=1, help='Camera index (1..4)')
    args = ap.parse_args()

    base = Path(args.base)
    cam = f'C{args.cam}'

    body_src = base / 'body-imu' / f'tri_{cam}_imu.bin'
    left_src = base / 'left-wheel-imu' / f'tri_{cam}_imu.bin'
    right_src = base / 'right-wheel-imu' / f'tri_{cam}_imu.bin'
    truth_src = base / 'ground_truth' / 'GINS-body.bin'

    out_root = base / 'converted' / cam
    body_dst = out_root / 'Body-IMU' / f'{cam}_imu_increments.txt'
    left_dst = out_root / 'Wheel-IMU-Left' / f'{cam}_imu_increments.txt'
    right_dst = out_root / 'Wheel-IMU-Right' / f'{cam}_imu_increments.txt'
    pos_dst = out_root / 'Ground Truth' / 'GINS_all.pos'
    truth200_dst = out_root / 'Ground Truth' / 'truth_200hz.nav'

    t0b, t1b, nb = imu_rates_to_increments(body_src, body_dst)
    print('Body IMU increments:', body_dst, 'rows=', nb)
    t0l, t1l, nl = imu_rates_to_increments(left_src, left_dst)
    print('Left IMU increments:', left_dst, 'rows=', nl)
    t0r, t1r, nr = imu_rates_to_increments(right_src, right_dst)
    print('Right IMU increments:', right_dst, 'rows=', nr)

    p0, p1, np1 = gnss_bin_to_pos_and_truth(truth_src, pos_dst, truth200_dst)
    print('GNSS POS 1Hz      :', pos_dst, 'rows=', np1)
    print('Truth 200Hz       :', truth200_dst)

    # Suggest time window overlap
    tmin = max(int(np.ceil(min(t0b, t0l, t0r))), int(np.ceil(p0)))
    tmax = min(int(np.floor(max(t1b, t1l, t1r))), int(np.floor(p1)))
    print('Suggested time window: starttime=', tmin, 'endtime=', min(tmin + 120, tmax))


if __name__ == '__main__':
    main()
