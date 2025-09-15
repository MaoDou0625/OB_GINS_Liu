# -*- coding: utf-8 -*-

import argparse, numpy as np
from pathlib import Path

def imu_bin_to_increments(src: Path, out: Path, start: float=None, end: float=None):
    M = np.fromfile(src, dtype=np.float64)
    if M.size % 7 != 0:
        raise SystemExit(f'Unexpected IMU bin size: {M.size}, not divisible by 7')
    M = M.reshape(-1, 7)
    t  = M[:,0]
    gyr= M[:,1:4]
    acc= M[:,4:7]

    if start is not None:
        t0 = float(start)
    else:
        t0 = t[0]
    if end is not None:
        t1 = float(end)
    else:
        t1 = t[-1]

    # keep margin for trapezoidal endpoints
    mask = (t >= (t0 - 0.01)) & (t <= (t1 + 0.02))
    t = t[mask]; gyr = gyr[mask]; acc = acc[mask]
    if t.size < 3:
        raise SystemExit('Not enough samples after windowing')
    dt = np.diff(t)
    dtheta = 0.5*(gyr[:-1]+gyr[1:]) * dt[:,None]
    dvel   = 0.5*(acc[:-1]+acc[1:]) * dt[:,None]
    out_arr = np.column_stack([t[1:], dtheta, dvel])
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(out, out_arr, fmt='%.9f %.9e %.9e %.9e %.9e %.9e %.9e')
    print(f'Wrote {out} rows={out_arr.shape[0]} from {src}')

def main():
    ap = argparse.ArgumentParser(description='Convert IMU bin (t,gx,gy,gz,ax,ay,az float64) to increments txt (t,dtheta,dvel)')
    ap.add_argument('--in', dest='inp', required=True)
    ap.add_argument('--out', dest='out', required=True)
    ap.add_argument('--start', type=float, default=None)
    ap.add_argument('--end', type=float, default=None)
    args = ap.parse_args()
    imu_bin_to_increments(Path(args.inp), Path(args.out), args.start, args.end)

if __name__ == '__main__':
    main()

