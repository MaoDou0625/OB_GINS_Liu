# -*- coding: utf-8 -*-
"""
Prepare full-span car dataset for OB_GINS from Wheel-INS sources.

Generates:
- IMU increments (7 cols): dataset/car/Body-IMU/C1_imu_increments.txt
- GNSS POS at 1 Hz:        dataset/car/Ground Truth/GINS_all.pos
- (Optional) 200 Hz truth: dataset/car/Ground Truth/truth_200hz.nav (if not present)

Assumes Wheel-INS layout under D:/Code/Wheel-INS/dataset/car
and writes outputs under D:/Code/OB_GINS/dataset/car
"""
import numpy as np
from pathlib import Path

SRC = Path(r'D:/Code/Wheel-INS/dataset/car')
DST = Path(r'D:/Code/OB_GINS/dataset/car')

def imu_rates_to_increments():
    src = SRC/'Body-IMU'/'C1_imu.bin'
    dst = DST/'Body-IMU'/'C1_imu_increments.txt'
    dst.parent.mkdir(parents=True, exist_ok=True)
    M = np.fromfile(src, dtype=np.float64)
    if M.size % 7 != 0:
        raise RuntimeError(f'IMU size not multiple of 7: {M.size}')
    M = M.reshape(-1,7)
    M = M[np.argsort(M[:,0])]
    t = M[:,0]; gyr = M[:,1:4]; acc = M[:,4:7]
    dt = np.diff(t)
    dtheta = 0.5*(gyr[:-1]+gyr[1:]) * dt[:,None]
    dvel   = 0.5*(acc[:-1]+acc[1:]) * dt[:,None]
    out = np.column_stack([t[1:], dtheta, dvel])
    np.savetxt(dst, out, fmt='%.9f %.9e %.9e %.9e %.9e %.9e %.9e')
    return t[0], t[-1], out.shape[0], dst

def gnss_bin_to_pos_full(t_min, t_max):
    src = SRC/'Ground Truth'/'GINS.bin'
    dst = DST/'Ground Truth'/'GINS_all.pos'
    dst.parent.mkdir(parents=True, exist_ok=True)
    G = np.fromfile(src, dtype=np.float64).reshape(-1,5)
    G = G[G[:,0] > 1.0]
    G = G[np.argsort(G[:,0])]
    Tg = G[:,0]; La = G[:,1]; Lo = G[:,2]; H = G[:,3]
    # integer seconds within overlap
    t0 = max(int(np.ceil(Tg[0])), int(np.ceil(t_min)))
    t1 = min(int(np.floor(Tg[-1])), int(np.floor(t_max)))
    Ti = np.arange(t0, t1+1, 1.0)
    def lin(t, x, ti):
        out = np.empty_like(ti)
        j = 0
        n = len(t)
        for i, tt in enumerate(ti):
            while j < n-2 and t[j+1] < tt:
                j += 1
            if tt <= t[0]: out[i] = x[0]; continue
            if tt >= t[-1]: out[i] = x[-1]; continue
            t0,t1 = t[j], t[j+1]
            a = (tt - t0)/(t1 - t0) if t1>t0 else 0.0
            out[i] = x[j] + a*(x[j+1]-x[j])
        return out
    La_i = lin(Tg, La, Ti)
    Lo_i = lin(Tg, Lo, Ti)
    H_i  = lin(Tg, H,  Ti)
    with open(dst,'w') as f:
        for tt,la,lo,hh in zip(Ti, La_i, Lo_i, H_i):
            f.write(f"{tt:10.3f}  {la:.10f}  {lo:.10f}     {hh:.3f}    0.020    0.020    0.050\n")
    return Ti[0], Ti[-1], Ti.size, dst

def ensure_truth_200hz():
    dst = DST/'Ground Truth'/'truth_200hz.nav'
    if dst.exists():
        return dst
    src = SRC/'Ground Truth'/'GINS.bin'
    M = np.fromfile(src, dtype=np.float64).reshape(-1,5)
    M = M[M[:,0] > 1.0]
    M = M[np.argsort(M[:,0])]
    dst.parent.mkdir(parents=True, exist_ok=True)
    with open(dst,'w') as f:
        for t, la, lo, h, _ in M:
            f.write(f"2140 {t:.3f} {la:.10f} {lo:.10f} {h:.3f}\n")
    return dst

def main():
    t0, t1, nimu, imu_path = imu_rates_to_increments()
    p0, p1, npos, pos_path = gnss_bin_to_pos_full(t0, t1)
    tnav = ensure_truth_200hz()
    print('IMU increments:', imu_path, 'rows=', nimu)
    print('GNSS POS full :', pos_path, 'rows=', npos)
    print('Truth 200Hz   :', tnav)

if __name__ == '__main__':
    main()

