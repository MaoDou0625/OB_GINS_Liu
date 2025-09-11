# -*- coding: utf-8 -*-
import os, numpy as np
from pathlib import Path
src_root = Path(r'D:/Code/Wheel-INS/dataset/car')
out_root = Path(r'D:/Code/OB_GINS/dataset/car')
start, end = 353650.0, 353680.0
# 1) IMU: rates -> increments (7 cols)
imu_bin = src_root/'Body-IMU'/'C1_imu.bin'
M = np.fromfile(imu_bin, dtype=np.float64).reshape(-1,7)
mask = (M[:,0] >= start-0.02) & (M[:,0] <= end+0.02)
M = M[mask]
# sort by time just in case
M = M[np.argsort(M[:,0])]
t = M[:,0]; gyr = M[:,1:4]; acc = M[:,4:7]
dt = np.diff(t)
dtheta = 0.5*(gyr[:-1]+gyr[1:]) * dt[:,None]
dvel   = 0.5*(acc[:-1]+acc[1:]) * dt[:,None]
out_imu = np.column_stack([t[1:], dtheta, dvel])
imu_out_path = out_root/'Body-IMU'/'C1_imu_increments.txt'
imu_out_path.parent.mkdir(parents=True, exist_ok=True)
np.savetxt(imu_out_path, out_imu, fmt='%.9f %.9e %.9e %.9e %.9e %.9e %.9e')
# 2) GNSS: extract integer-second POS from GINS.bin (interpolate if needed)
gins_bin = src_root/'Ground Truth'/'GINS.bin'
G = np.fromfile(gins_bin, dtype=np.float64).reshape(-1,5)
# keep rows with time > 1 (positions)
Gpos = G[G[:,0] > 1.0]
Gpos = Gpos[np.argsort(Gpos[:,0])]
Tg = Gpos[:,0]; La = Gpos[:,1]; Lo = Gpos[:,2]; H = Gpos[:,3]
# target integer seconds
Ti = np.arange(int(start), int(end)+1, 1.0, dtype=float)
# linear interp helper
def lin_interp(t, x, ti):
    out = np.empty_like(ti)
    j = 0
    n = len(t)
    for i, tt in enumerate(ti):
        while j < n-2 and t[j+1] < tt:
            j += 1
        if tt <= t[0]:
            out[i] = x[0]
        elif tt >= t[-1]:
            out[i] = x[-1]
        else:
            t0, t1 = t[j], t[j+1]
            a = (tt - t0)/(t1 - t0) if t1>t0 else 0.0
            out[i] = x[j] + a*(x[j+1]-x[j])
    return out
La_i = lin_interp(Tg, La, Ti)
Lo_i = lin_interp(Tg, Lo, Ti)
H_i  = lin_interp(Tg, H,  Ti)
# write POS (time lat lon h sN sE sU)
pos_out = out_root/'Ground Truth'/'GINS_all.pos'
pos_out.parent.mkdir(parents=True, exist_ok=True)
with open(pos_out, 'w') as f:
    for ttt, la, lo, hh in zip(Ti, La_i, Lo_i, H_i):
        f.write(f"{ttt:10.3f}  {la:.10f}  {lo:.10f}     {hh:.3f}    0.020    0.020    0.050\n")
# 3) Odometer (optional): split left/right if exists
odo_bin = src_root/'Odometer'/'odo.bin'
if odo_bin.exists():
    O = np.fromfile(odo_bin, dtype=np.float64).reshape(-1,4)
    tL, vL, tR, vR = O[:,0], O[:,1], O[:,2], O[:,3]
    # keep window
    mL = (tL>=start) & (tL<=end); mR = (tR>=start) & (tR<=end)
    L = np.column_stack([tL[mL], vL[mL]])
    R = np.column_stack([tR[mR], vR[mR]])
    np.savetxt(out_root/'Odometer'/'odo_left.txt', L, fmt='%.3f %.6f')
    np.savetxt(out_root/'Odometer'/'odo_right.txt', R, fmt='%.3f %.6f')
print('Wrote:', imu_out_path)
print('Wrote:', pos_out)
