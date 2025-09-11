# -*- coding: utf-8 -*-
import numpy as np
from pathlib import Path
src = Path(r'D:/Code/Wheel-INS/dataset/car/Odometer/odo.bin')
out_dir = Path(r'D:/Code/OB_GINS/dataset/car/Odometer')
out_dir.mkdir(parents=True, exist_ok=True)
M = np.fromfile(src, dtype=np.float64)
if M.size % 4 != 0:
    raise SystemExit(f'Unexpected odo.bin size: {M.size}')
M = M.reshape(-1,4)
tL, vL, tR, vR = M[:,0], M[:,1], M[:,2], M[:,3]
L = np.column_stack([tL, vL])
R = np.column_stack([tR, vR])
lf = out_dir/'odo_left.txt'
rf = out_dir/'odo_right.txt'
np.savetxt(lf, L, fmt='%.3f %.6f')
np.savetxt(rf, R, fmt='%.3f %.6f')
print('Left span:', f"[{tL.min():.3f} .. {tL.max():.3f}] n={L.shape[0]}")
print('Right span:', f"[{tR.min():.3f} .. {tR.max():.3f}] n={R.shape[0]}")
print('Wrote:', lf)
print('Wrote:', rf)
