# -*- coding: utf-8 -*-
import numpy as np
from pathlib import Path
src = Path(r'D:/Code/OB_GINS/dataset/car/Wheel-IMU/C1_imu.bin')
out = Path(r'D:/Code/OB_GINS/dataset/car/Wheel-IMU/C1_imu.txt')
out.parent.mkdir(parents=True, exist_ok=True)
M = np.fromfile(src, dtype=np.float64)
if M.size % 7 != 0:
    raise SystemExit(f'Unexpected Wheel-IMU size: {M.size}')
M = M.reshape(-1,7)
# Save as ascii: time gx gy gz ax ay az
np.savetxt(out, M, fmt='%.9f %.9e %.9e %.9e %.9e %.9e %.9e')
print('Wrote', out, 'rows=', M.shape[0])
