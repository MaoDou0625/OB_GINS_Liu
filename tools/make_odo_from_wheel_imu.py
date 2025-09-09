# -*- coding: utf-8 -*-
import re, numpy as np
from pathlib import Path

cfg_text = Path(r'D:/Code/Wheel-INS/config/car.yaml').read_text(encoding='utf-8')
m = re.search(r'Wheel_Radius:\s*([0-9.]+)', cfg_text)
R = float(m.group(1)) if m else 0.3

wheel_txt = Path(r'D:/Code/OB_GINS/dataset/car/Wheel-IMU/C1_imu.txt')
if wheel_txt.exists():
    M = np.loadtxt(str(wheel_txt))
else:
    M = np.fromfile(r'D:/Code/OB_GINS/dataset/car/Wheel-IMU/C1_imu.bin', dtype=np.float64).reshape(-1,7)

M = M[np.argsort(M[:,0])]
omega = np.linalg.norm(M[:,1:4], axis=1)
vel = omega * R
T = M[:,0]
ODO = np.column_stack([T, vel])
odo_out = Path(r'D:/Code/OB_GINS/dataset/car/Odometer/odo_wheelimu.txt')
odo_out.parent.mkdir(parents=True, exist_ok=True)
np.savetxt(str(odo_out), ODO, fmt='%.3f %.6f')
print('radius=', R, 'rows=', ODO.shape[0], '->', odo_out)
