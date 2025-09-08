# -*- coding: utf-8 -*-
import numpy as np, os
in_path=r'D:\Code\OB_GINS\dataset\ADIS16465.txt'
odo_out=r'D:\Code\OB_GINS\dataset\odometer.txt'
M=np.loadtxt(in_path)
# 如果第8/9列是轮速(vL,vR)，导出平均速度；若列数<8则输出零序列
if M.shape[1] >= 9:
    v = 0.5*(M[:,7]+M[:,8])
elif M.shape[1] == 8:
    v = M[:,7]
else:
    v = np.zeros(M.shape[0])
out = np.column_stack([M[:,0], v])
os.makedirs(os.path.dirname(odo_out), exist_ok=True)
np.savetxt(odo_out, out, fmt='%.3f %.6f')
print('wrote', out.shape, '->', odo_out)
