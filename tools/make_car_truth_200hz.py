# -*- coding: utf-8 -*-
import numpy as np
from pathlib import Path
src_bin = Path(r'D:/Code/Wheel-INS/dataset/car/Ground Truth/GINS.bin')
out_nav = Path(r'D:/Code/OB_GINS/dataset/car/Ground Truth/truth_200hz.nav')
M = np.fromfile(src_bin, dtype=np.float64).reshape(-1,5)
# Keep only rows with time>1 (position rows assumed)
M = M[M[:,0] > 1.0]
# Sort by time
M = M[np.argsort(M[:,0])]
with open(out_nav, 'w') as f:
    for t, la, lo, h, _ in M:
        f.write(f"2140 {t:.3f} {la:.10f} {lo:.10f} {h:.3f}\n")
print('Wrote', out_nav)
