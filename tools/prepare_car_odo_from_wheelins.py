# -*- coding: utf-8 -*-
import os, re, sys
import numpy as np
from pathlib import Path

def parse_window(yaml_path):
    start = 353650.0; end = 353680.0
    try:
        with open(yaml_path,'r',encoding='utf-8') as f:
            txt=f.read()
        m1=re.search(r'(?m)^starttime:\s*(\d+(?:\.\d+)?)', txt)
        m2=re.search(r'(?m)^endtime:\s*(\d+(?:\.\d+)?)', txt)
        if m1: start=float(m1.group(1))
        if m2: end=float(m2.group(1))
    except Exception:
        pass
    return start, end

def main():
    yaml_path = r'D:/Code/OB_GINS/config/run_car_30s_no_odo.yaml'
    if len(sys.argv) > 1:
        yaml_path = sys.argv[1]
    start, end = parse_window(yaml_path)
    src = Path(r'D:/Code/Wheel-INS/dataset/car/Odometer/odo.bin')
    if not src.exists():
        print('Missing source odo.bin:', src)
        return 1
    M = np.fromfile(src, dtype=np.float64)
    if M.size % 4 != 0:
        print('Unexpected size in odo.bin:', M.size)
        return 1
    M = M.reshape(-1,4)
    tL, vL, tR, vR = M[:,0], M[:,1], M[:,2], M[:,3]
    mL = (tL >= start) & (tL <= end)
    mR = (tR >= start) & (tR <= end)
    L = np.column_stack([tL[mL], vL[mL]])
    R = np.column_stack([tR[mR], vR[mR]])
    out_dir = Path(r'D:/Code/OB_GINS/dataset/car/Odometer')
    out_dir.mkdir(parents=True, exist_ok=True)
    lf = out_dir/'odo_left.txt'
    rf = out_dir/'odo_right.txt'
    # Overwrite with correct two-column files
    np.savetxt(lf, L, fmt='%.3f %.6f')
    np.savetxt(rf, R, fmt='%.3f %.6f')
    def span(A):
        if A.size==0:
            return 'EMPTY'
        return f"[{A[0,0]:.3f} .. {A[-1,0]:.3f}] n={A.shape[0]}"
    print('Left span:', span(L))
    print('Right span:', span(R))
    print('Wrote:', lf)
    print('Wrote:', rf)
    return 0

if __name__=='__main__':
    raise SystemExit(main())
