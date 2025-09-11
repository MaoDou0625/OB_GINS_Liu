# -*- coding: utf-8 -*-
import math, sys, argparse
from pathlib import Path

def read_nav(path):
    t=[]; la=[]; lo=[]; h=[]
    with open(path,'r') as f:
        for ln in f:
            s=ln.strip().split()
            if len(s)<11: continue
            t.append(float(s[1]))
            la.append(float(s[2])); lo.append(float(s[3])); h.append(float(s[4]))
    return t, la, lo, h

def read_truth(path):
    t=[]; la=[]; lo=[]; h=[]
    with open(path,'r') as f:
        for ln in f:
            s=ln.strip().split()
            if len(s)<5: continue
            t.append(float(s[1]))
            la.append(float(s[2])); lo.append(float(s[3])); h.append(float(s[4]))
    return t, la, lo, h

def interp_seq(tn, xn, tq):
    # piecewise-linear interpolate xn(tn) onto tq; assumes tn sorted
    m=len(tn)
    out=[None]*len(tq)
    j=0
    for i, tq_i in enumerate(tq):
        # move j so that tn[j] <= tq_i <= tn[j+1]
        while j < m-2 and tn[j+1] < tq_i:
            j+=1
        if tq_i < tn[0] or tq_i > tn[-1]:
            out[i]=None; continue
        if tq_i==tn[j] or j==m-1:
            out[i]=xn[j]; continue
        t0, t1 = tn[j], tn[j+1]
        x0, x1 = xn[j], xn[j+1]
        a = (tq_i - t0)/(t1 - t0) if t1>t0 else 0.0
        out[i]= x0 + a*(x1-x0)
    return out

def compare(nav_path, truth_path, out_csv=None):
    tn, lan, lon, hn = read_nav(nav_path)
    tt, lat, lot, ht = read_truth(truth_path)
    if not tn or not tt:
        print('Empty data'); return 1
    # interpolate nav onto truth timestamps
    lan_i = interp_seq(tn, lan, tt)
    lon_i = interp_seq(tn, lon, tt)
    hn_i  = interp_seq(tn, hn,  tt)
    # compute errors where defined
    R=6378137.0
    idx0 = next((i for i,(la,lo,h) in enumerate(zip(lan_i, lon_i, hn_i)) if la is not None), None)
    if idx0 is None:
        print('No overlap'); return 1
    lat0 = math.radians(lat[idx0])
    rows=[]
    for i, (la_n, lo_n, h_n) in enumerate(zip(lan_i, lon_i, hn_i)):
        if la_n is None: continue
        dlat = math.radians(la_n - lat[i])
        dlon = math.radians(lo_n - lot[i])
        dx = dlon*math.cos(lat0)*R
        dy = dlat*R
        dz = (h_n - ht[i])
        h  = math.hypot(dx,dy)
        e3 = math.hypot(h, dz)
        rows.append((tt[i], dx, dy, dz, h, e3))
    if not rows:
        print('No valid rows'); return 1
    n=len(rows)
    rmse_h = (sum(r[4]*r[4] for r in rows)/n)**0.5
    rmse_3 = (sum(r[5]*r[5] for r in rows)/n)**0.5
    mean_h = sum(r[4] for r in rows)/n
    max_h  = max(r[4] for r in rows)
    print(f'Samples: {n}')
    print(f'RMSE_h={rmse_h:.3f} m, RMSE_3D={rmse_3:.3f} m, mean_h={mean_h:.3f}, max_h={max_h:.3f}')
    if out_csv:
        out = Path(out_csv)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, 'w') as f:
            f.write('time,dx,dy,dz,horiz,err3d\n')
            for t,dx,dy,dz,h,e3 in rows:
                f.write(f"{t:.3f},{dx:.3f},{dy:.3f},{dz:.3f},{h:.3f},{e3:.3f}\n")
        print('Saved 200Hz errors to:', out)
    return 0

if __name__=='__main__':
    ap = argparse.ArgumentParser(description='Compare OB_GINS nav against 200 Hz truth with interpolation')
    ap.add_argument('nav', help='Path to OB_GINS_TXT.nav')
    ap.add_argument('truth', help='Path to truth.nav (t,lat,lon,h)')
    ap.add_argument('--out', help='Output CSV path for per-sample errors', default=None)
    args = ap.parse_args()
    raise SystemExit(compare(args.nav, args.truth, args.out))
