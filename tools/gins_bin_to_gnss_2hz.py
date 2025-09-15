# -*- coding: utf-8 -*-
import argparse, numpy as np, re
from pathlib import Path

def find_cols(M):
    n, m = M.shape
    # detect time column: monotonic non-decreasing and sufficient span
    time_idx = None
    for i in range(m):
        col = M[:,i]
        d = np.diff(col)
        if np.all(d >= -1e-9) and (col.max() - col.min()) > 1.0:
            time_idx = i; break
    if time_idx is None:
        return None
    # detect lat/lon in deg or rad (search among remaining columns)
    best = None
    for il in range(m):
        if il == time_idx: continue
        col = M[:,il]
        frac_deg = np.mean((col>=-90) & (col<=90))
        frac_rad = np.mean((col>=-np.pi/2) & (col<=np.pi/2))
        if best is None or frac_deg>best[0]:
            best = (frac_deg, il, 'deg')
        if frac_rad>best[0]:
            best = (frac_rad, il, 'rad')
    lat_idx = best[1]; lat_unit = best[2]
    best = None
    for io in range(m):
        if io==lat_idx or io==time_idx: continue
        col = M[:,io]
        frac_deg = np.mean((col>=-180) & (col<=180))
        frac_rad = np.mean((col>=-np.pi) & (col<=np.pi))
        # pick higher score
        if best is None or frac_deg>best[0]:
            best = (frac_deg, io, 'deg')
        if frac_rad>best[0]:
            best = (frac_rad, io, 'rad')
    lon_idx, lon_unit = best[1], best[2]
    # height column: broad range
    h_idx = None; best_var=-1
    for ih in range(m):
        if ih in (lat_idx, lon_idx, time_idx): continue
        col = M[:,ih]
        in_range = np.mean((col>-500) & (col<10000))
        var = np.var(col)
        if in_range>0.9 and var>best_var:
            best_var = var; h_idx = ih
    if h_idx is None:
        return None
    return time_idx, lat_idx, lon_idx, h_idx, lat_unit, lon_unit

def to_gnss(M):
    cols = find_cols(M)
    if cols is None:
        raise SystemExit('Failed to detect columns for time/lat/lon/h')
    ti, la_i, lo_i, h_i, la_u, lo_u = cols
    t  = M[:,ti]
    la = M[:,la_i]; lo = M[:,lo_i]; h = M[:,h_i]
    if la_u=='rad': la = np.degrees(la)
    if lo_u=='rad': lo = np.degrees(lo)
    # default stds
    std = np.zeros((len(t),3)); std[:,0]=0.1; std[:,1]=0.1; std[:,2]=0.2
    return t, la, lo, h, std

def parse_init_blh(yaml_path: Path):
    if yaml_path is None or not yaml_path.exists():
        return None
    txt = yaml_path.read_text(encoding='utf-8', errors='ignore')
    m = re.search(r'initBLH\s*:\s*\[\s*([0-9.+-eE]+)\s*,\s*([0-9.+-eE]+)\s*,\s*([0-9.+-eE]+)\s*\]', txt)
    if not m: return None
    la = float(m.group(1)); lo = float(m.group(2)); h = float(m.group(3))
    return la, lo, h

def resample_2hz(t, la, lo, h, std):
    out = []
    if len(t)==0: return out
    t0 = t[0]; last = t0-1.0
    for i in range(len(t)):
        if t[i] - last >= 0.5 - 1e-3:
            out.append((t[i], la[i], lo[i], h[i], std[i,0], std[i,1], std[i,2]))
            last = t[i]
    return out

def main():
    ap = argparse.ArgumentParser(description='Convert GINS-body.bin to 2 Hz GNSS text (time lat lon h stds)')
    ap.add_argument('--in', dest='inp', required=True)
    ap.add_argument('--out', dest='out', required=True)
    ap.add_argument('--yaml', dest='yaml', default=None, help='Optional YAML with initBLH for local->global fallback')
    args = ap.parse_args()
    src = Path(args.inp); out = Path(args.out)
    M = np.fromfile(src, dtype=np.float64)
    # try a few column counts
    shape_found = None
    for c in (7,8,10,11,12,13,16,18,20,22,24,28,32,37):
        if M.size % c == 0:
            MM = M.reshape(-1,c)
            if find_cols(MM) is not None:
                shape_found = ('geo', MM); break
    if shape_found is None:
        # Fallback: treat columns [1:4] as local ENU in meters, need initBLH
        for c in (11,37,16,20):
            if M.size % c == 0:
                shape_found = ('enu', M.reshape(-1,c)); break
    if shape_found is None:
        raise SystemExit(f'Cannot determine column layout for {src}')
    mode, A = shape_found
    if mode=='geo':
        t, la, lo, h, std = to_gnss(A)
    else:
        init = parse_init_blh(Path(args.yaml)) if args.yaml else None
        if init is None:
            raise SystemExit('Local ENU fallback requires --yaml with initBLH')
        la0, lo0, h0 = init
        t = A[:,0]
        e = A[:,1]; n = A[:,2]; u = A[:,3] if A.shape[1]>3 else np.zeros_like(e)
        R = 6378137.0
        la = la0 + (n / R) * 180.0/np.pi
        lo = lo0 + (e / (R*np.cos(np.radians(la0)))) * 180.0/np.pi
        h  = h0 + u
        std = np.zeros((len(t),3)); std[:,0]=0.1; std[:,1]=0.1; std[:,2]=0.2
    rows = resample_2hz(t, la, lo, h, std)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(out, np.array(rows), fmt='%.3f %.9f %.9f %.3f %.3f %.3f %.3f')
    print('Wrote 2Hz GNSS:', out, 'rows=', len(rows))

if __name__=='__main__':
    main()
