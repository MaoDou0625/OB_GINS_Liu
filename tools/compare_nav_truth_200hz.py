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

def compare(nav_path, truth_path, out_csv=None, series_csv=None, png_path=None, traj_png=None, err_png=None):
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
    rows=[]  # per-sample errors aligned to truth timestamps
    # Precompute ENU time series for nav and truth (aligned on truth timestamps)
    def to_ENU(lat_deg, lon_deg, h_m, lat_ref_deg, lon_ref_deg, h_ref):
        dlat = math.radians(lat_deg - lat_ref_deg)
        dlon = math.radians(lon_deg - lon_ref_deg)
        E = dlon * math.cos(lat0) * R
        N = dlat * R
        U = h_m - h_ref
        return E, N, U

    lat_ref = lat[idx0]
    lon_ref = lot[idx0]
    h_ref   = ht[idx0]
    ts_plot = []
    nav_E = []; nav_N = []; nav_U = []
    tru_E = []; tru_N = []; tru_U = []
    dxs = []; dys = []; dzs = []; hs = []; e3s = []
    for i, (la_n, lo_n, h_n) in enumerate(zip(lan_i, lon_i, hn_i)):
        if la_n is None:
            continue
        dlat = math.radians(la_n - lat[i])
        dlon = math.radians(lo_n - lot[i])
        dx = dlon*math.cos(lat0)*R
        dy = dlat*R
        dz = (h_n - ht[i])
        h  = math.hypot(dx,dy)
        e3 = math.hypot(h, dz)
        rows.append((tt[i], dx, dy, dz, h, e3))
        dxs.append(dx); dys.append(dy); dzs.append(dz); hs.append(h); e3s.append(e3)
        # ENU series (nav/truth)
        e1, n1, u1 = to_ENU(la_n, lo_n, h_n, lat_ref, lon_ref, h_ref)
        e0, n0, u0 = to_ENU(lat[i], lot[i], ht[i], lat_ref, lon_ref, h_ref)
        ts_plot.append(tt[i])
        nav_E.append(e1); nav_N.append(n1); nav_U.append(u1)
        tru_E.append(e0); tru_N.append(n0); tru_U.append(u0)
    if not rows:
        print('No valid rows'); return 1
    n=len(rows)
    rmse_h = (sum(r[4]*r[4] for r in rows)/n)**0.5
    rmse_3 = (sum(r[5]*r[5] for r in rows)/n)**0.5
    rmse_dx = (sum(x*x for x in dxs)/n)**0.5
    rmse_dy = (sum(y*y for y in dys)/n)**0.5
    rmse_dz = (sum(z*z for z in dzs)/n)**0.5
    mean_h = sum(r[4] for r in rows)/n
    max_h  = max(r[4] for r in rows)
    print(f'Samples: {n}')
    print(f'RMSE_h={rmse_h:.3f} m, RMSE_3D={rmse_3:.3f} m, mean_h={mean_h:.3f}, max_h={max_h:.3f}')
    print(f'RMSE components: dx={rmse_dx:.3f} m, dy={rmse_dy:.3f} m, dz={rmse_dz:.3f} m')
    if out_csv:
        out = Path(out_csv)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, 'w') as f:
            f.write('time,dx,dy,dz,horiz,err3d\n')
            for t,dx,dy,dz,h,e3 in rows:
                f.write(f"{t:.3f},{dx:.3f},{dy:.3f},{dz:.3f},{h:.3f},{e3:.3f}\n")
        print('Saved 200Hz errors to:', out)
    if series_csv:
        out = Path(series_csv)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, 'w') as f:
            f.write('time,nav_E,nav_N,nav_U,truth_E,truth_N,truth_U\n')
            for t,e1,n1,u1,e0,n0,u0 in zip(ts_plot, nav_E, nav_N, nav_U, tru_E, tru_N, tru_U):
                f.write(f"{t:.3f},{e1:.3f},{n1:.3f},{u1:.3f},{e0:.3f},{n0:.3f},{u0:.3f}\n")
        print('Saved ENU series to:', out)

    # Optional plotting
    if png_path:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
            axs[0].plot(ts_plot, tru_E, 'k-', lw=1.2, label='Truth')
            axs[0].plot(ts_plot, nav_E, 'b--', lw=1.0, label='Nav')
            axs[0].set_ylabel('X / East [m]')
            axs[0].grid(True, ls='--', alpha=0.4)
            axs[0].legend()

            axs[1].plot(ts_plot, tru_N, 'k-', lw=1.2, label='Truth')
            axs[1].plot(ts_plot, nav_N, 'g--', lw=1.0, label='Nav')
            axs[1].set_ylabel('Y / North [m]')
            axs[1].grid(True, ls='--', alpha=0.4)

            axs[2].plot(ts_plot, tru_U, 'k-', lw=1.2, label='Truth')
            axs[2].plot(ts_plot, nav_U, 'r--', lw=1.0, label='Nav')
            axs[2].set_ylabel('Z / Up [m]')
            axs[2].set_xlabel('time [s]')
            axs[2].grid(True, ls='--', alpha=0.4)

            fig.suptitle(f'Nav vs Truth (X/Y/Z in ENU)\nRMSE_h={rmse_h:.3f} m, RMSE_3D={rmse_3:.3f} m')
            fig.tight_layout(rect=[0, 0.06, 1, 0.94])
            outp = Path(png_path)
            outp.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(outp, dpi=150)
            print('Saved plot to:', outp)
        except Exception as e:
            print('plot skipped (matplotlib not available?):', e)
    if err_png:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, axs = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
            axs[0].plot(ts_plot, dxs, 'r-', lw=1.0)
            axs[0].set_ylabel('dx [m]')
            axs[0].grid(True, ls='--', alpha=0.4)
            axs[0].set_title(f'Errors vs Time (200 Hz)\nRMSE dx={rmse_dx:.3f}, dy={rmse_dy:.3f}, dz={rmse_dz:.3f}, horiz={rmse_h:.3f}, 3D={rmse_3:.3f} [m]')
            axs[1].plot(ts_plot, dys, 'g-', lw=1.0)
            axs[1].set_ylabel('dy [m]')
            axs[1].grid(True, ls='--', alpha=0.4)
            axs[2].plot(ts_plot, dzs, 'b-', lw=1.0)
            axs[2].set_ylabel('dz [m]')
            axs[2].grid(True, ls='--', alpha=0.4)
            axs[3].plot(ts_plot, hs, 'k-', lw=1.0)
            axs[3].set_ylabel('|h| [m]')
            axs[3].set_xlabel('time [s]')
            axs[3].grid(True, ls='--', alpha=0.4)
            fig.tight_layout()
            outp = Path(err_png)
            outp.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(outp, dpi=150)
            print('Saved error plot to:', outp)
        except Exception as e:
            print('error plot skipped (matplotlib not available?):', e)
    if traj_png:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(1, 1, figsize=(7, 7))
            ax.plot(tru_E, tru_N, 'k-', lw=1.2, label='Truth')
            ax.plot(nav_E, nav_N, 'b--', lw=1.0, label='Nav')
            # start/end markers
            if ts_plot:
                ax.plot(tru_E[0], tru_N[0], 'ko', ms=6)
                ax.plot(nav_E[0], nav_N[0], 'bo', ms=6)
                ax.plot(tru_E[-1], tru_N[-1], 'k^', ms=7)
                ax.plot(nav_E[-1], nav_N[-1], 'b^', ms=7)
            ax.set_aspect('equal', adjustable='box')
            ax.grid(True, ls='--', alpha=0.4)
            ax.set_xlabel('East [m]')
            ax.set_ylabel('North [m]')
            ax.set_title(f'Trajectory (ENU) — Nav vs Truth\nRMSE_h={rmse_h:.3f} m, RMSE_3D={rmse_3:.3f} m')
            ax.legend()
            outp = Path(traj_png)
            outp.parent.mkdir(parents=True, exist_ok=True)
            fig.tight_layout()
            fig.savefig(outp, dpi=150)
            print('Saved 2D trajectory plot to:', outp)
        except Exception as e:
            print('traj plot skipped (matplotlib not available?):', e)
    return 0

if __name__=='__main__':
    ap = argparse.ArgumentParser(description='Compare OB_GINS nav against 200 Hz truth with interpolation')
    ap.add_argument('nav', help='Path to OB_GINS_TXT.nav')
    ap.add_argument('truth', help='Path to truth.nav (t,lat,lon,h)')
    ap.add_argument('--out', help='Output CSV path for per-sample errors', default=None)
    ap.add_argument('--series', help='Output CSV path for ENU time series (nav & truth)', default=None)
    ap.add_argument('--png', help='Output PNG path for X/Y/Z time series plot', default=None)
    ap.add_argument('--traj', help='Output PNG path for 2D ENU trajectory plot', default=None)
    ap.add_argument('--errpng', help='Output PNG path for error time series (dx/dy/dz/horiz)', default=None)
    args = ap.parse_args()
    raise SystemExit(compare(args.nav, args.truth, args.out, args.series, args.png, args.traj, args.errpng))

