import sys
from pathlib import Path

def read_gnss(path):
    T = {}
    with open(path, 'r') as f:
        for line in f:
            if not line.strip():
                continue
            parts = line.strip().split()
            if len(parts) < 7:
                continue
            t = float(parts[0])
            lat_deg = float(parts[1])
            lon_deg = float(parts[2])
            h = float(parts[3])
            T[round(t)] = (lat_deg, lon_deg, h)
    return T

def read_nav(path):
    T = {}
    with open(path, 'r') as f:
        for line in f:
            if not line.strip():
                continue
            parts = line.strip().split()
            if len(parts) < 11:
                continue
            t = float(parts[1])
            # select near integer seconds
            ti = round(t)
            if abs(t - ti) <= 0.01:
                lat = float(parts[2])
                lon = float(parts[3])
                h = float(parts[4])
                T[ti] = (lat, lon, h)
    return T

def errors_vs_gnss(nav, gnss):
    import math
    times = sorted(set(nav.keys()) & set(gnss.keys()))
    if not times:
        return None
    # base latitude for projection
    lat0 = math.radians(gnss[times[0]][0])
    R = 6378137.0
    errs = []
    for t in times:
        lat_n, lon_n, h_n = nav[t]
        lat_g, lon_g, h_g = gnss[t]
        dlat = math.radians(lat_n - lat_g)
        dlon = math.radians(lon_n - lon_g)
        dx = dlon * math.cos(lat0) * R
        dy = dlat * R
        dz = h_n - h_g
        h_err = math.hypot(dx, dy)
        err3d = math.sqrt(h_err*h_err + dz*dz)
        errs.append((t, dx, dy, dz, h_err, err3d))
    return errs

def summarize(errs):
    import math
    if not errs:
        return {}
    n = len(errs)
    he = [e[4] for e in errs]
    e3 = [e[5] for e in errs]
    rmse_h = math.sqrt(sum(x*x for x in he)/n)
    rmse_3d = math.sqrt(sum(x*x for x in e3)/n)
    mean_h = sum(he)/n
    mean_3d = sum(e3)/n
    max_h = max(he)
    max_3d = max(e3)
    return dict(n=n, rmse_h=rmse_h, rmse_3d=rmse_3d, mean_h=mean_h, mean_3d=mean_3d, max_h=max_h, max_3d=max_3d)

def main():
    if len(sys.argv) < 4:
        print("Usage: compare_nav_gnss.py <nav1> <nav2> <gnss>")
        sys.exit(1)
    nav1 = read_nav(sys.argv[1])
    nav2 = read_nav(sys.argv[2])
    gnss = read_gnss(sys.argv[3])
    e1 = errors_vs_gnss(nav1, gnss)
    e2 = errors_vs_gnss(nav2, gnss)
    s1 = summarize(e1)
    s2 = summarize(e2)
    print("Samples:", s1.get('n'), s2.get('n'))
    print("NHC ON  RMSE_h=%.3f m, RMSE_3D=%.3f m, mean_h=%.3f, max_h=%.3f" % (s1['rmse_h'], s1['rmse_3d'], s1['mean_h'], s1['max_h']))
    print("NHC OFF RMSE_h=%.3f m, RMSE_3D=%.3f m, mean_h=%.3f, max_h=%.3f" % (s2['rmse_h'], s2['rmse_3d'], s2['mean_h'], s2['max_h']))
    # dump per-second errors
    out = Path(sys.argv[1]).parent.parent / 'liu_nhc_compare.csv'
    with open(out, 'w') as f:
        f.write('time,dx_on,dy_on,dz_on,h_on,err3d_on,dx_off,dy_off,dz_off,h_off,err3d_off\n')
        times = sorted(set([t for t,_1,_2,_3,_4,_5 in e1]) & set([t for t,_1,_2,_3,_4,_5 in e2]))
        d1 = {t:(dx,dy,dz,h,err3d) for t,dx,dy,dz,h,err3d in e1}
        d2 = {t:(dx,dy,dz,h,err3d) for t,dx,dy,dz,h,err3d in e2}
        for t in times:
            a = d1[t]; b = d2[t]
            f.write(f"{t},{a[0]:.3f},{a[1]:.3f},{a[2]:.3f},{a[3]:.3f},{a[4]:.3f},{b[0]:.3f},{b[1]:.3f},{b[2]:.3f},{b[3]:.3f},{b[4]:.3f}\n")
    print("Saved per-second errors to:", out)

if __name__ == '__main__':
    main()

