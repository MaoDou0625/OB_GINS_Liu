#!/usr/bin/env python3
"""
merge_imu_odo_trapezoid_txt.py
---------------------------------
Merge a high-rate IMU stream (rates) with a low-rate ODO stream (1 Hz by default)
into a single IMU+ODO file for OB_GINS using the 9-column *trapezoid* scheme.

OB_GINS expects:
  t, dtheta_x, dtheta_y, dtheta_z, dvel_x, dvel_y, dvel_z, v_prev, v_curr

This version adds:
  - Support for .txt/.csv input & output
  - Custom input delimiters (--imu-sep, --odo-sep): 'comma', 'tab', 'space', or a literal char
  - Custom output delimiter (--out-sep): 'comma', 'tab', 'space', or a literal char

USAGE (Windows paths need quotes)
-----
python merge_imu_odo_trapezoid_txt.py ^
  --imu "D:\\Code\\OB_GINS\\dataset\\car\\Body_imu.txt" ^
  --odo "D:\\Code\\OB_GINS\\dataset\\car\\odo.txt" ^
  --out "D:\\Code\\OB_GINS\\dataset\\car\\imu_odo_9col.txt" ^
  --imu-rate 200 ^
  --imu-cols t,wx,wy,wz,ax,ay,az ^
  --odo-cols t,v ^
  --imu-sep space --odo-sep space --out-sep space

Notes
-----
- IMU inputs must be *rates*: wx, wy, wz [rad/s], ax, ay, az [m/s^2].
  This script integrates per IMU-step to Δθ and Δv.
- ODO inputs can be either velocity v [m/s] or distance increment ds [m] (use --odo-cols t,ds).
- ODO is linearly interpolated to each IMU timestamp to form v_curr;
  v_prev is the previous IMU step's interpolated velocity (v_prev = v_curr at the first row).
- Output uses 9 columns; OB_GINS will compute Δs = 0.5*(v_prev+v_curr)*Δt_imu.
"""

import argparse
import sys
from typing import Tuple, List
import numpy as np
import pandas as pd

def parse_cols(spec: str, expected_len: int) -> List[str]:
    cols = [c.strip() for c in spec.split(",")]
    if len(cols) != expected_len:
        raise ValueError(f"Expect {expected_len} column names, got {len(cols)} in '{spec}'")
    return cols

def sep_to_params(sep: str):
    sep = sep.lower()
    if sep in ("comma", ","):
        return { "sep": "," }
    if sep in ("tab", "\\t"):
        return { "sep": "\t" }
    if sep in ("space", "whitespace"):
        # use delim_whitespace for robustness (multiple spaces)
        return { "delim_whitespace": True }
    # literal one-char or multi-char separator
    return { "sep": sep }

def write_with_sep(df: pd.DataFrame, path: str, out_sep: str):
    out_sep = out_sep.lower()
    if out_sep in ("comma", ","):
        sep = ","
    elif out_sep in ("tab", "\\t"):
        sep = "\t"
    elif out_sep in ("space", "whitespace"):
        sep = " "
    else:
        sep = out_sep
    df.to_csv(path, index=False, sep=sep, float_format="%.9f")

def load_imu(path: str, has_header: bool, imu_cols: List[str], time_unit: str, sep_spec: str) -> pd.DataFrame:
    read_kwargs = sep_to_params(sep_spec)
    if has_header:
        df = pd.read_csv(path, **read_kwargs)
        missing = [c for c in imu_cols if c not in df.columns]
        if missing:
            raise ValueError(f"IMU file missing columns: {missing}")
        df = df[imu_cols].copy()
        df.columns = ["t","wx","wy","wz","ax","ay","az"]
    else:
        df = pd.read_csv(path, header=None, **read_kwargs,
                         names=["t","wx","wy","wz","ax","ay","az"])
    if time_unit.lower() == "ns":
        df["t"] = df["t"].astype(np.float64) * 1e-9
    df = df.sort_values("t").reset_index(drop=True)
    return df

def load_odo(path: str, has_header: bool, odo_cols: List[str], time_unit: str, sep_spec: str) -> pd.DataFrame:
    read_kwargs = sep_to_params(sep_spec)
    if has_header:
        df = pd.read_csv(path, **read_kwargs)
        missing = [c for c in odo_cols if c not in df.columns]
        if missing:
            raise ValueError(f"ODO file missing columns: {missing}")
        df = df[odo_cols].copy()
        df.columns = ["t", odo_cols[1]]  # keep second name
    else:
        df = pd.read_csv(path, header=None, **read_kwargs)
        if df.shape[1] != 2:
            raise ValueError("ODO file without header must have exactly 2 columns (t, v) or (t, ds)")
        df.columns = ["t","val"]
        odo_cols = ["t","val"]
    if time_unit.lower() == "ns":
        df["t"] = df["t"].astype(np.float64) * 1e-9
    df = df.sort_values("t").reset_index(drop=True)
    # Normalize to have a column 'v' in m/s
    second = df.columns[1]
    if second.lower() == "ds":
        # compute per-ODO interval velocity; forward-fill last value
        t = df["t"].to_numpy()
        ds = df["ds"].to_numpy(dtype=np.float64)
        dt = np.diff(t, prepend=t[0])
        dt[0] = dt[1] if len(dt) > 1 else 1.0  # fallback
        v = np.divide(ds, dt, out=np.zeros_like(ds), where=dt>0)
        df = pd.DataFrame({"t": t, "v": v})
    elif second.lower() == "v":
        df = pd.DataFrame({"t": df["t"].to_numpy(dtype=np.float64),
                           "v": df[second].to_numpy(dtype=np.float64)})
    else:
        # assume it's velocity if not named 'ds'
        df = pd.DataFrame({"t": df["t"].to_numpy(dtype=np.float64),
                           "v": df[second].to_numpy(dtype=np.float64)})
    return df

def interp_odo_to_imu(imu_t: np.ndarray, odo_t: np.ndarray, odo_v: np.ndarray, mode: str="hold") -> np.ndarray:
    if len(odo_t) == 0:
        raise ValueError("ODO stream is empty.")
    imu_t = imu_t.astype(np.float64)
    odo_t = odo_t.astype(np.float64)
    odo_v = odo_v.astype(np.float64)
    v_interp = np.interp(imu_t, odo_t, odo_v)  # linear inside, edge hold outside
    if mode not in ("hold","edge"):
        print(f"[warn] Unknown mode '{mode}', using 'hold'.", file=sys.stderr)
    return v_interp

def build_output(imu: pd.DataFrame, v_interp: np.ndarray) -> pd.DataFrame:
    t = imu["t"].to_numpy()
    dt = np.diff(t, prepend=t[0])
    if len(dt) > 1 and dt[0] <= 0:
        dt[0] = dt[1]
    dtheta = imu[["wx","wy","wz"]].to_numpy() * dt[:,None]
    dvel   = imu[["ax","ay","az"]].to_numpy() * dt[:,None]
    v_curr = v_interp
    v_prev = np.roll(v_curr, 1)
    if len(v_prev) > 0:
        v_prev[0] = v_curr[0]
    out = pd.DataFrame({
        "t": t,
        "dtheta_x": dtheta[:,0],
        "dtheta_y": dtheta[:,1],
        "dtheta_z": dtheta[:,2],
        "dvel_x":   dvel[:,0],
        "dvel_y":   dvel[:,1],
        "dvel_z":   dvel[:,2],
        "v_prev":   v_prev,
        "v_curr":   v_curr
    })
    return out

def main():
    ap = argparse.ArgumentParser(description="Merge IMU (rates) and ODO (1 Hz) to OB_GINS 9-column (trapezoid). Supports .txt and custom separators.")
    ap.add_argument("--imu", required=True, help="IMU file path (.txt/.csv) with t,wx,wy,wz,ax,ay,az.")
    ap.add_argument("--odo", required=True, help="ODO file path (.txt/.csv) with t,v or t,ds.")
    ap.add_argument("--out", required=True, help="Output path (.txt/.csv).")
    ap.add_argument("--imu-rate", type=float, default=None, help="(Optional) nominal IMU rate for sanity check.")
    ap.add_argument("--imu-cols", default="t,wx,wy,wz,ax,ay,az", help="IMU column names if header exists.")
    ap.add_argument("--odo-cols", default="t,v", help="ODO column names if header exists. Use 't,ds' if ds given.")
    ap.add_argument("--time-unit", choices=["s","ns"], default="s", help="Timestamp unit in input files.")
    ap.add_argument("--odo-fill", choices=["hold","edge"], default="hold", help="Extrapolation outside ODO range.")
    ap.add_argument("--no-header", action="store_true", help="Set if input files have NO header.")
    ap.add_argument("--imu-sep", default="comma", help="IMU input separator: 'comma','tab','space', or literal char.")
    ap.add_argument("--odo-sep", default="comma", help="ODO input separator: 'comma','tab','space', or literal char.")
    ap.add_argument("--out-sep", default="comma", help="Output separator: 'comma','tab','space', or literal char.")
    args = ap.parse_args()

    imu_cols = parse_cols(args.imu_cols, 7)
    odo_cols = parse_cols(args.odo_cols, 2)

    imu = load_imu(args.imu, has_header=(not args.no_header), imu_cols=imu_cols, time_unit=args.time_unit, sep_spec=args.imu_sep)
    odo = load_odo(args.odo, has_header=(not args.no_header), odo_cols=odo_cols, time_unit=args.time_unit, sep_spec=args.odo_sep)

    if args.imu_rate is not None and len(imu) >= 2:
        est_rate = 1.0 / np.median(np.diff(imu["t"].to_numpy()))
        if abs(est_rate - args.imu_rate) > 0.1:
            print(f"[warn] Estimated IMU rate {est_rate:.3f} Hz differs from --imu-rate {args.imu_rate:.3f} Hz.", file=sys.stderr)

    v_interp = interp_odo_to_imu(imu["t"].to_numpy(), odo["t"].to_numpy(), odo["v"].to_numpy(), mode=args.odo_fill)
    out = build_output(imu, v_interp)
    write_with_sep(out, args.out, args.out_sep)
    print(f"[ok] Wrote {len(out)} rows to {args.out}")
    if len(out) >= 3:
        dt0 = out.loc[1,"t"] - out.loc[0,"t"]
        ds0 = 0.5*(out.loc[0,"v_prev"] + out.loc[0,"v_curr"]) * dt0
        print(f"[info] First Δt≈{dt0:.6f}s, trapezoid Δs≈{ds0:.6f} m")

if __name__ == "__main__":
    main()
