#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Convert Ground Truth GINS.bin (float64, N×5) to human-readable text files.

Outputs in the same folder:
- GINS.txt           : raw 5-column text dump
- truth_200hz.nav    : text with columns: week time lat lon h (200 Hz rows)

Usage:
  python convert_gins_bin.py "D:/Code/OB_GINS/dataset/robot/Ground Truth/GINS.bin"
"""
import argparse
from pathlib import Path
import numpy as np


def convert(bin_path: Path):
    bin_path = Path(bin_path)
    if not bin_path.exists():
        raise SystemExit(f"Input not found: {bin_path}")
    out_dir = bin_path.parent

    # Load and reshape
    M = np.fromfile(bin_path, dtype=np.float64)
    if M.size % 5 != 0:
        raise SystemExit(f"Unexpected size in {bin_path} (not multiple of 5)")
    M = M.reshape(-1, 5)

    # Dump raw text
    txt_path = out_dir / 'GINS.txt'
    np.savetxt(txt_path, M, fmt='%.3f %.10f %.10f %.3f %.6f')

    # Keep position rows (time > 1) and sort by time
    pos = M[M[:, 0] > 1.0]
    pos = pos[np.argsort(pos[:, 0])]

    # Write 200 Hz truth NAV text: week time lat lon h
    nav200 = out_dir / 'truth_200hz.nav'
    with open(nav200, 'w') as f:
        for t, la, lo, h, _ in pos:
            f.write(f"2140 {t:.3f} {la:.10f} {lo:.10f} {h:.3f}\n")

    print('Wrote:', txt_path)
    print('Wrote:', nav200)


def main():
    ap = argparse.ArgumentParser(description='Convert GINS.bin -> GINS.txt and truth_200hz.nav')
    ap.add_argument('bin', help='Path to Ground Truth/GINS.bin')
    args = ap.parse_args()
    convert(Path(args.bin))


if __name__ == '__main__':
    main()

