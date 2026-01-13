import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import glob

def load_nav(filepath):
    """
    Load OB_GINS navigation output file.
    Expected format: Index, Time, Lat(deg), Lon(deg), H(m), Vn, Ve, Vd, Roll, Pitch, Yaw
    """
    try:
        # Use sep='\s+' to handle varying spaces
        df = pd.read_csv(filepath, sep=r'\s+', header=None)
        
        # Check if we have enough columns (at least 11)
        if df.shape[1] >= 11:
            # Assign standard column names
            # Note: We assume the file follows the standard OB_GINS output format
            cols = ['idx', 'time', 'lat', 'lon', 'h', 'vn', 've', 'vd', 'roll', 'pitch', 'yaw']
            # If there are extra columns (e.g. from modified code), append generic names
            if df.shape[1] > 11:
                cols += [f'col_{i}' for i in range(11, df.shape[1])]
            
            df.columns = cols
            return df
        else:
            print(f"[Error] Nav file has fewer columns than expected (11): {df.shape[1]}")
            return None
    except Exception as e:
        print(f"[Error] Failed to load nav file: {e}")
        return None

def load_gnss(filepath):
    """
    Load GNSS/Truth file.
    Expected format: Time, Lat(deg), Lon(deg), H(m), ...
    """
    try:
        df = pd.read_csv(filepath, sep=r'\s+', header=None)
        if df.shape[1] >= 4:
            cols = ['time', 'lat', 'lon', 'h']
            if df.shape[1] > 4:
                cols += [f'col_{i}' for i in range(4, df.shape[1])]
            df.columns = cols
            return df
        else:
            print(f"[Error] GNSS file has fewer columns than expected (4): {df.shape[1]}")
            return None
    except Exception as e:
        print(f"[Error] Failed to load GNSS file: {e}")
        return None

def calculate_errors(nav_df, truth_df):
    """
    Calculate position errors by aligning Nav and Truth data on time.
    """
    print("[Info] Aligning data...")
    
    # Drop any rows with NaN in 'time' column to avoid merge_asof errors
    nav_df = nav_df.dropna(subset=['time'])
    truth_df = truth_df.dropna(subset=['time'])

    # Ensure sorted by time
    nav_df = nav_df.sort_values('time')
    truth_df = truth_df.sort_values('time')
    
    # Use merge_asof to find the nearest truth timestamp for each nav timestamp
    # tolerance=0.01s ensures we don't match points that are too far apart
    merged = pd.merge_asof(nav_df, truth_df, on='time', suffixes=('_est', '_true'), direction='nearest', tolerance=0.01)
    
    # Filter out rows where no match was found (NaNs in truth columns)
    merged = merged.dropna(subset=['lat_true'])
    
    if merged.empty:
        print("[Warning] No matching timestamps found between Nav and Truth (tolerance=10ms).")
        return pd.DataFrame() # Return empty
    
    print(f"[Info] Matched {len(merged)} points.")

    # Calculate position errors in meters (approximate flat earth for small diffs)
    # Earth radius
    R = 6378137.0
    
    # Lat/Lon differences in radians
    d_lat_rad = np.deg2rad(merged['lat_est'] - merged['lat_true'])
    d_lon_rad = np.deg2rad(merged['lon_est'] - merged['lon_true'])
    
    # North/East errors
    merged['error_n'] = d_lat_rad * R
    merged['error_e'] = d_lon_rad * R * np.cos(np.deg2rad(merged['lat_true']))
    
    # Down error (Height estimated - Height truth). 
    # Note: NED frame 'Down' is opposite to Height. 
    # If H_est > H_true, we are "above", so error_d should be negative?
    # Usually Error = Est - Truth.
    # Error_Height = H_est - H_true. 
    # Error_Down = -(Error_Height)
    merged['error_h'] = merged['h_est'] - merged['h_true']
    merged['error_d'] = -merged['error_h'] 
    
    return merged

def plot_results(merged_df, output_dir, nav_filename):
    print("[Info] Plotting results...")
    
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(f'OB_GINS Navigation vs GNSS Truth\n({nav_filename})', fontsize=16)

    # 1. Trajectory (Lat vs Lon)
    ax1 = plt.subplot(2, 2, 1)
    ax1.plot(merged_df['lon_true'], merged_df['lat_true'], 'k-', label='Truth', alpha=0.5, linewidth=2)
    ax1.plot(merged_df['lon_est'], merged_df['lat_est'], 'r--', label='Est', linewidth=1)
    ax1.set_xlabel('Longitude (deg)')
    ax1.set_ylabel('Latitude (deg)')
    ax1.set_title('2D Trajectory')
    ax1.legend()
    ax1.grid(True)
    # Make aspect ratio equal for map
    ax1.set_aspect('equal', 'box')

    # 2. Position Errors (N, E, D vs Time)
    ax2 = plt.subplot(2, 2, 2)
    ax2.plot(merged_df['time'], merged_df['error_n'], label='North', alpha=0.8)
    ax2.plot(merged_df['time'], merged_df['error_e'], label='East', alpha=0.8)
    # ax2.plot(merged_df['time'], merged_df['error_d'], label='Down', alpha=0.8)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error (m)')
    ax2.set_title('Position Errors (NE)')
    ax2.legend()
    ax2.grid(True)
    
    # 3. Height Profile
    ax3 = plt.subplot(2, 2, 3)
    ax3.plot(merged_df['time'], merged_df['h_true'], 'k-', label='Truth')
    ax3.plot(merged_df['time'], merged_df['h_est'], 'r--', label='Est')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Height (m)')
    ax3.set_title('Height Profile')
    ax3.legend()
    ax3.grid(True)

    # 4. Statistics Text
    ax4 = plt.subplot(2, 2, 4)
    ax4.axis('off') # Turn off axis
    
    rmse_n = np.sqrt(np.mean(merged_df['error_n']**2))
    rmse_e = np.sqrt(np.mean(merged_df['error_e']**2))
    rmse_d = np.sqrt(np.mean(merged_df['error_d']**2))
    rmse_h = np.sqrt(np.mean(merged_df['error_h']**2))
    
    max_n = np.max(np.abs(merged_df['error_n']))
    max_e = np.max(np.abs(merged_df['error_e']))
    max_d = np.max(np.abs(merged_df['error_d']))

    stats = [
        ["Metric", "North (m)", "East (m)", "Down (m)"],
        ["RMSE", f"{rmse_n:.3f}", f"{rmse_e:.3f}", f"{rmse_d:.3f}"],
        ["Max Abs", f"{max_n:.3f}", f"{max_e:.3f}", f"{max_d:.3f}"]
    ]
    
    table = ax4.table(cellText=stats, loc='center', cellLoc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(12)
    table.scale(1, 2)
    ax4.set_title('Error Statistics')

    # Save
    out_filename = f'comparison_plot_{os.path.splitext(nav_filename)[0]}.png'
    out_path = os.path.join(output_dir, out_filename)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(out_path)
    print(f"[Success] Plot saved to: {out_path}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python plot_comparison.py <nav_file_or_dir> <truth_file>")
        sys.exit(1)
        
    nav_input = sys.argv[1]
    truth_path = sys.argv[2]
    
    print("------------------------------------------------")
    print(" Running OB_GINS Comparison Script")
    print("------------------------------------------------")
    print(f"Nav Input:  {nav_input}")
    print(f"Truth File: {truth_path}")
    
    if not os.path.exists(truth_path):
        print(f"[Error] Truth file not found: {truth_path}")
        sys.exit(1)

    # Determine search directory
    if os.path.isdir(nav_input):
        search_dir = nav_input
    else:
        search_dir = os.path.dirname(nav_input)
        if not search_dir: # Handle current directory case if path is just "file.nav"
            search_dir = "."

    print(f"[Info] Scanning directory: {search_dir}")
    
    # Find files matching patterns
    nav_files = []
    patterns = ["*.nav"]
    for p in patterns:
        full_pattern = os.path.join(search_dir, p)
        found = glob.glob(full_pattern)
        nav_files.extend(found)
    
    # Remove duplicates and sort
    nav_files = sorted(list(set(nav_files)))
    
    # If explicit input file was given but not found by pattern (e.g. different name), add it
    if os.path.isfile(nav_input) and os.path.abspath(nav_input) not in [os.path.abspath(f) for f in nav_files]:
        print(f"[Info] Adding explicitly provided file: {nav_input}")
        nav_files.append(nav_input)

    if not nav_files:
        print("[Warning] No matching navigation files found.")
        sys.exit(0)
    
    print(f"[Info] Found {len(nav_files)} files to process.")

    # Load GNSS once
    gnss_df = load_gnss(truth_path)
    
    if gnss_df is not None:
        for nav_file in nav_files:
            print(f"\nProcessing: {os.path.basename(nav_file)}")
            nav_df = load_nav(nav_file)
            
            if nav_df is not None:
                merged_df = calculate_errors(nav_df, gnss_df)
                
                if not merged_df.empty:
                    # Output directory is same as nav file directory
                    output_dir = os.path.dirname(nav_file)
                    nav_filename = os.path.basename(nav_file)
                    plot_results(merged_df, output_dir, nav_filename)
                    
                    # Save errors to CSV for inspection
                    csv_out_filename = f'errors_{os.path.splitext(nav_filename)[0]}.csv'
                    csv_out = os.path.join(output_dir, csv_out_filename)
                    merged_df[['time', 'error_n', 'error_e', 'error_d', 'lat_est', 'lat_true']].to_csv(csv_out, index=False)
                    print(f"[Success] Error data saved to: {csv_out}")
                else:
                    print(f"[Warning] Comparison failed for {nav_filename} due to no data match.")
    else:
        print("[Error] Failed to load GNSS file.")
