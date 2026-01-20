import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
import glob

def blh_to_enu(blh, origin_blh):
    """
    将 BLH (纬度, 经度, 高度) 转换为局部 ENU 坐标系。
    blh: Nx3 数组 (度, 度, 米)
    origin_blh: 1x3 数组 (起始点)
    """
    lat = np.radians(blh[:, 0])
    lon = np.radians(blh[:, 1])
    h = blh[:, 2]
    
    lat0 = np.radians(origin_blh[0])
    lon0 = np.radians(origin_blh[1])
    h0 = origin_blh[2]
    
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = f * (2 - f)
    
    def ecef(lat, lon, h):
        v = a / np.sqrt(1 - e2 * np.sin(lat)**2)
        x = (v + h) * np.cos(lat) * np.cos(lon)
        y = (v + h) * np.cos(lat) * np.sin(lon)
        z = (v * (1 - e2) + h) * np.sin(lat)
        if np.isscalar(lat) or lat.ndim == 0:
             return np.array([x, y, z])
        return np.stack([x, y, z], axis=1)

    p = ecef(lat, lon, h)
    p0 = ecef(lat0, lon0, h0)
    
    dp = p - p0
    
    sin_lat0, cos_lat0 = np.sin(lat0), np.cos(lat0)
    sin_lon0, cos_lon0 = np.sin(lon0), np.cos(lon0)
    
    t = np.array([
        [-sin_lon0, cos_lon0, 0],
        [-sin_lat0 * cos_lon0, -sin_lat0 * sin_lon0, cos_lat0],
        [cos_lat0 * cos_lon0, cos_lat0 * sin_lon0, sin_lat0]
    ])
    
    return dp @ t.T

def main():
    parser = argparse.ArgumentParser(description="Plot OB-GINS-CT results vs Truth")
    parser.add_argument("--result", type=str, required=True, help="Path to ct_trajectory.txt")
    parser.add_argument("--truth", type=str, required=True, help="Path to GNSS truth file")
    parser.add_argument("--label", type=str, default="OB_GINS_CT")
    args = parser.parse_args()

    if not os.path.exists(args.result):
        print(f"Error: Result file {args.result} not found.")
        return

    # 1. Load Navigation Results
    # Format: time, lat, lon, alt, vx, vy, vz, roll, pitch, yaw
    print(f"Loading result: {args.result}")
    res_data = np.loadtxt(args.result)
    res_time = res_data[:, 0]
    res_blh = res_data[:, 1:4]
    res_vel = res_data[:, 4:7]
    res_att = res_data[:, 7:10]

    # 2. Load Truth Data
    # Format: time, lat, lon, alt, ...
    print(f"Loading truth: {args.truth}")
    truth_data = np.loadtxt(args.truth)
    truth_time = truth_data[:, 0]
    truth_blh = truth_data[:, 1:4]

    # 3. Convert to ENU for comparison
    origin_blh = truth_blh[0]
    truth_enu = blh_to_enu(truth_blh, origin_blh)
    res_enu = blh_to_enu(res_blh, origin_blh)

    # Align trajectories (time-based check)
    t0 = res_time[0]
    t_end = res_time[-1]
    
    # Filter truth to match result time window roughly for easier plotting
    mask = (truth_time >= t0 - 1.0) & (truth_time <= t_end + 1.0)
    truth_time_plot = truth_time[mask]
    truth_enu_plot = truth_enu[mask]
    truth_blh_plot = truth_blh[mask]

    # --- Plot 1: Position (ENU) ---
    fig1, axs1 = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    labels = ['East (m)', 'North (m)', 'Up (m)']
    for i in range(3):
        axs1[i].plot(truth_time_plot, truth_enu_plot[:, i], 'k--', label='Truth')
        axs1[i].plot(res_time, res_enu[:, i], 'r-', label=args.label)
        axs1[i].set_ylabel(labels[i])
        axs1[i].grid(True)
        axs1[i].legend()
    axs1[2].set_xlabel('Time (s)')
    fig1.suptitle('Position Comparison (Local ENU)')
    plt.tight_layout()

    # --- Plot 2: 2D Trajectory ---
    plt.figure(figsize=(8, 8))
    plt.plot(truth_enu_plot[:, 0], truth_enu_plot[:, 1], 'k--', label='Truth')
    plt.plot(res_enu[:, 0], res_enu[:, 1], 'r-', label=args.label)
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('Horizontal Trajectory')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)

    # --- Plot 3: Position Errors ---
    # Interpolate Truth to Result Time
    truth_east_interp = np.interp(res_time, truth_time, truth_enu[:, 0])
    truth_north_interp = np.interp(res_time, truth_time, truth_enu[:, 1])
    truth_alt_interp = np.interp(res_time, truth_time, truth_blh[:, 2])
    
    res_east = res_enu[:, 0]
    res_north = res_enu[:, 1]
    res_alt = res_blh[:, 2]

    error_east = res_east - truth_east_interp
    error_north = res_north - truth_north_interp
    error_horiz = np.sqrt(error_east**2 + error_north**2)
    error_height = res_alt - truth_alt_interp

    plt.figure(figsize=(10, 8))
    
    plt.subplot(3, 1, 1)
    plt.plot(res_time, error_horiz, 'r-', label='Horizontal Error')
    plt.ylabel('Error (m)')
    plt.title('Horizontal Position Error')
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(res_time, error_east, label='East Error')
    plt.plot(res_time, error_north, label='North Error')
    plt.ylabel('Error (m)')
    plt.title('East/North Errors')
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(res_time, error_height, 'b-', label='Height Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m)')
    plt.title('Height Error (Nav - Truth)')
    plt.grid(True)
    plt.legend()
    
    plt.tight_layout()

    # --- Plot 4+: IMU Errors ---
    output_dir = os.path.dirname(args.result)
    error_files = glob.glob(os.path.join(output_dir, "errors_*.txt"))
    
    for ef in error_files:
        imu_name = os.path.basename(ef).replace("errors_", "").replace(".txt", "")
        print(f"Plotting errors for: {imu_name}")
        
        # Format: t, bg(3), ba(3), l(3), r(4)
        err_data = np.loadtxt(ef)
        if err_data.ndim < 2: continue # Skip empty or single line
        
        e_time = err_data[:, 0]
        bg = err_data[:, 1:4]
        ba = err_data[:, 4:7]
        
        fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        
        axs[0].plot(e_time, bg[:, 0], label='X')
        axs[0].plot(e_time, bg[:, 1], label='Y')
        axs[0].plot(e_time, bg[:, 2], label='Z')
        axs[0].set_ylabel('Gyro Bias (rad/s)')
        axs[0].set_title(f'{imu_name} Gyro Bias')
        axs[0].grid(True)
        axs[0].legend()
        
        axs[1].plot(e_time, ba[:, 0], label='X')
        axs[1].plot(e_time, ba[:, 1], label='Y')
        axs[1].plot(e_time, ba[:, 2], label='Z')
        axs[1].set_ylabel('Accel Bias (m/s^2)')
        axs[1].set_title(f'{imu_name} Accel Bias')
        axs[1].grid(True)
        axs[1].legend()
        
        axs[1].set_xlabel('Time (s)')
        plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    main()