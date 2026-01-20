import numpy as np
import matplotlib.pyplot as plt
import argparse
import os

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

    # 加载结果数据: time, x, y, z, qx, qy, qz, qw (局部坐标系)
    res_data = np.loadtxt(args.result)
    res_time = res_data[:, 0]
    res_pos = res_data[:, 1:4]

    # 加载真值数据: time, lat, lon, h, ...
    # 假设 GnssFileLoader 格式为: time, lat, lon, h
    truth_data = np.loadtxt(args.truth)
    truth_time = truth_data[:, 0]
    truth_blh = truth_data[:, 1:4]

    # 使用第一个真值点作为原点，将真值转换为 ENU
    origin_blh = truth_blh[0]
    truth_enu = blh_to_enu(truth_blh, origin_blh)

    # 绘制三轴位置对比图
    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    labels = ['East (m)', 'North (m)', 'Up (m)']
    for i in range(3):
        axs[i].plot(truth_time, truth_enu[:, i], 'k--', label='Truth (GNSS)')
        axs[i].plot(res_time, res_pos[:, i], 'r-', label=args.label)
        axs[i].set_ylabel(labels[i])
        axs[i].legend()
        axs[i].grid(True)
    axs[2].set_xlabel('Time (s)')
    plt.suptitle('Trajectory Comparison (Local ENU)')
    plt.tight_layout()

    # 绘制 2D 水平轨迹对比图
    plt.figure(figsize=(8, 8))
    plt.plot(truth_enu[:, 0], truth_enu[:, 1], 'k--', label='Truth')
    plt.plot(res_pos[:, 0], res_pos[:, 1], 'r-', label=args.label)
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('Horizontal Trajectory Comparison')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    
    plt.show()

if __name__ == "__main__":
    main()