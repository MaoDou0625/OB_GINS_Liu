import matplotlib.pyplot as plt
import pandas as pd
import sys

def plot_results(file_path):
    # Read the file, skipping the header line starts with #
    try:
        data = pd.read_csv(file_path, sep='\s+', comment='#', header=None)
        # Columns: Time Tx Ty Tz Qx Qy Qz Qw Vx Vy Vz Bgx Bgy Bgz Bax Bay Baz
        columns = ['Time', 'Tx', 'Ty', 'Tz', 'Qx', 'Qy', 'Qz', 'Qw', 
                   'Vx', 'Vy', 'Vz', 'Bgx', 'Bgy', 'Bgz', 'Bax', 'Bay', 'Baz']
        data.columns = columns
    except Exception as e:
        print(f"Error reading file: {e}")
        return

    # Try to load GNSS data
    gnss_path = file_path.replace('OB_GINS_CT.txt', 'OB_GINS_CT_GNSS.txt')
    gnss_data = None
    try:
        gnss_data = pd.read_csv(gnss_path, sep='\s+', comment='#', header=None)
        # Time Tx Ty Tz
        gnss_data.columns = ['Time', 'Tx', 'Ty', 'Tz']
        print(f"Loaded GNSS data from {gnss_path}")
    except:
        print(f"Could not load GNSS data from {gnss_path}, skipping comparison.")

    # Plot Position
    plt.figure(figsize=(10, 8))
    plt.subplot(3, 1, 1)
    plt.plot(data['Time'], data['Tx'], label='Est Tx', linewidth=1.5)
    if gnss_data is not None:
        plt.scatter(gnss_data['Time'], gnss_data['Tx'], c='r', marker='x', s=10, label='GNSS Tx', alpha=0.5)
    plt.ylabel('X (m)')
    plt.title('Position Comparison')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3, 1, 2)
    plt.plot(data['Time'], data['Ty'], label='Est Ty', linewidth=1.5)
    if gnss_data is not None:
        plt.scatter(gnss_data['Time'], gnss_data['Ty'], c='r', marker='x', s=10, label='GNSS Ty', alpha=0.5)
    plt.ylabel('Y (m)')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3, 1, 3)
    plt.plot(data['Time'], data['Tz'], label='Est Tz', linewidth=1.5)
    if gnss_data is not None:
        plt.scatter(gnss_data['Time'], gnss_data['Tz'], c='r', marker='x', s=10, label='GNSS Tz', alpha=0.5)
    plt.ylabel('Z (m)')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('ct_position.png')

    # Plot Error (if GNSS available)
    if gnss_data is not None:
        import numpy as np
        
        # Interpolate Estimate to GNSS time
        # Handle time range: only interp within overlapping range
        t_est = data['Time'].values
        t_gnss = gnss_data['Time'].values
        
        # Find common time range
        t_min = max(t_est[0], t_gnss[0])
        t_max = min(t_est[-1], t_gnss[-1])
        
        mask = (t_gnss >= t_min) & (t_gnss <= t_max)
        t_eval = t_gnss[mask]
        gnss_eval = gnss_data.loc[mask, ['Tx', 'Ty', 'Tz']].values
        
        # Interpolate
        # est_pos = np.zeros_like(gnss_eval)
        est_tx = np.interp(t_eval, t_est, data['Tx'].values)
        est_ty = np.interp(t_eval, t_est, data['Ty'].values)
        est_tz = np.interp(t_eval, t_est, data['Tz'].values)
        
        err_x = est_tx - gnss_eval[:, 0]
        err_y = est_ty - gnss_eval[:, 1]
        err_z = est_tz - gnss_eval[:, 2]
        
        rmse_x = np.sqrt(np.mean(err_x**2))
        rmse_y = np.sqrt(np.mean(err_y**2))
        rmse_z = np.sqrt(np.mean(err_z**2))
        
        print(f"Error RMSE -> X: {rmse_x:.4f} m, Y: {rmse_y:.4f} m, Z: {rmse_z:.4f} m")
        
        plt.figure(figsize=(10, 8))
        plt.subplot(3, 1, 1)
        plt.plot(t_eval, err_x, label=f'Err X (RMSE={rmse_x:.2f})')
        plt.ylabel('Error X (m)')
        plt.title('Position Error (Est - GNSS)')
        plt.grid(True)
        plt.legend()
        
        plt.subplot(3, 1, 2)
        plt.plot(t_eval, err_y, label=f'Err Y (RMSE={rmse_y:.2f})')
        plt.ylabel('Error Y (m)')
        plt.grid(True)
        plt.legend()
        
        plt.subplot(3, 1, 3)
        plt.plot(t_eval, err_z, label=f'Err Z (RMSE={rmse_z:.2f})')
        plt.ylabel('Error Z (m)')
        plt.xlabel('Time (s)')
        plt.grid(True)
        plt.legend()
        
        plt.tight_layout()
        plt.savefig('ct_error.png')
        print("Saved error plot to ct_error.png")

    # Plot Velocity
    plt.figure(figsize=(10, 8))
    plt.subplot(3, 1, 1)
    plt.plot(data['Time'], data['Vx'], label='Vx')
    plt.ylabel('Vx (m/s)')
    plt.title('Velocity')
    plt.grid(True)
    
    plt.subplot(3, 1, 2)
    plt.plot(data['Time'], data['Vy'], label='Vy')
    plt.ylabel('Vy (m/s)')
    plt.grid(True)
    
    plt.subplot(3, 1, 3)
    plt.plot(data['Time'], data['Vz'], label='Vz')
    plt.ylabel('Vz (m/s)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('ct_velocity.png')

    # Plot Bias
    plt.figure(figsize=(10, 8))
    plt.subplot(2, 1, 1)
    plt.plot(data['Time'], data['Bgx'], label='Bgx')
    plt.plot(data['Time'], data['Bgy'], label='Bgy')
    plt.plot(data['Time'], data['Bgz'], label='Bgz')
    plt.ylabel('Gyro Bias (rad/s)')
    plt.legend()
    plt.title('IMU Bias')
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(data['Time'], data['Bax'], label='Bax')
    plt.plot(data['Time'], data['Bay'], label='Bay')
    plt.plot(data['Time'], data['Baz'], label='Baz')
    plt.ylabel('Accel Bias (m/s^2)')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('ct_bias.png')
    
    print("Plots saved to ct_position.png, ct_velocity.png, ct_bias.png")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        plot_results(sys.argv[1])
    else:
        # Default path based on config
        plot_results(r"D:/Code/dataset/WID/Datasets/transformedData2/output/OB_GINS_CT.txt")
