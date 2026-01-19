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

    # Plot Position
    plt.figure(figsize=(10, 8))
    plt.subplot(3, 1, 1)
    plt.plot(data['Time'], data['Tx'], label='Tx')
    plt.ylabel('X (m)')
    plt.title('Position')
    plt.grid(True)
    
    plt.subplot(3, 1, 2)
    plt.plot(data['Time'], data['Ty'], label='Ty')
    plt.ylabel('Y (m)')
    plt.grid(True)
    
    plt.subplot(3, 1, 3)
    plt.plot(data['Time'], data['Tz'], label='Tz')
    plt.ylabel('Z (m)')
    plt.xlabel('Time (s)')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('ct_position.png')

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
