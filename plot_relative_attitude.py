
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import os

# --- Configuration ---
output_dir = "D:/Code/dataset/wheel-ins2_dataset/converted/out/C1-Multi-Test/"
main_nav_file = os.path.join(output_dir, "OB_GINS_TXT.nav")
left_nav_file = os.path.join(output_dir, "OB_GINS_TXT_wheel_left.nav")
right_nav_file = os.path.join(output_dir, "OB_GINS_TXT_wheel_right.nav")

output_csv_file = "relative_attitudes.csv"
plot_left_file = "relative_attitude_left.png"
plot_right_file = "relative_attitude_right.png"


# --- Helper Functions ---
def euler_to_rotmat(roll, pitch, yaw):
    """Converts ZYX Euler angles (in degrees) to a rotation matrix."""
    return R.from_euler('zyx', [yaw, pitch, roll], degrees=True).as_matrix()

def rotmat_to_euler(rotmat):
    """Converts a rotation matrix to ZYX Euler angles (in degrees)."""
    return R.from_matrix(rotmat).as_euler('zyx', degrees=True)

def load_attitude_data(filepath):
    """Loads attitude data from a .nav file into a dictionary mapping time to attitude."""
    data = np.loadtxt(filepath, usecols=(1, 8, 9, 10)) # time, roll, pitch, yaw
    attitude_dict = {row[0]: row[1:] for row in data}
    return attitude_dict

# --- Main Logic ---
print("Loading attitude data...")
att_main = load_attitude_data(main_nav_file)
att_left = load_attitude_data(left_nav_file)
att_right = load_attitude_data(right_nav_file)

print("Calculating relative attitudes for all timestamps...")
results = []
# Use timestamps from the main IMU as the reference
common_timestamps = sorted(att_main.keys())

for t in common_timestamps:
    if t in att_left and t in att_right:
        # Get attitudes for the current timestamp
        rpy_main = att_main[t]
        rpy_left = att_left[t]
        rpy_right = att_right[t]

        # Convert to rotation matrices
        R_main = euler_to_rotmat(rpy_main[0], rpy_main[1], rpy_main[2])
        R_left = euler_to_rotmat(rpy_left[0], rpy_left[1], rpy_left[2])
        R_right = euler_to_rotmat(rpy_right[0], rpy_right[1], rpy_right[2])

        # Calculate rotation from Wheel to Main: R_main_from_wheel = R_wheel^T * R_main
        R_main_from_left = R_left.T @ R_main
        R_main_from_right = R_right.T @ R_main

        # Convert back to Euler angles [Yaw, Pitch, Roll]
        euler_main_from_left = rotmat_to_euler(R_main_from_left)
        euler_main_from_right = rotmat_to_euler(R_main_from_right)

        # Store results (time, roll, pitch, yaw)
        results.append([
            t,
            euler_main_from_left[2], euler_main_from_left[1], euler_main_from_left[0],
            euler_main_from_right[2], euler_main_from_right[1], euler_main_from_right[0]
        ])

# --- Save and Plot ---
if results:
    results_array = np.array(results)
    
    # Save to CSV
    print(f"Saving relative attitude data to {output_csv_file}...")
    np.savetxt(output_csv_file, results_array, delimiter=",",
               header="time,roll_L,pitch_L,yaw_L,roll_R,pitch_R,yaw_R",
               fmt="%.6f")

    # Plotting
    time = results_array[:, 0]
    
    # Plot for Left IMU
    print(f"Generating plot for Left IMU: {plot_left_file}")
    fig_l, axs_l = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    fig_l.suptitle('Relative Attitude: Main IMU from Left IMU', fontsize=16)
    
    axs_l[0].plot(time, results_array[:, 1])
    axs_l[0].set_ylabel('Roll (degrees)')
    axs_l[0].grid(True)
    
    axs_l[1].plot(time, results_array[:, 2])
    axs_l[1].set_ylabel('Pitch (degrees)')
    axs_l[1].grid(True)
    
    axs_l[2].plot(time, results_array[:, 3])
    axs_l[2].set_ylabel('Yaw (degrees)')
    axs_l[2].set_xlabel('Time (s)')
    axs_l[2].grid(True)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(plot_left_file)
    plt.close(fig_l)

    # Plot for Right IMU
    print(f"Generating plot for Right IMU: {plot_right_file}")
    fig_r, axs_r = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    fig_r.suptitle('Relative Attitude: Main IMU from Right IMU', fontsize=16)
    
    axs_r[0].plot(time, results_array[:, 4])
    axs_r[0].set_ylabel('Roll (degrees)')
    axs_r[0].grid(True)
    
    axs_r[1].plot(time, results_array[:, 5])
    axs_r[1].set_ylabel('Pitch (degrees)')
    axs_r[1].grid(True)
    
    axs_r[2].plot(time, results_array[:, 6])
    axs_r[2].set_ylabel('Yaw (degrees)')
    axs_r[2].set_xlabel('Time (s)')
    axs_r[2].grid(True)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(plot_right_file)
    plt.close(fig_r)

    print("Done.")
else:
    print("No common timestamps found to process.")

